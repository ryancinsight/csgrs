//! GPU buffer management for csgrs
//!
//! This module provides utilities for managing GPU buffers, data transfer,
//! and memory allocation for geometric operations.

use super::{GpuContext, GpuError, GpuResult};
use std::collections::HashMap;
use std::fmt::Debug;

/// GPU buffers for sparse voxel octree operations
pub struct VoxelGpuBuffers<'a> {
    /// Octree header buffer (uniform)
    pub header: &'a wgpu::Buffer,
    /// Node data buffer (storage)
    pub nodes: &'a wgpu::Buffer,
    /// Number of nodes in the octree
    pub node_count: usize,
}

/// GPU buffer pool for efficient memory management
pub struct BufferPool {
    context: GpuContext,
    buffers: Vec<wgpu::Buffer>,
    total_allocated: u64,
}

impl BufferPool {
    /// Create a new buffer pool
    pub const fn new(context: GpuContext) -> Self {
        Self {
            context,
            buffers: Vec::new(),
            total_allocated: 0,
        }
    }

    /// Allocate a new buffer
    pub fn allocate(
        &mut self,
        size: u64,
        usage: wgpu::BufferUsages,
        label: Option<&str>,
    ) -> GpuResult<&wgpu::Buffer> {
        let buffer = self.context.create_buffer(size, usage, label);
        self.buffers.push(buffer);
        self.total_allocated += size;
        Ok(self.buffers.last().unwrap())
    }

    /// Allocate buffer with initial data
    pub fn allocate_init(
        &mut self,
        usage: wgpu::BufferUsages,
        data: &[u8],
        label: Option<&str>,
    ) -> GpuResult<&wgpu::Buffer> {
        let buffer = self.context.create_buffer_init(usage, data, label);
        self.buffers.push(buffer);
        self.total_allocated += data.len() as u64;
        Ok(self.buffers.last().unwrap())
    }

    /// Get total allocated memory
    pub const fn total_allocated(&self) -> u64 {
        self.total_allocated
    }

    /// Convert sparse voxel octree to GPU buffers
    pub fn voxel_octree_to_gpu_buffers<S>(
        &mut self,
        octree: &crate::voxels::octree::SparseVoxelOctree<S>,
        label_prefix: &str,
    ) -> GpuResult<VoxelGpuBuffers<'_>>
    where
        S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
    {
        // Linearize the octree into GPU-compatible structures
        let (header, nodes) = self.linearize_voxel_octree(octree)?;

        // Create buffers using context directly to avoid borrow issues
        let header_vec = vec![header];
        let header_data = bytemuck::cast_slice(&header_vec);
        let nodes_data = bytemuck::cast_slice(&nodes);

        let header_buffer = self.context.create_buffer_init(
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            header_data,
            Some(&format!("{}_header", label_prefix)),
        );
        self.buffers.push(header_buffer);
        self.total_allocated += header_data.len() as u64;

        let nodes_buffer = self.context.create_buffer_init(
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            nodes_data,
            Some(&format!("{}_nodes", label_prefix)),
        );
        self.buffers.push(nodes_buffer);
        self.total_allocated += nodes_data.len() as u64;

        // Return references to the buffers in the pool
        let buffer_count = self.buffers.len();
        Ok(VoxelGpuBuffers {
            header: &self.buffers[buffer_count - 2],
            nodes: &self.buffers[buffer_count - 1],
            node_count: nodes.len(),
        })
    }

    /// Linearize sparse voxel octree for GPU processing
    fn linearize_voxel_octree<S>(
        &self,
        octree: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<(super::GpuVoxelHeader, Vec<super::GpuVoxelNode>)>
    where
        S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
    {
        use std::collections::HashMap;

        let mut nodes = Vec::new();
        let mut node_map = HashMap::new();
        let mut node_index = 0;

        // Recursively traverse and linearize the octree
        self.linearize_node(
            &octree.root,
            &mut nodes,
            &mut node_map,
            &mut node_index,
            0, // root depth
            octree.origin,
            octree.size,
        );

        let header = super::GpuVoxelHeader {
            node_count: nodes.len() as u32,
            max_depth: octree.max_depth as u32,
            root_index: 0,
            occupied_leaves: octree.occupied_leaves as u32,
            origin: [octree.origin.x as f32, octree.origin.y as f32, octree.origin.z as f32],
            root_size: octree.size as f32,
        };

        Ok((header, nodes))
    }

    /// Recursively linearize a voxel node
    #[allow(clippy::too_many_arguments)]
    fn linearize_node<S>(
        &self,
        node_rc: &std::rc::Rc<std::cell::RefCell<crate::voxels::octree::SparseVoxelNode<S>>>,
        nodes: &mut Vec<super::GpuVoxelNode>,
        node_map: &mut HashMap<usize, usize>,
        node_index: &mut usize,
        depth: u32,
        origin: nalgebra::Point3<crate::float_types::Real>,
        size: crate::float_types::Real,
    ) where
        S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
    {

        let node_ptr = std::rc::Rc::as_ptr(node_rc) as usize;
        if node_map.contains_key(&node_ptr) {
            return; // Node already processed
        }

        let current_index = *node_index;
        node_map.insert(node_ptr, current_index);
        *node_index += 1;

        let node_ref = node_rc.borrow();

        match &*node_ref {
            crate::voxels::octree::SparseVoxelNode::Leaf { occupied, .. } => {
                let gpu_node = super::GpuVoxelNode {
                    node_type: if *occupied { 3 } else { 2 }, // 2=empty leaf, 3=occupied leaf
                    children: [-1; 8],
                    depth,
                    origin: [origin.x as f32, origin.y as f32, origin.z as f32],
                    size: size as f32,
                    metadata_index: -1, // Metadata support planned for future enhancement
                };
                nodes.push(gpu_node);
            }
            crate::voxels::octree::SparseVoxelNode::Internal { children, .. } => {
                // Process children first to populate the node_map
                for (i, child_option) in children.iter().enumerate() {
                    if let Some(child_rc) = child_option {
                        let child_origin = self.child_origin(origin, size, i);
                        let child_size = size * 0.5;
                        self.linearize_node(
                            child_rc,
                            nodes,
                            node_map,
                            node_index,
                            depth + 1,
                            child_origin,
                            child_size,
                        );
                    }
                }

                // Now collect child indices and create the parent node
                let mut child_indices = [-1i32; 8];
                for (i, child_option) in children.iter().enumerate() {
                    if let Some(child_rc) = child_option {
                        child_indices[i] = node_map[&(std::rc::Rc::as_ptr(child_rc) as usize)] as i32;
                    }
                }

                let gpu_node = super::GpuVoxelNode {
                    node_type: 0, // 0=internal node
                    children: child_indices,
                    depth,
                    origin: [origin.x as f32, origin.y as f32, origin.z as f32],
                    size: size as f32,
                    metadata_index: -1,
                };
                nodes.push(gpu_node);
            }
        }
    }

    /// Calculate child node origin for given octant
    fn child_origin(
        &self,
        parent_origin: nalgebra::Point3<crate::float_types::Real>,
        parent_size: crate::float_types::Real,
        octant: usize,
    ) -> nalgebra::Point3<crate::float_types::Real> {
        let child_size = parent_size * 0.5;
        let x_offset = if (octant & 1) != 0 { child_size } else { 0.0 };
        let y_offset = if (octant & 2) != 0 { child_size } else { 0.0 };
        let z_offset = if (octant & 4) != 0 { child_size } else { 0.0 };

        nalgebra::Point3::new(
            parent_origin.x + x_offset,
            parent_origin.y + y_offset,
            parent_origin.z + z_offset,
        )
    }

    /// Clear all buffers
    pub fn clear(&mut self) {
        self.buffers.clear();
        self.total_allocated = 0;
    }

    /// Get buffer count
    pub fn buffer_count(&self) -> usize {
        self.buffers.len()
    }
}

/// Buffer utilities for data transfer
pub struct BufferUtils<'a> {
    context: &'a GpuContext,
}

impl<'a> BufferUtils<'a> {
    /// Create new buffer utilities
    pub const fn new(context: &'a GpuContext) -> Self {
        Self { context }
    }

    /// Copy data from CPU to GPU buffer
    pub async fn upload_data<T: bytemuck::Pod>(
        &self,
        buffer: &wgpu::Buffer,
        data: &[T],
    ) -> GpuResult<()> {
        let bytes = bytemuck::cast_slice(data);

        // Create staging buffer
        let staging_buffer = self.context.create_buffer_init(
            wgpu::BufferUsages::COPY_SRC,
            bytes,
            Some("staging-upload"),
        );

        // Copy from staging to target buffer
        let mut encoder =
            self.context
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("upload-encoder"),
                });

        encoder.copy_buffer_to_buffer(&staging_buffer, 0, buffer, 0, bytes.len() as u64);

        self.context.submit(encoder);
        self.context.poll();

        Ok(())
    }

    /// Copy data from GPU buffer to CPU
    pub async fn download_data<T: bytemuck::Pod + Default + Clone>(
        &self,
        buffer: &wgpu::Buffer,
        size: usize,
    ) -> GpuResult<Vec<T>> {
        // Create staging buffer for download
        let staging_buffer = self.context.create_buffer(
            (size * std::mem::size_of::<T>()) as u64,
            wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            Some("staging-download"),
        );

        // Copy from source to staging buffer
        let mut encoder =
            self.context
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("download-encoder"),
                });

        encoder.copy_buffer_to_buffer(
            buffer,
            0,
            &staging_buffer,
            0,
            (size * std::mem::size_of::<T>()) as u64,
        );

        self.context.submit(encoder);
        self.context.poll();

        // Map buffer for reading
        let buffer_slice = staging_buffer.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();

        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            tx.send(result).unwrap();
        });

        self.context.device().poll(wgpu::Maintain::Wait);

        match rx.recv().unwrap() {
            Ok(()) => {
                let data = buffer_slice.get_mapped_range();
                let result: Vec<T> = data
                    .chunks_exact(std::mem::size_of::<T>())
                    .map(|chunk| *bytemuck::from_bytes(chunk))
                    .collect();
                drop(data);
                staging_buffer.unmap();
                Ok(result)
            },
            Err(e) => Err(GpuError::TransferError(format!(
                "Buffer mapping failed: {:?}",
                e
            ))),
        }
    }

    /// Copy between GPU buffers
    pub fn copy_buffer(
        &self,
        source: &wgpu::Buffer,
        destination: &wgpu::Buffer,
        size: u64,
    ) -> GpuResult<()> {
        let mut encoder =
            self.context
                .device()
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("buffer-copy-encoder"),
                });

        encoder.copy_buffer_to_buffer(source, 0, destination, 0, size);
        self.context.submit(encoder);

        Ok(())
    }
}

/// Memory layout utilities for GPU data structures
pub mod layout {
    use super::*;

    /// Calculate aligned buffer size
    pub const fn aligned_size(size: usize, alignment: usize) -> usize {
        (size + alignment - 1) & !(alignment - 1)
    }

    /// Get WGSL struct alignment requirements
    pub const fn wgsl_alignment() -> usize {
        16 // WGSL requires 16-byte alignment for most types
    }

    /// Calculate buffer size for array of T
    pub const fn array_buffer_size<T>(count: usize) -> u64 {
        (count * std::mem::size_of::<T>()) as u64
    }

    /// Validate buffer size against data
    pub fn validate_buffer_size<T: bytemuck::Pod>(
        buffer: &wgpu::Buffer,
        data: &[T],
    ) -> GpuResult<()> {
        let expected_size = std::mem::size_of_val(data) as u64;
        if buffer.size() < expected_size {
            return Err(GpuError::BufferError(format!(
                "Buffer too small: {} < {}",
                buffer.size(),
                expected_size
            )));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpu::GpuVertex;

    #[tokio::test]
    async fn test_buffer_pool_allocation() {
        match GpuContext::new().await {
            Ok(gpu) => {
                let mut pool = BufferPool::new(gpu.clone());

                // Test buffer allocation
                let buffer = pool.allocate(1024, wgpu::BufferUsages::STORAGE, Some("test"));
                assert!(buffer.is_ok());

                // Check allocation tracking
                assert_eq!(pool.buffer_count(), 1);
                assert_eq!(pool.total_allocated(), 1024);

                // Test buffer init
                let data = vec![1u32, 2u32, 3u32];
                let init_buffer = pool.allocate_init(
                    wgpu::BufferUsages::STORAGE,
                    bytemuck::cast_slice(&data),
                    Some("test-init"),
                );
                assert!(init_buffer.is_ok());
                assert_eq!(pool.buffer_count(), 2);
                assert_eq!(pool.total_allocated(), 1024 + 12);

                // Test clear
                pool.clear();
                assert_eq!(pool.buffer_count(), 0);
                assert_eq!(pool.total_allocated(), 0);
            },
            Err(_) => {
                // Skip test if GPU unavailable
            },
        }
    }

    #[tokio::test]
    async fn test_buffer_utils_data_transfer() {
        match GpuContext::new().await {
            Ok(gpu) => {
                let utils = BufferUtils::new(&gpu);

                // Test data upload/download roundtrip
                let original_data = vec![
                    GpuVertex {
                        position: [1.0, 2.0, 3.0],
                        normal: [0.0, 1.0, 0.0],
                        metadata: 42,
                    },
                    GpuVertex {
                        position: [4.0, 5.0, 6.0],
                        normal: [0.0, 0.0, 1.0],
                        metadata: 43,
                    },
                ];

                // Create buffer
                let buffer = gpu.create_buffer(
                    (original_data.len() * std::mem::size_of::<GpuVertex>()) as u64,
                    wgpu::BufferUsages::STORAGE
                        | wgpu::BufferUsages::COPY_DST
                        | wgpu::BufferUsages::COPY_SRC,
                    Some("test-transfer"),
                );

                // Upload data
                let upload_result = utils.upload_data(&buffer, &original_data).await;
                assert!(upload_result.is_ok());

                // Download data
                let downloaded_data: Vec<GpuVertex> = utils
                    .download_data(&buffer, original_data.len())
                    .await
                    .unwrap();

                // Verify roundtrip
                assert_eq!(downloaded_data.len(), original_data.len());
                for (orig, down) in original_data.iter().zip(downloaded_data.iter()) {
                    assert_eq!(orig.position, down.position);
                    assert_eq!(orig.normal, down.normal);
                    assert_eq!(orig.metadata, down.metadata);
                }
            },
            Err(_) => {
                // Skip test if GPU unavailable
            },
        }
    }

    #[test]
    fn test_layout_utilities() {
        // Test alignment calculations
        assert_eq!(layout::aligned_size(10, 16), 16);
        assert_eq!(layout::aligned_size(16, 16), 16);
        assert_eq!(layout::aligned_size(17, 16), 32);

        // Test array buffer size
        assert_eq!(layout::array_buffer_size::<u32>(4), 16);
        assert_eq!(layout::array_buffer_size::<f32>(3), 12);
    }
}
