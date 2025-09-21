//! GPU-accelerated operations for csgrs
//!
//! This module provides a framework for GPU-accelerated geometric operations.
//! Currently implements placeholder functions that fall back to CPU operations.
//! Future versions will implement actual GPU compute shaders.

use super::{GpuContext, GpuResult};
#[cfg(test)]
use super::GpuError;
use crate::indexed_mesh::IndexedMesh;

/// GPU-accelerated boolean operations trait
pub trait GpuBooleanOps<S: Clone + Send + Sync + std::fmt::Debug> {
    /// Perform union operation on GPU
    fn gpu_union(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> impl std::future::Future<Output = GpuResult<IndexedMesh<S>>> + Send;

    /// Perform difference operation on GPU
    fn gpu_difference(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> impl std::future::Future<Output = GpuResult<IndexedMesh<S>>> + Send;

    /// Perform intersection operation on GPU
    fn gpu_intersection(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> impl std::future::Future<Output = GpuResult<IndexedMesh<S>>> + Send;

    /// Perform XOR operation on GPU
    fn gpu_xor(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> impl std::future::Future<Output = GpuResult<IndexedMesh<S>>> + Send;
}

/// GPU-accelerated voxel operations trait
pub trait GpuVoxelOps<S: Clone + Send + Sync + std::fmt::Debug + std::hash::Hash + std::cmp::PartialEq> {
    /// Perform union operation on GPU for voxel octrees (currently CPU fallback)
    fn gpu_voxel_union(
        &self,
        gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>>;

    /// Perform difference operation on GPU for voxel octrees (currently CPU fallback)
    fn gpu_voxel_difference(
        &self,
        gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>>;

    /// Perform intersection operation on GPU for voxel octrees (currently CPU fallback)
    fn gpu_voxel_intersection(
        &self,
        gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>>;

    /// Perform XOR operation on GPU for voxel octrees (currently CPU fallback)
    fn gpu_voxel_xor(
        &self,
        gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>>;
}

/// Check if GPU acceleration is beneficial for the given meshes
pub fn should_use_gpu(
    mesh1: &IndexedMesh<impl Clone + Send + Sync + std::fmt::Debug>,
    mesh2: &IndexedMesh<impl Clone + Send + Sync + std::fmt::Debug>,
) -> bool {
    // Determine if GPU acceleration is beneficial based on mesh characteristics

    let total_vertices = mesh1.vertices.len() + mesh2.vertices.len();
    let total_faces = mesh1.faces.len() + mesh2.faces.len();

    // GPU acceleration is beneficial for:
    // - Large meshes (> 10K vertices)
    // - Complex boolean operations (> 5K faces)
    // - Meshes that would benefit from parallel processing

    const GPU_THRESHOLD_VERTICES: usize = 10_000;
    const GPU_THRESHOLD_FACES: usize = 5_000;

    total_vertices > GPU_THRESHOLD_VERTICES || total_faces > GPU_THRESHOLD_FACES
}

// GPU data structures for compute shaders
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Default)]
#[repr(C)]
struct GpuMeshHeader {
    vertex_count: u32,
    face_count: u32,
    max_vertices: u32,
    max_faces: u32,
}

#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Default)]
#[repr(C)]
struct GpuFace {
    vertices: [u32; 3],
    normal: [f32; 3],
}

// Implement GpuBooleanOps for IndexedMesh
impl<S: Clone + Send + Sync + std::fmt::Debug> GpuBooleanOps<S> for IndexedMesh<S> {
    async fn gpu_union(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> GpuResult<IndexedMesh<S>> {
        self.execute_boolean_operation(gpu, other, 0).await // 0 = union
    }

    async fn gpu_difference(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> GpuResult<IndexedMesh<S>> {
        self.execute_boolean_operation(gpu, other, 1).await // 1 = difference
    }

    async fn gpu_intersection(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> GpuResult<IndexedMesh<S>> {
        self.execute_boolean_operation(gpu, other, 2).await // 2 = intersection
    }

    async fn gpu_xor(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
    ) -> GpuResult<IndexedMesh<S>> {
        self.execute_boolean_operation(gpu, other, 3).await // 3 = XOR
    }
}

impl<S: Clone + Send + Sync + std::fmt::Debug> IndexedMesh<S> {
    /// Execute boolean operation on GPU
    async fn execute_boolean_operation(
        &self,
        gpu: &GpuContext,
        other: &IndexedMesh<S>,
        operation_type: u32,
    ) -> GpuResult<IndexedMesh<S>> {

        // Calculate output mesh size (conservative estimate)
        let max_vertices = self.vertices.len() + other.vertices.len();
        let max_faces = self.faces.len() + other.faces.len();

        // Prepare GPU data structures
        let mesh1_header = GpuMeshHeader {
            vertex_count: self.vertices.len() as u32,
            face_count: self.faces.len() as u32,
            max_vertices: max_vertices as u32,
            max_faces: max_faces as u32,
        };

        let mesh2_header = GpuMeshHeader {
            vertex_count: other.vertices.len() as u32,
            face_count: other.faces.len() as u32,
            max_vertices: max_vertices as u32,
            max_faces: max_faces as u32,
        };

        let output_header = GpuMeshHeader {
            vertex_count: 0, // Will be computed by shader
            face_count: 0,   // Will be computed by shader
            max_vertices: max_vertices as u32,
            max_faces: max_faces as u32,
        };

        // Convert vertices to GPU format
        let mesh1_gpu_vertices: Vec<super::GpuVertex> = self.vertices
            .iter()
            .map(|v| super::GpuVertex {
                position: [v.pos.x as f32, v.pos.y as f32, v.pos.z as f32],
                normal: [0.0, 0.0, 0.0], // Will be computed
                metadata: 0,
            })
            .collect();

        let mesh2_gpu_vertices: Vec<super::GpuVertex> = other.vertices
            .iter()
            .map(|v| super::GpuVertex {
                position: [v.pos.x as f32, v.pos.y as f32, v.pos.z as f32],
                normal: [0.0, 0.0, 0.0], // Will be computed
                metadata: 0,
            })
            .collect();

        // Convert faces to GPU format
        let mesh1_gpu_faces: Vec<GpuFace> = self.faces
            .iter()
            .map(|f| GpuFace {
                vertices: [f.vertices[0] as u32, f.vertices[1] as u32, f.vertices[2] as u32],
                normal: [0.0, 0.0, 0.0], // Will be computed
            })
            .collect();

        let mesh2_gpu_faces: Vec<GpuFace> = other.faces
            .iter()
            .map(|f| GpuFace {
                vertices: [f.vertices[0] as u32, f.vertices[1] as u32, f.vertices[2] as u32],
                normal: [0.0, 0.0, 0.0], // Will be computed
            })
            .collect();

        // Create GPU buffers

        // Input buffers
        let mesh1_header_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::UNIFORM,
            bytemuck::bytes_of(&mesh1_header),
            Some("mesh1-header"),
        );

        let mesh1_vertex_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::STORAGE,
            bytemuck::cast_slice(&mesh1_gpu_vertices),
            Some("mesh1-vertices"),
        );

        let mesh1_face_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::STORAGE,
            bytemuck::cast_slice(&mesh1_gpu_faces),
            Some("mesh1-faces"),
        );

        let mesh2_header_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::UNIFORM,
            bytemuck::bytes_of(&mesh2_header),
            Some("mesh2-header"),
        );

        let mesh2_vertex_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::STORAGE,
            bytemuck::cast_slice(&mesh2_gpu_vertices),
            Some("mesh2-vertices"),
        );

        let mesh2_face_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::STORAGE,
            bytemuck::cast_slice(&mesh2_gpu_faces),
            Some("mesh2-faces"),
        );

        let operation_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::UNIFORM,
            bytemuck::bytes_of(&operation_type),
            Some("operation-type"),
        );

        // Output buffers
        let output_vertex_buf = gpu.create_buffer(
            (max_vertices * std::mem::size_of::<super::GpuVertex>()) as u64,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            Some("output-vertices"),
        );

        let output_face_buf = gpu.create_buffer(
            (max_faces * std::mem::size_of::<GpuFace>()) as u64,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            Some("output-faces"),
        );

        let output_header_buf = gpu.create_buffer_init(
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_SRC,
            bytemuck::bytes_of(&output_header),
            Some("output-header"),
        );

        // Get compute pipeline and bind group layout
        let mut pipelines = super::pipeline::ComputePipelines::new(gpu.clone());
        pipelines.boolean_ops_pipeline()?; // Create the pipeline and layout

        // Create bind group in a separate scope to avoid borrowing conflicts
        let bind_group = {
            let bind_group_layout = pipelines.get_bind_group_layout("boolean_ops_layout")
                .ok_or_else(|| super::GpuError::PipelineCreation("Bind group layout not found".to_string()))?;

            gpu.create_bind_group(
                bind_group_layout,
                &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &mesh1_header_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &mesh1_vertex_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &mesh1_face_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &mesh2_header_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &mesh2_vertex_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &mesh2_face_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &output_vertex_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &output_face_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &output_header_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                    wgpu::BindGroupEntry {
                        binding: 9,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: &operation_buf,
                            offset: 0,
                            size: None,
                        }),
                    },
                ],
                Some("boolean-ops-bind-group"),
            )
        };

        // Get pipeline reference
        let pipeline = pipelines.boolean_ops_pipeline()?;

        // Execute compute shader
        let mut encoder = gpu.device().create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("boolean-ops-encoder"),
        });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("boolean-ops-pass"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);

            // Dispatch workgroups (one per face, rounded up)
            let workgroups = (mesh1_header.face_count + mesh2_header.face_count).div_ceil(256).max(1);
            compute_pass.dispatch_workgroups(workgroups, 1, 1);
        }

        gpu.submit(encoder);
        gpu.poll();

        // Extract results from GPU buffers
        // Current implementation provides framework with CPU fallback for correctness
        // Future GPU shader implementations will replace CPU operations with native GPU computation

        // Read back the output header to get actual result counts
        let output_header_data: Vec<GpuMeshHeader> = super::buffers::BufferUtils::new(gpu)
            .download_data(&output_header_buf, 1)
            .await?;

        let result_header = output_header_data[0];

        // If GPU computation produced valid results, extract them
        if result_header.vertex_count > 0 && result_header.face_count > 0 {
            // Read back vertices and faces from GPU
            let gpu_vertices: Vec<super::GpuVertex> = super::buffers::BufferUtils::new(gpu)
                .download_data(&output_vertex_buf, result_header.vertex_count as usize)
                .await?;

            let gpu_faces: Vec<GpuFace> = super::buffers::BufferUtils::new(gpu)
                .download_data(&output_face_buf, result_header.face_count as usize)
                .await?;

            // Convert back to IndexedMesh format
            let vertices: Vec<nalgebra::Point3<f64>> = gpu_vertices
                .into_iter()
                .map(|v| nalgebra::Point3::new(v.position[0] as f64, v.position[1] as f64, v.position[2] as f64))
                .collect();

            let faces: Vec<Vec<usize>> = gpu_faces
                .into_iter()
                .map(|f| vec![f.vertices[0] as usize, f.vertices[1] as usize, f.vertices[2] as usize])
                .collect();

            // Create result mesh
            let result_mesh = crate::indexed_mesh::IndexedMesh::from_vertices_and_faces(vertices, faces, None::<S>);
            // Log successful GPU computation
            println!("GPU computation completed: {} vertices, {} faces",
                   result_mesh.vertices.len(), result_mesh.faces.len());
            return Ok(result_mesh);
        }

        // Fallback to CPU implementation if GPU computation failed or produced invalid results
        println!("GPU computation incomplete, using CPU fallback");
        use crate::traits::CSG;
        match operation_type {
            0 => Ok(self.union(other)),
            1 => Ok(self.difference(other)),
            2 => Ok(self.intersection(other)),
            3 => Ok(self.xor(other)),
            _ => Err(super::GpuError::Unsupported("Invalid operation type".to_string())),
        }
    }
}

#[cfg(test)]
// Implement GpuVoxelOps for SparseVoxelOctree
impl<S: Clone + Send + Sync + std::fmt::Debug + std::hash::Hash + std::cmp::PartialEq> GpuVoxelOps<S>
    for crate::voxels::octree::SparseVoxelOctree<S>
{
    fn gpu_voxel_union(
        &self,
        _gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>> {
        // Current implementation falls back to CPU operations
        // Future GPU shader implementations will replace this with actual GPU computation
        Ok(crate::traits::CSG::union(self, other))
    }

    fn gpu_voxel_difference(
        &self,
        _gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>> {
        // Current implementation falls back to CPU operations
        // Future GPU shader implementations will replace this with actual GPU computation
        Ok(crate::traits::CSG::difference(self, other))
    }

    fn gpu_voxel_intersection(
        &self,
        _gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>> {
        // Current implementation falls back to CPU operations
        // Future GPU shader implementations will replace this with actual GPU computation
        Ok(crate::traits::CSG::intersection(self, other))
    }

    fn gpu_voxel_xor(
        &self,
        _gpu: &GpuContext,
        other: &crate::voxels::octree::SparseVoxelOctree<S>,
    ) -> GpuResult<crate::voxels::octree::SparseVoxelOctree<S>> {
        // Current implementation falls back to CPU operations
        // Future GPU shader implementations will replace this with actual GPU computation
        Ok(crate::traits::CSG::xor(self, other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::indexed_mesh::shapes;

    #[tokio::test]
    async fn test_gpu_operations_fallback() {
        // Test fallback behavior when GPU unavailable
        match GpuContext::new().await {
            Ok(gpu) => {
                let cube1 = shapes::cube::<()>(2.0, None);
                let cube2 = shapes::cube::<()>(1.0, None);

                // Test union operation
                let result = cube1.gpu_union(&gpu, &cube2).await;
                assert!(result.is_ok());

                // Test that result has reasonable vertex/face counts
                let result_mesh = result.unwrap();
                assert!(!result_mesh.vertices.is_empty());
                assert!(!result_mesh.faces.is_empty());
            },
            Err(GpuError::NotAvailable(_)) => {
                // Expected on systems without GPU
                println!("GPU not available, skipping GPU tests");
            },
            Err(e) => panic!("Unexpected GPU error: {:?}", e),
        }
    }

    #[test]
    fn test_should_use_gpu() {
        let small_cube = shapes::cube::<()>(1.0, None);
        let large_cube = shapes::cube::<()>(10.0, None); // More vertices

        // For now, GPU is always disabled (placeholder implementation)
        assert!(!should_use_gpu(&small_cube, &large_cube));
    }

    #[tokio::test]
    async fn test_gpu_voxel_operations_fallback() {
        use crate::voxels::octree::SparseVoxelOctree;

        // Create test voxel octrees
        let octree1 = SparseVoxelOctree::<()>::new(
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            2.0,
            3, // Small depth for testing
            None,
        );

        let octree2 = SparseVoxelOctree::<()>::new(
            nalgebra::Point3::new(0.0, 0.0, 0.0), // Same origin for CSG compatibility
            2.0,
            3,
            None,
        );

        // Test CPU fallback (create GPU context but operations fall back to CPU)
        match GpuContext::new().await {
            Ok(gpu) => {
                let result = octree1.gpu_voxel_union(&gpu, &octree2);
                assert!(result.is_ok());

                let result_octree = result.unwrap();
                assert!(result_octree.total_nodes >= octree1.total_nodes);
            },
            Err(GpuError::NotAvailable(_)) => {
                // Skip test if GPU unavailable (this is expected behavior)
                println!("GPU not available, skipping voxel GPU tests");
            },
            Err(e) => panic!("Unexpected GPU error: {:?}", e),
        }
    }
}
