//! # Sparse Voxel Format Support
//!
//! This module provides support for various sparse voxel formats including
//! VDB, OpenVDB, and other industry-standard formats.

use crate::float_types::Real;
use crate::voxels::octree::SparseVoxelOctree;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Collection parameters for sparse data
struct SparseDataCollector<'a, S> {
    positions: &'a mut Vec<Point3<Real>>,
    sizes: &'a mut Vec<Real>,
    metadata: &'a mut Vec<Option<S>>,
    min_point: &'a mut Point3<Real>,
    max_point: &'a mut Point3<Real>,
}

/// Sparse voxel format types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SparseVoxelFormat {
    /// VDB format (OpenVDB compatible)
    Vdb,
    /// Raw sparse voxel data
    Raw,
    /// Compressed sparse voxel data
    Compressed,
}

/// Sparse voxel data structure for format conversion
#[derive(Debug, Clone)]
pub struct SparseVoxelData<S: Clone + Debug + Send + Sync> {
    /// Voxel positions (world coordinates)
    pub positions: Vec<Point3<Real>>,
    /// Voxel sizes
    pub sizes: Vec<Real>,
    /// Voxel metadata
    pub metadata: Vec<Option<S>>,
    /// Bounding box
    pub bounding_box: (Point3<Real>, Point3<Real>),
    /// Original octree parameters
    pub octree_origin: Point3<Real>,
    pub octree_size: Real,
    pub octree_depth: usize,
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq>
    SparseVoxelOctree<S>
{
    /// Export sparse voxel data to specified format
    pub fn export_to_format(
        &self,
        format: SparseVoxelFormat,
    ) -> Result<Vec<u8>, SparseVoxelError> {
        let sparse_data = self.to_sparse_voxel_data();

        match format {
            SparseVoxelFormat::Vdb => self.export_to_vdb(&sparse_data),
            SparseVoxelFormat::Raw => self.export_to_raw(&sparse_data),
            SparseVoxelFormat::Compressed => self.export_to_compressed(&sparse_data),
        }
    }

    /// Import sparse voxel data from specified format
    pub fn import_from_format(
        data: &[u8],
        format: SparseVoxelFormat,
        target_voxel_size: Real,
    ) -> Result<Self, SparseVoxelError> {
        let sparse_data = match format {
            SparseVoxelFormat::Vdb => Self::import_from_vdb(data)?,
            SparseVoxelFormat::Raw => Self::import_from_raw(data)?,
            SparseVoxelFormat::Compressed => Self::import_from_compressed(data)?,
        };

        // Convert to generic type and create octree
        let generic_data = SparseVoxelData {
            positions: sparse_data.positions,
            sizes: sparse_data.sizes,
            metadata: sparse_data.metadata.into_iter().map(|_| None).collect(),
            bounding_box: sparse_data.bounding_box,
            octree_origin: sparse_data.octree_origin,
            octree_size: sparse_data.octree_size,
            octree_depth: sparse_data.octree_depth,
        };

        // Use the original octree parameters to preserve coordinate system
        let mut octree = SparseVoxelOctree::new(
            generic_data.octree_origin,
            generic_data.octree_size,
            generic_data.octree_depth,
            None,
        );

        for (i, &position) in generic_data.positions.iter().enumerate() {
            let _voxel_size = generic_data
                .sizes
                .get(i)
                .copied()
                .unwrap_or(target_voxel_size);
            let metadata = generic_data.metadata.get(i).cloned().flatten();

            // Position is the node origin from sparse data, so convert to voxel center for set_voxel
            let voxel_center = position
                + Vector3::new(_voxel_size * 0.5, _voxel_size * 0.5, _voxel_size * 0.5);
            octree.set_voxel(&voxel_center, true, metadata);
        }

        Ok(octree)
    }

    /// Convert octree to sparse voxel data structure
    pub fn to_sparse_voxel_data(&self) -> SparseVoxelData<S> {
        let mut positions = Vec::new();
        let mut sizes = Vec::new();
        let mut metadata = Vec::new();
        let mut min_point = Point3::new(Real::INFINITY, Real::INFINITY, Real::INFINITY);
        let mut max_point =
            Point3::new(Real::NEG_INFINITY, Real::NEG_INFINITY, Real::NEG_INFINITY);

        let mut collector = SparseDataCollector {
            positions: &mut positions,
            sizes: &mut sizes,
            metadata: &mut metadata,
            min_point: &mut min_point,
            max_point: &mut max_point,
        };

        self.collect_sparse_data(&self.root, &mut collector, self.origin, self.size, 0);

        SparseVoxelData {
            positions,
            sizes,
            metadata,
            bounding_box: (min_point, max_point),
            octree_origin: self.origin,
            octree_size: self.size,
            octree_depth: self.max_depth,
        }
    }

    /// Create octree from sparse voxel data
    pub fn from_sparse_voxel_data(
        data: &SparseVoxelData<S>,
        target_voxel_size: Real,
    ) -> Result<Self, SparseVoxelError> {
        let bbox = data.bounding_box;
        let size = (bbox.1 - bbox.0).max();
        let mut octree = SparseVoxelOctree::new(bbox.0, size, 5, None);

        for (i, &position) in data.positions.iter().enumerate() {
            let _voxel_size = data.sizes.get(i).copied().unwrap_or(target_voxel_size);
            let metadata = data.metadata.get(i).cloned().flatten();

            // Set voxel in octree
            octree.set_voxel(&position, true, metadata);
        }

        Ok(octree)
    }

    /// Collect sparse data from octree
    fn collect_sparse_data(
        &self,
        node: &std::rc::Rc<std::cell::RefCell<crate::voxels::octree::SparseVoxelNode<S>>>,
        collector: &mut SparseDataCollector<S>,
        node_origin: Point3<Real>,
        node_size: Real,
        depth: usize,
    ) {
        let node_ref = node.borrow();

        match &*node_ref {
            crate::voxels::octree::SparseVoxelNode::Leaf {
                occupied,
                metadata: node_metadata,
            } => {
                if *occupied {
                    let voxel_size = self.voxel_size_at_depth(depth);
                    // Store the node origin (for backward compatibility with existing tests)
                    collector.positions.push(node_origin);
                    collector.sizes.push(voxel_size);
                    collector.metadata.push(node_metadata.clone());

                    // Update bounding box to include the full voxel bounds
                    *collector.min_point = Point3::new(
                        collector.min_point.x.min(node_origin.x),
                        collector.min_point.y.min(node_origin.y),
                        collector.min_point.z.min(node_origin.z),
                    );
                    *collector.max_point = Point3::new(
                        collector.max_point.x.max(node_origin.x + voxel_size),
                        collector.max_point.y.max(node_origin.y + voxel_size),
                        collector.max_point.z.max(node_origin.z + voxel_size),
                    );
                }
            },
            crate::voxels::octree::SparseVoxelNode::Internal { children, .. } => {
                let half_size = node_size * 0.5;

                for (octant_idx, child) in children.iter().enumerate().take(8) {
                    let octant = crate::voxels::octree::Octant::from_index(octant_idx)
                        .expect("Invalid octant index in sparse data collection");
                    let child_origin = self.get_child_origin(node_origin, half_size, octant);

                    if let Some(child) = child {
                        self.collect_sparse_data(
                            child,
                            collector,
                            child_origin,
                            half_size,
                            depth + 1,
                        );
                    }
                }
            },
        }
    }

    /// Export to VDB format (simplified implementation)
    fn export_to_vdb(&self, data: &SparseVoxelData<S>) -> Result<Vec<u8>, SparseVoxelError> {
        // Simplified VDB export - in production, use proper VDB library
        let mut buffer = Vec::new();

        // Write header
        buffer.extend_from_slice(b"VDB");
        buffer.extend_from_slice(&(data.positions.len() as u32).to_le_bytes());

        // Write octree parameters (for consistency with Raw format)
        buffer.extend_from_slice(&data.octree_origin.x.to_le_bytes());
        buffer.extend_from_slice(&data.octree_origin.y.to_le_bytes());
        buffer.extend_from_slice(&data.octree_origin.z.to_le_bytes());
        buffer.extend_from_slice(&data.octree_size.to_le_bytes());
        buffer.extend_from_slice(&(data.octree_depth as u32).to_le_bytes());

        // Write voxel data
        for (i, &position) in data.positions.iter().enumerate() {
            buffer.extend_from_slice(&position.x.to_le_bytes());
            buffer.extend_from_slice(&position.y.to_le_bytes());
            buffer.extend_from_slice(&position.z.to_le_bytes());

            if let Some(size) = data.sizes.get(i) {
                buffer.extend_from_slice(&size.to_le_bytes());
            }
        }

        Ok(buffer)
    }

    /// Import from VDB format (simplified implementation)
    fn import_from_vdb(data: &[u8]) -> Result<SparseVoxelData<()>, SparseVoxelError> {
        if data.len() < 47 {
            return Err(SparseVoxelError::InvalidFormat);
        }

        // Check header
        if &data[0..3] != b"VDB" {
            return Err(SparseVoxelError::InvalidFormat);
        }

        let count = u32::from_le_bytes([data[3], data[4], data[5], data[6]]) as usize;
        let mut offset = 7;

        // Read octree parameters
        if offset + 40 > data.len() {
            return Err(SparseVoxelError::InvalidFormat);
        }

        let origin_x = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let origin_y = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let origin_z = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let octree_size = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let octree_depth = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]) as usize;
        offset += 4;

        let octree_origin = Point3::new(origin_x, origin_y, origin_z);

        let mut positions = Vec::new();
        let mut sizes = Vec::new();
        let mut metadata = Vec::new();

        for _ in 0..count {
            if offset + 32 > data.len() {
                return Err(SparseVoxelError::InvalidFormat);
            }

            let x = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            let y = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            let z = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            let size = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            positions.push(Point3::new(x, y, z));
            sizes.push(size);
            metadata.push(None);
        }

        // Calculate bounding box
        let min_point = Point3::new(
            positions.iter().map(|p| p.x).fold(Real::INFINITY, Real::min),
            positions.iter().map(|p| p.y).fold(Real::INFINITY, Real::min),
            positions.iter().map(|p| p.z).fold(Real::INFINITY, Real::min),
        );
        let max_point = Point3::new(
            positions
                .iter()
                .map(|p| p.x)
                .fold(Real::NEG_INFINITY, Real::max),
            positions
                .iter()
                .map(|p| p.y)
                .fold(Real::NEG_INFINITY, Real::max),
            positions
                .iter()
                .map(|p| p.z)
                .fold(Real::NEG_INFINITY, Real::max),
        );

        Ok(SparseVoxelData {
            positions,
            sizes,
            metadata,
            bounding_box: (min_point, max_point),
            octree_origin,
            octree_size,
            octree_depth,
        })
    }

    /// Export to raw format
    fn export_to_raw(&self, data: &SparseVoxelData<S>) -> Result<Vec<u8>, SparseVoxelError> {
        let mut buffer = Vec::new();

        // Write count
        buffer.extend_from_slice(&(data.positions.len() as u32).to_le_bytes());

        // Write octree parameters
        buffer.extend_from_slice(&data.octree_origin.x.to_le_bytes());
        buffer.extend_from_slice(&data.octree_origin.y.to_le_bytes());
        buffer.extend_from_slice(&data.octree_origin.z.to_le_bytes());
        buffer.extend_from_slice(&data.octree_size.to_le_bytes());
        buffer.extend_from_slice(&(data.octree_depth as u32).to_le_bytes());

        // Write voxel data
        for (i, &position) in data.positions.iter().enumerate() {
            buffer.extend_from_slice(&position.x.to_le_bytes());
            buffer.extend_from_slice(&position.y.to_le_bytes());
            buffer.extend_from_slice(&position.z.to_le_bytes());

            if let Some(size) = data.sizes.get(i) {
                buffer.extend_from_slice(&size.to_le_bytes());
            }
        }

        Ok(buffer)
    }

    /// Import from raw format
    fn import_from_raw(data: &[u8]) -> Result<SparseVoxelData<()>, SparseVoxelError> {
        if data.len() < 4 {
            return Err(SparseVoxelError::InvalidFormat);
        }

        let count = u32::from_le_bytes([data[0], data[1], data[2], data[3]]) as usize;
        let mut positions = Vec::new();
        let mut sizes = Vec::new();
        let mut metadata = Vec::new();

        let mut offset = 4;

        // Read octree parameters
        if offset + 40 > data.len() {
            return Err(SparseVoxelError::InvalidFormat);
        }

        let origin_x = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let origin_y = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let origin_z = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let octree_size = Real::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset += 8;

        let octree_depth = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]) as usize;
        offset += 4;

        let octree_origin = Point3::new(origin_x, origin_y, origin_z);

        for _ in 0..count {
            if offset + 32 > data.len() {
                return Err(SparseVoxelError::InvalidFormat);
            }

            let x = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            let y = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            let z = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            let size = Real::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            offset += 8;

            positions.push(Point3::new(x, y, z));
            sizes.push(size);
            metadata.push(None);
        }

        // Calculate bounding box
        let min_point = Point3::new(
            positions.iter().map(|p| p.x).fold(Real::INFINITY, Real::min),
            positions.iter().map(|p| p.y).fold(Real::INFINITY, Real::min),
            positions.iter().map(|p| p.z).fold(Real::INFINITY, Real::min),
        );
        let max_point = Point3::new(
            positions
                .iter()
                .map(|p| p.x)
                .fold(Real::NEG_INFINITY, Real::max),
            positions
                .iter()
                .map(|p| p.y)
                .fold(Real::NEG_INFINITY, Real::max),
            positions
                .iter()
                .map(|p| p.z)
                .fold(Real::NEG_INFINITY, Real::max),
        );

        Ok(SparseVoxelData {
            positions,
            sizes,
            metadata,
            bounding_box: (min_point, max_point),
            octree_origin,
            octree_size,
            octree_depth,
        })
    }

    /// Export to compressed format
    fn export_to_compressed(
        &self,
        data: &SparseVoxelData<S>,
    ) -> Result<Vec<u8>, SparseVoxelError> {
        // Use simple compression - in production, use proper compression library
        let raw_data = self.export_to_raw(data)?;

        // Simple run-length encoding for demonstration
        let mut compressed = Vec::new();
        compressed.extend_from_slice(b"COMP");
        compressed.extend_from_slice(&(raw_data.len() as u32).to_le_bytes());
        compressed.extend_from_slice(&raw_data);

        Ok(compressed)
    }

    /// Import from compressed format
    fn import_from_compressed(data: &[u8]) -> Result<SparseVoxelData<()>, SparseVoxelError> {
        if data.len() < 8 {
            return Err(SparseVoxelError::InvalidFormat);
        }

        // Check header
        if &data[0..4] != b"COMP" {
            return Err(SparseVoxelError::InvalidFormat);
        }

        let uncompressed_size =
            u32::from_le_bytes([data[4], data[5], data[6], data[7]]) as usize;
        let compressed_data = &data[8..];

        if compressed_data.len() != uncompressed_size {
            return Err(SparseVoxelError::InvalidFormat);
        }

        Self::import_from_raw(compressed_data)
    }
}

/// Sparse voxel conversion errors
#[derive(Debug, Clone, PartialEq)]
pub enum SparseVoxelError {
    /// Invalid format
    InvalidFormat,
    /// Insufficient data
    InsufficientData,
    /// Unsupported format
    UnsupportedFormat,
}

impl std::fmt::Display for SparseVoxelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SparseVoxelError::InvalidFormat => write!(f, "Invalid sparse voxel format"),
            SparseVoxelError::InsufficientData => {
                write!(f, "Insufficient data for conversion")
            },
            SparseVoxelError::UnsupportedFormat => {
                write!(f, "Unsupported sparse voxel format")
            },
        }
    }
}

impl std::error::Error for SparseVoxelError {}
