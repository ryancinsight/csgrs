//! Conversion utilities between Mesh and SvoMesh
//!
//! This module provides efficient conversion methods between the traditional BSP-based
//! Mesh structure and the new sparse voxel octree SvoMesh structure, enabling seamless
//! interoperability and migration paths.
//!
//! ## Key Features
//!
//! - **Zero-Copy Conversions**: Where possible, avoid data duplication
//! - **Metadata Preservation**: Maintain all metadata during conversions
//! - **Precision Control**: Handle precision differences between representations
//! - **Performance Optimization**: Efficient algorithms for large meshes

use crate::float_types::Real;
use crate::mesh::{polygon::Polygon, Mesh};
use crate::traits::CSG;
use crate::voxels::{precision::PrecisionConfig, svo_mesh::SvoMesh};
use nalgebra::Point3;
use crate::float_types::parry3d::bounding_volume::{Aabb, BoundingVolume};
use std::fmt::Debug;

/// Conversion trait for transforming between mesh representations
pub trait MeshConversion<S: Clone + Send + Sync + Debug> {
    /// Convert to SvoMesh with default precision
    fn to_svo_mesh(&self) -> SvoMesh<S>;
    
    /// Convert to SvoMesh with custom precision configuration
    fn to_svo_mesh_with_precision(&self, precision: PrecisionConfig) -> SvoMesh<S>;
    
    /// Convert from SvoMesh
    fn from_svo_mesh(svo_mesh: &SvoMesh<S>) -> Self;
}

/// Implementation for Mesh -> SvoMesh conversion
impl<S: Clone + Send + Sync + Debug> MeshConversion<S> for Mesh<S> {
    fn to_svo_mesh(&self) -> SvoMesh<S> {
        self.to_svo_mesh_with_precision(PrecisionConfig::default())
    }
    
    fn to_svo_mesh_with_precision(&self, precision: PrecisionConfig) -> SvoMesh<S> {
        // Extract polygons from the BSP tree
        let polygons = &self.polygons;
        
        // Create SvoMesh with the extracted polygons
        let mut svo_mesh = SvoMesh::with_precision(precision);
        svo_mesh.metadata = self.metadata.clone();
        
        if !polygons.is_empty() {
            svo_mesh.insert_polygons(&polygons);
        }
        
        svo_mesh
    }
    
    fn from_svo_mesh(svo_mesh: &SvoMesh<S>) -> Self {
        // Extract polygons from SvoMesh
        let polygons = svo_mesh.polygons();
        
        // Create traditional Mesh from polygons
        let mesh = Mesh::from_polygons(&polygons, svo_mesh.metadata.clone());
        
        mesh
    }
}

/// Implementation for SvoMesh -> Mesh conversion
impl<S: Clone + Send + Sync + Debug> MeshConversion<S> for SvoMesh<S> {
    fn to_svo_mesh(&self) -> SvoMesh<S> {
        self.clone()
    }
    
    fn to_svo_mesh_with_precision(&self, precision: PrecisionConfig) -> SvoMesh<S> {
        if self.precision_config == precision {
            self.clone()
        } else {
            // Need to rebuild with new precision
            let polygons = self.polygons();
            let mut new_mesh = SvoMesh::with_precision(precision);
            new_mesh.metadata = self.metadata.clone();
            
            if !polygons.is_empty() {
                new_mesh.insert_polygons(&polygons);
            }
            
            new_mesh
        }
    }
    
    fn from_svo_mesh(svo_mesh: &SvoMesh<S>) -> Self {
        svo_mesh.clone()
    }
}

/// Batch conversion utilities for collections of meshes
pub struct BatchConverter;

impl BatchConverter {
    /// Convert multiple Mesh instances to SvoMesh instances
    pub fn meshes_to_svo<S: Clone + Send + Sync + Debug>(
        meshes: &[Mesh<S>],
        precision: Option<PrecisionConfig>,
    ) -> Vec<SvoMesh<S>> {
        let precision = precision.unwrap_or_default();
        
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            meshes
                .par_iter()
                .map(|mesh| mesh.to_svo_mesh_with_precision(precision))
                .collect()
        }
        
        #[cfg(not(feature = "parallel"))]
        {
            meshes
                .iter()
                .map(|mesh| mesh.to_svo_mesh_with_precision(precision))
                .collect()
        }
    }
    
    /// Convert multiple SvoMesh instances to Mesh instances
    pub fn svo_to_meshes<S: Clone + Send + Sync + Debug>(
        svo_meshes: &[SvoMesh<S>],
    ) -> Vec<Mesh<S>> {
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            svo_meshes
                .par_iter()
                .map(|svo_mesh| Mesh::from_svo_mesh(svo_mesh))
                .collect()
        }
        
        #[cfg(not(feature = "parallel"))]
        {
            svo_meshes
                .iter()
                .map(|svo_mesh| Mesh::from_svo_mesh(svo_mesh))
                .collect()
        }
    }
    
    /// Merge multiple meshes into a single SvoMesh
    pub fn merge_to_svo<S: Clone + Send + Sync + Debug>(
        meshes: &[Mesh<S>],
        precision: Option<PrecisionConfig>,
    ) -> SvoMesh<S> {
        if meshes.is_empty() {
            return SvoMesh::new();
        }
        
        let precision = precision.unwrap_or_default();
        
        // Collect all polygons from all meshes
        let all_polygons: Vec<Polygon<S>> = meshes
            .iter()
            .flat_map(|mesh| mesh.polygons.iter().cloned())
            .collect();
        
        // Use metadata from first non-empty mesh
        let metadata = meshes
            .iter()
            .find_map(|mesh| mesh.metadata.clone());
        
        let mut result = SvoMesh::with_precision(precision);
        result.metadata = metadata;
        
        if !all_polygons.is_empty() {
            result.insert_polygons(&all_polygons);
        }
        
        result
    }
    
    /// Split an SvoMesh into multiple Mesh instances based on spatial regions
    pub fn split_svo_spatially<S: Clone + Send + Sync + Debug>(
        svo_mesh: &SvoMesh<S>,
        grid_size: usize,
    ) -> Vec<Mesh<S>> {
        if svo_mesh.is_empty() || grid_size == 0 {
            return vec![];
        }
        
        let bounds = svo_mesh.bounding_box();
        let polygons = svo_mesh.polygons();
        
        // Create spatial grid
        let grid_step = (bounds.maxs - bounds.mins) / (grid_size as Real);
        let mut grid_meshes = Vec::new();
        
        for x in 0..grid_size {
            for y in 0..grid_size {
                for z in 0..grid_size {
                    let cell_min = bounds.mins + Point3::new(
                        x as Real * grid_step.x,
                        y as Real * grid_step.y,
                        z as Real * grid_step.z,
                    ).coords;
                    
                    let cell_max = cell_min + grid_step;
                    let cell_bounds = Aabb::new(cell_min, cell_max);
                    
                    // Filter polygons that intersect this cell
                    let cell_polygons: Vec<_> = polygons
                        .iter()
                        .filter(|poly| {
                            let poly_bounds = poly.bounding_box();
                            cell_bounds.intersects(&poly_bounds)
                        })
                        .cloned()
                        .collect();
                    
                    if !cell_polygons.is_empty() {
                        let cell_mesh = Mesh::from_polygons(&cell_polygons, svo_mesh.metadata.clone());
                        grid_meshes.push(cell_mesh);
                    }
                }
            }
        }
        
        grid_meshes
    }
}

/// Conversion statistics for analyzing conversion efficiency
#[derive(Debug, Clone)]
pub struct ConversionStats {
    pub source_polygons: usize,
    pub target_polygons: usize,
    pub source_memory: usize,
    pub target_memory: usize,
    pub conversion_time_ms: u64,
    pub precision_loss: bool,
}

impl ConversionStats {
    /// Calculate memory efficiency ratio (target/source)
    pub fn memory_ratio(&self) -> f64 {
        if self.source_memory == 0 {
            0.0
        } else {
            self.target_memory as f64 / self.source_memory as f64
        }
    }
    
    /// Calculate polygon preservation ratio
    pub fn polygon_preservation(&self) -> f64 {
        if self.source_polygons == 0 {
            1.0
        } else {
            self.target_polygons as f64 / self.source_polygons as f64
        }
    }
}

/// Conversion utilities with performance monitoring
pub struct MonitoredConverter;

impl MonitoredConverter {
    /// Convert Mesh to SvoMesh with performance statistics
    pub fn mesh_to_svo_with_stats<S: Clone + Send + Sync + Debug>(
        mesh: &Mesh<S>,
        precision: Option<PrecisionConfig>,
    ) -> (SvoMesh<S>, ConversionStats) {
        let start_time = std::time::Instant::now();
        
        let source_polygons = mesh.polygons.len();
        let source_memory = std::mem::size_of_val(mesh); // Approximate
        
        let svo_mesh = match precision {
            Some(p) => mesh.to_svo_mesh_with_precision(p),
            None => mesh.to_svo_mesh(),
        };
        
        let target_polygons = svo_mesh.polygons().len();
        let target_memory = svo_mesh.memory_usage();
        let conversion_time_ms = start_time.elapsed().as_millis() as u64;
        
        let stats = ConversionStats {
            source_polygons,
            target_polygons,
            source_memory,
            target_memory,
            conversion_time_ms,
            precision_loss: false, // SVO typically preserves precision
        };
        
        (svo_mesh, stats)
    }
    
    /// Convert SvoMesh to Mesh with performance statistics
    pub fn svo_to_mesh_with_stats<S: Clone + Send + Sync + Debug>(
        svo_mesh: &SvoMesh<S>,
    ) -> (Mesh<S>, ConversionStats) {
        let start_time = std::time::Instant::now();
        
        let source_polygons = svo_mesh.polygons().len();
        let source_memory = svo_mesh.memory_usage();
        
        let mesh = Mesh::from_svo_mesh(svo_mesh);
        
        let target_polygons = mesh.polygons.len();
        let target_memory = std::mem::size_of_val(&mesh); // Approximate
        let conversion_time_ms = start_time.elapsed().as_millis() as u64;
        
        let stats = ConversionStats {
            source_polygons,
            target_polygons,
            source_memory,
            target_memory,
            conversion_time_ms,
            precision_loss: false, // Conversion preserves data
        };
        
        (mesh, stats)
    }
}

/// Validation utilities for ensuring conversion correctness
pub struct ConversionValidator;

impl ConversionValidator {
    /// Validate that conversion preserves geometric properties
    pub fn validate_geometric_preservation<S: Clone + Send + Sync + Debug + PartialEq>(
        original: &Mesh<S>,
        converted: &SvoMesh<S>,
        tolerance: Real,
    ) -> bool {
        // Check polygon count
        let orig_polygons = &original.polygons;
        let conv_polygons = converted.polygons();
        
        if orig_polygons.len() != conv_polygons.len() {
            return false;
        }
        
        // Check bounding boxes
        let orig_bounds = original.bounding_box();
        let conv_bounds = converted.bounding_box();
        
        let bounds_diff = (orig_bounds.maxs - conv_bounds.maxs).norm() +
                         (orig_bounds.mins - conv_bounds.mins).norm();
        
        if bounds_diff > tolerance {
            return false;
        }
        
        // Check metadata preservation
        original.metadata == converted.metadata
    }
    
    /// Validate round-trip conversion (Mesh -> SvoMesh -> Mesh)
    pub fn validate_round_trip<S: Clone + Send + Sync + Debug + PartialEq>(
        original: &Mesh<S>,
        tolerance: Real,
    ) -> bool {
        let svo_mesh = original.to_svo_mesh();
        let round_trip = Mesh::from_svo_mesh(&svo_mesh);
        
        // Compare polygon counts
        let orig_polygons = &original.polygons;
        let trip_polygons = &round_trip.polygons;
        
        if orig_polygons.len() != trip_polygons.len() {
            return false;
        }
        
        // Compare bounding boxes
        let orig_bounds = original.bounding_box();
        let trip_bounds = round_trip.bounding_box();
        
        let bounds_diff = (orig_bounds.maxs - trip_bounds.maxs).norm() +
                         (orig_bounds.mins - trip_bounds.mins).norm();
        
        bounds_diff <= tolerance as Real && original.metadata == round_trip.metadata
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Test imports removed as they were unused
    
    #[test]
    fn test_empty_mesh_conversion() {
        let mesh = Mesh::<()>::new();
        let svo_mesh = mesh.to_svo_mesh();
        
        assert!(svo_mesh.is_empty());
        assert_eq!(svo_mesh.polygons().len(), 0);
        
        let back_to_mesh = Mesh::from_svo_mesh(&svo_mesh);
        assert!(back_to_mesh.polygons.is_empty());
    }
    
    #[test]
    fn test_batch_conversion() {
        let meshes = vec![Mesh::<()>::new(), Mesh::<()>::new()];
        let svo_meshes = BatchConverter::meshes_to_svo(&meshes, None);
        
        assert_eq!(svo_meshes.len(), 2);
        assert!(svo_meshes.iter().all(|m| m.is_empty()));
        
        let back_to_meshes = BatchConverter::svo_to_meshes(&svo_meshes);
        assert_eq!(back_to_meshes.len(), 2);
        assert!(back_to_meshes.iter().all(|m| m.polygons.is_empty()));
    }
    
    #[test]
    fn test_conversion_stats() {
        let mesh = Mesh::<()>::new();
        let (svo_mesh, stats) = MonitoredConverter::mesh_to_svo_with_stats(&mesh, None);
        
        assert_eq!(stats.source_polygons, 0);
        assert_eq!(stats.target_polygons, 0);
        assert!(!stats.precision_loss);
        assert!(svo_mesh.is_empty());
    }
    
    #[test]
    fn test_round_trip_validation() {
        let mesh = Mesh::<()>::new();
        let tolerance = 1e-6;
        
        assert!(ConversionValidator::validate_round_trip(&mesh, tolerance));
    }
}