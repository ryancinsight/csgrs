//! Traits for Dependency Inversion Principle (DIP)
//! 
//! High-level modules should not depend on low-level modules.
//! Both should depend on abstractions.

use crate::float_types::Real;
use crate::voxels::error::VoxelResult;
use crate::voxels::polygon::Polygon;
use nalgebra::Point3;
use std::fmt::Debug;

/// Abstraction for SDF functions (DIP - depend on abstraction, not concretion)
pub trait SignedDistanceFunction: Fn(&Point3<Real>) -> Real + Send + Sync {}

impl<F> SignedDistanceFunction for F 
where 
    F: Fn(&Point3<Real>) -> Real + Send + Sync 
{}

/// Abstraction for voxelization strategies (DIP)
pub trait VoxelizationStrategy<S: Clone> {
    /// Voxelize using this strategy
    fn voxelize(&self, bounds_min: Point3<Real>, bounds_max: Point3<Real>, resolution: u8) -> VoxelResult<Vec<Polygon<S>>>;
}

/// Abstraction for surface extraction strategies (DIP)
pub trait SurfaceExtractionStrategy<S: Clone> {
    /// Extract surface using this strategy
    fn extract_surface(&self, data: &dyn SurfaceData<S>) -> VoxelResult<Vec<Polygon<S>>>;
}

/// Abstraction for surface data (DIP)
pub trait SurfaceData<S: Clone> {
    /// Get surface information at a point
    fn surface_info_at(&self, point: &Point3<Real>) -> Option<Real>;
    
    /// Get bounds
    fn bounds(&self) -> (Point3<Real>, Point3<Real>);
    
    /// Get metadata
    fn metadata(&self) -> Option<&S>;
}

/// Abstraction for BSP construction (DIP)
pub trait BspConstructor<S: Clone + Debug + Send + Sync> {
    /// Construct BSP tree from polygons
    fn construct_bsp(&self, polygons: &[Polygon<S>]) -> VoxelResult<crate::voxels::bsp::Node<S>>;
}

/// Abstraction for memory optimization (DIP)
pub trait MemoryOptimizer {
    /// Optimize memory usage
    fn optimize_memory(&mut self) -> VoxelResult<usize>; // Returns bytes saved
    
    /// Get memory usage
    fn memory_usage(&self) -> usize;
}

/// Abstraction for spatial queries (DIP)
pub trait SpatialQuery<S: Clone> {
    /// Query occupancy at point
    fn query_occupancy(&self, point: &Point3<Real>) -> Option<crate::voxels::Occupancy>;
    
    /// Query within bounds
    fn query_bounds(&self, min: &Point3<Real>, max: &Point3<Real>) -> Vec<&S>;
}

/// Default BSP constructor implementation
pub struct DefaultBspConstructor;

impl<S: Clone + Debug + Send + Sync> BspConstructor<S> for DefaultBspConstructor {
    fn construct_bsp(&self, polygons: &[Polygon<S>]) -> VoxelResult<crate::voxels::bsp::Node<S>> {
        if polygons.is_empty() {
            return Err(crate::voxels::error::VoxelError::BspConstructionFailed(
                "Cannot construct BSP from empty polygon list".to_string()
            ));
        }
        
        Ok(crate::voxels::bsp::Node::from_polygons(polygons))
    }
}

/// SDF-based voxelization strategy
pub struct SdfVoxelizationStrategy<F> {
    pub sdf: F,
    pub iso_value: Real,
}

impl<F, S> VoxelizationStrategy<S> for SdfVoxelizationStrategy<F>
where
    F: SignedDistanceFunction,
    S: Clone + Debug + Send + Sync,
{
    fn voxelize(&self, bounds_min: Point3<Real>, bounds_max: Point3<Real>, resolution: u8) -> VoxelResult<Vec<Polygon<S>>> {
        crate::voxels::error::VoxelValidator::validate_resolution(resolution)?;
        
        let bounds_min_array = [bounds_min.x, bounds_min.y, bounds_min.z];
        let bounds_max_array = [bounds_max.x, bounds_max.y, bounds_max.z];
        crate::voxels::error::VoxelValidator::validate_bounds(&bounds_min_array, &bounds_max_array)?;
        
        // Use the existing SDF voxelization logic
        let grid_resolution = (resolution as usize * 4, resolution as usize * 4, resolution as usize * 4);
        let voxels = crate::voxels::csg::Voxels::sdf(&self.sdf, grid_resolution, bounds_min, bounds_max, self.iso_value, None);
        
        Ok(voxels.polygons().to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sdf_trait() {
        let sphere_sdf = |p: &Point3<Real>| p.coords.norm() - 1.0;
        let _: &dyn SignedDistanceFunction = &sphere_sdf;
    }
    
    #[test]
    fn test_bsp_constructor() {
        let constructor = DefaultBspConstructor;
        let empty_polygons: Vec<Polygon<()>> = Vec::new();
        assert!(constructor.construct_bsp(&empty_polygons).is_err());
    }
}
