//! BVH configuration and construction algorithms
//!
//! This module defines configuration options for BVH construction and operation,
//! including different construction algorithms optimized for ray tracing performance.

use crate::core::float_types::Real;

/// Construction algorithms for BVH building
///
/// Different algorithms provide trade-offs between construction speed and ray tracing quality.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::bvh::ConstructionAlgorithm;
///
/// // Choose construction algorithm based on requirements
/// let fast_construction = ConstructionAlgorithm::Median;        // Fastest, lower quality
/// let balanced = ConstructionAlgorithm::BinnedSAH;              // Good balance
/// let high_quality = ConstructionAlgorithm::SAH;                // Best quality, slower
/// let spatial_splits = ConstructionAlgorithm::SpatialSAH;       // Highest quality
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConstructionAlgorithm {
    /// Median split algorithm - O(n log n) complexity, fastest construction
    Median,
    /// Surface Area Heuristic - optimal ray tracing performance, slower construction
    SAH,
    /// Binned SAH - fast approximation of SAH for large datasets
    BinnedSAH,
    /// Spatial SAH - allows primitive splitting for highest quality
    SpatialSAH,
}

impl Default for ConstructionAlgorithm {
    fn default() -> Self {
        ConstructionAlgorithm::BinnedSAH
    }
}

/// Configuration for BVH construction and operation
///
/// This struct provides fine-grained control over BVH behavior and performance
/// characteristics, particularly optimized for ray tracing applications.
///
/// # Examples
///
/// ## Default Configuration
///
/// ```rust
/// use csgrs::spatial::bvh::BVHConfig;
///
/// let config = BVHConfig::default();
/// assert_eq!(config.max_polygons_per_leaf, 4);
/// assert_eq!(config.max_depth, 25);
/// ```
///
/// ## Ray Tracing Optimized
///
/// ```rust
/// use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};
///
/// let config = BVHConfig::for_ray_tracing();
/// assert_eq!(config.construction_algorithm, ConstructionAlgorithm::SAH);
/// assert_eq!(config.max_polygons_per_leaf, 4);
/// ```
///
/// ## Dynamic Scene Optimized
///
/// ```rust
/// use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};
///
/// let config = BVHConfig::for_dynamic_scenes();
/// assert_eq!(config.construction_algorithm, ConstructionAlgorithm::Median);
/// assert!(config.enable_incremental_updates);
/// ```
#[derive(Debug, Clone)]
pub struct BVHConfig {
    /// Maximum number of polygons per leaf node
    /// Typical values: 1-8. Lower values improve ray tracing, higher values reduce memory.
    pub max_polygons_per_leaf: usize,
    
    /// Maximum tree depth to prevent excessive recursion
    /// Typical values: 20-30. Higher values allow better subdivision but increase traversal cost.
    pub max_depth: usize,
    
    /// Construction algorithm to use for building the BVH
    pub construction_algorithm: ConstructionAlgorithm,
    
    /// SAH traversal cost parameter (cost of traversing an internal node)
    /// Typical value: 1.0. Higher values favor fewer internal nodes.
    pub sah_traversal_cost: Real,
    
    /// SAH intersection cost parameter (cost of testing ray-primitive intersection)
    /// Typical value: 1.0. Higher values favor fewer primitives per leaf.
    pub sah_intersection_cost: Real,
    
    /// Number of bins for binned SAH algorithm
    /// Typical values: 16-64. More bins improve quality but increase construction time.
    pub sah_bins: usize,
    
    /// Enable spatial splitting for primitives (SpatialSAH only)
    /// Allows splitting large primitives for better ray performance.
    pub enable_spatial_splits: bool,
    
    /// Enable incremental updates for dynamic scenes
    /// Allows insert/remove/refit operations without full reconstruction.
    pub enable_incremental_updates: bool,
    
    /// Quality threshold for triggering BVH rebuilds
    /// Range: 0.0-1.0. Lower values trigger rebuilds more frequently.
    pub rebuild_quality_threshold: Real,
    
    /// Maximum number of refit operations before forcing rebuild
    /// Prevents degradation from too many incremental updates.
    pub max_refits_before_rebuild: usize,
}

impl Default for BVHConfig {
    fn default() -> Self {
        Self {
            max_polygons_per_leaf: 4,
            max_depth: 25,
            construction_algorithm: ConstructionAlgorithm::BinnedSAH,
            sah_traversal_cost: 1.0,
            sah_intersection_cost: 1.0,
            sah_bins: 32,
            enable_spatial_splits: false,
            enable_incremental_updates: true,
            rebuild_quality_threshold: 0.3,
            max_refits_before_rebuild: 10,
        }
    }
}

impl BVHConfig {
    /// Create configuration optimized for ray tracing performance
    ///
    /// Uses SAH construction for optimal ray traversal performance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};
    ///
    /// let config = BVHConfig::for_ray_tracing();
    /// assert_eq!(config.construction_algorithm, ConstructionAlgorithm::SAH);
    /// assert_eq!(config.max_polygons_per_leaf, 4);
    /// assert_eq!(config.sah_traversal_cost, 1.0);
    /// ```
    pub fn for_ray_tracing() -> Self {
        Self {
            max_polygons_per_leaf: 4,
            max_depth: 30,
            construction_algorithm: ConstructionAlgorithm::SAH,
            sah_traversal_cost: 1.0,
            sah_intersection_cost: 1.0,
            sah_bins: 64,
            enable_spatial_splits: false,
            enable_incremental_updates: false,
            rebuild_quality_threshold: 0.2,
            max_refits_before_rebuild: 5,
        }
    }
    
    /// Create configuration optimized for dynamic scenes
    ///
    /// Uses faster construction with incremental update support.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};
    ///
    /// let config = BVHConfig::for_dynamic_scenes();
    /// assert_eq!(config.construction_algorithm, ConstructionAlgorithm::Median);
    /// assert!(config.enable_incremental_updates);
    /// ```
    pub fn for_dynamic_scenes() -> Self {
        Self {
            max_polygons_per_leaf: 8,
            max_depth: 20,
            construction_algorithm: ConstructionAlgorithm::Median,
            sah_traversal_cost: 1.0,
            sah_intersection_cost: 1.0,
            sah_bins: 16,
            enable_spatial_splits: false,
            enable_incremental_updates: true,
            rebuild_quality_threshold: 0.4,
            max_refits_before_rebuild: 20,
        }
    }
    
    /// Create configuration for high-quality ray tracing
    ///
    /// Uses spatial SAH with primitive splitting for best possible ray performance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};
    ///
    /// let config = BVHConfig::for_high_quality_rays();
    /// assert_eq!(config.construction_algorithm, ConstructionAlgorithm::SpatialSAH);
    /// assert!(config.enable_spatial_splits);
    /// ```
    pub fn for_high_quality_rays() -> Self {
        Self {
            max_polygons_per_leaf: 2,
            max_depth: 35,
            construction_algorithm: ConstructionAlgorithm::SpatialSAH,
            sah_traversal_cost: 1.0,
            sah_intersection_cost: 1.5,
            sah_bins: 128,
            enable_spatial_splits: true,
            enable_incremental_updates: false,
            rebuild_quality_threshold: 0.1,
            max_refits_before_rebuild: 3,
        }
    }
    
    /// Create configuration for fast construction
    ///
    /// Uses median split for fastest possible BVH construction.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};
    ///
    /// let config = BVHConfig::for_fast_construction();
    /// assert_eq!(config.construction_algorithm, ConstructionAlgorithm::Median);
    /// assert_eq!(config.max_polygons_per_leaf, 16);
    /// ```
    pub fn for_fast_construction() -> Self {
        Self {
            max_polygons_per_leaf: 16,
            max_depth: 15,
            construction_algorithm: ConstructionAlgorithm::Median,
            sah_traversal_cost: 1.0,
            sah_intersection_cost: 1.0,
            sah_bins: 8,
            enable_spatial_splits: false,
            enable_incremental_updates: true,
            rebuild_quality_threshold: 0.5,
            max_refits_before_rebuild: 50,
        }
    }
    
    /// Validate the configuration parameters
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::BVHConfig;
    ///
    /// let config = BVHConfig::default();
    /// assert!(config.validate().is_ok());
    ///
    /// let mut invalid_config = BVHConfig::default();
    /// invalid_config.max_polygons_per_leaf = 0;
    /// assert!(invalid_config.validate().is_err());
    /// ```
    pub fn validate(&self) -> crate::spatial::SpatialResult<()> {
        use crate::spatial::SpatialError;
        
        if self.max_polygons_per_leaf == 0 {
            return Err(SpatialError::configuration(
                "max_polygons_per_leaf",
                &self.max_polygons_per_leaf.to_string(),
                "at least 1"
            ));
        }
        
        if self.max_depth == 0 {
            return Err(SpatialError::configuration(
                "max_depth",
                &self.max_depth.to_string(),
                "at least 1"
            ));
        }
        
        if self.sah_traversal_cost <= 0.0 {
            return Err(SpatialError::configuration(
                "sah_traversal_cost",
                &self.sah_traversal_cost.to_string(),
                "greater than 0.0"
            ));
        }
        
        if self.sah_intersection_cost <= 0.0 {
            return Err(SpatialError::configuration(
                "sah_intersection_cost",
                &self.sah_intersection_cost.to_string(),
                "greater than 0.0"
            ));
        }
        
        if self.sah_bins == 0 {
            return Err(SpatialError::configuration(
                "sah_bins",
                &self.sah_bins.to_string(),
                "at least 1"
            ));
        }
        
        if self.rebuild_quality_threshold < 0.0 || self.rebuild_quality_threshold > 1.0 {
            return Err(SpatialError::configuration(
                "rebuild_quality_threshold",
                &self.rebuild_quality_threshold.to_string(),
                "between 0.0 and 1.0"
            ));
        }
        
        Ok(())
    }
}
