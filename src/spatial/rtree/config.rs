//! R-tree configuration and split algorithms
//!
//! This module defines configuration options for R-tree construction and operation,
//! including different split algorithms and performance tuning parameters.

use crate::core::float_types::Real;

/// Split algorithms for R-tree node overflow handling
///
/// Different algorithms provide trade-offs between construction speed and tree quality.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::rtree::SplitAlgorithm;
///
/// // Choose split algorithm based on requirements
/// let fast_construction = SplitAlgorithm::Linear;     // Fastest, lower quality
/// let balanced = SplitAlgorithm::Quadratic;           // Good balance
/// let high_quality = SplitAlgorithm::RStarTree;       // Best quality, slower
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SplitAlgorithm {
    /// Linear split algorithm - O(M) complexity, fastest but lowest quality
    Linear,
    /// Quadratic split algorithm - O(M²) complexity, good balance of speed and quality
    Quadratic,
    /// R*-tree split algorithm - highest quality with forced reinsertion
    RStarTree,
}

impl Default for SplitAlgorithm {
    fn default() -> Self {
        SplitAlgorithm::Quadratic
    }
}

/// Configuration for R-tree construction and operation
///
/// This struct provides fine-grained control over R-tree behavior and performance
/// characteristics.
///
/// # Examples
///
/// ## Default Configuration
///
/// ```rust
/// use csgrs::spatial::rtree::RTreeConfig;
///
/// let config = RTreeConfig::default();
/// assert_eq!(config.max_children, 8);
/// assert_eq!(config.min_children, 4);
/// ```
///
/// ## Custom Configuration
///
/// ```rust
/// use csgrs::spatial::rtree::{RTreeConfig, SplitAlgorithm};
///
/// let config = RTreeConfig {
///     max_children: 16,
///     min_children: 8,
///     split_algorithm: SplitAlgorithm::RStarTree,
///     reinsert_factor: 0.3,
///     bulk_load_threshold: 100,
/// };
/// ```
///
/// ## Optimized Configurations
///
/// ```rust
/// use csgrs::spatial::rtree::RTreeConfig;
///
/// // Fast insertion, lower query performance
/// let fast_insert = RTreeConfig::for_fast_insertion();
///
/// // Optimized for range queries
/// let range_optimized = RTreeConfig::for_range_queries();
///
/// // Memory-efficient configuration
/// let memory_efficient = RTreeConfig::for_memory_efficiency();
/// ```
#[derive(Debug, Clone)]
pub struct RTreeConfig {
    /// Maximum number of children per node (M)
    /// Typical values: 4-16. Higher values reduce tree height but increase node processing cost.
    pub max_children: usize,
    
    /// Minimum number of children per node (m)
    /// Typically M/2. Must be ≤ max_children/2 for proper tree balance.
    pub min_children: usize,
    
    /// Split algorithm to use when nodes overflow
    pub split_algorithm: SplitAlgorithm,
    
    /// Fraction of entries to reinsert during R*-tree forced reinsertion (0.0-1.0)
    /// Typical value: 0.3 (30%). Only used with SplitAlgorithm::RStarTree.
    pub reinsert_factor: Real,
    
    /// Threshold for bulk loading vs incremental insertion
    /// If initial polygon count ≥ threshold, use bulk loading for better tree quality.
    pub bulk_load_threshold: usize,
}

impl Default for RTreeConfig {
    fn default() -> Self {
        Self {
            max_children: 8,
            min_children: 4,
            split_algorithm: SplitAlgorithm::Quadratic,
            reinsert_factor: 0.3,
            bulk_load_threshold: 50,
        }
    }
}

impl RTreeConfig {
    /// Create configuration optimized for fast insertion operations
    ///
    /// Uses linear split algorithm and higher branching factor for speed.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::{RTreeConfig, SplitAlgorithm};
    ///
    /// let config = RTreeConfig::for_fast_insertion();
    /// assert_eq!(config.split_algorithm, SplitAlgorithm::Linear);
    /// assert_eq!(config.max_children, 16);
    /// ```
    pub fn for_fast_insertion() -> Self {
        Self {
            max_children: 16,
            min_children: 8,
            split_algorithm: SplitAlgorithm::Linear,
            reinsert_factor: 0.2,
            bulk_load_threshold: 100,
        }
    }
    
    /// Create configuration optimized for range query performance
    ///
    /// Uses R*-tree algorithm with moderate branching factor for best query performance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::{RTreeConfig, SplitAlgorithm};
    ///
    /// let config = RTreeConfig::for_range_queries();
    /// assert_eq!(config.split_algorithm, SplitAlgorithm::RStarTree);
    /// assert_eq!(config.max_children, 6);
    /// ```
    pub fn for_range_queries() -> Self {
        Self {
            max_children: 6,
            min_children: 3,
            split_algorithm: SplitAlgorithm::RStarTree,
            reinsert_factor: 0.3,
            bulk_load_threshold: 25,
        }
    }
    
    /// Create configuration optimized for memory efficiency
    ///
    /// Uses smaller branching factor to reduce memory usage per node.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::RTreeConfig;
    ///
    /// let config = RTreeConfig::for_memory_efficiency();
    /// assert_eq!(config.max_children, 4);
    /// assert_eq!(config.min_children, 2);
    /// ```
    pub fn for_memory_efficiency() -> Self {
        Self {
            max_children: 4,
            min_children: 2,
            split_algorithm: SplitAlgorithm::Quadratic,
            reinsert_factor: 0.25,
            bulk_load_threshold: 20,
        }
    }
    
    /// Create configuration for large datasets
    ///
    /// Uses higher branching factor and bulk loading for large-scale spatial indexing.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::RTreeConfig;
    ///
    /// let config = RTreeConfig::for_large_datasets();
    /// assert_eq!(config.max_children, 12);
    /// assert_eq!(config.bulk_load_threshold, 200);
    /// ```
    pub fn for_large_datasets() -> Self {
        Self {
            max_children: 12,
            min_children: 6,
            split_algorithm: SplitAlgorithm::Quadratic,
            reinsert_factor: 0.3,
            bulk_load_threshold: 200,
        }
    }
    
    /// Validate the configuration parameters
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::RTreeConfig;
    ///
    /// let config = RTreeConfig::default();
    /// assert!(config.validate().is_ok());
    ///
    /// let mut invalid_config = RTreeConfig::default();
    /// invalid_config.min_children = invalid_config.max_children + 1;
    /// assert!(invalid_config.validate().is_err());
    /// ```
    pub fn validate(&self) -> crate::spatial::SpatialResult<()> {
        use crate::spatial::SpatialError;
        
        if self.max_children < 2 {
            return Err(SpatialError::configuration(
                "max_children",
                &self.max_children.to_string(),
                "at least 2"
            ));
        }
        
        if self.min_children < 1 {
            return Err(SpatialError::configuration(
                "min_children",
                &self.min_children.to_string(),
                "at least 1"
            ));
        }
        
        if self.min_children > self.max_children / 2 {
            return Err(SpatialError::configuration(
                "min_children",
                &format!("{} (max_children: {})", self.min_children, self.max_children),
                "at most max_children/2 for proper tree balance"
            ));
        }
        
        if self.reinsert_factor < 0.0 || self.reinsert_factor > 1.0 {
            return Err(SpatialError::configuration(
                "reinsert_factor",
                &self.reinsert_factor.to_string(),
                "between 0.0 and 1.0"
            ));
        }
        
        Ok(())
    }
}
