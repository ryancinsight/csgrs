//! **Minkowski Operation Data Models (The Skeleton)**
//!
//! This module defines the core data structures and types used in Minkowski operations,
//! following Cathedral Engineering principles where models represent the "skeleton"
//! that provides structural foundation for the architectural space.
//!
//! ## **Design Philosophy**
//!
//! These data structures are designed to be:
//! - **Immutable by default** - Preventing accidental state mutations
//! - **Type-safe** - Using Rust's type system to prevent invalid operations
//! - **Memory-efficient** - Optimized for the specific needs of Minkowski operations
//! - **Mathematically sound** - Reflecting the underlying mathematical concepts

use crate::core::float_types::Real;
use nalgebra::Point3;
use std::fmt::Debug;

/// **Configuration parameters for Minkowski operations**
///
/// This structure encapsulates all tunable parameters that affect the behavior
/// and performance of Minkowski operations, providing a centralized configuration
/// mechanism following the principle of explicit configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct MinkowskiConfig {
    /// **Numerical precision tolerance**
    ///
    /// Used for floating-point comparisons and degenerate case detection.
    /// Smaller values increase precision but may cause numerical instability.
    pub epsilon: Real,

    /// **Maximum number of vertices to process**
    ///
    /// Prevents excessive memory usage and computation time for very large geometries.
    /// Set to 0 to disable the limit.
    pub max_vertices: usize,

    /// **Enable convex hull optimization**
    ///
    /// When true, computes convex hulls of inputs before Minkowski operation.
    /// This can significantly improve performance for complex non-convex shapes.
    pub use_convex_hull_optimization: bool,

    /// **Memory allocation strategy**
    ///
    /// Controls how memory is allocated for intermediate results during computation.
    pub memory_strategy: MemoryStrategy,

    /// **Parallel processing configuration**
    ///
    /// Controls whether and how to use parallel processing for large operations.
    pub parallel_config: ParallelConfig,
}

impl Default for MinkowskiConfig {
    fn default() -> Self {
        Self {
            epsilon: Real::EPSILON * 10.0, // Slightly larger than machine epsilon
            max_vertices: 100_000,          // Reasonable default for most applications
            use_convex_hull_optimization: true,
            memory_strategy: MemoryStrategy::Balanced,
            parallel_config: ParallelConfig::Auto,
        }
    }
}

impl MinkowskiConfig {
    /// **Create a configuration optimized for high precision**
    pub fn high_precision() -> Self {
        Self {
            epsilon: Real::EPSILON,
            max_vertices: 50_000,
            use_convex_hull_optimization: false, // Preserve exact geometry
            memory_strategy: MemoryStrategy::Quality,
            parallel_config: ParallelConfig::Disabled,
        }
    }

    /// **Create a configuration optimized for performance**
    pub fn high_performance() -> Self {
        Self {
            epsilon: Real::EPSILON * 100.0,
            max_vertices: 10_000,
            use_convex_hull_optimization: true,
            memory_strategy: MemoryStrategy::Speed,
            parallel_config: ParallelConfig::Aggressive,
        }
    }

    /// **Create a configuration optimized for memory efficiency**
    pub fn memory_efficient() -> Self {
        Self {
            epsilon: Real::EPSILON * 50.0,
            max_vertices: 5_000,
            use_convex_hull_optimization: true,
            memory_strategy: MemoryStrategy::Compact,
            parallel_config: ParallelConfig::Conservative,
        }
    }

    /// **Validate the configuration parameters**
    ///
    /// Ensures that all configuration values are within reasonable ranges
    /// and compatible with each other.
    pub fn validate(&self) -> Result<(), String> {
        if self.epsilon <= 0.0 {
            return Err("Epsilon must be positive".to_string());
        }

        if self.epsilon > 1.0 {
            return Err("Epsilon is too large (> 1.0), may cause incorrect results".to_string());
        }

        // Additional validation logic can be added here
        Ok(())
    }
}

/// **Memory allocation strategy for Minkowski operations**
///
/// This enum defines different approaches to memory management during
/// Minkowski operations, allowing users to optimize for their specific
/// use case and system constraints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MemoryStrategy {
    /// **Optimize for speed** - Pre-allocate large buffers, use more memory for faster access
    Speed,
    /// **Optimize for memory usage** - Minimize memory footprint, may be slower
    Compact,
    /// **Balanced approach** - Reasonable trade-off between speed and memory usage
    Balanced,
    /// **Optimize for result quality** - Use extra memory to improve numerical precision
    Quality,
}

impl MemoryStrategy {
    /// **Get the memory allocation factor for this strategy**
    ///
    /// Returns a multiplier for buffer sizes relative to the minimum required.
    pub fn allocation_factor(&self) -> f32 {
        match self {
            MemoryStrategy::Speed => 2.0,     // Double the minimum allocation
            MemoryStrategy::Compact => 1.0,   // Minimum allocation only
            MemoryStrategy::Balanced => 1.5,  // 50% extra allocation
            MemoryStrategy::Quality => 3.0,   // Triple allocation for precision
        }
    }

    /// **Check if this strategy should use pre-allocation**
    pub fn use_preallocation(&self) -> bool {
        match self {
            MemoryStrategy::Speed | MemoryStrategy::Quality => true,
            MemoryStrategy::Compact => false,
            MemoryStrategy::Balanced => true,
        }
    }
}

/// **Parallel processing configuration**
///
/// This enum controls how Minkowski operations utilize multiple CPU cores
/// for improved performance on large datasets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParallelConfig {
    /// **Disable parallel processing** - Use single-threaded execution only
    Disabled,
    /// **Conservative parallelism** - Use parallel processing only for very large operations
    Conservative,
    /// **Automatic parallelism** - Let the system decide based on available cores and data size
    Auto,
    /// **Aggressive parallelism** - Use parallel processing whenever possible
    Aggressive,
    /// **Custom thread count** - Use a specific number of threads
    Custom(usize),
}

impl ParallelConfig {
    /// **Get the recommended thread count for this configuration**
    ///
    /// Returns the number of threads to use based on the configuration
    /// and system capabilities.
    pub fn thread_count(&self) -> usize {
        // Use a simple heuristic for thread count without external dependencies
        let available_parallelism = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);

        match self {
            ParallelConfig::Disabled => 1,
            ParallelConfig::Conservative => std::cmp::min(2, available_parallelism),
            ParallelConfig::Auto => available_parallelism,
            ParallelConfig::Aggressive => available_parallelism * 2,
            ParallelConfig::Custom(count) => *count,
        }
    }

    /// **Get the minimum operation size for parallel processing**
    ///
    /// Returns the minimum number of operations below which parallel
    /// processing should not be used due to overhead.
    pub fn parallel_threshold(&self) -> usize {
        match self {
            ParallelConfig::Disabled => usize::MAX,
            ParallelConfig::Conservative => 10_000,
            ParallelConfig::Auto => 1_000,
            ParallelConfig::Aggressive => 100,
            ParallelConfig::Custom(_) => 1_000,
        }
    }
}

/// **Intermediate computation state for Minkowski operations**
///
/// This structure holds temporary data during Minkowski computation,
/// providing a clean interface for managing complex intermediate states.
#[derive(Debug, Clone)]
pub struct MinkowskiState {
    /// **Vertex points from the first operand**
    pub vertices_a: Vec<Point3<Real>>,
    
    /// **Vertex points from the second operand**
    pub vertices_b: Vec<Point3<Real>>,
    
    /// **Computed Minkowski sum points**
    pub sum_points: Vec<Point3<Real>>,
    
    /// **Configuration used for this computation**
    pub config: MinkowskiConfig,
    
    /// **Performance metrics**
    pub metrics: ComputationMetrics,
}

impl MinkowskiState {
    /// **Create a new computation state**
    pub fn new(config: MinkowskiConfig) -> Self {
        Self {
            vertices_a: Vec::new(),
            vertices_b: Vec::new(),
            sum_points: Vec::new(),
            config,
            metrics: ComputationMetrics::default(),
        }
    }

    /// **Estimate memory usage for the current state**
    pub fn estimated_memory_bytes(&self) -> usize {
        let point_size = std::mem::size_of::<Point3<Real>>();
        (self.vertices_a.len() + self.vertices_b.len() + self.sum_points.len()) * point_size
    }

    /// **Check if the state exceeds memory limits**
    pub fn exceeds_memory_limit(&self, limit_bytes: usize) -> bool {
        self.estimated_memory_bytes() > limit_bytes
    }
}

/// **Performance metrics for Minkowski operations**
///
/// This structure tracks various performance metrics during computation,
/// enabling optimization and debugging of Minkowski operations.
#[derive(Debug, Clone, Default)]
pub struct ComputationMetrics {
    /// **Number of vertex pairs processed**
    pub vertex_pairs_processed: usize,
    
    /// **Number of degenerate cases encountered**
    pub degenerate_cases: usize,
    
    /// **Time spent on convex hull computation (nanoseconds)**
    pub convex_hull_time_ns: u64,
    
    /// **Time spent on vertex enumeration (nanoseconds)**
    pub vertex_enumeration_time_ns: u64,
    
    /// **Peak memory usage (bytes)**
    pub peak_memory_bytes: usize,
    
    /// **Number of parallel threads used**
    pub threads_used: usize,
}

impl ComputationMetrics {
    /// **Get total computation time in nanoseconds**
    pub fn total_time_ns(&self) -> u64 {
        self.convex_hull_time_ns + self.vertex_enumeration_time_ns
    }

    /// **Get total computation time in milliseconds**
    pub fn total_time_ms(&self) -> f64 {
        self.total_time_ns() as f64 / 1_000_000.0
    }

    /// **Calculate operations per second**
    pub fn operations_per_second(&self) -> f64 {
        if self.total_time_ns() == 0 {
            0.0
        } else {
            (self.vertex_pairs_processed as f64) / (self.total_time_ns() as f64 / 1_000_000_000.0)
        }
    }
}
