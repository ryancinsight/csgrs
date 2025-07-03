//! **Spatial Data Structure Traits (Cathedral Engineering Compliant)**
//!
//! This module provides a comprehensive trait system for spatial data structures
//! following Cathedral Engineering principles with deep hierarchical organization
//! and single responsibility modules.
//!
//! ## **Module Architecture (Cathedral Engineering)**
//!
//! This module follows the **Law of Master Masonry** with clear functional roles:
//!
//! - **`errors`** (Immune System) - Error types and handling for spatial operations
//! - **`geometry`** (Skeleton) - Core geometric types (Aabb, Ray, Intersection)
//! - **`index`** (Soul) - The primary SpatialIndex trait definition
//! - **`config`** (Skeleton) - Configuration types and query specifications
//! - **`factory`** (Mind) - Factory for creating spatial structures
//! - **`selector`** (Mind) - Automatic structure selection algorithms
//! - **`benchmark`** (Mind) - Performance benchmarking utilities
//! - **`performance`** (Skeleton) - Performance matrices and ratings
//!
//! ## **Unified Configuration System**
//!
//! The module provides a unified configuration system that works across all spatial structures:
//!
//! ```rust
//! use csgrs::spatial::traits::{SpatialConfig, factory::SpatialStructureFactory, config::QueryType};
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create test data
//! let vertices = vec![
//!     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//!     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
//! ];
//! let polygon: Polygon<i32> = Polygon::new(vertices, None);
//! let polygons = vec![polygon];
//!
//! // Use predefined optimized configurations
//! let point_config = SpatialConfig::for_point_queries();
//! let volume_config = SpatialConfig::for_volume_queries();
//! let boolean_config = SpatialConfig::for_boolean_operations();
//!
//! // Create structures with specific configurations
//! let kdtree = SpatialStructureFactory::create_kdtree_with_config(&polygons, &point_config);
//! let octree = SpatialStructureFactory::create_octree_with_config(&polygons, &volume_config);
//! let bsp = SpatialStructureFactory::create_bsp_with_config(&polygons, &boolean_config);
//! ```
//!
//! ## **Polymorphic Usage**
//!
//! Write generic algorithms that work with any spatial structure:
//!
//! ```rust
//! use csgrs::spatial::traits::{index::SpatialIndex, factory::SpatialStructureFactory, config::QueryType};
//! use csgrs::geometry::{Polygon, Vertex};
//! use nalgebra::{Point3, Vector3};
//!
//! // Generic function that works with any spatial structure
//! fn analyze_structure<S: Clone + Send + Sync + std::fmt::Debug>(
//!     structure: &dyn SpatialIndex<S>
//! ) -> (usize, usize) {
//!     let stats = structure.statistics();
//!     let polygon_count = structure.all_polygons().len();
//!     (polygon_count, stats.node_count)
//! }
//! ```
//!
//! ## **Performance Optimization**
//!
//! The module includes comprehensive performance analysis tools:
//!
//! ```rust
//! use csgrs::spatial::traits::{benchmark::SpatialBenchmark, selector::SpatialStructureSelector};
//!
//! // Benchmark different structures
//! let results = SpatialBenchmark::benchmark_construction(&polygons);
//! let fastest = results.fastest_construction();
//!
//! // Automatic structure selection
//! let recommended = SpatialStructureSelector::recommend_structure(&polygons, QueryType::PointLocation);
//! ```
//!
//! ## **Theoretical References**
//!
//! - Samet, H. (2006). *Foundations of Multidimensional and Metric Data Structures*
//! - de Berg, M., et al. (2008). *Computational Geometry: Algorithms and Applications*
//! - Ericson, C. (2004). *Real-Time Collision Detection*

// Public API exports following the façade pattern
pub mod errors;
pub mod geometry;
pub mod index;
pub mod config;
pub mod factory;
pub mod selector;
pub mod benchmark;
pub mod performance;

// Re-export key types and traits for convenient access
pub use errors::{SpatialError, SpatialResult};
pub use geometry::{Aabb, Ray, Intersection};
pub use index::{SpatialIndex, SpatialStatistics, DatasetCharacteristics};
pub use config::{SpatialConfig, QueryType, SpatialDistribution, SpatialStructureType};
pub use factory::SpatialStructureFactory;
pub use selector::SpatialStructureSelector;
pub use benchmark::{SpatialBenchmark, BenchmarkResults};
pub use performance::{PerformanceMatrix, StructurePerformance, PerformanceRating};
