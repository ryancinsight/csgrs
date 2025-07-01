//! Common traits and interfaces for spatial data structures
//!
//! This module defines the unified trait system that enables polymorphic usage
//! of different spatial data structures (BSP, KD-tree, Octree) while maintaining
//! performance and type safety.
//!
//! # Unified Configuration System
//!
//! The module provides a unified configuration system that works across all spatial structures:
//!
//! ```rust
//! use csgrs::spatial::{SpatialConfig, SpatialStructureFactory, QueryType};
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
//!
//! // All structures implement the same SpatialIndex trait
//! assert_eq!(kdtree.all_polygons().len(), 1);
//! assert_eq!(octree.all_polygons().len(), 1);
//! assert_eq!(bsp.all_polygons().len(), 1);
//! ```
//!
//! # Polymorphic Usage
//!
//! Write generic algorithms that work with any spatial structure:
//!
//! ```rust
//! use csgrs::spatial::{SpatialIndex, SpatialStructureFactory, QueryType};
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
//! // Create different structures as trait objects
//! let kdtree = SpatialStructureFactory::create_optimal(&polygons, QueryType::PointLocation);
//! let octree = SpatialStructureFactory::create_optimal(&polygons, QueryType::VolumeQuery);
//!
//! // Use them polymorphically
//! let (kd_polys, kd_nodes) = analyze_structure(kdtree.as_ref());
//! let (oct_polys, oct_nodes) = analyze_structure(octree.as_ref());
//!
//! assert_eq!(kd_polys, 1);
//! assert_eq!(oct_polys, 1);
//! assert!(kd_nodes >= 1);
//! assert!(oct_nodes >= 1);
//! ```

use crate::geometry::Polygon;
use crate::core::float_types::Real;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Unified error type for all spatial data structure operations
///
/// This enum provides consistent error handling across all spatial structures
/// (BSP, KD-tree, Octree) with detailed error information for debugging.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::{SpatialError, SpatialStructureFactory};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Example of handling construction errors
/// let empty_polygons: Vec<Polygon<i32>> = vec![];
///
/// // This would typically succeed, but demonstrates error handling pattern
/// let result = std::panic::catch_unwind(|| {
///     SpatialStructureFactory::create_kdtree(&empty_polygons)
/// });
///
/// // In practice, most operations don't fail, but when they do,
/// // SpatialError provides detailed information
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialError {
    /// Error during spatial structure construction
    ConstructionError {
        /// The type of structure being constructed
        structure_type: String,
        /// Detailed error message
        message: String,
        /// Optional source error
        source: Option<String>,
    },

    /// Error during spatial query operations
    QueryError {
        /// The type of query being performed
        query_type: String,
        /// Detailed error message
        message: String,
        /// Optional context information
        context: Option<String>,
    },

    /// Configuration validation error
    ConfigurationError {
        /// The configuration parameter that failed validation
        parameter: String,
        /// The invalid value
        value: String,
        /// Expected value or range
        expected: String,
    },

    /// Geometric computation error
    GeometricError {
        /// The geometric operation that failed
        operation: String,
        /// Detailed error message
        message: String,
        /// Optional geometric context
        context: Option<String>,
    },

    /// Memory allocation or capacity error
    MemoryError {
        /// The operation that caused the memory error
        operation: String,
        /// Requested size or capacity
        requested: usize,
        /// Available size or capacity
        available: Option<usize>,
    },

    /// Invalid input data error
    InvalidInput {
        /// Description of the invalid input
        input_type: String,
        /// Detailed error message
        message: String,
        /// Suggestion for fixing the input
        suggestion: Option<String>,
    },

    /// Feature not supported error
    UnsupportedOperation {
        /// The operation that is not supported
        operation: String,
        /// The structure type that doesn't support it
        structure_type: String,
        /// Suggested alternative
        alternative: Option<String>,
    },
}

impl std::fmt::Display for SpatialError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpatialError::ConstructionError { structure_type, message, source } => {
                write!(f, "Construction error in {}: {}", structure_type, message)?;
                if let Some(src) = source {
                    write!(f, " (source: {})", src)?;
                }
                Ok(())
            },
            SpatialError::QueryError { query_type, message, context } => {
                write!(f, "Query error in {}: {}", query_type, message)?;
                if let Some(ctx) = context {
                    write!(f, " (context: {})", ctx)?;
                }
                Ok(())
            },
            SpatialError::ConfigurationError { parameter, value, expected } => {
                write!(f, "Configuration error: parameter '{}' has invalid value '{}', expected {}",
                       parameter, value, expected)
            },
            SpatialError::GeometricError { operation, message, context } => {
                write!(f, "Geometric error in {}: {}", operation, message)?;
                if let Some(ctx) = context {
                    write!(f, " (context: {})", ctx)?;
                }
                Ok(())
            },
            SpatialError::MemoryError { operation, requested, available } => {
                write!(f, "Memory error in {}: requested {} bytes", operation, requested)?;
                if let Some(avail) = available {
                    write!(f, ", available {} bytes", avail)?;
                }
                Ok(())
            },
            SpatialError::InvalidInput { input_type, message, suggestion } => {
                write!(f, "Invalid input ({}): {}", input_type, message)?;
                if let Some(sug) = suggestion {
                    write!(f, " (suggestion: {})", sug)?;
                }
                Ok(())
            },
            SpatialError::UnsupportedOperation { operation, structure_type, alternative } => {
                write!(f, "Unsupported operation '{}' for {}", operation, structure_type)?;
                if let Some(alt) = alternative {
                    write!(f, " (try: {})", alt)?;
                }
                Ok(())
            },
        }
    }
}

impl std::error::Error for SpatialError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None // We store source as String for simplicity
    }
}

/// Result type for spatial operations that can fail
pub type SpatialResult<T> = Result<T, SpatialError>;

/// Utility functions for creating common spatial errors
impl SpatialError {
    /// Create a construction error
    pub fn construction(structure_type: &str, message: &str) -> Self {
        Self::ConstructionError {
            structure_type: structure_type.to_string(),
            message: message.to_string(),
            source: None,
        }
    }

    /// Create a construction error with source
    pub fn construction_with_source(structure_type: &str, message: &str, source: &str) -> Self {
        Self::ConstructionError {
            structure_type: structure_type.to_string(),
            message: message.to_string(),
            source: Some(source.to_string()),
        }
    }

    /// Create a query error
    pub fn query(query_type: &str, message: &str) -> Self {
        Self::QueryError {
            query_type: query_type.to_string(),
            message: message.to_string(),
            context: None,
        }
    }

    /// Create a query error with context
    pub fn query_with_context(query_type: &str, message: &str, context: &str) -> Self {
        Self::QueryError {
            query_type: query_type.to_string(),
            message: message.to_string(),
            context: Some(context.to_string()),
        }
    }

    /// Create a configuration error
    pub fn configuration(parameter: &str, value: &str, expected: &str) -> Self {
        Self::ConfigurationError {
            parameter: parameter.to_string(),
            value: value.to_string(),
            expected: expected.to_string(),
        }
    }

    /// Create a geometric error
    pub fn geometric(operation: &str, message: &str) -> Self {
        Self::GeometricError {
            operation: operation.to_string(),
            message: message.to_string(),
            context: None,
        }
    }

    /// Create an invalid input error
    pub fn invalid_input(input_type: &str, message: &str) -> Self {
        Self::InvalidInput {
            input_type: input_type.to_string(),
            message: message.to_string(),
            suggestion: None,
        }
    }

    /// Create an invalid input error with suggestion
    pub fn invalid_input_with_suggestion(input_type: &str, message: &str, suggestion: &str) -> Self {
        Self::InvalidInput {
            input_type: input_type.to_string(),
            message: message.to_string(),
            suggestion: Some(suggestion.to_string()),
        }
    }

    /// Create an unsupported operation error
    pub fn unsupported(operation: &str, structure_type: &str) -> Self {
        Self::UnsupportedOperation {
            operation: operation.to_string(),
            structure_type: structure_type.to_string(),
            alternative: None,
        }
    }

    /// Create an unsupported operation error with alternative
    pub fn unsupported_with_alternative(operation: &str, structure_type: &str, alternative: &str) -> Self {
        Self::UnsupportedOperation {
            operation: operation.to_string(),
            structure_type: structure_type.to_string(),
            alternative: Some(alternative.to_string()),
        }
    }
}

/// Axis-aligned bounding box for spatial queries
#[derive(Debug, Clone, PartialEq)]
pub struct Aabb {
    pub min: Point3<Real>,
    pub max: Point3<Real>,
}

impl Aabb {
    /// Create a new AABB from min and max points
    pub fn new(min: Point3<Real>, max: Point3<Real>) -> Self {
        Self { min, max }
    }

    /// Create an AABB that encompasses all given points
    pub fn from_points(points: &[Point3<Real>]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }

        let mut min = points[0];
        let mut max = points[0];

        for point in points.iter().skip(1) {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }

        Some(Self { min, max })
    }

    /// Create an AABB from a collection of polygons
    pub fn from_polygons<S: Clone>(polygons: &[Polygon<S>]) -> Option<Self> {
        let points: Vec<Point3<Real>> = polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();
        Self::from_points(&points)
    }

    /// Check if this AABB contains a point
    pub fn contains_point(&self, point: &Point3<Real>) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y &&
        point.z >= self.min.z && point.z <= self.max.z
    }

    /// Check if this AABB intersects with another AABB
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
        self.min.y <= other.max.y && self.max.y >= other.min.y &&
        self.min.z <= other.max.z && self.max.z >= other.min.z
    }

    /// Get the center point of the AABB
    pub fn center(&self) -> Point3<Real> {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the size (dimensions) of the AABB
    pub fn size(&self) -> Vector3<Real> {
        self.max - self.min
    }

    /// Get the volume of the AABB
    pub fn volume(&self) -> Real {
        let size = self.size();
        size.x * size.y * size.z
    }
}

/// Ray for spatial intersection queries
#[derive(Debug, Clone)]
pub struct Ray {
    pub origin: Point3<Real>,
    pub direction: Vector3<Real>,
}

impl Ray {
    /// Create a new ray
    pub fn new(origin: Point3<Real>, direction: Vector3<Real>) -> Self {
        Self { origin, direction: direction.normalize() }
    }

    /// Get a point along the ray at parameter t
    pub fn point_at(&self, t: Real) -> Point3<Real> {
        self.origin + self.direction * t
    }
}

/// Result of a ray-geometry intersection
#[derive(Debug, Clone)]
pub struct Intersection<S: Clone> {
    pub distance: Real,
    pub point: Point3<Real>,
    pub normal: Vector3<Real>,
    pub polygon: Polygon<S>,
}

/// Common interface for all spatial data structures
///
/// This trait enables polymorphic usage of different spatial structures
/// while maintaining type safety and performance.
///
/// # Examples
///
/// ## Polymorphic Usage
///
/// ```rust
/// use csgrs::spatial::{SpatialIndex, SpatialStructureFactory, QueryType};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Generic function that works with any spatial structure
/// fn analyze_spatial_structure<S: Clone + Send + Sync + std::fmt::Debug>(
///     structure: &dyn SpatialIndex<S>
/// ) -> (usize, usize, bool) {
///     let stats = structure.statistics();
///     let polygon_count = structure.all_polygons().len();
///     let has_bounds = structure.bounding_box().is_some();
///     (polygon_count, stats.node_count, has_bounds)
/// }
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Create different structures as trait objects
/// let kdtree = SpatialStructureFactory::create_kdtree(&polygons);
/// let octree = SpatialStructureFactory::create_octree(&polygons);
///
/// // Use them polymorphically
/// let (kd_polys, kd_nodes, kd_bounds) = analyze_spatial_structure(kdtree.as_ref());
/// let (oct_polys, oct_nodes, oct_bounds) = analyze_spatial_structure(octree.as_ref());
///
/// assert_eq!(kd_polys, 1);
/// assert_eq!(oct_polys, 1);
/// assert!(kd_bounds);
/// assert!(oct_bounds);
/// ```
///
/// ## Collection Storage
///
/// ```rust
/// use csgrs::spatial::{SpatialIndex, SpatialStructureFactory};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Store different structures in a collection
/// let mut structures: Vec<Box<dyn SpatialIndex<i32>>> = Vec::new();
/// structures.push(SpatialStructureFactory::create_bsp(&polygons));
/// structures.push(SpatialStructureFactory::create_kdtree(&polygons));
/// structures.push(SpatialStructureFactory::create_octree(&polygons));
///
/// // Process them uniformly
/// for (i, structure) in structures.iter().enumerate() {
///     let stats = structure.statistics();
///     assert!(stats.polygon_count > 0, "Structure {} should have polygons", i);
///     assert!(stats.node_count >= 1, "Structure {} should have at least 1 node", i);
/// }
/// ```
pub trait SpatialIndex<S: Clone + Send + Sync + Debug> {
    /// Build the spatial structure from a collection of polygons
    fn build(polygons: &[Polygon<S>]) -> Self where Self: Sized;

    /// Create an empty spatial structure
    fn new() -> Self where Self: Sized;

    /// Get all polygons stored in the structure
    fn all_polygons(&self) -> Vec<Polygon<S>>;

    /// Find all polygons within a given bounding box
    fn query_range(&self, bounds: &Aabb) -> Vec<&Polygon<S>>;

    /// Find the nearest polygon to a given point
    fn nearest_neighbor(&self, point: &Point3<Real>) -> Option<&Polygon<S>>;

    /// Find all polygons intersected by a ray
    fn ray_intersections(&self, ray: &Ray) -> Vec<Intersection<S>>;

    /// Check if a point is inside the geometry represented by this structure
    fn contains_point(&self, point: &Point3<Real>) -> bool;

    /// Get the bounding box of all geometry in this structure
    fn bounding_box(&self) -> Option<Aabb>;

    /// Get statistics about the structure (depth, node count, etc.)
    fn statistics(&self) -> SpatialStatistics;
}

/// Statistics about a spatial data structure
#[derive(Debug, Clone)]
pub struct SpatialStatistics {
    pub node_count: usize,
    pub max_depth: usize,
    pub polygon_count: usize,
    pub memory_usage_bytes: usize,
}

/// Characteristics of a polygon dataset for structure selection
#[derive(Debug, Clone)]
pub struct DatasetCharacteristics {
    pub polygon_count: usize,
    pub bounding_box: Aabb,
    pub density: Real,  // polygons per unit volume
    pub aspect_ratio: Real,  // max dimension / min dimension
    pub spatial_distribution: SpatialDistribution,
}

/// Types of spatial distribution patterns
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialDistribution {
    Uniform,
    Clustered,
    Sparse,
    Linear,
    Planar,
}

/// Query type for optimal structure selection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QueryType {
    BooleanOperations,
    PointLocation,
    NearestNeighbor,
    RangeQuery,
    RayIntersection,
    VolumeQuery,
    RayTracing,
    CollisionDetection,
}

/// Automatic spatial structure selector
pub struct SpatialStructureSelector;

/// Factory for creating spatial structures as trait objects
///
/// This factory provides a unified interface for creating any spatial structure type
/// with consistent configuration and automatic optimization.
///
/// # Examples
///
/// ## Automatic Optimal Creation
///
/// ```rust
/// use csgrs::spatial::{SpatialStructureFactory, QueryType};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Factory automatically selects optimal structure and configuration
/// let point_structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::PointLocation);
/// let volume_structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::VolumeQuery);
/// let boolean_structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::BooleanOperations);
///
/// // All structures implement the same SpatialIndex trait
/// assert_eq!(point_structure.all_polygons().len(), 1);
/// assert_eq!(volume_structure.all_polygons().len(), 1);
/// assert_eq!(boolean_structure.all_polygons().len(), 1);
/// ```
///
/// ## Specific Structure Creation
///
/// ```rust
/// use csgrs::spatial::SpatialStructureFactory;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Create specific structure types with optimized configurations
/// let bsp = SpatialStructureFactory::create_bsp(&polygons);
/// let kdtree = SpatialStructureFactory::create_kdtree(&polygons);
/// let octree = SpatialStructureFactory::create_octree(&polygons);
///
/// // All use optimized configurations for their intended use cases
/// assert_eq!(bsp.all_polygons().len(), 1);
/// assert_eq!(kdtree.all_polygons().len(), 1);
/// assert_eq!(octree.all_polygons().len(), 1);
/// ```
///
/// ## Specialized Factory Methods
///
/// ```rust
/// use csgrs::spatial::{SpatialStructureFactory, QueryType};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Specialized factory methods for specific scenarios
/// let large_dataset = SpatialStructureFactory::create_for_large_dataset(&polygons, QueryType::PointLocation);
/// let memory_optimized = SpatialStructureFactory::create_memory_optimized(&polygons, QueryType::VolumeQuery);
///
/// assert_eq!(large_dataset.all_polygons().len(), 1);
/// assert_eq!(memory_optimized.all_polygons().len(), 1);
/// ```
pub struct SpatialStructureFactory;

impl SpatialStructureFactory {
    /// Create a spatial structure as a trait object based on the recommended type
    pub fn create_structure<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        structure_type: SpatialStructureType,
    ) -> Box<dyn SpatialIndex<S>> {
        let config = SpatialConfig::default();
        Self::create_structure_with_config(polygons, structure_type, &config)
    }

    /// Create a spatial structure with error handling
    pub fn try_create_structure<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        structure_type: SpatialStructureType,
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let config = SpatialConfig::default();
        Self::try_create_structure_with_config(polygons, structure_type, &config)
    }

    /// Create a spatial structure with custom configuration
    pub fn create_structure_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        structure_type: SpatialStructureType,
        config: &SpatialConfig,
    ) -> Box<dyn SpatialIndex<S>> {
        // For backward compatibility, panic on error (existing behavior)
        Self::try_create_structure_with_config(polygons, structure_type, config)
            .unwrap_or_else(|e| panic!("Failed to create spatial structure: {}", e))
    }

    /// Create a spatial structure with custom configuration and error handling
    pub fn try_create_structure_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        structure_type: SpatialStructureType,
        config: &SpatialConfig,
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        // Validate configuration first
        config.validate()?;

        // Validate input
        if polygons.is_empty() {
            return Err(SpatialError::invalid_input_with_suggestion(
                "polygons",
                "empty polygon array",
                "provide at least one polygon for spatial structure construction"
            ));
        }

        match structure_type {
            SpatialStructureType::Bsp => {
                Ok(Box::new(crate::spatial::bsp::Node::from_polygons_with_config(polygons, &config.to_bsp_config())))
            },
            SpatialStructureType::Kdtree => {
                Ok(Box::new(crate::spatial::kdtree::Node::from_polygons_with_config(polygons, &config.to_kdtree_config())))
            },
            SpatialStructureType::Octree => {
                Ok(Box::new(crate::spatial::octree::Node::from_polygons_with_config(polygons, &config.to_octree_config())))
            },
            SpatialStructureType::Rtree => {
                Ok(Box::new(crate::spatial::rtree::Node::from_polygons_with_config(polygons, &config.to_rtree_config())))
            },
            SpatialStructureType::Bvh => {
                Ok(Box::new(crate::spatial::bvh::Node::from_polygons_with_config(polygons, &config.to_bvh_config())))
            },
            SpatialStructureType::Hybrid => {
                // For hybrid, choose the best structure based on data characteristics
                let characteristics = SpatialStructureSelector::analyze_dataset(polygons);
                let recommended = SpatialStructureSelector::recommend_structure(
                    &characteristics,
                    QueryType::RangeQuery // Default to range queries for hybrid
                );
                Self::try_create_structure_with_config(polygons, recommended, config)
            },
        }
    }

    /// Create a spatial structure automatically based on data analysis and query type
    pub fn create_optimal<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        query_type: QueryType,
    ) -> Box<dyn SpatialIndex<S>> {
        let characteristics = SpatialStructureSelector::analyze_dataset(polygons);
        let recommended = SpatialStructureSelector::recommend_structure(&characteristics, query_type);

        // Use optimized configuration based on query type
        let config = match query_type {
            QueryType::BooleanOperations => SpatialConfig::for_boolean_operations(),
            QueryType::PointLocation | QueryType::NearestNeighbor => SpatialConfig::for_point_queries(),
            QueryType::VolumeQuery => SpatialConfig::for_volume_queries(),
            _ => SpatialConfig::default(),
        };

        Self::create_structure_with_config(polygons, recommended, &config)
    }

    /// Create a spatial structure with optimized configuration for large datasets
    pub fn create_for_large_dataset<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        query_type: QueryType,
    ) -> Box<dyn SpatialIndex<S>> {
        let characteristics = SpatialStructureSelector::analyze_dataset(polygons);
        let recommended = SpatialStructureSelector::recommend_structure(&characteristics, query_type);
        let config = SpatialConfig::for_large_datasets();
        Self::create_structure_with_config(polygons, recommended, &config)
    }

    /// Create a spatial structure with memory-optimized configuration
    pub fn create_memory_optimized<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        query_type: QueryType,
    ) -> Box<dyn SpatialIndex<S>> {
        let characteristics = SpatialStructureSelector::analyze_dataset(polygons);
        let recommended = SpatialStructureSelector::recommend_structure(&characteristics, query_type);
        let config = SpatialConfig::for_memory_efficiency();
        Self::create_structure_with_config(polygons, recommended, &config)
    }

    /// Create a spatial structure with error handling and validation
    pub fn try_create_optimal<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        query_type: QueryType,
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let characteristics = SpatialStructureSelector::analyze_dataset(polygons);
        let recommended = SpatialStructureSelector::recommend_structure(&characteristics, query_type);

        // Use optimized configuration based on query type
        let config = match query_type {
            QueryType::BooleanOperations => SpatialConfig::for_boolean_operations(),
            QueryType::PointLocation | QueryType::NearestNeighbor => SpatialConfig::for_point_queries(),
            QueryType::VolumeQuery => SpatialConfig::for_volume_queries(),
            _ => SpatialConfig::default(),
        };

        Self::try_create_structure_with_config(polygons, recommended, &config)
    }

    /// Create a BSP tree with error handling
    pub fn try_create_bsp<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let config = SpatialConfig::for_boolean_operations();
        config.validate()?;

        if polygons.is_empty() {
            return Err(SpatialError::invalid_input(
                "polygons",
                "empty polygon array for BSP tree construction"
            ));
        }

        Ok(Box::new(crate::spatial::bsp::Node::from_polygons_with_config(polygons, &config.to_bsp_config())))
    }

    /// Create a KD-tree with error handling
    pub fn try_create_kdtree<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let config = SpatialConfig::for_point_queries();
        config.validate()?;

        if polygons.is_empty() {
            return Err(SpatialError::invalid_input(
                "polygons",
                "empty polygon array for KD-tree construction"
            ));
        }

        Ok(Box::new(crate::spatial::kdtree::Node::from_polygons_with_config(polygons, &config.to_kdtree_config())))
    }

    /// Create an Octree with error handling
    pub fn try_create_octree<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let config = SpatialConfig::for_volume_queries();
        config.validate()?;

        if polygons.is_empty() {
            return Err(SpatialError::invalid_input(
                "polygons",
                "empty polygon array for Octree construction"
            ));
        }

        Ok(Box::new(crate::spatial::octree::Node::from_polygons_with_config(polygons, &config.to_octree_config())))
    }

    /// Create an R-tree with error handling
    pub fn try_create_rtree<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let config = SpatialConfig::for_range_queries();
        config.validate()?;

        if polygons.is_empty() {
            return Err(SpatialError::invalid_input(
                "polygons",
                "empty polygon array for R-tree construction"
            ));
        }

        Ok(Box::new(crate::spatial::rtree::Node::from_polygons_with_config(polygons, &config.to_rtree_config())))
    }

    /// Create a BVH with error handling
    pub fn try_create_bvh<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> SpatialResult<Box<dyn SpatialIndex<S>>> {
        let config = SpatialConfig::for_ray_tracing();
        config.validate()?;

        if polygons.is_empty() {
            return Err(SpatialError::invalid_input(
                "polygons",
                "empty polygon array for BVH construction"
            ));
        }

        Ok(Box::new(crate::spatial::bvh::Node::from_polygons_with_config(polygons, &config.to_bvh_config())))
    }

    /// Create a BSP tree as a trait object
    pub fn create_bsp<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> Box<dyn SpatialIndex<S>> {
        let config = SpatialConfig::for_boolean_operations();
        Box::new(crate::spatial::bsp::Node::from_polygons_with_config(polygons, &config.to_bsp_config()))
    }

    /// Create a BSP tree with custom configuration
    pub fn create_bsp_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        config: &SpatialConfig,
    ) -> Box<dyn SpatialIndex<S>> {
        Box::new(crate::spatial::bsp::Node::from_polygons_with_config(polygons, &config.to_bsp_config()))
    }

    /// Create a KD-tree as a trait object
    pub fn create_kdtree<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> Box<dyn SpatialIndex<S>> {
        let config = SpatialConfig::for_point_queries();
        Box::new(crate::spatial::kdtree::Node::from_polygons_with_config(polygons, &config.to_kdtree_config()))
    }

    /// Create a KD-tree with custom configuration
    pub fn create_kdtree_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        config: &SpatialConfig,
    ) -> Box<dyn SpatialIndex<S>> {
        Box::new(crate::spatial::kdtree::Node::from_polygons_with_config(polygons, &config.to_kdtree_config()))
    }

    /// Create an Octree as a trait object
    pub fn create_octree<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> Box<dyn SpatialIndex<S>> {
        let config = SpatialConfig::for_volume_queries();
        Box::new(crate::spatial::octree::Node::from_polygons_with_config(polygons, &config.to_octree_config()))
    }

    /// Create an Octree with custom configuration
    pub fn create_octree_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        config: &SpatialConfig,
    ) -> Box<dyn SpatialIndex<S>> {
        Box::new(crate::spatial::octree::Node::from_polygons_with_config(polygons, &config.to_octree_config()))
    }

    /// Create an R-tree as a trait object
    pub fn create_rtree<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> Box<dyn SpatialIndex<S>> {
        let config = SpatialConfig::for_range_queries();
        Box::new(crate::spatial::rtree::Node::from_polygons_with_config(polygons, &config.to_rtree_config()))
    }

    /// Create an R-tree with custom configuration
    pub fn create_rtree_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        config: &SpatialConfig,
    ) -> Box<dyn SpatialIndex<S>> {
        Box::new(crate::spatial::rtree::Node::from_polygons_with_config(polygons, &config.to_rtree_config()))
    }

    /// Create a BVH as a trait object
    pub fn create_bvh<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
    ) -> Box<dyn SpatialIndex<S>> {
        let config = SpatialConfig::for_ray_tracing();
        Box::new(crate::spatial::bvh::Node::from_polygons_with_config(polygons, &config.to_bvh_config()))
    }

    /// Create a BVH with custom configuration
    pub fn create_bvh_with_config<S: Clone + Send + Sync + Debug + 'static>(
        polygons: &[Polygon<S>],
        config: &SpatialConfig,
    ) -> Box<dyn SpatialIndex<S>> {
        Box::new(crate::spatial::bvh::Node::from_polygons_with_config(polygons, &config.to_bvh_config()))
    }
}

impl SpatialStructureSelector {
    /// Analyze polygon dataset characteristics
    pub fn analyze_dataset<S: Clone>(polygons: &[Polygon<S>]) -> DatasetCharacteristics {
        let polygon_count = polygons.len();
        
        let bounding_box = Aabb::from_polygons(polygons)
            .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::origin()));
        
        let volume = bounding_box.volume();
        let density = if volume > 0.0 { polygon_count as Real / volume } else { 0.0 };
        
        let size = bounding_box.size();
        let max_dim = size.x.max(size.y).max(size.z);
        let min_dim = size.x.min(size.y).min(size.z);
        let aspect_ratio = if min_dim > 0.0 { max_dim / min_dim } else { Real::INFINITY };
        
        let spatial_distribution = Self::analyze_distribution(polygons, &bounding_box);
        
        DatasetCharacteristics {
            polygon_count,
            bounding_box,
            density,
            aspect_ratio,
            spatial_distribution,
        }
    }

    /// Recommend optimal spatial structure based on dataset and query type
    pub fn recommend_structure(
        characteristics: &DatasetCharacteristics,
        query_type: QueryType,
    ) -> SpatialStructureType {
        match query_type {
            QueryType::BooleanOperations => {
                // BSP trees are optimal for Boolean operations
                SpatialStructureType::Bsp
            },
            QueryType::PointLocation | QueryType::NearestNeighbor => {
                // KD-trees excel at point-based queries
                if characteristics.polygon_count < 1000 {
                    SpatialStructureType::Kdtree
                } else if characteristics.spatial_distribution == SpatialDistribution::Sparse {
                    // For sparse data, Octree might be better even for point queries
                    SpatialStructureType::Octree
                } else {
                    SpatialStructureType::Hybrid
                }
            },
            QueryType::RangeQuery => {
                // R-trees are optimal for range queries and bounding box operations
                if characteristics.polygon_count > 100 {
                    // R-trees excel with larger datasets
                    SpatialStructureType::Rtree
                } else if characteristics.aspect_ratio > 5.0 {
                    // High aspect ratio favors KD-trees for small datasets
                    SpatialStructureType::Kdtree
                } else if characteristics.spatial_distribution == SpatialDistribution::Sparse {
                    SpatialStructureType::Octree
                } else {
                    // Default to R-tree for range queries
                    SpatialStructureType::Rtree
                }
            },
            QueryType::VolumeQuery => {
                // Octrees are optimal for volume-based queries
                if characteristics.spatial_distribution == SpatialDistribution::Clustered {
                    SpatialStructureType::Octree
                } else if characteristics.polygon_count < 500 {
                    // Small datasets might benefit from simpler structures
                    SpatialStructureType::Kdtree
                } else {
                    SpatialStructureType::Octree
                }
            },
            QueryType::RayIntersection => {
                // BVH is optimal for ray intersection queries
                SpatialStructureType::Bvh
            },
            QueryType::RayTracing => {
                // BVH is specifically designed for ray tracing
                SpatialStructureType::Bvh
            },
            QueryType::CollisionDetection => {
                // BVH excels at collision detection with dynamic updates
                if characteristics.polygon_count > 50 {
                    SpatialStructureType::Bvh
                } else {
                    // For small datasets, KD-tree is sufficient
                    SpatialStructureType::Kdtree
                }
            },
        }
    }

    /// Get performance characteristics for different structure types
    pub fn get_performance_characteristics() -> PerformanceMatrix {
        PerformanceMatrix::default()
    }

    fn analyze_distribution<S: Clone>(polygons: &[Polygon<S>], bounds: &Aabb) -> SpatialDistribution {
        if polygons.is_empty() {
            return SpatialDistribution::Uniform;
        }

        // Simple heuristic based on polygon center distribution
        let centers: Vec<Point3<Real>> = polygons
            .iter()
            .map(|poly| {
                let sum = poly.vertices.iter().fold(Point3::origin(), |acc, v| acc + v.pos.coords);
                Point3::from(sum.coords / poly.vertices.len() as Real)
            })
            .collect();

        // Analyze clustering by checking variance
        let center_of_centers = centers.iter().fold(Point3::origin(), |acc, p| acc + p.coords) / centers.len() as Real;
        let variance = centers.iter()
            .map(|p| (p - center_of_centers).norm_squared())
            .sum::<Real>() / centers.len() as Real;

        let bounds_size = bounds.size().norm();
        let normalized_variance = variance / (bounds_size * bounds_size);

        if normalized_variance < 0.1 {
            SpatialDistribution::Clustered
        } else if normalized_variance > 0.5 {
            SpatialDistribution::Sparse
        } else {
            SpatialDistribution::Uniform
        }
    }
}

/// Types of spatial data structures
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialStructureType {
    Bsp,
    Bvh,
    Kdtree,
    Octree,
    Rtree,
    Hybrid,
}

/// Unified configuration for all spatial data structures
///
/// This struct provides a single configuration interface that works across all spatial
/// structures (BSP, KD-tree, Octree), enabling consistent behavior and easy optimization.
///
/// # Examples
///
/// ## Using Predefined Configurations
///
/// ```rust
/// use csgrs::spatial::{SpatialConfig, SpatialStructureFactory, QueryType};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Use predefined configurations optimized for specific use cases
/// let point_config = SpatialConfig::for_point_queries();
/// let volume_config = SpatialConfig::for_volume_queries();
/// let boolean_config = SpatialConfig::for_boolean_operations();
/// let large_config = SpatialConfig::for_large_datasets();
/// let memory_config = SpatialConfig::for_memory_efficiency();
///
/// // Create structures with optimized configurations
/// let kdtree = SpatialStructureFactory::create_kdtree_with_config(&polygons, &point_config);
/// let octree = SpatialStructureFactory::create_octree_with_config(&polygons, &volume_config);
///
/// assert_eq!(kdtree.all_polygons().len(), 1);
/// assert_eq!(octree.all_polygons().len(), 1);
/// ```
///
/// ## Custom Configuration
///
/// ```rust
/// use csgrs::spatial::{SpatialConfig, SpatialStructureFactory, SpatialStructureType};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Create test data
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let polygons = vec![polygon];
///
/// // Start with a predefined configuration and customize it
/// let mut config = SpatialConfig::for_point_queries();
/// config.max_depth = 15;
/// config.max_polygons_per_leaf = 5;
/// config.parallel_threshold = 200;
///
/// // Apply custom configuration to any structure type
/// let structure = SpatialStructureFactory::create_structure_with_config(
///     &polygons,
///     SpatialStructureType::Kdtree,
///     &config
/// );
///
/// assert_eq!(structure.all_polygons().len(), 1);
/// ```
///
/// ## Configuration Comparison
///
/// ```rust
/// use csgrs::spatial::SpatialConfig;
///
/// let default_config = SpatialConfig::default();
/// let point_config = SpatialConfig::for_point_queries();
/// let volume_config = SpatialConfig::for_volume_queries();
///
/// // Point queries use smaller leaf sizes for better precision
/// assert!(point_config.max_polygons_per_leaf <= default_config.max_polygons_per_leaf);
///
/// // Volume queries use different depth limits for better performance
/// assert_ne!(volume_config.max_depth, default_config.max_depth);
/// ```
#[derive(Debug, Clone)]
pub struct SpatialConfig {
    /// Maximum depth of the tree structure
    pub max_depth: usize,
    /// Minimum number of polygons per leaf node before stopping subdivision
    pub min_polygons_per_leaf: usize,
    /// Maximum number of polygons per leaf node before forcing subdivision
    pub max_polygons_per_leaf: usize,
    /// Threshold for automatic parallel processing (polygon count)
    pub parallel_threshold: usize,
    /// Whether to use Surface Area Heuristic for splitting (KD-trees)
    pub use_sah: bool,
    /// Whether to enable adaptive refinement (Octrees)
    pub adaptive_refinement: bool,
    /// Minimum volume for subdivision (Octrees only)
    pub min_subdivision_volume: Real,
    /// Balance factor for BSP plane selection (0.0 to 1.0)
    pub balance_factor: Real,
    /// Spanning factor for BSP plane selection (0.0 to 1.0)
    pub spanning_factor: Real,
}

impl Default for SpatialConfig {
    fn default() -> Self {
        Self {
            max_depth: 20,
            min_polygons_per_leaf: 1,
            max_polygons_per_leaf: 10,
            parallel_threshold: 100,
            use_sah: true,
            adaptive_refinement: true,
            min_subdivision_volume: 0.001,
            balance_factor: 0.5,
            spanning_factor: 0.5,
        }
    }
}

impl SpatialConfig {
    /// Create a configuration optimized for Boolean operations (BSP trees)
    pub fn for_boolean_operations() -> Self {
        Self {
            max_depth: 25,
            max_polygons_per_leaf: 8,
            parallel_threshold: 50,
            balance_factor: 0.6,
            spanning_factor: 0.4,
            ..Default::default()
        }
    }

    /// Create a configuration optimized for point queries (KD-trees)
    pub fn for_point_queries() -> Self {
        Self {
            max_depth: 20,
            max_polygons_per_leaf: 5,
            parallel_threshold: 500,
            use_sah: true,
            ..Default::default()
        }
    }

    /// Create a configuration optimized for volume queries (Octrees)
    pub fn for_volume_queries() -> Self {
        Self {
            max_depth: 8,
            max_polygons_per_leaf: 8,
            parallel_threshold: 200,
            adaptive_refinement: true,
            min_subdivision_volume: 0.001,
            ..Default::default()
        }
    }

    /// Create a configuration optimized for large datasets
    pub fn for_large_datasets() -> Self {
        Self {
            max_depth: 15,
            max_polygons_per_leaf: 20,
            parallel_threshold: 100,
            use_sah: true,
            adaptive_refinement: true,
            ..Default::default()
        }
    }

    /// Create a configuration optimized for memory efficiency
    pub fn for_memory_efficiency() -> Self {
        Self {
            max_depth: 12,
            max_polygons_per_leaf: 15,
            parallel_threshold: 1000, // Prefer sequential for memory efficiency
            adaptive_refinement: false,
            ..Default::default()
        }
    }

    /// Create a configuration optimized for range queries (R-trees)
    pub fn for_range_queries() -> Self {
        Self {
            max_depth: 15,
            min_polygons_per_leaf: 2,
            max_polygons_per_leaf: 8,
            parallel_threshold: 100,
            use_sah: false, // Not applicable for R-trees
            adaptive_refinement: false, // Not applicable for R-trees
            min_subdivision_volume: 0.001,
            balance_factor: 0.3, // Used as reinsert_factor for R-trees
            spanning_factor: 0.5,
        }
    }

    /// Create a configuration optimized for ray tracing (BVH)
    pub fn for_ray_tracing() -> Self {
        Self {
            max_depth: 30,
            min_polygons_per_leaf: 1,
            max_polygons_per_leaf: 4,
            parallel_threshold: 200,
            use_sah: true, // Enable SAH for BVH
            adaptive_refinement: false,
            min_subdivision_volume: 0.0001,
            balance_factor: 1.0, // SAH traversal cost
            spanning_factor: 1.0, // SAH intersection cost
        }
    }

    /// Convert to BSP-specific configuration
    pub fn to_bsp_config(&self) -> crate::spatial::bsp::BspConfig {
        crate::spatial::bsp::BspConfig {
            max_depth: self.max_depth,
            min_polygons_per_leaf: self.min_polygons_per_leaf,
            max_polygons_per_leaf: self.max_polygons_per_leaf,
            balance_factor: self.balance_factor,
            spanning_factor: self.spanning_factor,
        }
    }

    /// Convert to KD-tree specific configuration
    pub fn to_kdtree_config(&self) -> crate::spatial::kdtree::construction::KdTreeConfig {
        crate::spatial::kdtree::construction::KdTreeConfig {
            max_depth: self.max_depth,
            min_polygons_per_leaf: self.min_polygons_per_leaf,
            max_polygons_per_leaf: self.max_polygons_per_leaf,
            use_sah: self.use_sah,
        }
    }

    /// Convert to Octree specific configuration
    pub fn to_octree_config(&self) -> crate::spatial::octree::construction::OctreeConfig {
        crate::spatial::octree::construction::OctreeConfig {
            max_depth: self.max_depth,
            min_polygons_per_leaf: self.min_polygons_per_leaf,
            max_polygons_per_leaf: self.max_polygons_per_leaf,
            min_subdivision_volume: self.min_subdivision_volume,
            adaptive_refinement: self.adaptive_refinement,
        }
    }

    /// Convert to R-tree specific configuration
    pub fn to_rtree_config(&self) -> crate::spatial::rtree::RTreeConfig {
        crate::spatial::rtree::RTreeConfig {
            max_children: self.max_polygons_per_leaf,
            min_children: self.min_polygons_per_leaf,
            split_algorithm: crate::spatial::rtree::SplitAlgorithm::Quadratic,
            reinsert_factor: self.spanning_factor, // Reuse spanning_factor for reinsertion
            bulk_load_threshold: self.parallel_threshold / 2, // Use half of parallel threshold
        }
    }

    /// Convert to BVH specific configuration
    pub fn to_bvh_config(&self) -> crate::spatial::bvh::BVHConfig {
        let construction_algorithm = if self.use_sah {
            crate::spatial::bvh::ConstructionAlgorithm::SAH
        } else {
            crate::spatial::bvh::ConstructionAlgorithm::BinnedSAH
        };

        crate::spatial::bvh::BVHConfig {
            max_polygons_per_leaf: self.max_polygons_per_leaf,
            max_depth: self.max_depth,
            construction_algorithm,
            sah_traversal_cost: self.balance_factor,
            sah_intersection_cost: self.spanning_factor,
            sah_bins: 32,
            enable_spatial_splits: false,
            enable_incremental_updates: true,
            rebuild_quality_threshold: 0.3,
            max_refits_before_rebuild: 10,
        }
    }

    /// Validate the configuration parameters
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::SpatialConfig;
    ///
    /// let mut config = SpatialConfig::default();
    /// assert!(config.validate().is_ok());
    ///
    /// // Invalid configuration
    /// config.max_depth = 0;
    /// assert!(config.validate().is_err());
    /// ```
    pub fn validate(&self) -> SpatialResult<()> {
        if self.max_depth == 0 {
            return Err(SpatialError::configuration(
                "max_depth",
                &self.max_depth.to_string(),
                "greater than 0"
            ));
        }

        if self.max_depth > 50 {
            return Err(SpatialError::configuration(
                "max_depth",
                &self.max_depth.to_string(),
                "50 or less (to prevent excessive memory usage)"
            ));
        }

        if self.min_polygons_per_leaf > self.max_polygons_per_leaf {
            return Err(SpatialError::configuration(
                "min_polygons_per_leaf",
                &format!("{} (max_polygons_per_leaf: {})", self.min_polygons_per_leaf, self.max_polygons_per_leaf),
                "less than or equal to max_polygons_per_leaf"
            ));
        }

        if self.max_polygons_per_leaf == 0 {
            return Err(SpatialError::configuration(
                "max_polygons_per_leaf",
                "0",
                "greater than 0"
            ));
        }

        if self.parallel_threshold == 0 {
            return Err(SpatialError::configuration(
                "parallel_threshold",
                "0",
                "greater than 0"
            ));
        }

        if self.min_subdivision_volume < 0.0 {
            return Err(SpatialError::configuration(
                "min_subdivision_volume",
                &self.min_subdivision_volume.to_string(),
                "greater than or equal to 0.0"
            ));
        }

        if self.balance_factor < 0.0 || self.balance_factor > 1.0 {
            return Err(SpatialError::configuration(
                "balance_factor",
                &self.balance_factor.to_string(),
                "between 0.0 and 1.0"
            ));
        }

        if self.spanning_factor < 0.0 || self.spanning_factor > 1.0 {
            return Err(SpatialError::configuration(
                "spanning_factor",
                &self.spanning_factor.to_string(),
                "between 0.0 and 1.0"
            ));
        }

        Ok(())
    }

    /// Create a validated configuration, returning an error if invalid
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::SpatialConfig;
    ///
    /// let config = SpatialConfig::validated(10, 1, 5, 100, true, true, 0.001, 0.5, 0.5);
    /// assert!(config.is_ok());
    ///
    /// let invalid_config = SpatialConfig::validated(0, 1, 5, 100, true, true, 0.001, 0.5, 0.5);
    /// assert!(invalid_config.is_err());
    /// ```
    pub fn validated(
        max_depth: usize,
        min_polygons_per_leaf: usize,
        max_polygons_per_leaf: usize,
        parallel_threshold: usize,
        use_sah: bool,
        adaptive_refinement: bool,
        min_subdivision_volume: Real,
        balance_factor: Real,
        spanning_factor: Real,
    ) -> SpatialResult<Self> {
        let config = Self {
            max_depth,
            min_polygons_per_leaf,
            max_polygons_per_leaf,
            parallel_threshold,
            use_sah,
            adaptive_refinement,
            min_subdivision_volume,
            balance_factor,
            spanning_factor,
        };

        config.validate()?;
        Ok(config)
    }
}

/// Performance characteristics matrix for different spatial structures
#[derive(Debug, Clone)]
pub struct PerformanceMatrix {
    pub construction_time: StructurePerformance,
    pub memory_usage: StructurePerformance,
    pub point_queries: StructurePerformance,
    pub range_queries: StructurePerformance,
    pub volume_queries: StructurePerformance,
    pub boolean_operations: StructurePerformance,
}

/// Performance ratings for different structures
#[derive(Debug, Clone)]
pub struct StructurePerformance {
    pub bsp: PerformanceRating,
    pub kdtree: PerformanceRating,
    pub octree: PerformanceRating,
}

/// Performance rating scale
#[derive(Debug, Clone, PartialEq)]
pub enum PerformanceRating {
    Excellent,  // Best choice for this operation
    Good,       // Good performance, suitable choice
    Fair,       // Acceptable performance
    Poor,       // Not recommended for this operation
}

impl Default for PerformanceMatrix {
    fn default() -> Self {
        Self {
            construction_time: StructurePerformance {
                bsp: PerformanceRating::Good,
                kdtree: PerformanceRating::Excellent,
                octree: PerformanceRating::Good,
            },
            memory_usage: StructurePerformance {
                bsp: PerformanceRating::Good,
                kdtree: PerformanceRating::Excellent,
                octree: PerformanceRating::Fair,
            },
            point_queries: StructurePerformance {
                bsp: PerformanceRating::Fair,
                kdtree: PerformanceRating::Excellent,
                octree: PerformanceRating::Good,
            },
            range_queries: StructurePerformance {
                bsp: PerformanceRating::Fair,
                kdtree: PerformanceRating::Excellent,
                octree: PerformanceRating::Good,
            },
            volume_queries: StructurePerformance {
                bsp: PerformanceRating::Fair,
                kdtree: PerformanceRating::Good,
                octree: PerformanceRating::Excellent,
            },
            boolean_operations: StructurePerformance {
                bsp: PerformanceRating::Excellent,
                kdtree: PerformanceRating::Poor,
                octree: PerformanceRating::Poor,
            },
        }
    }
}

/// Benchmarking utilities for spatial structures
pub struct SpatialBenchmark;

impl SpatialBenchmark {
    /// Compare construction time for different structures
    pub fn benchmark_construction<S: Clone + Send + Sync + std::fmt::Debug>(
        polygons: &[Polygon<S>],
    ) -> BenchmarkResults {
        use std::time::Instant;

        let start = Instant::now();
        let _bsp = crate::spatial::bsp::Node::from_polygons(polygons);
        let bsp_time = start.elapsed();

        let start = Instant::now();
        let _kdtree = crate::spatial::kdtree::Node::from_polygons(polygons);
        let kdtree_time = start.elapsed();

        let start = Instant::now();
        let _octree = crate::spatial::octree::Node::from_polygons(polygons);
        let octree_time = start.elapsed();

        BenchmarkResults {
            bsp_time,
            kdtree_time,
            octree_time,
        }
    }
}

/// Results from benchmarking different spatial structures
#[derive(Debug)]
pub struct BenchmarkResults {
    pub bsp_time: std::time::Duration,
    pub kdtree_time: std::time::Duration,
    pub octree_time: std::time::Duration,
}

impl BenchmarkResults {
    /// Get the fastest structure for construction
    pub fn fastest_construction(&self) -> SpatialStructureType {
        if self.bsp_time <= self.kdtree_time && self.bsp_time <= self.octree_time {
            SpatialStructureType::Bsp
        } else if self.kdtree_time <= self.octree_time {
            SpatialStructureType::Kdtree
        } else {
            SpatialStructureType::Octree
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_aabb_creation() {
        let min = Point3::new(0.0, 0.0, 0.0);
        let max = Point3::new(1.0, 1.0, 1.0);
        let aabb = Aabb::new(min, max);
        
        assert_eq!(aabb.min, min);
        assert_eq!(aabb.max, max);
        assert_eq!(aabb.volume(), 1.0);
    }

    #[test]
    fn test_aabb_contains_point() {
        let aabb = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        
        assert!(aabb.contains_point(&Point3::new(0.5, 0.5, 0.5)));
        assert!(!aabb.contains_point(&Point3::new(1.5, 0.5, 0.5)));
    }

    #[test]
    fn test_ray_creation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let direction = Vector3::new(1.0, 0.0, 0.0);
        let ray = Ray::new(origin, direction);
        
        assert_eq!(ray.origin, origin);
        assert_eq!(ray.direction, direction);
        assert_eq!(ray.point_at(1.0), Point3::new(1.0, 0.0, 0.0));
    }
}
