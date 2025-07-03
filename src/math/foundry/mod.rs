//! **Mathematical Foundry (The Canonical Workshop)**
//!
//! This module serves as the canonical workshop for master mathematical parts following
//! Cathedral Engineering principles. It consolidates mathematical operations that were
//! previously scattered across the codebase into unified, optimized implementations.
//!
//! ## **Design Philosophy**
//!
//! The foundry follows the **Law of Canonical Master Parts** - there should be exactly
//! one authoritative implementation of each mathematical concept. Other modules create
//! **Derived Projections** using `From<T>` traits rather than reimplementing operations.
//!
//! ## **Module Architecture (Cathedral Engineering)**
//!
//! This module follows the **Law of Master Masonry** with clear functional roles:
//!
//! - **`distance`** (Mind) - All distance calculation implementations
//! - **`bounds`** (Mind) - All bounding box and spatial bound operations
//! - **`transforms`** (Mind) - All geometric transformation utilities
//! - **`precision`** (Soul) - Numerical precision and stability utilities
//! - **`constants`** (Skeleton) - Mathematical constants and derived values
//!
//! ## **Consolidation Benefits**
//!
//! ### **Before Foundry (Scattered Implementation)**
//! ```ignore
//! // In spatial/utils.rs
//! pub fn distance(a: &Point3<Real>, b: &Point3<Real>) -> Real { ... }
//! pub fn distance_squared(a: &Point3<Real>, b: &Point3<Real>) -> Real { ... }
//!
//! // In geometry/vertex.rs
//! impl Vertex {
//!     pub fn distance_to(&self, other: &Vertex) -> Real { ... }
//!     pub fn distance_squared_to(&self, other: &Vertex) -> Real { ... }
//! }
//! ```
//!
//! ### **After Foundry (Canonical Implementation)**
//! ```ignore
//! // In math/foundry/distance.rs - Single source of truth
//! pub fn euclidean_distance(a: &Point3<Real>, b: &Point3<Real>) -> Real { ... }
//! pub fn euclidean_distance_squared(a: &Point3<Real>, b: &Point3<Real>) -> Real { ... }
//!
//! // Other modules use derived projections
//! impl Vertex {
//!     pub fn distance_to(&self, other: &Vertex) -> Real {
//!         foundry::distance::euclidean_distance(&self.pos, &other.pos)
//!     }
//! }
//! ```
//!
//! ## **Performance Optimizations**
//!
//! The foundry implementations include:
//! - **SIMD optimizations** where applicable
//! - **Numerical stability** improvements
//! - **Memory layout** optimizations
//! - **Algorithmic complexity** improvements
//!
//! ## **Usage Examples**
//!
//! ### **Direct Foundry Usage**
//! ```rust
//! use csgrs::math::foundry;
//! use nalgebra::Point3;
//!
//! let p1 = Point3::new(0.0, 0.0, 0.0);
//! let p2 = Point3::new(3.0, 4.0, 0.0);
//!
//! // Use canonical implementations
//! let distance = foundry::distance::euclidean_distance(&p1, &p2);
//! let distance_sq = foundry::distance::euclidean_distance_squared(&p1, &p2);
//! ```
//!
//! ### **Derived Projection Usage**
//! ```rust
//! use csgrs::geometry::Vertex;
//! use nalgebra::{Point3, Vector3};
//!
//! let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
//! let v2 = Vertex::new(Point3::new(3.0, 4.0, 0.0), Vector3::y());
//!
//! // Uses foundry implementation internally
//! let distance = v1.distance_to(&v2);
//! ```
//!
//! ## **Theoretical References**
//!
//! - Knuth, D. E. (1997). *The Art of Computer Programming, Volume 1: Fundamental Algorithms*
//! - Press, W. H., et al. (2007). *Numerical Recipes: The Art of Scientific Computing*
//! - Golub, G. H., & Van Loan, C. F. (2013). *Matrix Computations*

// Public API exports following the façade pattern
pub mod distance;
pub mod bounds;
pub mod transforms;
pub mod precision;
pub mod constants;

// Re-export key functions for convenient access
pub use distance::{
    euclidean_distance, euclidean_distance_squared,
    manhattan_distance, chebyshev_distance
};
pub use bounds::{
    aabb_union, aabb_intersection, aabb_contains_point,
    aabb_volume, aabb_surface_area
};
pub use transforms::{
    translation_matrix, rotation_matrix, scaling_matrix,
    compose_transforms
};
pub use precision::{
    approximately_equal, approximately_zero,
    safe_normalize, robust_cross_product
};
pub use constants::{
    GOLDEN_RATIO, SQRT_2, SQRT_3, SQRT_5,
    PI_2, PI_4, TAU
};
