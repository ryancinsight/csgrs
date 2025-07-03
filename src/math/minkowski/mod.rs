//! **Minkowski Operations Module**
//!
//! This module implements Minkowski sum and difference operations following Cathedral Engineering
//! principles with deep hierarchical organization and single responsibility modules.
//!
//! ## **Mathematical Foundation**
//!
//! ### **Minkowski Sum Definition**
//! For sets A and B in Euclidean space, the Minkowski sum A ⊕ B is defined as:
//! ```text
//! A ⊕ B = {a + b | a ∈ A, b ∈ B}
//! ```
//!
//! ### **Geometric Interpretation**
//! - **Shape Expansion**: The Minkowski sum expands shape A by the geometry of shape B
//! - **Collision Detection**: Useful for computing safety margins and buffer zones
//! - **Morphological Operations**: Foundation for dilation and erosion in image processing
//!
//! ### **Algorithmic Complexity**
//! - **Vertex Enumeration**: O(|A| × |B|) for generating all pairwise sums
//! - **Convex Hull Computation**: O(n log n) where n = |A| × |B|
//! - **Total Complexity**: O(|A| × |B| + n log n)
//!
//! ## **Module Architecture (Cathedral Engineering)**
//!
//! This module follows the **Law of Master Masonry** with clear functional roles:
//!
//! - **`sum`** (Mind) - Minkowski sum operations and implementations
//! - **`difference`** (Mind) - Minkowski difference operations and implementations  
//! - **`contracts`** (Soul) - Traits defining Minkowski operation interfaces
//! - **`models`** (Skeleton) - Core data structures for Minkowski operations
//! - **`errors`** (Immune System) - Error types and handling for Minkowski operations
//!
//! ## **Usage Examples**
//!
//! ### **Basic Minkowski Sum**
//! ```rust
//! use csgrs::CSG;
//! 
//! let cube: CSG<()> = CSG::cube(2.0, None);
//! let sphere: CSG<()> = CSG::sphere(0.5, 16, 8, None);
//! 
//! // Compute Minkowski sum (expands cube by sphere shape)
//! let expanded = cube.minkowski_sum(&sphere);
//! ```
//!
//! ### **Advanced Modular Usage**
//! ```ignore
//! use csgrs::math::minkowski;
//!
//! // Direct module access for advanced scenarios
//! let result = minkowski::sum::compute(&csg_a, &csg_b);
//! ```
//!
//! ## **Theoretical References**
//!
//! - Preparata, F. P., & Shamos, M. I. (1985). *Computational Geometry: An Introduction*
//! - de Berg, M., et al. (2008). *Computational Geometry: Algorithms and Applications*
//! - Ghosh, P. K. (1993). "A unified computational framework for Minkowski operations"

// Public API exports following the façade pattern
pub mod sum;
pub mod difference;
pub mod contracts;
pub mod models;
pub mod errors;

// Re-export key types and functions for convenient access
pub use sum::compute as minkowski_sum;
pub use difference::compute as minkowski_difference;
pub use contracts::{MinkowskiOperation, MinkowskiComputable};
pub use errors::{MinkowskiError, MinkowskiResult};
