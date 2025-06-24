//! Complex 2D shapes that require advanced geometric calculations
//!
//! This module organizes sophisticated 2D shapes into logical categories:
//! - Decorative shapes (stars, hearts, crescents, teardrops, etc.)
//! - Functional shapes (rounded rectangles, keyholes, rings, circles with modifications)
//! - Parametric shapes (squircles, reuleaux polygons, supershapes, pie slices)

pub mod decorative;
pub mod functional;
pub mod parametric;

// Re-export all public functions for unified API
#[allow(unused_imports)]
pub use decorative::*;
#[allow(unused_imports)]
pub use functional::*;
#[allow(unused_imports)]
pub use parametric::*; 
