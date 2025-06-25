//! Primitive generation module
//!
//! This module provides functions for generating various geometric primitives
//! including 2D shapes, 3D shapes, and related utilities.

pub mod d2;
pub mod d3;

// Re-export all primitive creation functions for convenient access
// Note: These are impl block methods, so wildcard imports may appear unused but provide the functionality
// 2D primitive functions
#[allow(unused_imports)]
pub use d2::basic::*;
#[allow(unused_imports)]
pub use d2::complex::*;
#[allow(unused_imports)]
pub use d2::curves::*;
#[allow(unused_imports)]
pub use d2::gears::*;

// 3D primitive functions
#[allow(unused_imports)]
pub use d3::basic::*;
#[allow(unused_imports)]
pub use d3::complex::*;
#[allow(unused_imports)]
pub use d3::gears::*;
#[allow(unused_imports)]
pub use d3::platonic::*;
#[allow(unused_imports)]
pub use d3::specialized::*;
