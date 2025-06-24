//! Primitive generation module
//!
//! This module provides functions for generating various geometric primitives
//! including 2D shapes, 3D shapes, and related utilities.

pub mod d2;
pub mod d3;

// Re-export all primitives for convenience
pub use d2::*;
pub use d3::*; 
