//! 2D primitive shapes and curves
//!
//! This module provides a comprehensive collection of 2D geometric primitives
//! organized by category for optimal maintainability and discoverability.

pub mod basic;
pub mod complex;
pub mod curves;
pub mod gears;
pub mod utils;

// Re-export all public functions for unified API
#[allow(unused_imports)]
pub use basic::*;
#[allow(unused_imports)]
pub use complex::*;
#[allow(unused_imports)]
pub use curves::*;
#[allow(unused_imports)]
pub use gears::*;
