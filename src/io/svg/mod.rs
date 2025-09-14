//! Modular SVG parsing and export functionality
//!
//! This module provides comprehensive SVG support for the csgrs library,
//! broken down into focused submodules for better maintainability.

pub mod transform;
pub mod path;
pub mod parser;
pub mod export;

// Re-export main functionality
pub use transform::*;
pub use path::*;
pub use parser::*;
pub use export::*;
