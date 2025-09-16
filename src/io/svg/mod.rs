//! Modular SVG parsing and export functionality
//!
//! This module provides comprehensive SVG support for the csgrs library,
//! broken down into focused submodules for better maintainability.

pub mod export;
pub mod parser;
pub mod path;
pub mod transform;

// Re-export main functionality
pub use export::*;
pub use parser::*;
pub use path::*;
pub use transform::*;
