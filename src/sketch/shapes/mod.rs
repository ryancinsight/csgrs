//! Modular 2D shape implementations for Sketch
//!
//! This module provides a clean separation of different shape categories
//! to improve maintainability and reduce monolithic file sizes.

pub mod basic;
pub mod complex;
pub mod curves;
pub mod gears;

// Note: Re-exports will be added as modules implement public functions
// For now, functions are accessed via their respective modules
