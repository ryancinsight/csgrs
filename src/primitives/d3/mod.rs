//! 3D primitive generation module
//!
//! This module provides functions for generating various 3D geometric primitives
//! organized into logical categories for better maintainability and code clarity.

pub mod basic;
pub mod complex;
pub mod gears;
pub mod platonic;
pub mod specialized;

// Note: impl blocks cannot be re-exported, they are automatically available when modules are included
