//! Core types and foundational definitions

pub mod errors;
pub mod float_types;

// Re-export commonly used items
pub use errors::ValidationError;
pub use float_types::{EPSILON, FRAC_PI_2, PI, Real, TAU};
