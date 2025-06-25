//! Mathematical algorithms and computational geometry

#[cfg(feature = "chull-io")]
pub mod convex_hull;

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(feature = "offset")]
pub mod offset;

#[cfg(feature = "sdf")]
pub mod sdf;

#[cfg(feature = "sdf")]
pub mod tpms;
