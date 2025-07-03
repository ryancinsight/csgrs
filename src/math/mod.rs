//! Mathematical algorithms and computational geometry
//!
//! This module provides a comprehensive suite of mathematical operations and computational
//! geometry algorithms following Cathedral Engineering principles with deep hierarchical
//! organization and single responsibility modules.
//!
//! ## **Module Architecture**
//!
//! The mathematical module is organized into specialized sub-modules:
//!
//! - **`convex_hull`** - Pure convex hull algorithms and operations
//! - **`minkowski`** - Minkowski sum and difference operations (Cathedral Engineering compliant)
//! - **`foundry`** - Canonical master parts for mathematical operations (Cathedral Engineering compliant)
//! - **`metaballs`** - Metaball and isosurface generation
//! - **`offset`** - 2D polygon offsetting and buffering operations
//! - **`sdf`** - Signed distance field meshing and operations
//! - **`tpms`** - Triply-periodic minimal surfaces
//!
//! ## **Cathedral Engineering Compliance**
//!
//! The `minkowski` and `foundry` modules follow Cathedral Engineering principles:
//! - **Deep hierarchical organization** with proper module anatomy
//! - **Single responsibility** modules with clear functional roles
//! - **Canonical master parts** to eliminate code duplication
//! - **Comprehensive documentation** with mathematical foundations

#[cfg(feature = "chull-io")]
pub mod convex_hull;

#[cfg(feature = "chull-io")]
pub mod minkowski;

pub mod foundry;

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(feature = "offset")]
pub mod offset;

#[cfg(feature = "sdf")]
pub mod sdf;

#[cfg(feature = "sdf")]
pub mod tpms;
