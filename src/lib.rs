//! A fast, optionally multithreaded **Constructive Solid Geometry (CSG)** library,
//! built around Boolean operations (*union*, *difference*, *intersection*, *xor*) on sets of polygons stored in [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) trees.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]
//! ## Surface Quality Considerations
//!
//! **Mesh/IndexedMesh Systems**: Provide smooth, mathematically precise surfaces using polygon-based representations
//! **Sparse Voxel System**: Uses discrete volume sampling for efficient CSG operations but produces blocky surfaces
//!
//! For applications requiring smooth surfaces, use Mesh or IndexedMesh primitives. The sparse voxel system
//! excels at volume-based boolean operations but inherently produces blocky, low-resolution surfaces due to
//! discrete voxel sampling and cube-based mesh reconstruction.
//!
//! ## Documentation
//! - **[User Guide](https://github.com/timschmidt/csgrs/blob/main/docs/user_guide.md)**: Practical examples and usage patterns
//! - **[Mathematical Foundations](https://github.com/timschmidt/csgrs/blob/main/docs/mathematical_foundations.md)**: Theoretical basis and algorithmic principles
//! - **[API Reference](https://docs.rs/csgrs)**: Generated Rust documentation
//!
//! # Features
//! #### Default
//! - **f64**: use f64 as Real
//! - [**stl-io**](https://en.wikipedia.org/wiki/STL_(file_format)): `.stl` import/export
//! - [**dxf-io**](https://en.wikipedia.org/wiki/AutoCAD_DXF): `.dxf` import/export
//! - **chull-io**: convex hull and minkowski sum
//! - **metaballs**: enables an implementation of [metaballs](https://en.wikipedia.org/wiki/Metaballs)
//! - **sdf**: signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) using [fast-surface-nets](https://crates.io/crates/fast-surface-nets)
//! - **offset**: use `geo-buf` for offset operations
//! - **delaunay**: use `geo`s `spade` feature for triangulation
//!
//! #### Optional
//! - **f32**: use f32 as Real, this conflicts with f64
//! - **parallel**: use rayon for multithreading
//! - **svg-io**: create `Sketch`s from and convert `Sketch`s to SVG's
//! - **truetype-text**: create `Sketch`s using TrueType fonts `.ttf`
//! - **hershey-text**: create `Sketch`s using Hershey fonts (`.jhf`)
//! - **image-io**: make `Sketch`s from images
//! - **earcut**: use `geo`s `earcutr` feature for triangulation
//! - **bevymesh**: for conversion to a bevy `Mesh`

#![forbid(unsafe_code)]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]

pub mod errors;
pub mod examples;
pub mod float_types;
pub mod indexed_mesh;
pub mod io;
pub mod mesh;
pub mod sketch;
pub mod traits;
pub mod voxels;

// SIMD optimizations (optional feature)
#[cfg(feature = "simd")]
pub mod simd;

// GPU acceleration for indexed mesh operations (optional feature)
#[cfg(feature = "gpu")]
pub mod gpu;

// Procedural generation and mesh synthesis algorithms (temporarily disabled)
// pub mod procedural;

// Note: delaunay and earcut features can both be enabled, but delaunay takes precedence
// This allows --all-features to work while maintaining backward compatibility

// Note: f64 and f32 features can both be enabled, but f64 takes precedence
// This allows --all-features to work while maintaining backward compatibility

#[cfg(not(any(feature = "f64", feature = "f32")))]
compile_error!("Either 'f64' or 'f32' feature must be specified");

#[cfg(test)]
mod tests;
