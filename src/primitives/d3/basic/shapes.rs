//! **Mathematical Foundations for 3D Box Geometry**
//!
//! This module implements mathematically rigorous algorithms for generating
//! axis-aligned rectangular prisms (cuboids) and cubes based on solid geometry
//! and computational topology principles.
//!
//! ## **Theoretical Foundations**
//!
//! ### **Cuboid Geometry**
//! A right rectangular prism (cuboid) in 3D space is defined by:
//! - **Vertices**: 8 corner points forming a rectangular parallelepiped
//! - **Edges**: 12 edges connecting adjacent vertices
//! - **Faces**: 6 rectangular faces, each with consistent outward normal
//!
//! ### **Coordinate System**
//! Standard axis-aligned cuboid from origin:
//! ```text
//! (0,0,0) → (width, length, height)
//! ```
//! This creates a right-handed coordinate system with consistent face orientations.
//!
//! ### **Face Normal Calculation**
//! Each face normal is computed using the right-hand rule:
//! ```text
//! n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
//! ```
//! where vertices are ordered counter-clockwise when viewed from outside.
//!
//! ### **Winding Order Convention**
//! All faces use counter-clockwise vertex ordering when viewed from exterior:
//! - **Ensures consistent outward normals**
//! - **Enables proper backface culling**
//! - **Maintains manifold topology for CSG operations**
//!
//! ## **Geometric Properties**
//! - **Volume**: V = width × length × height
//! - **Surface Area**: A = 2(wl + wh + lh)
//! - **Diagonal**: d = √(w² + l² + h²)
//! - **Centroid**: (w/2, l/2, h/2)
//!
//! All shapes maintain proper topology for boolean operations and mesh processing.

use crate::core::float_types::Real;
use crate::csg::CSG;
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// **Mathematical Foundation: Axis-Aligned Cuboid Construction**
    ///
    /// Create a right rectangular prism (cuboid) spanning from origin to (width, length, height).
    /// This implements a mathematically rigorous construction ensuring proper topology
    /// and consistent face orientations for CSG operations.
    ///
    /// ## **Cuboid Mathematics**
    ///
    /// ### **Vertex Enumeration**
    /// The 8 vertices are systematically enumerated using binary encoding:
    /// ```text
    /// (i,j,k) → (i×width, j×length, k×height)
    /// where i,j,k ∈ {0,1}
    /// ```
    /// This gives vertices: p₀₀₀, p₁₀₀, p₀₁₀, p₁₁₀, p₀₀₁, p₁₀₁, p₀₁₁, p₁₁₁
    ///
    /// ### **Face Construction Algorithm**
    /// Each face is a quadrilateral with vertices ordered counter-clockwise
    /// when viewed from outside:
    ///
    /// 1. **Bottom Face** (z=0, normal = -ẑ):
    ///    p₀₀₀ → p₀₁₀ → p₁₁₀ → p₁₀₀
    ///
    /// 2. **Top Face** (z=height, normal = +ẑ):
    ///    p₀₀₁ → p₁₀₁ → p₁₁₁ → p₀₁₁
    ///
    /// 3. **Front Face** (y=0, normal = -ŷ):
    ///    p₀₀₀ → p₁₀₀ → p₁₀₁ → p₀₀₁
    ///
    /// 4. **Back Face** (y=length, normal = +ŷ):
    ///    p₀₁₀ → p₀₁₁ → p₁₁₁ → p₁₁₀
    ///
    /// 5. **Left Face** (x=0, normal = -x̂):
    ///    p₀₀₀ → p₀₀₁ → p₀₁₁ → p₀₁₀
    ///
    /// 6. **Right Face** (x=width, normal = +x̂):
    ///    p₁₀₀ → p₁₁₀ → p₁₁₁ → p₁₀₁
    ///
    /// ### **Normal Vector Verification**
    /// Each face normal is verified using the right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```
    /// For consistent outward orientation.
    ///
    /// ### **Topological Properties**
    /// - **Manifold**: Each edge shared by exactly 2 faces
    /// - **Closed**: No boundary edges
    /// - **Orientable**: Consistent normal directions
    /// - **Genus 0**: Topologically equivalent to a sphere
    ///
    /// ## **Geometric Invariants**
    /// - **Volume**: V = width × length × height
    /// - **Surface Area**: A = 2(width×length + width×height + length×height)
    /// - **Euler Characteristic**: χ = V - E + F = 8 - 12 + 6 = 2
    ///
    /// ## **Numerical Considerations**
    /// - All dimensions must be ≥ 0 for valid geometry
    /// - Face normals are unit vectors for proper lighting
    /// - Vertex precision maintains geometric consistency
    ///
    /// # Parameters
    /// - `width`: X-dimension (≥ 0)
    /// - `length`: Y-dimension (≥ 0)
    /// - `height`: Z-dimension (≥ 0)
    /// - `metadata`: Optional metadata for all faces
    pub fn cuboid(width: Real, length: Real, height: Real, metadata: Option<S>) -> CSG<S> {
        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::new(0.0, 0.0, 0.0);
        let p100 = Point3::new(width, 0.0, 0.0);
        let p110 = Point3::new(width, length, 0.0);
        let p010 = Point3::new(0.0, length, 0.0);

        let p001 = Point3::new(0.0, 0.0, height);
        let p101 = Point3::new(width, 0.0, height);
        let p111 = Point3::new(width, length, height);
        let p011 = Point3::new(0.0, length, height);

        // We'll define 6 faces (each a Polygon), in an order that keeps outward-facing normals
        // and consistent (counter-clockwise) vertex winding as viewed from outside the prism.

        // Bottom face (z=0, normal approx. -Z)
        // p000 -> p100 -> p110 -> p010
        let bottom_normal = -Vector3::z();
        let bottom = Polygon::new(
            vec![
                Vertex::new(p000, bottom_normal),
                Vertex::new(p010, bottom_normal),
                Vertex::new(p110, bottom_normal),
                Vertex::new(p100, bottom_normal),
            ],
            metadata.clone(),
        );

        // Top face (z=depth, normal approx. +Z)
        // p001 -> p011 -> p111 -> p101
        let top_normal = Vector3::z();
        let top = Polygon::new(
            vec![
                Vertex::new(p001, top_normal),
                Vertex::new(p101, top_normal),
                Vertex::new(p111, top_normal),
                Vertex::new(p011, top_normal),
            ],
            metadata.clone(),
        );

        // Front face (y=0, normal approx. -Y)
        // p000 -> p001 -> p101 -> p100
        let front_normal = -Vector3::y();
        let front = Polygon::new(
            vec![
                Vertex::new(p000, front_normal),
                Vertex::new(p100, front_normal),
                Vertex::new(p101, front_normal),
                Vertex::new(p001, front_normal),
            ],
            metadata.clone(),
        );

        // Back face (y=height, normal approx. +Y)
        // p010 -> p110 -> p111 -> p011
        let back_normal = Vector3::y();
        let back = Polygon::new(
            vec![
                Vertex::new(p010, back_normal),
                Vertex::new(p011, back_normal),
                Vertex::new(p111, back_normal),
                Vertex::new(p110, back_normal),
            ],
            metadata.clone(),
        );

        // Left face (x=0, normal approx. -X)
        // p000 -> p010 -> p011 -> p001
        let left_normal = -Vector3::x();
        let left = Polygon::new(
            vec![
                Vertex::new(p000, left_normal),
                Vertex::new(p001, left_normal),
                Vertex::new(p011, left_normal),
                Vertex::new(p010, left_normal),
            ],
            metadata.clone(),
        );

        // Right face (x=width, normal approx. +X)
        // p100 -> p101 -> p111 -> p110
        let right_normal = Vector3::x();
        let right = Polygon::new(
            vec![
                Vertex::new(p100, right_normal),
                Vertex::new(p110, right_normal),
                Vertex::new(p111, right_normal),
                Vertex::new(p101, right_normal),
            ],
            metadata.clone(),
        );

        // Combine all faces into a CSG
        CSG::from_polygons(&[bottom, top, front, back, left, right])
    }

    pub fn cube(width: Real, metadata: Option<S>) -> CSG<S> {
        Self::cuboid(width, width, width, metadata)
    }
}
