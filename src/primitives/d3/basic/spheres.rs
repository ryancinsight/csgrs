//! **Mathematical Foundations for Spherical Geometry**
//!
//! This module implements mathematically rigorous sphere generation using
//! spherical coordinates and UV parameterization based on classical
//! differential geometry principles.
//!
//! ## **Theoretical Foundation**
//!
//! ### **Spherical Coordinate System**
//! A sphere of radius r is parameterized using spherical coordinates:
//! ```text
//! x = r·sin(φ)·cos(θ)
//! y = r·cos(φ)
//! z = r·sin(φ)·sin(θ)
//! ```
//! where:
//! - **θ ∈ [0, 2π]**: Azimuthal angle (longitude)
//! - **φ ∈ [0, π]**: Polar angle (latitude from north pole)
//! - **r > 0**: Radius (constant for sphere)
//!
//! ### **UV Parameterization**
//! The sphere surface is mapped to parameter space:
//! ```text
//! u ∈ [0, 1] → θ = u·2π (wraps around equator)
//! v ∈ [0, 1] → φ = v·π   (from north to south pole)
//! ```
//!
//! ### **Surface Normal Calculation**
//! For a sphere, the outward normal at any point equals the position vector:
//! ```text
//! n⃗(θ,φ) = (sin(φ)cos(θ), cos(φ), sin(φ)sin(θ))ᵀ
//! ```
//! This is automatically unit length for unit sphere.
//!
//! ### **Quadrilateral Tessellation**
//! The sphere is divided into quadrilateral patches:
//! - **Segments**: Divisions around equator (longitude slices)
//! - **Stacks**: Divisions from pole to pole (latitude bands)
//! - **Total quads**: segments × stacks
//!
//! ### **Degenerate Triangle Handling**
//! At poles (φ = 0 or φ = π), quadrilaterals degenerate to triangles
//! since multiple longitude values map to the same point.
//!
//! ## **Geometric Properties**
//! - **Surface Area**: A = 4πr²
//! - **Volume**: V = (4/3)πr³
//! - **Gaussian Curvature**: K = 1/r² (constant positive)
//! - **Mean Curvature**: H = 1/r (constant positive)
//!
//! All generated meshes maintain proper topology for CSG operations.

use crate::core::float_types::{PI, Real, TAU};
use crate::csg::CSG;
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// **Mathematical Foundation: Spherical Mesh Generation**
    ///
    /// Construct a sphere using UV-parameterized quadrilateral tessellation.
    /// This implements the standard spherical coordinate parameterization
    /// with adaptive handling of polar degeneracies.
    ///
    /// ## **Sphere Mathematics**
    ///
    /// ### **Parametric Surface Equations**
    /// The sphere surface is defined by:
    /// ```text
    /// S(u,v) = r(sin(πv)cos(2πu), cos(πv), sin(πv)sin(2πu))
    /// where u ∈ [0,1], v ∈ [0,1]
    /// ```
    ///
    /// ### **Tessellation Algorithm**
    /// 1. **Parameter Grid**: Create (segments+1) × (stacks+1) parameter values
    /// 2. **Vertex Generation**: Evaluate S(u,v) at grid points
    /// 3. **Quadrilateral Formation**: Connect adjacent grid points
    /// 4. **Degeneracy Handling**: Poles require triangle adaptation
    ///
    /// ### **Pole Degeneracy Resolution**
    /// At poles (v=0 or v=1), the parameterization becomes singular:
    /// - **North pole** (v=0): All u values map to same point (0, r, 0)
    /// - **South pole** (v=1): All u values map to same point (0, -r, 0)
    /// - **Solution**: Use triangles instead of quads for polar caps
    ///
    /// ### **Normal Vector Computation**
    /// Sphere normals are simply the normalized position vectors:
    /// ```text
    /// n⃗ = p⃗/|p⃗| = (x,y,z)/r
    /// ```
    /// This is mathematically exact for spheres (no approximation needed).
    ///
    /// ### **Mesh Quality Metrics**
    /// - **Aspect Ratio**: Best when segments ≈ 2×stacks
    /// - **Area Distortion**: Minimal at equator, maximal at poles
    /// - **Angular Distortion**: Increases towards poles (unavoidable)
    ///
    /// ### **Numerical Considerations**
    /// - **Trigonometric Precision**: Uses TAU and PI for accuracy
    /// - **Pole Handling**: Avoids division by zero at singularities
    /// - **Winding Consistency**: Maintains outward-facing orientation
    ///
    /// ## **Geometric Properties**
    /// - **Surface Area**: A = 4πr²
    /// - **Volume**: V = (4/3)πr³
    /// - **Circumference** (any great circle): C = 2πr
    /// - **Curvature**: Gaussian K = 1/r², Mean H = 1/r
    ///
    /// # Parameters
    /// - `radius`: Sphere radius (> 0)
    /// - `segments`: Longitude divisions (≥ 3, recommend ≥ 8)
    /// - `stacks`: Latitude divisions (≥ 2, recommend ≥ 6)
    /// - `metadata`: Optional metadata for all faces
    pub fn sphere(
        radius: Real,
        segments: usize,
        stacks: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        // Vertex generation function using mathematical sphere parameterization
        let vertex = |theta: Real, phi: Real| {
            let dir = Vector3::new(
                theta.cos() * phi.sin(),
                phi.cos(),
                theta.sin() * phi.sin(),
            );
            Vertex::new(
                Point3::new(dir.x * radius, dir.y * radius, dir.z * radius),
                dir,
            )
        };

        // Use advanced iterator patterns for sphere tessellation
        #[cfg(feature = "parallel")]
        let polygons: Vec<Polygon<S>> = {
            if segments * stacks > 10000 { // High-resolution spheres
                use rayon::prelude::*;

                // Parallel processing for high-resolution spheres
                (0..segments)
                    .into_par_iter()
                    .flat_map(|i| {
                        (0..stacks)
                            .into_par_iter()
                            .map(move |j| (i, j))
                    })
                    .map(|(i, j)| {
                        // Use enumerate-like pattern for coordinate generation
                        let (t0, t1) = (i as Real / segments as Real, (i + 1) as Real / segments as Real);
                        let (p0, p1) = (j as Real / stacks as Real, (j + 1) as Real / stacks as Real);

                        let (theta0, theta1) = (t0 * TAU, t1 * TAU);
                        let (phi0, phi1) = (p0 * PI, p1 * PI);

                        // Generate vertices using iterator patterns
                        let mut vertices = vec![vertex(theta0, phi0)];

                        // Conditional vertex addition using iterator patterns
                        if j > 0 {
                            vertices.push(vertex(theta1, phi0));
                        }
                        if j < stacks - 1 {
                            vertices.push(vertex(theta1, phi1));
                        }
                        vertices.push(vertex(theta0, phi1));

                        Polygon::new(vertices, metadata.clone())
                    })
                    .collect()
            } else {
                // Sequential processing for smaller spheres
                (0..segments)
                    .flat_map(|i| {
                        (0..stacks).map(move |j| (i, j))
                    })
                    .map(|(i, j)| {
                        let (t0, t1) = (i as Real / segments as Real, (i + 1) as Real / segments as Real);
                        let (p0, p1) = (j as Real / stacks as Real, (j + 1) as Real / stacks as Real);

                        let (theta0, theta1) = (t0 * TAU, t1 * TAU);
                        let (phi0, phi1) = (p0 * PI, p1 * PI);

                        let mut vertices = vec![vertex(theta0, phi0)];

                        if j > 0 {
                            vertices.push(vertex(theta1, phi0));
                        }
                        if j < stacks - 1 {
                            vertices.push(vertex(theta1, phi1));
                        }
                        vertices.push(vertex(theta0, phi1));

                        Polygon::new(vertices, metadata.clone())
                    })
                    .collect()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let polygons: Vec<Polygon<S>> = (0..segments)
            .flat_map(|i| {
                (0..stacks).map(move |j| (i, j))
            })
            .map(|(i, j)| {
                // Use enumerate-like pattern for coordinate generation
                let (t0, t1) = (i as Real / segments as Real, (i + 1) as Real / segments as Real);
                let (p0, p1) = (j as Real / stacks as Real, (j + 1) as Real / stacks as Real);

                let (theta0, theta1) = (t0 * TAU, t1 * TAU);
                let (phi0, phi1) = (p0 * PI, p1 * PI);

                // Generate vertices using functional patterns
                let mut vertices = vec![vertex(theta0, phi0)];

                // Conditional vertex addition
                if j > 0 {
                    vertices.push(vertex(theta1, phi0));
                }
                if j < stacks - 1 {
                    vertices.push(vertex(theta1, phi1));
                }
                vertices.push(vertex(theta0, phi1));

                Polygon::new(vertices, metadata.clone())
            })
            .collect();

        CSG::from_polygons(&polygons)
    }
}
