//! The [convex hull](https://en.wikipedia.org/wiki/Convex_hull) of a shape is the smallest convex set that contains it.
//! It may be visualized as the shape enclosed by a rubber band stretched around the subset.
//!
//! This is the set:\
//! ![Pre-ConvexHull demo image][Pre-ConvexHull demo image]
//!
//! And this is the convex hull of that set:\
//! ![ConvexHull demo image][ConvexHull demo image]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Pre-ConvexHull demo image", "docs/convex_hull_before_nobackground.png"))]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("ConvexHull demo image", "docs/convex_hull_nobackground.png"))]

use crate::core::float_types::Real;
use crate::csg::CSG;
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use chull::ConvexHullWrapper;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Compute the [convex hull](https://en.wikipedia.org/wiki/Convex_hull) of all vertices in this CSG.
    pub fn convex_hull(&self) -> CSG<S> {
        // Gather all (x, y, z) coordinates from the polygons
        let points: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        let points_for_hull: Vec<Vec<Real>> =
            points.iter().map(|p| vec![p.x, p.y, p.z]).collect();

        // Attempt to compute the convex hull using the robust wrapper
        let hull = match ConvexHullWrapper::try_new(&points_for_hull, None) {
            Ok(h) => h,
            Err(_) => {
                // Fallback to an empty CSG if hull generation fails
                return CSG::new();
            },
        };

        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons as triangles
        let mut polygons = Vec::new();
        for tri in indices.chunks(3) {
            let v0 = &verts[tri[0]];
            let v1 = &verts[tri[1]];
            let v2 = &verts[tri[2]];
            let vv0 = Vertex::new(Point3::new(v0[0], v0[1], v0[2]), Vector3::zeros());
            let vv1 = Vertex::new(Point3::new(v1[0], v1[1], v1[2]), Vector3::zeros());
            let vv2 = Vertex::new(Point3::new(v2[0], v2[1], v2[2]), Vector3::zeros());
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], None));
        }

        CSG::from_polygons(&polygons)
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// **Mathematical Foundation**: For convex sets A and B, A ⊕ B = {a + b | a ∈ A, b ∈ B}.
    /// By the Minkowski sum theorem, the convex hull of all pairwise vertex sums equals
    /// the Minkowski sum of the convex hulls of A and B.
    ///
    /// **Algorithm**: O(|A| × |B|) vertex combinations followed by O(n log n) convex hull computation.
    pub fn minkowski_sum(&self, other: &CSG<S>) -> CSG<S> {
        // Extract vertices using iterator chains for better performance
        let verts_a: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        let verts_b: Vec<Point3<Real>> = other
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        // Handle empty input cases (fix missing return statement)
        if verts_a.is_empty() || verts_b.is_empty() {
            return CSG::new();
        }

        // Pre-allocate capacity for better memory performance
        let expected_capacity = verts_a.len() * verts_b.len();
        let mut sum_points = Vec::with_capacity(expected_capacity);

        // Compute Minkowski sum using optimized iterator pattern
        // Mathematical theorem: A ⊕ B = {a + b | a ∈ A, b ∈ B}
        for a in &verts_a {
            for b in &verts_b {
                sum_points.push(a + b.coords);
            }
        }

        // Early return if no points generated
        if sum_points.is_empty() {
            return CSG::new();
        }

        // Convert to format expected by hull library
        let points_for_hull: Vec<Vec<Real>> =
            sum_points.iter().map(|p| vec![p.x, p.y, p.z]).collect();

        // Compute convex hull with proper error handling
        let hull = match ConvexHullWrapper::try_new(&points_for_hull, None) {
            Ok(h) => h,
            Err(_) => return CSG::new(), // Robust fallback for degenerate cases
        };

        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons with proper normal vector calculation
        let polygons: Vec<Polygon<S>> = indices
            .chunks_exact(3)
            .filter_map(|tri| {
                let v0 = &verts[tri[0]];
                let v1 = &verts[tri[1]];
                let v2 = &verts[tri[2]];

                let p0 = Point3::new(v0[0], v0[1], v0[2]);
                let p1 = Point3::new(v1[0], v1[1], v1[2]);
                let p2 = Point3::new(v2[0], v2[1], v2[2]);

                // Calculate proper normal vector using cross product
                let edge1 = p1 - p0;
                let edge2 = p2 - p0;
                let normal = edge1.cross(&edge2);

                // Filter out degenerate triangles
                if normal.norm_squared() > Real::EPSILON {
                    let normalized_normal = normal.normalize();
                    let vv0 = Vertex::new(p0, normalized_normal);
                    let vv1 = Vertex::new(p1, normalized_normal);
                    let vv2 = Vertex::new(p2, normalized_normal);
                    Some(Polygon::new(vec![vv0, vv1, vv2], None))
                } else {
                    None
                }
            })
            .collect();

        CSG::from_polygons(&polygons)
    }
}
