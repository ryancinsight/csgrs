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

        // Reconstruct polygons as triangles using advanced iterator patterns
        #[cfg(feature = "parallel")]
        let polygons: Vec<Polygon<S>> = {
            if indices.len() > 3000 { // More than 1000 triangles
                use rayon::prelude::*;

                // Use parallel processing for large hull datasets
                indices
                    .par_chunks(3)
                    .filter_map(|tri| {
                        if tri.len() == 3 {
                            // Use iterator patterns for robust vertex creation
                            let vertices_result: Result<Vec<Vertex>, &'static str> = tri
                                .iter()
                                .map(|&idx| {
                                    if idx < verts.len() {
                                        let v = &verts[idx];
                                        if v.len() >= 3 && v.iter().all(|&coord| coord.is_finite()) {
                                            Ok(Vertex::new(
                                                Point3::new(v[0], v[1], v[2]),
                                                Vector3::zeros()
                                            ))
                                        } else {
                                            Err("Invalid vertex coordinates")
                                        }
                                    } else {
                                        Err("Vertex index out of bounds")
                                    }
                                })
                                .collect();

                            match vertices_result {
                                Ok(vertices) if vertices.len() == 3 => {
                                    Some(Polygon::new(vertices, None))
                                },
                                _ => None,
                            }
                        } else {
                            None
                        }
                    })
                    .collect()
            } else {
                // Sequential processing for smaller datasets
                indices
                    .chunks(3)
                    .filter_map(|tri| {
                        if tri.len() == 3 {
                            let vertices_result: Result<Vec<Vertex>, &'static str> = tri
                                .iter()
                                .map(|&idx| {
                                    if idx < verts.len() {
                                        let v = &verts[idx];
                                        if v.len() >= 3 && v.iter().all(|&coord| coord.is_finite()) {
                                            Ok(Vertex::new(
                                                Point3::new(v[0], v[1], v[2]),
                                                Vector3::zeros()
                                            ))
                                        } else {
                                            Err("Invalid vertex coordinates")
                                        }
                                    } else {
                                        Err("Vertex index out of bounds")
                                    }
                                })
                                .collect();

                            match vertices_result {
                                Ok(vertices) if vertices.len() == 3 => {
                                    Some(Polygon::new(vertices, None))
                                },
                                _ => None,
                            }
                        } else {
                            None
                        }
                    })
                    .collect()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let polygons: Vec<Polygon<S>> = indices
            .chunks(3)
            .filter_map(|tri| {
                if tri.len() == 3 {
                    let vertices_result: Result<Vec<Vertex>, &'static str> = tri
                        .iter()
                        .map(|&idx| {
                            if idx < verts.len() {
                                let v = &verts[idx];
                                if v.len() >= 3 && v.iter().all(|&coord| coord.is_finite()) {
                                    Ok(Vertex::new(
                                        Point3::new(v[0], v[1], v[2]),
                                        Vector3::zeros()
                                    ))
                                } else {
                                    Err("Invalid vertex coordinates")
                                }
                            } else {
                                Err("Vertex index out of bounds")
                            }
                        })
                        .collect();

                    match vertices_result {
                        Ok(vertices) if vertices.len() == 3 => {
                            Some(Polygon::new(vertices, None))
                        },
                        _ => None,
                    }
                } else {
                    None
                }
            })
            .collect();

        CSG::from_polygons(&polygons)
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// **Mathematical Foundation**: For convex sets A and B, A ⊕ B = {a + b | a ∈ A, b ∈ B}.
    /// By the Minkowski sum theorem, the convex hull of all pairwise vertex sums equals
    /// the Minkowski sum of the convex hulls of A and B.
    ///
    /// **Algorithm**: O(|A| × |B|) vertex combinations followed by O(n log n) convex hull computation.
    ///
    /// **Note**: This method delegates to the new modular Minkowski implementation in
    /// `crate::math::minkowski::sum` while preserving the existing API for backward compatibility.
    pub fn minkowski_sum(&self, other: &CSG<S>) -> CSG<S> {
        // Delegate to the new modular Minkowski implementation
        // This preserves the existing API while using the Cathedral Engineering architecture
        crate::math::minkowski::sum::compute(self, other)
    }
}
