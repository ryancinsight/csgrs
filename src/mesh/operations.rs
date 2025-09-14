//! Core mesh operations and utilities

use crate::float_types::Real;
use crate::mesh::{polygon::Polygon, vertex::Vertex};
use crate::traits::CSG;
use std::fmt::Debug;
use std::num::NonZeroU32;

#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator, IntoParallelIterator};

use super::Mesh;

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Build a Mesh from an existing polygon list
    /// **Performance Optimization**: Reserves capacity to avoid reallocations
    pub fn from_polygons(polygons: &[Polygon<S>], metadata: Option<S>) -> Self {
        let mut mesh = Mesh::new();
        mesh.polygons.reserve(polygons.len());
        mesh.polygons.extend_from_slice(polygons);
        mesh.metadata = metadata;
        mesh
    }

    /// Helper to collect all vertices from the CSG.
    #[cfg(not(feature = "parallel"))]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.iter().cloned())
            .collect()
    }

    /// Parallel helper to collect all vertices from the CSG.
    #[cfg(feature = "parallel")]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .par_iter()
            .flat_map(|p| p.vertices.clone().into_par_iter())
            .collect()
    }

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    ///
    /// ## Performance Characteristics
    /// - **Time Complexity**: O(n) where n is the total number of polygon vertices
    /// - **Space Complexity**: O(m) where m is the number of triangles created
    /// - **Memory Optimization**: Pre-allocates vector with estimated capacity to minimize reallocations
    /// - **Parallel Support**: Uses sequential processing (triangulation is typically I/O bound)
    pub fn triangulate(&self) -> Mesh<S> {
        // Pre-allocate vector with estimated capacity to avoid reallocations
        // Most polygons will triangulate to 2 triangles, some may need more
        let estimated_capacity = self.polygons.len() * 2;

        let mut triangles: Vec<Polygon<S>> = Vec::with_capacity(estimated_capacity);
        triangles.extend(
            self.polygons
                .iter()
                .flat_map(|poly| {
                    poly.triangulate().into_iter().map(move |triangle| {
                        Polygon::new(triangle.to_vec(), poly.metadata.clone())
                    })
                })
        );

        // Reclaim unused capacity if we over-allocated significantly
        triangles.shrink_to_fit();

        Mesh::from_polygons(&triangles, self.metadata.clone())
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Performance Characteristics
    /// - **Time Complexity**: O(4^levels × n) where n is the total number of polygon vertices
    /// - **Space Complexity**: O(4^levels × m) where m is the number of input polygons
    /// - **Memory Optimization**: Pre-allocates vector with exact capacity calculation
    /// - **Parallel Support**: Uses parallel iterators when available for large meshes
    /// - **Growth Factor**: Each subdivision level multiplies triangle count by 4
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<S> {
        // Estimate capacity: each polygon becomes 4^levels triangles
        let levels_usize = levels.get() as usize;
        let subdivision_factor = 4_usize.pow(levels_usize as u32);
        let estimated_capacity = self.polygons.len() * subdivision_factor;

        let mut new_polygons: Vec<Polygon<S>> = Vec::with_capacity(estimated_capacity);

        {
            #[cfg(feature = "parallel")]
            {
                let polygons: Vec<_> = self.polygons
                    .par_iter()
                    .flat_map(|poly| {
                        let sub_tris = poly.subdivide_triangles(levels);
                        // Convert each small tri back to a Polygon
                        sub_tris.into_par_iter().map(move |tri| {
                            Polygon::new(vec![tri[0], tri[1], tri[2]], poly.metadata.clone())
                        })
                    })
                    .collect();
                new_polygons.extend(polygons);
            }

            #[cfg(not(feature = "parallel"))]
            {
                new_polygons.extend(
                    self.polygons
                        .iter()
                        .flat_map(|poly| {
                            let sub_tris = poly.subdivide_triangles(levels);
                            sub_tris.into_iter().map(move |tri| {
                                Polygon::new(vec![tri[0], tri[1], tri[2]], poly.metadata.clone())
                            })
                        })
                );
            }
        }

        // Optimize memory usage
        new_polygons.shrink_to_fit();

        Mesh::from_polygons(&new_polygons, self.metadata.clone())
    }

    /// Subdivide all polygons in this Mesh 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use core::num::NonZeroU32;
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        #[cfg(feature = "parallel")]
        {
            // Pre-calculate total capacity to avoid reallocations
            let total_subdivisions = self
                .polygons
                .iter()
                .map(|poly| poly.subdivide_triangles(levels).len())
                .sum();
            let mut new_polygons = Vec::with_capacity(total_subdivisions);

            let polygons: Vec<_> = self.polygons.par_iter_mut().flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris
                    .into_par_iter()
                    .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
            }).collect();
            new_polygons.extend(polygons);

            self.polygons = new_polygons;
        }

        #[cfg(not(feature = "parallel"))]
        {
            // Pre-calculate total capacity to avoid reallocations
            let total_subdivisions = self
                .polygons
                .iter()
                .map(|poly| poly.subdivide_triangles(levels).len())
                .sum();
            let mut new_polygons = Vec::with_capacity(total_subdivisions);

            new_polygons.extend(self.polygons.iter().flat_map(|poly| {
                let polytri = poly.subdivide_triangles(levels);
                polytri
                    .into_iter()
                    .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
            }));

            self.polygons = new_polygons;
        }
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    ///
    /// Returns the angle in radians.
    pub(crate) fn dihedral_angle(p1: &Polygon<S>, p2: &Polygon<S>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }
}
