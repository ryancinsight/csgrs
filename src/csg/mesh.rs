use super::CSG;
use crate::polygon::Polygon;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Subdivide all polygons in this CSG 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::CSG;
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    /// # Examples
    ///
    /// ```ignore
    /// use csgrs::CSG;
    /// use core::num::NonZeroU32;
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// cube.subdivide_triangles_mut(NonZeroU32::new(1).unwrap());
    /// assert!(cube.polygons.len() > 6);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: core::num::NonZeroU32) {
        #[cfg(feature = "parallel")]
        {
            self.polygons = self
                .polygons
                .par_iter_mut()
                .flat_map(|poly| {
                    let sub_tris = poly.subdivide_triangles(levels.into());
                    // Convert each small tri back to a Polygon
                    sub_tris
                        .into_par_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.polygons = self
                .polygons
                .iter_mut()
                .flat_map(|poly| {
                    let polytri = poly.subdivide_triangles(levels.into());
                    polytri
                        .into_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }
    }

    /// Subdivide all polygons in this CSG 'levels' times, returning a new CSG.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: core::num::NonZeroU32) -> CSG<S> {
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris.into_par_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        CSG::from_polygons(&new_polygons)
    }

    /// Renormalize all polygons in this CSG by re-computing each polygon's plane
    /// and assigning that plane's normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }

    /// Triangulate each polygon in the CSG returning a CSG containing triangles
    pub fn tessellate(&self) -> CSG<S> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.tessellate()
                    .into_iter()
                    .map(move |triangle| Polygon::new(triangle.to_vec(), poly.metadata.clone()))
            })
            .collect::<Vec<_>>();

        CSG::from_polygons(&triangles)
    }
} 