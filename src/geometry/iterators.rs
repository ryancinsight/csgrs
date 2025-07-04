//! **Geometric Iterator Adaptors (The Mind)**
//!
//! This module provides domain-specific iterator adaptors for common geometric operations,
//! following Cathedral Engineering principles where iterator adaptors represent the
//! "mind" that processes geometric data with functional patterns.
//!
//! ## **Iterator Adaptor Categories**
//!
//! - **Vertex Transformations**: Transform vertex positions and normals
//! - **Normal Calculations**: Compute and update surface normals
//! - **Polygon Processing**: Process polygon collections with geometric operations
//! - **Spatial Filtering**: Filter geometric elements based on spatial criteria

use crate::core::float_types::Real;
use crate::geometry::{Polygon, Vertex};
use nalgebra::{Matrix4, Point3, Vector3};
use std::fmt::Debug;

/// **Iterator adaptor for vertex transformations**
///
/// This adaptor applies homogeneous transformations to vertices while handling
/// potential numerical instabilities and maintaining normal consistency.
pub struct VertexTransformIterator<I, S> {
    iter: I,
    transform_matrix: Matrix4<Real>,
    normal_matrix: Matrix4<Real>,
    _phantom: std::marker::PhantomData<S>,
}

impl<I, S> VertexTransformIterator<I, S>
where
    I: Iterator<Item = Vertex>,
    S: Clone + Debug + Send + Sync,
{
    /// Create a new vertex transform iterator
    pub fn new(iter: I, transform_matrix: Matrix4<Real>) -> Self {
        // Compute normal transformation matrix (inverse transpose)
        let normal_matrix = transform_matrix
            .try_inverse()
            .unwrap_or(Matrix4::identity())
            .transpose();

        Self {
            iter,
            transform_matrix,
            normal_matrix,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<I, S> Iterator for VertexTransformIterator<I, S>
where
    I: Iterator<Item = Vertex>,
    S: Clone + Debug + Send + Sync,
{
    type Item = Result<Vertex, &'static str>;

    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|vertex| {
            // Transform position using homogeneous coordinates
            let hom_pos = self.transform_matrix * vertex.pos.to_homogeneous();
            match Point3::from_homogeneous(hom_pos) {
                Some(transformed_pos) => {
                    // Transform normal using inverse transpose rule
                    let transformed_normal = self.normal_matrix
                        .transform_vector(&vertex.normal)
                        .normalize();

                    Ok(Vertex::new(transformed_pos, transformed_normal))
                },
                None => Err("Invalid homogeneous coordinates after transformation"),
            }
        })
    }
}

/// **Iterator adaptor for normal recalculation**
///
/// This adaptor recalculates vertex normals based on polygon face normals,
/// providing smooth or flat shading options.
pub struct NormalRecalcIterator<I, S: Clone> {
    polygons: Vec<Polygon<S>>,
    vertex_iter: I,
    smooth_normals: bool,
}

impl<I, S> NormalRecalcIterator<I, S>
where
    I: Iterator<Item = Vertex>,
    S: Clone + Debug + Send + Sync,
{
    /// Create a new normal recalculation iterator
    pub fn new(vertex_iter: I, polygons: Vec<Polygon<S>>, smooth_normals: bool) -> Self {
        Self {
            polygons,
            vertex_iter,
            smooth_normals,
        }
    }
}

impl<I, S> Iterator for NormalRecalcIterator<I, S>
where
    I: Iterator<Item = Vertex>,
    S: Clone + Debug + Send + Sync,
{
    type Item = Vertex;

    fn next(&mut self) -> Option<Self::Item> {
        self.vertex_iter.next().map(|vertex| {
            if self.smooth_normals {
                // Calculate smooth normal by averaging adjacent face normals
                let adjacent_normals: Vec<Vector3<Real>> = self.polygons
                    .iter()
                    .filter(|poly| {
                        poly.vertices.iter().any(|v| {
                            (v.pos - vertex.pos).norm() < crate::core::float_types::EPSILON
                        })
                    })
                    .map(|poly| poly.plane.normal())
                    .collect();

                if !adjacent_normals.is_empty() {
                    let avg_normal = adjacent_normals
                        .iter()
                        .fold(Vector3::zeros(), |acc, &normal| acc + normal)
                        / adjacent_normals.len() as Real;
                    
                    Vertex::new(vertex.pos, avg_normal.normalize())
                } else {
                    vertex
                }
            } else {
                // Keep original normal for flat shading
                vertex
            }
        })
    }
}

/// **Iterator adaptor for polygon edge processing**
///
/// This adaptor processes polygon edges with geometric operations like
/// edge length calculation, edge subdivision, or edge smoothing.
pub struct PolygonEdgeIterator<I, S> {
    polygon_iter: I,
    edge_processor: Box<dyn Fn(&Vertex, &Vertex) -> Vec<Vertex>>,
    current_edges: std::vec::IntoIter<Vertex>,
    _phantom: std::marker::PhantomData<S>,
}

impl<I, S> PolygonEdgeIterator<I, S>
where
    I: Iterator<Item = Polygon<S>>,
    S: Clone + Debug + Send + Sync,
{
    /// Create a new polygon edge iterator with custom edge processor
    pub fn new<F>(polygon_iter: I, edge_processor: F) -> Self
    where
        F: Fn(&Vertex, &Vertex) -> Vec<Vertex> + 'static,
    {
        Self {
            polygon_iter,
            edge_processor: Box::new(edge_processor),
            current_edges: Vec::new().into_iter(),
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<I, S> Iterator for PolygonEdgeIterator<I, S>
where
    I: Iterator<Item = Polygon<S>>,
    S: Clone + Debug + Send + Sync,
{
    type Item = Vertex;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            // Try to get next vertex from current edges
            if let Some(vertex) = self.current_edges.next() {
                return Some(vertex);
            }

            // Process next polygon if current edges are exhausted
            if let Some(polygon) = self.polygon_iter.next() {
                let mut new_edges = Vec::new();
                
                // Process each edge of the polygon
                for i in 0..polygon.vertices.len() {
                    let v1 = &polygon.vertices[i];
                    let v2 = &polygon.vertices[(i + 1) % polygon.vertices.len()];
                    let processed_vertices = (self.edge_processor)(v1, v2);
                    new_edges.extend(processed_vertices);
                }
                
                self.current_edges = new_edges.into_iter();
            } else {
                return None;
            }
        }
    }
}

/// **Extension trait for vertex iterator operations**
///
/// This trait provides convenient methods for applying geometric iterator adaptors
/// to vertex iterators, improving code expressiveness and reusability.
pub trait VertexIteratorExt: Iterator<Item = Vertex> + Sized {
    /// Apply vertex transformation with error handling
    fn transform_vertices<S>(self, matrix: Matrix4<Real>) -> VertexTransformIterator<Self, S>
    where
        S: Clone + Debug + Send + Sync,
    {
        VertexTransformIterator::new(self, matrix)
    }

    /// Recalculate normals with smooth or flat shading
    fn recalculate_normals<S>(
        self,
        polygons: Vec<Polygon<S>>,
        smooth: bool,
    ) -> NormalRecalcIterator<Self, S>
    where
        S: Clone + Debug + Send + Sync,
    {
        NormalRecalcIterator::new(self, polygons, smooth)
    }
}

/// **Extension trait for polygon iterator operations**
pub trait PolygonIteratorExt<S>: Iterator<Item = Polygon<S>> + Sized
where
    S: Clone + Debug + Send + Sync,
{
    /// Process polygon edges with custom operations
    fn process_edges<F>(self, edge_processor: F) -> PolygonEdgeIterator<Self, S>
    where
        F: Fn(&Vertex, &Vertex) -> Vec<Vertex> + 'static,
    {
        PolygonEdgeIterator::new(self, edge_processor)
    }
}

// Implement the extension traits for appropriate iterators
impl<I: Iterator<Item = Vertex>> VertexIteratorExt for I {}
impl<I, S> PolygonIteratorExt<S> for I
where
    I: Iterator<Item = Polygon<S>>,
    S: Clone + Debug + Send + Sync,
{}

/// **Batch Processing Functions for Large Datasets**
pub mod batch {
    use super::*;
    use nalgebra::Matrix4;

    /// **Batch vertex transformation with parallel processing**
    ///
    /// Processes large collections of vertices using par_chunks() for optimal
    /// memory usage and parallel efficiency.
    #[allow(unused_variables)] // chunk_size is used in parallel feature
    pub fn transform_vertices_batch<S>(
        vertices: Vec<Vertex>,
        transform_matrix: Matrix4<Real>,
        chunk_size: usize,
    ) -> Result<Vec<Vertex>, &'static str>
    where
        S: Clone + std::fmt::Debug + Send + Sync,
    {
        #[cfg(feature = "parallel")]
        {
            if vertices.len() > 1000 {
                use rayon::prelude::*;

                // Use par_chunks for memory-efficient parallel processing
                vertices
                    .par_chunks(chunk_size)
                    .map(|chunk| {
                        chunk
                            .iter()
                            .map(|vertex| {
                                let transform_iter = VertexTransformIterator::<_, S>::new(
                                    std::iter::once(vertex.clone()),
                                    transform_matrix,
                                );
                                transform_iter.collect::<Result<Vec<_>, _>>()
                            })
                            .collect::<Result<Vec<Vec<_>>, _>>()
                            .map(|nested| nested.into_iter().flatten().collect::<Vec<_>>())
                    })
                    .collect::<Result<Vec<_>, _>>()
                    .map(|chunks| chunks.into_iter().flatten().collect())
            } else {
                vertices
                    .into_iter()
                    .map(|vertex| {
                        let transform_iter = VertexTransformIterator::<_, S>::new(
                            std::iter::once(vertex),
                            transform_matrix,
                        );
                        transform_iter.collect::<Result<Vec<_>, _>>()
                    })
                    .collect::<Result<Vec<Vec<_>>, _>>()
                    .map(|nested| nested.into_iter().flatten().collect())
            }
        }

        #[cfg(not(feature = "parallel"))]
        {
            vertices
                .into_iter()
                .map(|vertex| {
                    let transform_iter = VertexTransformIterator::<_, S>::new(
                        std::iter::once(vertex),
                        transform_matrix,
                    );
                    transform_iter.collect::<Result<Vec<_>, _>>()
                })
                .collect::<Result<Vec<Vec<_>>, _>>()
                .map(|nested| nested.into_iter().flatten().collect())
        }
    }

    /// **Batch polygon subdivision with parallel processing**
    ///
    /// Subdivides large collections of polygons using parallel chunks
    /// for optimal performance on large meshes.
    #[allow(unused_variables)] // chunk_size is used in parallel feature
    pub fn subdivide_polygons_batch<S>(
        polygons: Vec<Polygon<S>>,
        subdivision_level: usize,
        chunk_size: usize,
    ) -> Vec<Polygon<S>>
    where
        S: Clone + std::fmt::Debug + Send + Sync,
    {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 100 {
                use rayon::prelude::*;

                polygons
                    .par_chunks(chunk_size)
                    .flat_map(|chunk| {
                        chunk
                            .par_iter()
                            .flat_map(|polygon| {
                                // Recursive subdivision using iterator patterns
                                (0..subdivision_level)
                                    .fold(vec![polygon.clone()], |current_polys, _| {
                                        current_polys
                                            .into_par_iter()
                                            .flat_map(|poly| subdivide_polygon(&poly))
                                            .collect()
                                    })
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect()
            } else {
                polygons
                    .into_iter()
                    .flat_map(|polygon| {
                        (0..subdivision_level)
                            .fold(vec![polygon], |current_polys, _| {
                                current_polys
                                    .into_iter()
                                    .flat_map(|poly| subdivide_polygon(&poly))
                                    .collect()
                            })
                    })
                    .collect()
            }
        }

        #[cfg(not(feature = "parallel"))]
        {
            polygons
                .into_iter()
                .flat_map(|polygon| {
                    (0..subdivision_level)
                        .fold(vec![polygon], |current_polys, _| {
                            current_polys
                                .into_iter()
                                .flat_map(|poly| subdivide_polygon(&poly))
                                .collect()
                        })
                })
                .collect()
        }
    }

    /// **Helper function for polygon subdivision**
    fn subdivide_polygon<S>(polygon: &Polygon<S>) -> Vec<Polygon<S>>
    where
        S: Clone + std::fmt::Debug + Send + Sync,
    {
        if polygon.vertices.len() < 3 {
            return vec![polygon.clone()];
        }

        // Simple midpoint subdivision using iterator patterns
        let midpoints: Vec<Vertex> = polygon.vertices
            .iter()
            .enumerate()
            .map(|(i, vertex)| {
                let next_vertex = &polygon.vertices[(i + 1) % polygon.vertices.len()];
                let mid_pos = Point3::from((vertex.pos.coords + next_vertex.pos.coords) / 2.0);
                let mid_normal = (vertex.normal + next_vertex.normal).normalize();
                Vertex::new(mid_pos, mid_normal)
            })
            .collect();

        // Create subdivided triangles (simplified approach)
        if polygon.vertices.len() == 3 && midpoints.len() == 3 {
            // Triangle subdivision into 4 triangles using clones
            vec![
                Polygon::new(vec![polygon.vertices[0].clone(), midpoints[0].clone(), midpoints[2].clone()], polygon.metadata.clone()),
                Polygon::new(vec![midpoints[0].clone(), polygon.vertices[1].clone(), midpoints[1].clone()], polygon.metadata.clone()),
                Polygon::new(vec![midpoints[2].clone(), midpoints[1].clone(), polygon.vertices[2].clone()], polygon.metadata.clone()),
                Polygon::new(vec![midpoints[0].clone(), midpoints[1].clone(), midpoints[2].clone()], polygon.metadata.clone()),
            ]
        } else {
            // For non-triangles, return original (could be enhanced)
            vec![polygon.clone()]
        }
    }
}

/// **Utility functions for common geometric operations**
pub mod operations {
    use super::*;

    /// Edge subdivision processor that adds midpoint vertices
    pub fn subdivide_edge(v1: &Vertex, v2: &Vertex) -> Vec<Vertex> {
        let midpoint_pos = Point3::from((v1.pos.coords + v2.pos.coords) / 2.0);
        let midpoint_normal = (v1.normal + v2.normal).normalize();
        let midpoint = Vertex::new(midpoint_pos, midpoint_normal);

        vec![v1.clone(), midpoint, v2.clone()]
    }

    /// Edge smoothing processor that applies Laplacian smoothing
    pub fn smooth_edge(v1: &Vertex, v2: &Vertex) -> Vec<Vertex> {
        let smoothing_factor = 0.1;
        let offset = (v2.pos - v1.pos) * smoothing_factor;

        let smoothed_v1 = Vertex::new(v1.pos + offset, v1.normal);
        let smoothed_v2 = Vertex::new(v2.pos - offset, v2.normal);

        vec![smoothed_v1, smoothed_v2]
    }
}
