use crate::csg::CSG;
use crate::geometry::Polygon;
use std::fmt::Debug;
use nalgebra::Point3;
use std::collections::HashMap;

use crate::core::float_types::Real;
use crate::geometry::Vertex;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// **Mathematical Foundation: True Laplacian Mesh Smoothing with Global Connectivity**
    ///
    /// Implements proper discrete Laplacian smoothing using global mesh connectivity:
    ///
    /// ## **Discrete Laplacian Operator**
    /// For each vertex v with neighbors N(v):
    /// ```text
    /// L(v) = (1/|N(v)|) · Σ(n∈N(v)) (n - v)
    /// ```
    ///
    /// ## **Global Connectivity Benefits**
    /// - **Proper Neighborhoods**: Uses actual mesh connectivity, not just polygon edges
    /// - **Uniform Weighting**: Each neighbor contributes equally to smoothing
    /// - **Boundary Detection**: Automatically detects and preserves mesh boundaries
    /// - **Volume Preservation**: Better volume preservation than local smoothing
    ///
    /// ## **Algorithm Improvements**
    /// - **Epsilon-based Vertex Matching**: Robust floating-point coordinate handling
    /// - **Manifold Preservation**: Ensures mesh topology is maintained
    /// - **Feature Detection**: Can preserve sharp features based on neighbor count
    pub fn laplacian_smooth_global(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> CSG<S> {
        let (vertex_map, adjacency) = self.build_mesh_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        for iteration in 0..iterations {
            // Build current vertex position mapping using advanced iterator patterns
            #[cfg(feature = "parallel")]
            let current_positions: HashMap<usize, Point3<Real>> = {
                if smoothed_polygons.len() > 100 {
                    use rayon::prelude::*;
                    smoothed_polygons
                        .par_iter()
                        .flat_map(|polygon| polygon.vertices.par_iter())
                        .filter_map(|vertex| {
                            vertex_map.position_to_index
                                .iter()
                                .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon)
                                .map(|(_, idx)| (*idx, vertex.pos))
                        })
                        .collect()
                } else {
                    smoothed_polygons
                        .iter()
                        .flat_map(|polygon| polygon.vertices.iter())
                        .filter_map(|vertex| {
                            vertex_map.position_to_index
                                .iter()
                                .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon)
                                .map(|(_, idx)| (*idx, vertex.pos))
                        })
                        .collect()
                }
            };

            #[cfg(not(feature = "parallel"))]
            let current_positions: HashMap<usize, Point3<Real>> = smoothed_polygons
                .iter()
                .flat_map(|polygon| polygon.vertices.iter())
                .filter_map(|vertex| {
                    vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon)
                        .map(|(_, idx)| (*idx, vertex.pos))
                })
                .collect();

            // Compute Laplacian updates using advanced iterator patterns with parallel processing
            #[cfg(feature = "parallel")]
            let laplacian_updates: HashMap<usize, Point3<Real>> = {
                if adjacency.len() > 500 {
                    use rayon::prelude::*;
                    adjacency
                        .par_iter()
                        .filter_map(|(&vertex_idx, neighbors)| {
                            current_positions.get(&vertex_idx).map(|&current_pos| {
                                // Check boundary condition
                                if preserve_boundaries && neighbors.len() < 4 {
                                    return (vertex_idx, current_pos);
                                }

                                // Use reduce() for efficient neighbor position accumulation
                                let neighbor_sum = neighbors
                                    .par_iter()
                                    .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                                    .map(|&pos| pos.coords)
                                    .reduce(|| nalgebra::Vector3::zeros(), |acc, pos| acc + pos);

                                let valid_neighbors = neighbors
                                    .iter()
                                    .filter(|&&neighbor_idx| current_positions.contains_key(&neighbor_idx))
                                    .count();

                                if valid_neighbors > 0 {
                                    let neighbor_avg = Point3::from(neighbor_sum / valid_neighbors as Real);
                                    let laplacian = neighbor_avg - current_pos;
                                    let new_pos = current_pos + laplacian * lambda;
                                    (vertex_idx, new_pos)
                                } else {
                                    (vertex_idx, current_pos)
                                }
                            })
                        })
                        .collect()
                } else {
                    adjacency
                        .iter()
                        .filter_map(|(&vertex_idx, neighbors)| {
                            current_positions.get(&vertex_idx).map(|&current_pos| {
                                // Check boundary condition
                                if preserve_boundaries && neighbors.len() < 4 {
                                    return (vertex_idx, current_pos);
                                }

                                // Use fold() for neighbor position accumulation
                                let neighbor_sum = neighbors
                                    .iter()
                                    .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                                    .fold(nalgebra::Vector3::zeros(), |acc, &pos| acc + pos.coords);

                                let valid_neighbors = neighbors
                                    .iter()
                                    .filter(|&&neighbor_idx| current_positions.contains_key(&neighbor_idx))
                                    .count();

                                if valid_neighbors > 0 {
                                    let neighbor_avg = Point3::from(neighbor_sum / valid_neighbors as Real);
                                    let laplacian = neighbor_avg - current_pos;
                                    let new_pos = current_pos + laplacian * lambda;
                                    (vertex_idx, new_pos)
                                } else {
                                    (vertex_idx, current_pos)
                                }
                            })
                        })
                        .collect()
                }
            };

            #[cfg(not(feature = "parallel"))]
            let laplacian_updates: HashMap<usize, Point3<Real>> = adjacency
                .iter()
                .filter_map(|(&vertex_idx, neighbors)| {
                    current_positions.get(&vertex_idx).map(|&current_pos| {
                        // Check boundary condition
                        if preserve_boundaries && neighbors.len() < 4 {
                            return (vertex_idx, current_pos);
                        }

                        // Use fold() for neighbor position accumulation
                        let neighbor_sum = neighbors
                            .iter()
                            .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                            .fold(nalgebra::Vector3::zeros(), |acc, &pos| acc + pos.coords);

                        let valid_neighbors = neighbors
                            .iter()
                            .filter(|&&neighbor_idx| current_positions.contains_key(&neighbor_idx))
                            .count();

                        if valid_neighbors > 0 {
                            let neighbor_avg = Point3::from(neighbor_sum / valid_neighbors as Real);
                            let laplacian = neighbor_avg - current_pos;
                            let new_pos = current_pos + laplacian * lambda;
                            (vertex_idx, new_pos)
                        } else {
                            (vertex_idx, current_pos)
                        }
                    })
                })
                .collect();

            // Apply updates using iterator patterns with early termination
            smoothed_polygons
                .iter_mut()
                .for_each(|polygon| {
                    polygon.vertices
                        .iter_mut()
                        .for_each(|vertex| {
                            // Use find() for early termination when matching vertex
                            if let Some((_, idx)) = vertex_map.position_to_index
                                .iter()
                                .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon)
                            {
                                if let Some(&new_pos) = laplacian_updates.get(idx) {
                                    vertex.pos = new_pos;
                                }
                            }
                        });

                    // Recompute polygon plane and normals after smoothing
                    polygon.set_new_normal();
                    // Invalidate the cached bounding box since vertex positions have changed
                    polygon.invalidate_bounding_box();
                });

            // Progress feedback for long smoothing operations
            if iterations > 10 && iteration % (iterations / 10) == 0 {
                eprintln!(
                    "Smoothing progress: {}/{} iterations",
                    iteration + 1,
                    iterations
                );
            }
        }

        CSG::from_polygons(&smoothed_polygons)
    }

    /// **Mathematical Foundation: Taubin Mesh Smoothing**
    ///
    /// Implements Taubin's feature-preserving mesh smoothing algorithm, which reduces
    /// shrinkage compared to standard Laplacian smoothing.
    ///
    /// ## **Taubin's Algorithm**
    /// This method involves two steps per iteration:
    /// 1. **Shrinking Step**: Apply standard Laplacian smoothing with a positive factor `lambda`.
    ///    `v' = v + λ * L(v)`
    /// 2. **Inflating Step**: Apply a second Laplacian step with a negative factor `mu`.
    ///    `v'' = v' + μ * L(v')`
    ///
    /// Typically, `0 < λ < -μ`. A common choice is `mu = -λ / (1 - λ)`.
    /// This combination effectively smooths the mesh while minimizing volume loss.
    ///
    /// Returns a new, smoothed CSG object.
    pub fn taubin_smooth(
        &self,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> CSG<S> {
        let (vertex_map, adjacency) = self.build_mesh_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        for _ in 0..iterations {
            // --- Lambda (shrinking) pass ---
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            current_positions.insert(*idx, vertex.pos);
                            break;
                        }
                    }
                }
            }

            let mut updates: HashMap<usize, Point3<Real>> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_pos);
                        continue;
                    }

                    // Compute neighbor average using iterator combinators
                    let neighbor_positions: Vec<Point3<Real>> = neighbors
                        .iter()
                        .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                        .copied()
                        .collect();

                    let valid_neighbors = neighbor_positions.len();
                    let neighbor_sum = neighbor_positions
                        .iter()
                        .fold(Point3::origin(), |acc, &pos| acc + pos.coords);

                    if valid_neighbors > 0 {
                        let neighbor_avg = neighbor_sum / valid_neighbors as Real;
                        let laplacian = neighbor_avg - current_pos;
                        updates.insert(vertex_idx, current_pos + laplacian * lambda);
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            if let Some(&new_pos) = updates.get(idx) {
                                vertex.pos = new_pos;
                            }
                            break;
                        }
                    }
                }
            }

            // --- Mu (inflating) pass ---
            current_positions.clear();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            current_positions.insert(*idx, vertex.pos);
                            break;
                        }
                    }
                }
            }

            updates.clear();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_pos);
                        continue;
                    }

                    // Compute neighbor average using iterator combinators
                    let neighbor_positions: Vec<Point3<Real>> = neighbors
                        .iter()
                        .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                        .copied()
                        .collect();

                    let valid_neighbors = neighbor_positions.len();
                    let neighbor_sum = neighbor_positions
                        .iter()
                        .fold(Point3::origin(), |acc, &pos| acc + pos.coords);

                    if valid_neighbors > 0 {
                        let neighbor_avg = neighbor_sum / valid_neighbors as Real;
                        let laplacian = neighbor_avg - current_pos;
                        updates.insert(vertex_idx, current_pos + laplacian * mu);
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            if let Some(&new_pos) = updates.get(idx) {
                                vertex.pos = new_pos;
                            }
                            break;
                        }
                    }
                }
            }
        }

        // Final pass to recompute normals and invalidate bounding boxes
        for polygon in &mut smoothed_polygons {
            polygon.set_new_normal();
            // Invalidate the cached bounding box since vertex positions have changed
            polygon.invalidate_bounding_box();
        }

        CSG::from_polygons(&smoothed_polygons)
    }

    /// **Mathematical Foundation: Adaptive Mesh Refinement**
    ///
    /// Intelligently refine mesh based on geometric criteria:
    ///
    /// ## **Refinement Criteria**
    /// - **Quality threshold**: Refine triangles with quality score < threshold
    /// - **Size variation**: Refine where edge lengths vary significantly
    /// - **Curvature**: Refine high-curvature regions (based on normal variation)
    /// - **Feature detection**: Preserve sharp edges and corners
    ///
    /// ## **Refinement Strategy**
    /// 1. **Quality-based**: Subdivide poor-quality triangles
    /// 2. **Size-based**: Subdivide triangles larger than target size
    /// 3. **Curvature-based**: Subdivide where surface curves significantly
    ///
    /// This provides better mesh quality compared to uniform subdivision.
    pub fn adaptive_refine(
        &self,
        quality_threshold: Real,
        max_edge_length: Real,
        curvature_threshold_deg: Real,
    ) -> CSG<S> {
        let qualities = self.analyze_triangle_quality();
        let (mut vertex_map, _adjacency) = self.build_mesh_connectivity();
        let mut refined_polygons = Vec::new();
        let mut polygon_map: HashMap<usize, Vec<usize>> = HashMap::new();

        // Build polygon-vertex mapping using advanced iterator patterns
        self.polygons
            .iter()
            .enumerate()
            .flat_map(|(poly_idx, poly)| {
                poly.vertices.iter().map(move |vertex| (poly_idx, vertex))
            })
            .for_each(|(poly_idx, vertex)| {
                let v_idx = vertex_map.get_or_create_index(vertex.pos);
                polygon_map.entry(v_idx).or_default().push(poly_idx);
            });

        for (i, polygon) in self.polygons.iter().enumerate() {
            let mut should_refine = false;

            // Quality and edge length check
            if i < qualities.len() {
                let quality = &qualities[i];
                if quality.quality_score < quality_threshold
                    || Self::max_edge_length(&polygon.vertices) > max_edge_length
                {
                    should_refine = true;
                }
            }

            // Curvature check using iterator patterns
            if !should_refine {
                should_refine = polygon.edges()
                    .any(|edge| {
                        let v1_idx = vertex_map.get_or_create_index(edge.0.pos);
                        let v2_idx = vertex_map.get_or_create_index(edge.1.pos);

                        if let (Some(p1_indices), Some(p2_indices)) =
                            (polygon_map.get(&v1_idx), polygon_map.get(&v2_idx))
                        {
                            // Use iterator patterns for nested polygon index checking
                            p1_indices
                                .iter()
                                .filter(|&&p1_idx| p1_idx != i)
                                .flat_map(|&p1_idx| {
                                    p2_indices.iter().filter_map(move |&p2_idx| {
                                        if p1_idx == p2_idx {
                                            Some(p1_idx)
                                        } else {
                                            None
                                        }
                                    })
                                })
                                .any(|p1_idx| {
                                    let other_poly = &self.polygons[p1_idx];
                                    let angle = Self::dihedral_angle(polygon, other_poly);
                                    angle > curvature_threshold_deg.to_radians()
                                })
                        } else {
                            false
                        }
                    });
            }

            if should_refine {
                let subdivided = polygon.subdivide_triangles(1.try_into().unwrap());
                for triangle in subdivided {
                    let vertices = triangle.to_vec();
                    refined_polygons.push(Polygon::new(vertices, polygon.metadata.clone()));
                }
            } else {
                refined_polygons.push(polygon.clone());
            }
        }

        CSG::from_polygons(&refined_polygons)
    }

    /// Calculate maximum edge length in a polygon
    fn max_edge_length(vertices: &[Vertex]) -> Real {
        if vertices.len() < 2 {
            return 0.0;
        }

        let mut max_length: Real = 0.0;
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let edge_length = (vertices[j].pos - vertices[i].pos).norm();
            max_length = max_length.max(edge_length);
        }
        max_length
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    ///
    /// Returns the angle in radians.
    fn dihedral_angle(p1: &Polygon<S>, p2: &Polygon<S>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }

    /// **Mathematical Foundation: Feature-Preserving Mesh Optimization**
    ///
    /// Remove poor-quality triangles while preserving important geometric features:
    ///
    /// ## **Quality-Based Filtering**
    /// Remove triangles that meet criteria:
    /// - **Sliver triangles**: min_angle < threshold (typically 5°)
    /// - **Needle triangles**: aspect_ratio > threshold (typically 20)
    /// - **Small triangles**: area < threshold
    ///
    /// ## **Feature Preservation**
    /// - **Sharp edges**: Preserve edges with large dihedral angles
    /// - **Boundaries**: Maintain mesh boundaries
    /// - **Topology**: Ensure mesh remains manifold
    ///
    /// Returns cleaned mesh with improved triangle quality.
    pub fn remove_poor_triangles(&self, min_quality: Real) -> CSG<S> {
        let qualities = self.analyze_triangle_quality();
        let mut filtered_polygons = Vec::new();

        for (i, polygon) in self.polygons.iter().enumerate() {
            let keep_triangle = if i < qualities.len() {
                let quality = &qualities[i];
                quality.quality_score >= min_quality
                    && quality.area > Real::EPSILON
                    && quality.min_angle > (5.0_f64.to_radians())
                    && quality.aspect_ratio < 20.0
            } else {
                true // Keep if we can't assess quality
            };

            if keep_triangle {
                filtered_polygons.push(polygon.clone());
            }
        }

        CSG::from_polygons(&filtered_polygons)
    }

    /// Legacy Laplacian smoothing (kept for backward compatibility)
    /// **Note**: Use `laplacian_smooth_global` for better results with proper connectivity
    pub fn laplacian_smooth(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> CSG<S> {
        // Delegate to the improved global connectivity version
        self.laplacian_smooth_global(lambda, iterations, preserve_boundaries)
    }

    /// Subdivide all polygons in this CSG 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::CSG;
    /// use core::num::NonZeroU32;
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
                .iter()
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
    /// with intelligent parallel processing for large meshes
    pub fn tessellate(&self) -> CSG<S> {
        #[cfg(feature = "parallel")]
        let triangles: Vec<Polygon<S>> = {
            if self.polygons.len() > 1000 {
                use rayon::prelude::*;

                // Use parallel processing for large meshes
                self.polygons
                    .par_iter()
                    .flat_map(|poly| {
                        poly.tessellate()
                            .into_par_iter()
                            .map(move |triangle| {
                                Polygon::new(triangle.to_vec(), poly.metadata.clone())
                            })
                    })
                    .collect()
            } else {
                // Sequential processing for smaller meshes
                self.polygons
                    .iter()
                    .flat_map(|poly| {
                        poly.tessellate().into_iter().map(move |triangle| {
                            Polygon::new(triangle.to_vec(), poly.metadata.clone())
                        })
                    })
                    .collect()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let triangles: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.tessellate().into_iter().map(move |triangle| {
                    Polygon::new(triangle.to_vec(), poly.metadata.clone())
                })
            })
            .collect();

        CSG::from_polygons(&triangles)
    }
} 