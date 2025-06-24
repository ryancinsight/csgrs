//! Non-parallel BSP tree operations (clip, build, slice)

use crate::core::float_types::EPSILON;
use crate::geometry::{BACK, COPLANAR, FRONT, Plane, SPANNING, Polygon, Vertex};
use super::core::Node;
use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Build a BSP tree from the given polygons
    /// **Mathematical Foundation**: Recursively partition 3D space using hyperplanes,
    /// classifying polygons as FRONT, BACK, COPLANAR, or SPANNING relative to the splitting plane.
    /// **Algorithm**: O(n log n) expected time for balanced trees, O(n²) worst case.
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose the first polygon's plane as the splitting plane if not already set.
        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.as_ref().unwrap();

        // Pre-allocate with estimated capacity for better performance
        let mut front = Vec::with_capacity(polygons.len() / 2);
        let mut back = Vec::with_capacity(polygons.len() / 2);

        // Optimized polygon classification using iterator pattern
        // **Mathematical Theorem**: Each polygon is classified relative to the splitting plane
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) = 
                plane.split_polygon(polygon);

            // Extend collections efficiently with iterator chains
            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);
            front.append(&mut front_parts);
            back.append(&mut back_parts);
        }

        // Build child nodes using lazy initialization pattern for memory efficiency
        if !front.is_empty() {
            self.front
                .get_or_insert_with(|| Box::new(Node::new()))
                .build(&front);
        }

        if !back.is_empty() {
            self.back
                .get_or_insert_with(|| Box::new(Node::new()))
                .build(&back);
        }
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    /// **Mathematical Foundation**: Uses plane classification to determine polygon visibility.
    /// Polygons entirely in BACK half-space are clipped (removed).
    /// **Algorithm**: O(n log d) where n is polygon count, d is tree depth.
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();
        
        // Pre-allocate for better performance
        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());

        // Optimized polygon splitting with iterator patterns
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) = 
                plane.split_polygon(polygon);

            // Efficient coplanar polygon classification using iterator chain
            for coplanar_poly in coplanar_front.into_iter().chain(coplanar_back.into_iter()) {
                if plane.orient_plane(&coplanar_poly.plane) == FRONT {
                    front_parts.push(coplanar_poly);
                } else {
                    back_parts.push(coplanar_poly);
                }
            }

            front_polys.append(&mut front_parts);
            back_polys.append(&mut back_parts);
        }

        // Recursively clip with optimized pattern
        let mut result = if let Some(front_node) = &self.front {
            front_node.clip_polygons(&front_polys)
        } else {
            front_polys
        };

        if let Some(back_node) = &self.back {
            result.extend(back_node.clip_polygons(&back_polys));
        }
        
        result
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front {
            front.clip_to(bsp);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(bsp);
        }
    }

    /// Slices this BSP node with `slicing_plane`, returning:
    /// - All polygons that are coplanar with the plane (within EPSILON),
    /// - A list of line‐segment intersections (each a [Vertex; 2]) from polygons that span the plane.
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();
        self.slice_recursive(slicing_plane, &mut coplanar_polygons, &mut intersection_edges);
        (coplanar_polygons, intersection_edges)
    }

    /// Recursive helper for slice operation
    fn slice_recursive(
        &self,
        slicing_plane: &Plane,
        coplanar_polygons: &mut Vec<Polygon<S>>,
        intersection_edges: &mut Vec<[Vertex; 2]>,
    ) {
        for poly in &self.polygons {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                continue; // degenerate polygon => skip
            }
            
            // Use iterator chain to compute vertex types more efficiently
            let types: Vec<_> = poly.vertices
                .iter()
                .map(|vertex| slicing_plane.orient_point(&vertex.pos))
                .collect();
            
            let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

            // Based on the combined classification of its vertices:
            match polygon_type {
                COPLANAR => {
                    // The entire polygon is in the plane, so push it to the coplanar list.
                    coplanar_polygons.push(poly.clone());
                }

                FRONT | BACK => {
                    // Entirely on one side => no intersection. We skip it.
                }

                SPANNING => {
                    // Use iterator chain to collect intersection points more efficiently
                    let crossing_points: Vec<_> = (0..vcount)
                        .filter_map(|i| {
                            let j = (i + 1) % vcount;
                            let ti = types[i];
                            let tj = types[j];
                            let vi = &poly.vertices[i];
                            let vj = &poly.vertices[j];

                            if (ti | tj) == SPANNING {
                                let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                                if denom.abs() > EPSILON {
                                    let intersection = (slicing_plane.offset()
                                        - slicing_plane.normal().dot(&vi.pos.coords))
                                        / denom;
                                    Some(vi.interpolate(vj, intersection))
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        })
                        .collect();

                    // Convert crossing points to intersection edges
                    intersection_edges.extend(
                        crossing_points.chunks_exact(2)
                            .map(|chunk| [chunk[0].clone(), chunk[1].clone()])
                    );
                }
                _ => {}
            }
        }

        if let Some(front) = &self.front {
            front.slice_recursive(slicing_plane, coplanar_polygons, intersection_edges);
        }
        if let Some(back) = &self.back {
            back.slice_recursive(slicing_plane, coplanar_polygons, intersection_edges);
        }
    }
} 