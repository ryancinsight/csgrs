//! Non-parallel BSP tree operations (clip, build, slice)

use crate::core::float_types::EPSILON;
use crate::geometry::{BACK, COPLANAR, FRONT, Plane, SPANNING, Polygon, Vertex};
use super::core::Node;
use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Build a BSP tree from the given polygons
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose the first polygon's plane as the splitting plane if not already set.
        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front: Vec<Polygon<S>> = Vec::new();
        let mut back: Vec<Polygon<S>> = Vec::new();

        // For each polygon, split it relative to the current node's plane.
        for p in polygons {
            let (coplanar_front, coplanar_back, f, b) = plane.split_polygon(p);

            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);

            front.extend(f);
            back.extend(b);
        }

        // Build child nodes using get_or_insert_with pattern
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
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();
        let mut front_polys: Vec<Polygon<S>> = Vec::new();
        let mut back_polys: Vec<Polygon<S>> = Vec::new();

        for poly in polygons {
            let (coplanar_front, coplanar_back, mut front, mut back) = plane.split_polygon(poly);

            // Handle coplanar polygons with iterator chains
            coplanar_front.into_iter()
                .chain(coplanar_back.into_iter())
                .for_each(|cp| {
                    if plane.orient_plane(&cp.plane) == FRONT {
                        front.push(cp);
                    } else {
                        back.push(cp);
                    }
                });

            front_polys.extend(front);
            back_polys.extend(back);
        }

        let mut final_front = if let Some(ref f) = self.front {
            f.clip_polygons(&front_polys)
        } else {
            front_polys
        };

        let back_clipped = if let Some(ref b) = self.back {
            b.clip_polygons(&back_polys)
        } else {
            Vec::new()
        };
        
        final_front.extend(back_clipped);
        final_front
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