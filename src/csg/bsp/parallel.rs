//! Parallel BSP tree operations (requires "parallel" feature)

#[cfg(feature = "parallel")]
use rayon::{join, prelude::*};

use super::core::Node;
use crate::core::float_types::EPSILON;
use crate::geometry::{BACK, COPLANAR, FRONT, Plane, Polygon, SPANNING, Vertex};
use std::fmt::Debug;

#[cfg(feature = "parallel")]
impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Parallel version of `build`.
    pub fn build_parallel(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        if self.plane.is_none() {
            self.plane = Some(self.pick_best_splitting_plane(polygons));
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front_polys = Vec::with_capacity(polygons.len() / 2);
        let mut back_polys = Vec::with_capacity(polygons.len() / 2);

        // Process polygons sequentially for deterministic behavior
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_polygon(polygon);

            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);
            front_polys.append(&mut front_parts);
            back_polys.append(&mut back_parts);
        }

        // Parallelize the recursive building of child nodes
        rayon::join(
            || {
                if !front_polys.is_empty() {
                    let mut front_node = Box::new(Node::new());
                    front_node.build_parallel(&front_polys);
                    self.front = Some(front_node);
                }
            },
            || {
                if !back_polys.is_empty() {
                    let mut back_node = Box::new(Node::new());
                    back_node.build_parallel(&back_polys);
                    self.back = Some(back_node);
                }
            },
        );
    }

    /// Parallel version of `clip_polygons`.
    pub fn clip_polygons_parallel(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());

        // Process polygons sequentially for deterministic behavior
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_polygon(polygon);

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

        // Parallelize the recursive clipping
        let (mut front_clipped, back_clipped) = rayon::join(
            || {
                if let Some(ref front_node) = self.front {
                    front_node.clip_polygons_parallel(&front_polys)
                } else {
                    front_polys
                }
            },
            || {
                if let Some(ref back_node) = self.back {
                    back_node.clip_polygons_parallel(&back_polys)
                } else {
                    Vec::new()
                }
            },
        );

        front_clipped.extend(back_clipped);
        front_clipped
    }

    /// Parallel version of `clip_to`.
    pub fn clip_to_parallel(&mut self, bsp: &Node<S>) {
        // The clipping of polygons can be done in parallel for different nodes.
        let (polygons, front_opt, back_opt) =
            (std::mem::take(&mut self.polygons), self.front.take(), self.back.take());

        let (clipped_polygons, (clipped_front, clipped_back)) = rayon::join(
            || bsp.clip_polygons_parallel(&polygons),
            || {
                rayon::join(
                    || {
                        if let Some(mut front) = front_opt {
                            front.clip_to_parallel(bsp);
                            Some(front)
                        } else {
                            None
                        }
                    },
                    || {
                        if let Some(mut back) = back_opt {
                            back.clip_to_parallel(bsp);
                            Some(back)
                        } else {
                            None
                        }
                    },
                )
            },
        );

        self.polygons = clipped_polygons;
        self.front = clipped_front;
        self.back = clipped_back;
    }

    /// Parallel version of `slice`.
    pub fn slice_parallel(
        &self,
        slicing_plane: &Plane,
    ) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();
        self.slice_recursive_parallel(
            slicing_plane,
            &mut coplanar_polygons,
            &mut intersection_edges,
        );
        (coplanar_polygons, intersection_edges)
    }

    /// Recursive helper for parallel slice operation
    fn slice_recursive_parallel(
        &self,
        slicing_plane: &Plane,
        coplanar_polygons: &mut Vec<Polygon<S>>,
        intersection_edges: &mut Vec<[Vertex; 2]>,
    ) {
        let (mut local_coplanar, mut local_edges) = self
            .polygons
            .par_iter()
            .map(|poly| {
                let vcount = poly.vertices.len();
                if vcount < 2 {
                    // Degenerate => skip
                    return (Vec::new(), Vec::new());
                }
                let mut polygon_type = 0;
                let mut types = Vec::with_capacity(vcount);

                for vertex in &poly.vertices {
                    let vertex_type = slicing_plane.orient_point(&vertex.pos);
                    polygon_type |= vertex_type;
                    types.push(vertex_type);
                }

                match polygon_type {
                    COPLANAR => {
                        // Entire polygon in plane
                        (vec![poly.clone()], Vec::new())
                    },
                    FRONT | BACK => {
                        // Entirely on one side => no intersection
                        (Vec::new(), Vec::new())
                    },
                    SPANNING => {
                        // The polygon crosses the plane => gather intersection edges
                        let mut crossing_points = Vec::new();
                        for i in 0..vcount {
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
                                    let intersect_vert = vi.interpolate(vj, intersection);
                                    crossing_points.push(intersect_vert);
                                }
                            }
                        }

                        // Pair up intersection points => edges
                        let mut edges = Vec::new();
                        for chunk in crossing_points.chunks_exact(2) {
                            edges.push([chunk[0].clone(), chunk[1].clone()]);
                        }
                        (Vec::new(), edges)
                    },
                    _ => (Vec::new(), Vec::new()),
                }
            })
            .reduce(
                || (Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc
                },
            );

        coplanar_polygons.append(&mut local_coplanar);
        intersection_edges.append(&mut local_edges);

        if let (Some(front), Some(back)) = (&self.front, &self.back) {
            let (mut front_coplanar, mut front_edges) = (Vec::new(), Vec::new());
            let (mut back_coplanar, mut back_edges) = (Vec::new(), Vec::new());
            join(
                || {
                    front.slice_recursive_parallel(
                        slicing_plane,
                        &mut front_coplanar,
                        &mut front_edges,
                    )
                },
                || {
                    back.slice_recursive_parallel(
                        slicing_plane,
                        &mut back_coplanar,
                        &mut back_edges,
                    )
                },
            );
            coplanar_polygons.append(&mut front_coplanar);
            intersection_edges.append(&mut front_edges);
            coplanar_polygons.append(&mut back_coplanar);
            intersection_edges.append(&mut back_edges);
        } else if let Some(front) = &self.front {
            front.slice_recursive_parallel(
                slicing_plane,
                coplanar_polygons,
                intersection_edges,
            );
        } else if let Some(back) = &self.back {
            back.slice_recursive_parallel(
                slicing_plane,
                coplanar_polygons,
                intersection_edges,
            );
        }
    }
}
