//! Parallel BSP tree operations (requires "parallel" feature)

#[cfg(feature = "parallel")]
use rayon::{join, prelude::*};

use super::core::Node;
use crate::core::float_types::EPSILON;
use crate::geometry::{BACK, COPLANAR, FRONT, Plane, Polygon, SPANNING, Vertex};
use std::fmt::Debug;

#[cfg(feature = "parallel")]
impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Build a BSP tree from the given polygons (parallel version)
    pub fn build_parallel(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.clone().unwrap();

        // Split polygons in parallel
        let (mut coplanar_front, mut coplanar_back, front, back) =
            polygons.par_iter().map(|p| plane.split_polygon(p)).reduce(
                || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc.2.extend(x.2);
                    acc.3.extend(x.3);
                    acc
                },
            );

        // Append coplanar fronts/backs to self.polygons
        self.polygons.append(&mut coplanar_front);
        self.polygons.append(&mut coplanar_back);

        // Recursively build front/back in parallel
        if let (Some(front_node), Some(back_node)) = (&mut self.front, &mut self.back) {
            join(|| front_node.build(&front), || back_node.build(&back));
        } else {
            if let Some(front_node) = &mut self.front {
                front_node.build(&front);
            }
            if let Some(back_node) = &mut self.back {
                back_node.build(&back);
            }
        }
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree (parallel version)
    pub fn clip_polygons_parallel(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // If this node has no plane, just return the original set
        if self.plane.is_none() {
            return polygons.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        // Split each polygon in parallel; gather results
        let (coplanar_front, coplanar_back, mut front, mut back) = polygons
            .par_iter()
            .map(|poly| plane.split_polygon(poly))
            .reduce(
                || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc.2.extend(x.2);
                    acc.3.extend(x.3);
                    acc
                },
            );

        // Decide where to send the coplanar polygons
        for cp in coplanar_front {
            if plane.orient_plane(&cp.plane) == FRONT {
                front.push(cp);
            } else {
                back.push(cp);
            }
        }
        for cp in coplanar_back {
            if plane.orient_plane(&cp.plane) == FRONT {
                front.push(cp);
            } else {
                back.push(cp);
            }
        }

        // Recursively clip front & back in parallel
        let (front_clipped, back_clipped) = join(
            || {
                if let Some(ref f) = self.front {
                    f.clip_polygons(&front)
                } else {
                    front
                }
            },
            || {
                if let Some(ref b) = self.back {
                    b.clip_polygons(&back)
                } else {
                    // If there's no back node, discard these polygons
                    Vec::new()
                }
            },
        );

        // Combine front and back
        let mut result = front_clipped;
        result.extend(back_clipped);
        result
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree (parallel version)
    pub fn clip_to_parallel(&mut self, bsp: &Node<S>) {
        // clip self.polygons in parallel
        let new_polygons = bsp.clip_polygons(&self.polygons);
        self.polygons = new_polygons;

        // Recurse in parallel over front/back
        if let (Some(front_node), Some(back_node)) = (&mut self.front, &mut self.back) {
            join(|| front_node.clip_to(bsp), || back_node.clip_to(bsp));
        } else {
            if let Some(front_node) = &mut self.front {
                front_node.clip_to(bsp);
            }
            if let Some(back_node) = &mut self.back {
                back_node.clip_to(bsp);
            }
        }
    }

    /// Slices this BSP node with `slicing_plane` (parallel version)
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
