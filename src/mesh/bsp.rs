//! [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node structure and operations

#[cfg(not(feature = "parallel"))]
use crate::float_types::EPSILON;

#[cfg(not(feature = "parallel"))]
use crate::mesh::vertex::Vertex;

use crate::float_types::Real;
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::mesh::polygon::Polygon;
use std::fmt::Debug;

/// A [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Splitting plane for this node *or* **None** for a leaf that
    /// only stores polygons.
    pub plane: Option<Plane>,

    /// Polygons in *front* half‑spaces.
    pub front: Option<Box<Node<S>>>,

    /// Polygons in *back* half‑spaces.
    pub back: Option<Box<Node<S>>>,

    /// Polygons that lie *exactly* on `plane`
    /// (after the node has been built).
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Default for Node<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create a new empty BSP node
    pub const fn new() -> Self {
        Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        }
    }

    /// Creates a new BSP node from polygons
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() {
            node.build(polygons);
        }
        node
    }

    /// Invert all polygons in the BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn invert(&mut self) {
        // Flip all polygons and plane in this node
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        if let Some(ref mut front) = self.front {
            front.invert();
        }
        if let Some(ref mut back) = self.back {
            back.invert();
        }

        std::mem::swap(&mut self.front, &mut self.back);
    }

    pub fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        //  Empirically-chosen base weights.  A lower K_SPANS strongly favours
        //  planes that do *not* slice polygons while K_BALANCE nudges the split
        //  towards a front/back equilibrium.  We derive an adaptive weight below
        //  so meshes of radically different complexity keep consistent behaviour.
        const K_SPANS_BASE: Real = 5.0;
        const K_BALANCE_BASE: Real = 1.0;

        // ------------------------------------------------------------------
        //  Adaptive coefficients – scale the balance penalty with the log10 of
        //  the polygon count so extremely large meshes do not over-penalise
        //  minor imbalances (KISS & ADP principles).
        // ------------------------------------------------------------------
        let poly_count = polygons.len().max(1) as Real;
        let k_balance = K_BALANCE_BASE * poly_count.log10().max(1.0);

        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // Larger meshes benefit from sampling more candidate planes – we use the
        // cubic-root of n (≤128 cap) which empirically balances cost vs.
        // quality and avoids missing good planes in dense areas.
        let sample_size = ((polygons.len() as f64).cbrt() as usize).max(1).min(128);
        let step = (polygons.len() / sample_size.max(1)).max(1);

        for p in polygons.iter().step_by(step) {
            let plane = &p.plane;
            let mut num_front = 0;
            let mut num_back = 0;
            let mut num_spanning = 0;

            for poly in polygons {
                match plane.classify_polygon(poly) {
                    COPLANAR => { /* ignored for balance */ },
                    FRONT => num_front += 1,
                    BACK => num_back += 1,
                    SPANNING | _ => num_spanning += 1,
                }
            }

            // The score is a weighted sum of the number of spanning polygons and the
            // imbalance between front and back polygons. The weights are chosen
            // to strongly penalize splitting polygons.
            let balance = (num_front as Real - num_back as Real).abs();
            let score = K_SPANS_BASE * num_spanning as Real + k_balance * balance;

            //  Fast-path: perfect plane (no spanning + nearly balanced) – stop early.
            if num_spanning == 0 && balance <= 1.0 {
                return plane.clone();
            }

            if score < best_score {
                best_score = score;
                best_plane = plane.clone();
            }
        }

        best_plane
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    /// **Mathematical Foundation**: Uses plane classification to determine polygon visibility.
    /// Polygons entirely in BACK half-space are clipped (removed).
    /// **Algorithm**: O(n log d) where n is polygon count, d is tree depth.
    #[cfg(not(feature = "parallel"))]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // If this node has no plane (i.e. it’s empty), just return
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
    #[cfg(not(feature = "parallel"))]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front {
            front.clip_to(bsp);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(bsp);
        }
    }

    /// Return all polygons in this BSP tree using an iterative approach,
    /// avoiding potential stack overflow of recursive approach
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Use iterator to add child nodes more efficiently
            stack.extend(
                [&node.front, &node.back]
                    .iter()
                    .filter_map(|child| child.as_ref().map(|boxed| boxed.as_ref())),
            );
        }
        result
    }

    /// Build a BSP tree from the given polygons
    #[cfg(not(feature = "parallel"))]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose the best splitting plane using a heuristic if not already set.
        if self.plane.is_none() {
            self.plane = Some(self.pick_best_splitting_plane(polygons));
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

    /// Slices this BSP node with `slicing_plane`, returning:
    /// - All polygons that are coplanar with the plane (within EPSILON),
    /// - A list of line‐segment intersections (each a [Vertex; 2]) from polygons that span the plane.
    #[cfg(not(feature = "parallel"))]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let all_polys = self.all_polygons();

        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();

        for poly in &all_polys {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                continue; // degenerate polygon => skip
            }

            // Use iterator chain to compute vertex types more efficiently
            let types: Vec<_> = poly
                .vertices
                .iter()
                .map(|vertex| slicing_plane.orient_point(&vertex.pos))
                .collect();

            let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

            // Based on the combined classification of its vertices:
            match polygon_type {
                COPLANAR => {
                    // The entire polygon is in the plane, so push it to the coplanar list.
                    coplanar_polygons.push(poly.clone());
                },

                FRONT | BACK => {
                    // Entirely on one side => no intersection. We skip it.
                },

                SPANNING => {
                    // The polygon crosses the plane. We'll gather the intersection points
                    // (the new vertices introduced on edges that cross the plane).
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

                    // -----------------------------------------------------------------
                    //  🔄  Robust Edge Pairing
                    // -----------------------------------------------------------------
                    // A well-behaved convex polygon must intersect a plane an **even**
                    // number of times (Jordan curve theorem).  With floating-point
                    // arithmetic we occasionally collect an **odd** number.  Instead of
                    // creating a hole (by discarding the orphan) we connect the last
                    // intersection back to the first, ensuring topological consistency
                    // while keeping the operation branch-free and cache-friendly.

                    let cp_count = crossing_points.len();
                    for chunk in crossing_points.chunks_exact(2) {
                        intersection_edges.push([chunk[0].clone(), chunk[1].clone()]);
                    }

                    if cp_count % 2 == 1 && cp_count >= 3 {
                        // Connect last to first to close the loop.
                        intersection_edges.push([
                            crossing_points[cp_count - 1].clone(),
                            crossing_points[0].clone(),
                        ]);
                    } else if cp_count % 2 == 1 {
                        // Rare degenerate: exactly one intersection -> skip but warn.
                        #[cfg(debug_assertions)]
                        eprintln!(
                            "[csgrs::bsp] Warning: single (unpaired) slice intersection at {:?}",
                            crossing_points[0].pos
                        );
                    }
                },

                _ => {
                    // Shouldn't happen in a typical classification, but we can ignore
                },
            }
        }

        (coplanar_polygons, intersection_edges)
    }
}

#[cfg(test)]
mod tests {
    use crate::mesh::bsp::Node;
    use crate::mesh::polygon::Polygon;
    use crate::mesh::vertex::Vertex;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_bsp_basic_functionality() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let node = Node::from_polygons(&polygons);
        assert!(!node.all_polygons().is_empty());
    }
}
