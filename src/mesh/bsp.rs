//! [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node structure and operations

#[cfg(not(feature = "parallel"))]
use crate::float_types::EPSILON;

#[cfg(not(feature = "parallel"))]
use crate::mesh::vertex::Vertex;

use crate::float_types::{Real, EPSILON as GLOBAL_EPSILON};
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::mesh::polygon::Polygon;
use nalgebra::Vector3;
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
    /// Calculate characteristic length for a set of polygons.
    /// Uses the bounding box diagonal as a measure of geometric scale.
    pub fn calculate_characteristic_length(polygons: &[Polygon<S>]) -> Real {
        if polygons.is_empty() {
            return 1.0; // Default fallback
        }

        // Find bounding box of all polygon vertices
        let mut min_point = polygons[0].vertices[0].pos;
        let mut max_point = min_point;

        for polygon in polygons {
            for vertex in &polygon.vertices {
                let pos = vertex.pos;
                min_point.x = min_point.x.min(pos.x);
                min_point.y = min_point.y.min(pos.y);
                min_point.z = min_point.z.min(pos.z);
                max_point.x = max_point.x.max(pos.x);
                max_point.y = max_point.y.max(pos.y);
                max_point.z = max_point.z.max(pos.z);
            }
        }

        // Return diagonal length as characteristic scale
        (max_point - min_point).norm()
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
        // Dynamic weights based on geometry characteristics
        let characteristic_length = Plane::calculate_polygon_characteristic_length(polygons);
        let polygon_count = polygons.len() as Real;

        // Adaptive weights: prioritize quality for complex geometry, balance for simple geometry
        let k_spans = if polygon_count > 50.0 { 6.0 } else { 8.0 };
        let k_balance = if polygon_count > 100.0 { 0.5 } else { 1.0 };
        let k_stability = 3.0; // Always important
        let k_quality = if polygon_count > 30.0 { 8.0 } else { 5.0 }; // Higher priority for complex meshes

        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // Improved sampling strategy: stratified sampling for better coverage
        let sample_size = if polygons.len() <= 20 {
            polygons.len()
        } else {
            // Sample every nth polygon plus some random ones for better distribution
            let _stride = polygons.len() / 15; // Get ~15 evenly distributed
            let extra_samples = 5.min(polygons.len() - 15); // Add up to 5 more
            15 + extra_samples
        };

        let mut candidates = Vec::with_capacity(sample_size);

        if polygons.len() <= 20 {
            // Use all polygons for small sets
            candidates.extend(polygons.iter());
        } else {
            // Stratified sampling: take every nth polygon
            let stride = polygons.len() / 15;
            for i in (0..polygons.len()).step_by(stride.max(1)).take(15) {
                candidates.push(&polygons[i]);
            }
            // Add some from the end to ensure good coverage
            let remaining = sample_size - candidates.len();
            let start_idx = polygons.len().saturating_sub(remaining);
            for poly in polygons.iter().skip(start_idx) {
                if candidates.len() < sample_size {
                    candidates.push(poly);
                }
            }
        }

        for p in candidates {
            let plane = &p.plane;
            let plane_normal = plane.normal();

            // Skip degenerate planes
            if plane_normal.norm() < GLOBAL_EPSILON {
                continue;
            }

            let mut num_front = 0;
            let mut num_back = 0;
            let mut num_spanning = 0;

            for poly in polygons {
                match plane.classify_polygon(poly) {
                    COPLANAR => {}, // Not counted for balance
                    FRONT => num_front += 1,
                    BACK => num_back += 1,
                    SPANNING => num_spanning += 1,
                    _ => num_spanning += 1, // Treat any other combination as spanning
                }
            }

            // Enhanced scoring with geometric awareness
            let balance_score = k_balance * ((num_front - num_back) as Real).abs();
            let spanning_score = k_spans * num_spanning as Real;
            let stability_score = k_stability * self.calculate_stability_penalty(&plane_normal, characteristic_length);

            // Calculate intersection quality score (lower is better, so we subtract it)
            let quality_score = self.calculate_intersection_quality_score(plane, polygons);
            let quality_penalty = k_quality * (1.0 - quality_score); // Convert to penalty

            // Additional geometric considerations for curved surfaces
            let curvature_bonus = self.calculate_curvature_alignment_bonus(plane, polygons);

            let total_score = spanning_score + balance_score + stability_score + quality_penalty - curvature_bonus;

            if total_score < best_score {
                best_score = total_score;
                best_plane = plane.clone();
            }
        }
        best_plane
    }

    /// Calculate numerical stability penalty for a plane normal.
    /// Penalizes normals that are nearly axis-aligned or have very small components,
    /// which can lead to precision issues in intersection calculations.
    fn calculate_stability_penalty(&self, normal: &Vector3<Real>, characteristic_length: Real) -> Real {
        let adaptive_eps = crate::float_types::adaptive_epsilon(characteristic_length);
        let mut penalty = 0.0;

        // Penalize normals with very small components (near axis-aligned)
        let abs_components = [normal.x.abs(), normal.y.abs(), normal.z.abs()];
        for &component in &abs_components {
            if component < adaptive_eps * 10.0 {
                penalty += 0.5; // Moderate penalty for near-axis alignment
            }
        }

        // Penalize normals that are nearly zero (degenerate)
        let normal_length = normal.norm();
        if normal_length < adaptive_eps {
            penalty += 10.0; // High penalty for degenerate normals
        }

        penalty
    }

    /// Calculate intersection quality score for a plane with the given polygons.
    /// Higher scores indicate better intersection quality (more perpendicular cuts).
    /// This helps avoid tangential intersections that create gaps and poor geometry.
    fn calculate_intersection_quality_score(&self, plane: &Plane, polygons: &[Polygon<S>]) -> Real {
        let plane_normal = plane.normal();
        let mut quality_score = 0.0;
        let mut intersection_count = 0;

        for polygon in polygons {
            // Calculate how perpendicular the plane is to the polygon's edges
            let vertices = &polygon.vertices;
            if vertices.len() < 2 {
                continue;
            }

            for i in 0..vertices.len() {
                let j = (i + 1) % vertices.len();
                let edge_vec = vertices[j].pos - vertices[i].pos;
                let edge_length = edge_vec.norm();

                if edge_length < GLOBAL_EPSILON {
                    continue; // Skip degenerate edges
                }

                let edge_dir = edge_vec / edge_length;

                // Calculate angle between plane normal and edge direction
                // cos(θ) = |normal · edge_dir|
                let cos_angle = plane_normal.dot(&edge_dir).abs();

                // For good intersections, we want the plane normal to be perpendicular
                // to the edge (cos_angle close to 0) or parallel (cos_angle close to 1)
                // Penalize angles around 45 degrees (cos_angle ≈ 0.707) which create
                // tangential intersections
                let perpendicularity_score = if cos_angle < 0.3 {
                    // Nearly perpendicular - excellent for clean cuts
                    1.0
                } else if cos_angle > 0.9 {
                    // Nearly parallel - good for avoiding intersections
                    0.8
                } else {
                    // Oblique angle - penalize heavily as this creates tangential cuts
                    let oblique_penalty = (cos_angle - 0.3) / 0.6; // 0 to 1 range
                    0.2 * (1.0 - oblique_penalty) // Heavily penalize oblique angles
                };

                quality_score += perpendicularity_score * edge_length;
                intersection_count += 1;
            }
        }

        if intersection_count > 0 {
            quality_score / intersection_count as Real
        } else {
            0.0
        }
    }

    /// Calculate bonus for planes that align well with curved surfaces.
    /// This helps BSP trees handle cylinder-cube intersections better by preferring
    /// planes that cut cleanly through curved geometry rather than tangentially.
    fn calculate_curvature_alignment_bonus(&self, plane: &Plane, polygons: &[Polygon<S>]) -> Real {
        let plane_normal = plane.normal();
        let mut alignment_bonus = 0.0;
        let mut surface_count = 0;

        for polygon in polygons {
            let poly_normal = polygon.plane.normal();

            // Calculate how well the splitting plane aligns with the polygon's surface
            let dot_product = plane_normal.dot(&poly_normal).abs();

            // Bonus for planes that are either very parallel (dot ≈ 1) or very perpendicular (dot ≈ 0)
            // to existing surfaces - these create clean cuts
            let alignment_score = if dot_product > 0.9 {
                // Nearly parallel - good for separating layers
                1.0
            } else if dot_product < 0.1 {
                // Nearly perpendicular - excellent for cutting through surfaces
                2.0
            } else {
                // Oblique angles - less desirable
                0.1
            };

            // Weight by polygon size (larger polygons have more influence)
            let polygon_size = polygon.vertices.len() as Real;
            alignment_bonus += alignment_score * polygon_size;
            surface_count += polygon_size as i32;
        }

        if surface_count > 0 {
            alignment_bonus / surface_count as Real
        } else {
            0.0
        }
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

        // Optimized polygon splitting with enhanced numerical stability
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

        // Enhanced pre-allocation based on polygon analysis
        let mut front = Vec::with_capacity(polygons.len() / 2);
        let mut back = Vec::with_capacity(polygons.len() / 2);
        let mut total_splits = 0;

        // Optimized polygon classification with split tracking for quality metrics
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_polygon(polygon);

            // Track splitting statistics for adaptive tree balancing
            if !front_parts.is_empty() && !back_parts.is_empty() {
                total_splits += 1;
            }

            // Extend collections efficiently with iterator chains
            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);
            front.append(&mut front_parts);
            back.append(&mut back_parts);
        }

        // Adaptive depth control: if we're creating too many splits,
        // prefer leaf nodes to prevent over-fragmentation
        let split_ratio = total_splits as Real / polygons.len() as Real;
        let should_continue_splitting = split_ratio < 0.8 && polygons.len() > 4;

        // Build child nodes with adaptive depth control for better mesh quality
        if !front.is_empty() && should_continue_splitting {
            self.front
                .get_or_insert_with(|| Box::new(Node::new()))
                .build(&front);
        } else if !front.is_empty() {
            // Convert remaining polygons to leaf node
            self.polygons.extend(front);
        }

        if !back.is_empty() && should_continue_splitting {
            self.back
                .get_or_insert_with(|| Box::new(Node::new()))
                .build(&back);
        } else if !back.is_empty() {
            // Convert remaining polygons to leaf node
            self.polygons.extend(back);
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
                                // Use the robust intersection method from plane
                                slicing_plane.compute_robust_edge_intersection(
                                    vi, vj, &slicing_plane.normal()
                                )
                            } else {
                                None
                            }
                        })
                        .collect();

                    // Convert crossing points to intersection edges
                    intersection_edges.extend(
                        crossing_points
                            .chunks_exact(2)
                            .map(|chunk| [chunk[0], chunk[1]]),
                    );
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
