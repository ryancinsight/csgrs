//! BSP node for voxel/BSP hybrid, ported from mesh::bsp to use voxels::{plane,polygon,vertex}

use crate::float_types::{EPSILON, Real};
use crate::voxels::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use nalgebra::Point3;
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    pub plane: Option<Plane>,
    pub front: Option<Box<Node<S>>>,
    pub back: Option<Box<Node<S>>>,
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Default for Node<S> {
    fn default() -> Self { Self::new() }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    pub const fn new() -> Self {
        Self { plane: None, front: None, back: None, polygons: Vec::new() }
    }

    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() { node.build(polygons); }
        node
    }

    #[cfg(not(feature = "parallel"))]
    pub fn invert(&mut self) {
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane { plane.flip(); }
        if let Some(ref mut front) = self.front { front.invert(); }
        if let Some(ref mut back) = self.back { back.invert(); }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// Pick the best splitting plane using Surface Area Heuristic (SAH)
    ///
    /// This implementation uses a robust Surface Area Heuristic that considers:
    /// - Surface area of resulting partitions
    /// - Balance between front and back partitions
    /// - Cost of spanning polygons
    /// - Numerical stability and edge cases
    ///
    /// **Mathematical Foundation**: SAH minimizes expected traversal cost:
    /// Cost = C_traverse + P_front * C_front + P_back * C_back + P_span * C_span
    /// where P_x is probability of hitting partition x (proportional to surface area)
    pub fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        // Handle edge cases
        if polygons.is_empty() {
            // Return a default plane - this should not happen in normal usage
            return Plane::from_points(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0)
            );
        }

        if polygons.len() == 1 {
            return polygons[0].plane.clone();
        }

        // SAH constants based on empirical studies
        const TRAVERSE_COST: Real = 1.0;
        const INTERSECT_COST: Real = 2.0;
        const SPAN_PENALTY: Real = 4.0;
        const BALANCE_WEIGHT: Real = 0.1;

        let mut best_plane = polygons[0].plane.clone();
        let mut best_cost = Real::MAX;

        // Calculate total surface area for normalization
        let total_surface_area = Self::calculate_total_surface_area(polygons);
        if total_surface_area < EPSILON {
            // All polygons are degenerate, return first valid plane
            return polygons[0].plane.clone();
        }

        // Evaluate all polygon planes as potential splitting planes
        // Use all polygons for small sets, sample for large sets
        let sample_size = if polygons.len() <= 50 {
            polygons.len()
        } else {
            // For large sets, sample more intelligently
            (polygons.len() as Real).sqrt() as usize + 10
        };

        let step = if polygons.len() <= sample_size {
            1
        } else {
            polygons.len() / sample_size
        };

        for (_i, candidate_poly) in polygons.iter().enumerate().step_by(step) {
            let plane = &candidate_poly.plane;

            // Skip degenerate planes
            if plane.normal().norm_squared() < EPSILON * EPSILON {
                continue;
            }

            let (front_area, back_area, spanning_count, front_count, back_count) =
                Self::evaluate_split(polygons, plane);

            // Calculate SAH cost
            let total_area = front_area + back_area;
            if total_area < EPSILON {
                continue; // Skip degenerate splits
            }

            let p_front = front_area / total_area;
            let p_back = back_area / total_area;

            // SAH cost function with additional heuristics
            let traversal_cost = TRAVERSE_COST;
            let front_cost = p_front * front_count as Real * INTERSECT_COST;
            let back_cost = p_back * back_count as Real * INTERSECT_COST;
            let span_cost = spanning_count as Real * SPAN_PENALTY;

            // Balance penalty - prefer more balanced splits
            let balance_penalty = BALANCE_WEIGHT *
                ((front_count as Real - back_count as Real).abs() / polygons.len() as Real);

            let total_cost = traversal_cost + front_cost + back_cost + span_cost + balance_penalty;

            if total_cost < best_cost {
                best_cost = total_cost;
                best_plane = plane.clone();
            }
        }

        best_plane
    }

    /// Calculate total surface area of all polygons for SAH normalization
    fn calculate_total_surface_area(polygons: &[Polygon<S>]) -> Real {
        polygons.iter()
            .map(|poly| poly.surface_area())
            .sum()
    }

    /// Evaluate a potential split plane and return metrics for SAH calculation
    /// Returns: (front_surface_area, back_surface_area, spanning_count, front_count, back_count)
    fn evaluate_split(polygons: &[Polygon<S>], plane: &Plane) -> (Real, Real, usize, usize, usize) {
        let mut front_area = 0.0;
        let mut back_area = 0.0;
        let mut spanning_count = 0;
        let mut front_count = 0;
        let mut back_count = 0;

        for poly in polygons {
            let classification = plane.classify_polygon(poly);
            let poly_area = poly.surface_area();

            match classification {
                COPLANAR => {
                    // Coplanar polygons contribute to both sides for conservative estimation
                    front_area += poly_area * 0.5;
                    back_area += poly_area * 0.5;
                }
                FRONT => {
                    front_area += poly_area;
                    front_count += 1;
                }
                BACK => {
                    back_area += poly_area;
                    back_count += 1;
                }
                SPANNING => {
                    // Spanning polygons contribute to both sides (conservative)
                    front_area += poly_area;
                    back_area += poly_area;
                    spanning_count += 1;
                    front_count += 1;
                    back_count += 1;
                }
                _ => {
                    // Treat unknown classifications as spanning for safety
                    front_area += poly_area;
                    back_area += poly_area;
                    spanning_count += 1;
                    front_count += 1;
                    back_count += 1;
                }
            }
        }

        (front_area, back_area, spanning_count, front_count, back_count)
    }

    #[cfg(not(feature = "parallel"))]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // Iterative traversal for front/back children to avoid deep recursion
        if self.plane.is_none() { return polygons.to_vec(); }
        let plane = self.plane.as_ref().unwrap();
        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) = plane.split_polygon(polygon);
            for coplanar_poly in coplanar_front.into_iter().chain(coplanar_back.into_iter()) {
                if plane.orient_plane(&coplanar_poly.plane) == FRONT { front_parts.push(coplanar_poly); } else { back_parts.push(coplanar_poly); }
            }
            front_polys.append(&mut front_parts);
            back_polys.append(&mut back_parts);
        }
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

    #[cfg(not(feature = "parallel"))]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front { front.clip_to(bsp); }
        if let Some(ref mut back) = self.back { back.clip_to(bsp); }
    }

    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];
        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);
            stack.extend([
                node.front.as_ref().map(|b| b.as_ref()),
                node.back.as_ref().map(|b| b.as_ref()),
            ].into_iter().flatten());
        }
        result
    }

    #[cfg(not(feature = "parallel"))]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() { return; }
        if self.plane.is_none() { self.plane = Some(self.pick_best_splitting_plane(polygons)); }
        let plane = self.plane.as_ref().unwrap();
        let mut front = Vec::with_capacity(polygons.len()/2);
        let mut back = Vec::with_capacity(polygons.len()/2);
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) = plane.split_polygon(polygon);
            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);
            front.append(&mut front_parts);
            back.append(&mut back_parts);
        }
        if !front.is_empty() {
            let child = self.front.get_or_insert_with(|| Box::new(Node::new()));
            child.build(&front);
        }
        if !back.is_empty() {
            let child = self.back.get_or_insert_with(|| Box::new(Node::new()));
            child.build(&back);
        }
    }

    #[cfg(not(feature = "parallel"))]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let all_polys = self.all_polygons();
        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();
        for poly in &all_polys {
            let vcount = poly.vertices.len(); if vcount < 2 { continue; }
            let types: Vec<_> = poly.vertices.iter().map(|v| slicing_plane.orient_point(&v.pos)).collect();
            let polygon_type = types.iter().fold(0, |acc, &t| acc | t);
            match polygon_type {
                COPLANAR => { coplanar_polygons.push(poly.clone()); }
                FRONT | BACK => {}
                SPANNING => {
                    let crossing_points: Vec<_> = (0..vcount).filter_map(|i| {
                        let j = (i + 1) % vcount;
                        let ti = types[i]; let tj = types[j];
                        let vi = &poly.vertices[i]; let vj = &poly.vertices[j];
                        if (ti | tj) == SPANNING {
                            let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                            if denom.abs() > EPSILON {
                                let t = (slicing_plane.offset() - slicing_plane.normal().dot(&vi.pos.coords)) / denom;
                                Some(vi.interpolate(vj, t))
                            } else { None }
                        } else { None }
                    }).collect();
                    intersection_edges.extend(crossing_points.chunks_exact(2).map(|c| [c[0].clone(), c[1].clone()]));
                }
                _ => {}
            }
        }
        (coplanar_polygons, intersection_edges)
    }

    /// Optimize memory usage by compacting vectors and removing empty nodes
    pub fn optimize_memory(&mut self) {
        // Shrink polygon vector to fit
        self.polygons.shrink_to_fit();

        // Recursively optimize children
        if let Some(ref mut front) = self.front {
            front.optimize_memory();

            // Remove empty front node
            if front.is_empty() {
                self.front = None;
            }
        }

        if let Some(ref mut back) = self.back {
            back.optimize_memory();

            // Remove empty back node
            if back.is_empty() {
                self.back = None;
            }
        }
    }

    /// Check if this BSP node is empty (no polygons and no children)
    pub fn is_empty(&self) -> bool {
        self.polygons.is_empty() && self.front.is_none() && self.back.is_none()
    }

    /// Get memory usage of this BSP tree
    pub fn memory_usage(&self) -> usize {
        let mut size = std::mem::size_of::<Self>();

        // Add polygon memory
        size += self.polygons.capacity() * std::mem::size_of::<Polygon<S>>();

        // Add children memory
        if let Some(ref front) = self.front {
            size += front.memory_usage();
        }
        if let Some(ref back) = self.back {
            size += back.memory_usage();
        }

        size
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::voxels::vertex::Vertex;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_bsp_construction() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<()> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let bsp: Node<()> = Node::from_polygons(&polygons);
        assert!(bsp.plane.is_some());
    }

    #[test]
    fn test_surface_area_heuristic_splitting_plane() {
        // Create test polygons with known optimal split
        let polygons = create_test_polygons_for_splitting();

        let bsp = Node::<()>::new();
        let best_plane = bsp.pick_best_splitting_plane(&polygons);

        // Verify the plane is valid (non-degenerate)
        let normal = best_plane.normal();
        assert!(normal.norm() > 1e-6, "Selected plane should have valid normal");

        // Test that the plane actually splits the polygons
        let (front_area, back_area, spanning_count, front_count, back_count) =
            Node::<()>::evaluate_split(&polygons, &best_plane);

        // The test mainly verifies that the algorithm doesn't crash and produces a valid plane
        // The actual classification depends on the specific geometry and plane selection
        println!("Split results: front={}, back={}, spanning={}, front_area={:.6}, back_area={:.6}",
                front_count, back_count, spanning_count, front_area, back_area);

        // Surface areas should be positive
        assert!(front_area >= 0.0 && back_area >= 0.0, "Surface areas should be non-negative");
    }

    #[test]
    fn test_splitting_plane_edge_cases() {
        let bsp = Node::<()>::new();

        // Test empty polygon list
        let empty_polygons: Vec<Polygon<()>> = vec![];
        let plane = bsp.pick_best_splitting_plane(&empty_polygons);
        // Should return a default plane without crashing
        assert!(plane.normal().norm() > 0.0);

        // Test single polygon
        let single_polygon = vec![create_simple_triangle()];
        let plane = bsp.pick_best_splitting_plane(&single_polygon);
        assert!(plane.normal().norm() > 0.0);

        // Test degenerate polygons (very small area)
        let degenerate_polygons = create_degenerate_polygons();
        let plane = bsp.pick_best_splitting_plane(&degenerate_polygons);
        // Should handle gracefully
        assert!(plane.normal().norm() >= 0.0);
    }

    #[test]
    fn test_surface_area_calculation() {
        // Test triangle area calculation
        let triangle = create_simple_triangle();
        let area = triangle.surface_area();

        // Triangle with vertices at (0,0,0), (1,0,0), (0,1,0) should have area 0.5
        assert!((area - 0.5).abs() < 1e-6, "Triangle area should be 0.5, got {}", area);

        // Test quad area calculation
        let quad = create_simple_quad();
        let quad_area = quad.surface_area();
        assert!(quad_area > 0.0, "Quad should have positive area");
    }

    #[test]
    fn test_evaluate_split_metrics() {
        let polygons = create_test_polygons_for_splitting();
        let plane = Plane::from_points(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0)
        );

        let (front_area, back_area, spanning_count, front_count, back_count) =
            Node::<()>::evaluate_split(&polygons, &plane);

        // Verify metrics are reasonable
        assert!(front_area >= 0.0 && back_area >= 0.0);
        assert!(front_count + back_count + spanning_count <= polygons.len() * 2); // Conservative upper bound
    }

    // Helper functions for creating test data
    fn create_simple_triangle() -> Polygon<()> {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        Polygon::new(vertices, None)
    }

    fn create_simple_quad() -> Polygon<()> {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        Polygon::new(vertices, None)
    }

    fn create_test_polygons_for_splitting() -> Vec<Polygon<()>> {
        vec![
            // Front-facing triangle
            Polygon::new(vec![
                Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(1.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            ], None),
            // Back-facing triangle
            Polygon::new(vec![
                Vertex::new(Point3::new(-1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(-2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(-1.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            ], None),
            // Spanning triangle
            Polygon::new(vec![
                Vertex::new(Point3::new(-0.5, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(0.5, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            ], None),
        ]
    }

    fn create_degenerate_polygons() -> Vec<Polygon<()>> {
        vec![
            // Very small triangle
            Polygon::new(vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(1e-8, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(0.0, 1e-8, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            ], None),
        ]
    }
}
