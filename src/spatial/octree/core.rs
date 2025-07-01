//! Core Octree node structure and basic operations

use crate::geometry::Polygon;
use crate::core::float_types::Real;
use crate::spatial::traits::{Aabb, SpatialIndex, SpatialStatistics};
use nalgebra::Point3;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

/// Octant indices for the 8 children of an octree node
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Octant {
    /// Bottom-front-left (min x, min y, min z)
    BottomFrontLeft = 0,
    /// Bottom-front-right (max x, min y, min z)
    BottomFrontRight = 1,
    /// Bottom-back-left (min x, max y, min z)
    BottomBackLeft = 2,
    /// Bottom-back-right (max x, max y, min z)
    BottomBackRight = 3,
    /// Top-front-left (min x, min y, max z)
    TopFrontLeft = 4,
    /// Top-front-right (max x, min y, max z)
    TopFrontRight = 5,
    /// Top-back-left (min x, max y, max z)
    TopBackLeft = 6,
    /// Top-back-right (max x, max y, max z)
    TopBackRight = 7,
}

impl Octant {
    /// Get all octants in order
    pub fn all() -> [Octant; 8] {
        [
            Octant::BottomFrontLeft,
            Octant::BottomFrontRight,
            Octant::BottomBackLeft,
            Octant::BottomBackRight,
            Octant::TopFrontLeft,
            Octant::TopFrontRight,
            Octant::TopBackLeft,
            Octant::TopBackRight,
        ]
    }

    /// Get the bounding box for this octant within a parent bounds
    pub fn bounds(self, parent_bounds: &Aabb) -> Aabb {
        let center = parent_bounds.center();
        let min = parent_bounds.min;
        let max = parent_bounds.max;

        match self {
            Octant::BottomFrontLeft => Aabb::new(min, center),
            Octant::BottomFrontRight => Aabb::new(
                Point3::new(center.x, min.y, min.z),
                Point3::new(max.x, center.y, center.z),
            ),
            Octant::BottomBackLeft => Aabb::new(
                Point3::new(min.x, center.y, min.z),
                Point3::new(center.x, max.y, center.z),
            ),
            Octant::BottomBackRight => Aabb::new(
                Point3::new(center.x, center.y, min.z),
                Point3::new(max.x, max.y, center.z),
            ),
            Octant::TopFrontLeft => Aabb::new(
                Point3::new(min.x, min.y, center.z),
                Point3::new(center.x, center.y, max.z),
            ),
            Octant::TopFrontRight => Aabb::new(
                Point3::new(center.x, min.y, center.z),
                Point3::new(max.x, center.y, max.z),
            ),
            Octant::TopBackLeft => Aabb::new(
                Point3::new(min.x, center.y, center.z),
                Point3::new(center.x, max.y, max.z),
            ),
            Octant::TopBackRight => Aabb::new(center, max),
        }
    }
}

/// An Octree node for hierarchical 3D space subdivision
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Bounding box of this octree node
    pub bounds: Aabb,
    
    /// Eight child nodes (one for each octant)
    pub children: [Option<Box<Node<S>>>; 8],
    
    /// Polygons stored in this node (for leaf nodes or when polygons span multiple octants)
    pub polygons: Vec<Polygon<S>>,
    
    /// Current subdivision level (0 = root)
    pub level: usize,
    
    /// Whether this node has been subdivided
    pub is_subdivided: bool,
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create a new empty Octree node with given bounds
    pub fn new_with_bounds(bounds: Aabb) -> Self {
        Self {
            bounds,
            children: [None, None, None, None, None, None, None, None],
            polygons: Vec::new(),
            level: 0,
            is_subdivided: false,
        }
    }

    /// Create a new empty Octree node (requires bounds to be set later)
    pub fn new() -> Self {
        Self::new_with_bounds(Aabb::new(Point3::origin(), Point3::origin()))
    }

    /// Check if this is a leaf node
    pub fn is_leaf(&self) -> bool {
        !self.is_subdivided
    }

    /// Get the depth of this subtree
    pub fn depth(&self) -> usize {
        if self.is_leaf() {
            1
        } else {
            let max_child_depth = self.children
                .iter()
                .filter_map(|child| child.as_ref().map(|n| n.depth()))
                .max()
                .unwrap_or(0);
            1 + max_child_depth
        }
    }

    /// Count the total number of nodes in this subtree
    pub fn node_count(&self) -> usize {
        let mut count = 1;
        for child in &self.children {
            if let Some(ref child_node) = child {
                count += child_node.node_count();
            }
        }
        count
    }

    /// Count the total number of polygons in this subtree
    pub fn polygon_count(&self) -> usize {
        let mut count = self.polygons.len();
        for child in &self.children {
            if let Some(ref child_node) = child {
                count += child_node.polygon_count();
            }
        }
        count
    }

    /// Get all polygons in this Octree using iterative traversal
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Add child nodes to stack
            for child in &node.children {
                if let Some(ref child_node) = child {
                    stack.push(child_node.as_ref());
                }
            }
        }
        result
    }

    /// Get the center point of a polygon (centroid of vertices)
    pub fn polygon_center(polygon: &Polygon<S>) -> Point3<Real> {
        let sum = polygon.vertices.iter()
            .fold(Point3::origin(), |acc, vertex| acc + vertex.pos.coords);
        Point3::from(sum.coords / polygon.vertices.len() as Real)
    }

    /// Determine which octant a point belongs to
    pub fn point_octant(&self, point: &Point3<Real>) -> Octant {
        let center = self.bounds.center();
        
        let x_bit = if point.x >= center.x { 1 } else { 0 };
        let y_bit = if point.y >= center.y { 2 } else { 0 };
        let z_bit = if point.z >= center.z { 4 } else { 0 };
        
        match x_bit | y_bit | z_bit {
            0 => Octant::BottomFrontLeft,
            1 => Octant::BottomFrontRight,
            2 => Octant::BottomBackLeft,
            3 => Octant::BottomBackRight,
            4 => Octant::TopFrontLeft,
            5 => Octant::TopFrontRight,
            6 => Octant::TopBackLeft,
            7 => Octant::TopBackRight,
            _ => unreachable!(),
        }
    }

    /// Determine which octant(s) a polygon belongs to
    pub fn polygon_octants(&self, polygon: &Polygon<S>) -> Vec<Octant> {
        // Simple implementation: check polygon center
        // A more sophisticated implementation would check if polygon spans multiple octants
        let center = Self::polygon_center(polygon);
        vec![self.point_octant(&center)]
    }

    /// Get the child node for a specific octant
    pub fn get_child(&self, octant: Octant) -> Option<&Node<S>> {
        self.children[octant as usize].as_ref().map(|boxed| boxed.as_ref())
    }

    /// Get the mutable child node for a specific octant
    pub fn get_child_mut(&mut self, octant: Octant) -> Option<&mut Node<S>> {
        self.children[octant as usize].as_mut().map(|boxed| boxed.as_mut())
    }

    /// Set the child node for a specific octant
    pub fn set_child(&mut self, octant: Octant, child: Node<S>) {
        self.children[octant as usize] = Some(Box::new(child));
    }

    /// Calculate the volume of this node's bounds
    pub fn volume(&self) -> Real {
        self.bounds.volume()
    }

    /// Calculate the surface area of this node's bounds
    pub fn surface_area(&self) -> Real {
        let size = self.bounds.size();
        2.0 * (size.x * size.y + size.y * size.z + size.z * size.x)
    }

    /// Estimate memory usage of this subtree in bytes
    pub fn memory_usage(&self) -> usize {
        let mut size = std::mem::size_of::<Self>();
        
        // Add polygon memory
        size += self.polygons.capacity() * std::mem::size_of::<Polygon<S>>();
        for polygon in &self.polygons {
            size += polygon.vertices.capacity() * std::mem::size_of_val(&polygon.vertices[0]);
        }
        
        // Add child memory
        for child in &self.children {
            if let Some(ref child_node) = child {
                size += child_node.memory_usage();
            }
        }
        
        size
    }
}

impl<S: Clone + Send + Sync + Debug> SpatialIndex<S> for Node<S> {
    fn build(polygons: &[Polygon<S>]) -> Self {
        Self::from_polygons(polygons)
    }

    fn new() -> Self {
        Self::new()
    }

    fn all_polygons(&self) -> Vec<Polygon<S>> {
        self.all_polygons()
    }

    fn query_range(&self, bounds: &Aabb) -> Vec<&Polygon<S>> {
        self.volume_query(bounds)
    }

    fn nearest_neighbor(&self, point: &Point3<Real>) -> Option<&Polygon<S>> {
        // For octrees, nearest neighbor is less efficient than for KD-trees
        // This is a simplified implementation
        let mut best_polygon = None;
        let mut best_distance_squared = Real::INFINITY;

        for polygon in &self.polygons {
            let center = Self::polygon_center(polygon);
            let distance_squared = (point - center).norm_squared();
            if distance_squared < best_distance_squared {
                best_distance_squared = distance_squared;
                best_polygon = Some(polygon);
            }
        }

        // Recursively search children
        for child in &self.children {
            if let Some(ref child_node) = child {
                if let Some(candidate) = child_node.nearest_neighbor(point) {
                    let center = Self::polygon_center(candidate);
                    let distance_squared = (point - center).norm_squared();
                    if distance_squared < best_distance_squared {
                        best_distance_squared = distance_squared;
                        best_polygon = Some(candidate);
                    }
                }
            }
        }

        best_polygon
    }

    fn ray_intersections(&self, ray: &crate::spatial::traits::Ray) -> Vec<crate::spatial::traits::Intersection<S>> {
        let mut intersections = Vec::new();
        self.ray_intersections_recursive(ray, &mut intersections);
        intersections
    }

    fn contains_point(&self, point: &Point3<Real>) -> bool {
        if !self.bounds.contains_point(point) {
            return false;
        }

        // Check polygons in this node
        for polygon in &self.polygons {
            if self.point_in_polygon(point, polygon) {
                return true;
            }
        }

        // Recursively check children
        for child in &self.children {
            if let Some(ref child_node) = child {
                if child_node.contains_point(point) {
                    return true;
                }
            }
        }

        false
    }

    fn bounding_box(&self) -> Option<Aabb> {
        Some(self.bounds.clone())
    }

    fn statistics(&self) -> SpatialStatistics {
        SpatialStatistics {
            node_count: self.node_count(),
            max_depth: self.depth(),
            polygon_count: self.polygon_count(),
            memory_usage_bytes: self.memory_usage(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Simple point-in-polygon test (simplified implementation)
    fn point_in_polygon(&self, _point: &Point3<Real>, _polygon: &Polygon<S>) -> bool {
        // TODO: Implement proper 3D point-in-polygon test
        // For now, return false as this is complex and depends on polygon orientation
        false
    }

    /// Recursive ray intersection implementation
    fn ray_intersections_recursive(&self, ray: &crate::spatial::traits::Ray, intersections: &mut Vec<crate::spatial::traits::Intersection<S>>) {
        // Check if ray intersects this node's bounds
        if !self.ray_intersects_aabb(ray, &self.bounds) {
            return;
        }

        // Check intersections with polygons in this node
        for polygon in &self.polygons {
            if let Some(intersection) = self.ray_polygon_intersection(ray, polygon) {
                intersections.push(intersection);
            }
        }

        // Recursively check children
        for child in &self.children {
            if let Some(ref child_node) = child {
                child_node.ray_intersections_recursive(ray, intersections);
            }
        }
    }

    /// Check if a ray intersects with an AABB (same as KD-tree implementation)
    fn ray_intersects_aabb(&self, ray: &crate::spatial::traits::Ray, aabb: &Aabb) -> bool {
        use nalgebra::Vector3;
        
        let inv_dir = Vector3::new(
            1.0 / ray.direction.x,
            1.0 / ray.direction.y,
            1.0 / ray.direction.z,
        );

        let t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
        let t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;
        let t3 = (aabb.min.y - ray.origin.y) * inv_dir.y;
        let t4 = (aabb.max.y - ray.origin.y) * inv_dir.y;
        let t5 = (aabb.min.z - ray.origin.z) * inv_dir.z;
        let t6 = (aabb.max.z - ray.origin.z) * inv_dir.z;

        let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
        let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

        tmax >= 0.0 && tmin <= tmax
    }

    /// Calculate ray-polygon intersection (simplified)
    fn ray_polygon_intersection(&self, ray: &crate::spatial::traits::Ray, polygon: &Polygon<S>) -> Option<crate::spatial::traits::Intersection<S>> {
        // Simplified ray-polygon intersection
        let center = Self::polygon_center(polygon);
        let to_center = center - ray.origin;
        let projection = to_center.dot(&ray.direction);
        
        if projection < 0.0 {
            return None; // Behind ray origin
        }
        
        let closest_point = ray.origin + ray.direction * projection;
        let distance = (closest_point - center).norm();
        
        if distance < 0.1 { // Arbitrary threshold
            Some(crate::spatial::traits::Intersection {
                distance: projection,
                point: closest_point,
                normal: polygon.plane.normal(),
                polygon: polygon.clone(),
            })
        } else {
            None
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Default for Node<S> {
    fn default() -> Self {
        Self::new()
    }
}
