//! Core KD-tree node structure and basic operations

use crate::geometry::Polygon;
use crate::core::float_types::Real;
use crate::spatial::traits::{Aabb, SpatialIndex, SpatialStatistics};
use nalgebra::Point3;
use std::fmt::Debug;



/// Splitting axis for KD-tree nodes
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

impl Axis {
    /// Get the next axis in the cycle (X -> Y -> Z -> X)
    pub fn next(self) -> Self {
        match self {
            Axis::X => Axis::Y,
            Axis::Y => Axis::Z,
            Axis::Z => Axis::X,
        }
    }

    /// Get the coordinate value for a point along this axis
    pub fn coordinate(self, point: &Point3<Real>) -> Real {
        match self {
            Axis::X => point.x,
            Axis::Y => point.y,
            Axis::Z => point.z,
        }
    }
}

/// A KD-tree node for 3D spatial indexing
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Splitting axis for this node (None for leaf nodes)
    pub axis: Option<Axis>,
    
    /// Splitting value along the axis (None for leaf nodes)
    pub split_value: Option<Real>,
    
    /// Left child (points with coordinate <= split_value)
    pub left: Option<Box<Node<S>>>,
    
    /// Right child (points with coordinate > split_value)
    pub right: Option<Box<Node<S>>>,
    
    /// Polygons stored in this node (for leaf nodes or when polygons span the split)
    pub polygons: Vec<Polygon<S>>,
    
    /// Bounding box of all geometry in this subtree
    pub bounds: Option<Aabb>,
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create a new empty KD-tree node
    pub fn new() -> Self {
        Self {
            axis: None,
            split_value: None,
            left: None,
            right: None,
            polygons: Vec::new(),
            bounds: None,
        }
    }

    /// Check if this is a leaf node
    pub fn is_leaf(&self) -> bool {
        self.left.is_none() && self.right.is_none()
    }

    /// Get the depth of this subtree
    pub fn depth(&self) -> usize {
        if self.is_leaf() {
            1
        } else {
            let left_depth = self.left.as_ref().map_or(0, |n| n.depth());
            let right_depth = self.right.as_ref().map_or(0, |n| n.depth());
            1 + left_depth.max(right_depth)
        }
    }

    /// Count the total number of nodes in this subtree
    pub fn node_count(&self) -> usize {
        let mut count = 1;
        if let Some(ref left) = self.left {
            count += left.node_count();
        }
        if let Some(ref right) = self.right {
            count += right.node_count();
        }
        count
    }

    /// Count the total number of polygons in this subtree
    pub fn polygon_count(&self) -> usize {
        let mut count = self.polygons.len();
        if let Some(ref left) = self.left {
            count += left.polygon_count();
        }
        if let Some(ref right) = self.right {
            count += right.polygon_count();
        }
        count
    }

    /// Get all polygons in this KD-tree using iterative traversal
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Add child nodes to stack
            if let Some(ref left) = node.left {
                stack.push(left.as_ref());
            }
            if let Some(ref right) = node.right {
                stack.push(right.as_ref());
            }
        }
        result
    }

    /// Update the bounding box for this node based on its polygons
    pub fn update_bounds(&mut self) {
        self.bounds = Aabb::from_polygons(&self.polygons);
        
        // Merge with child bounds
        if let Some(ref left) = self.left {
            if let Some(ref left_bounds) = left.bounds {
                self.bounds = match self.bounds {
                    Some(ref bounds) => Some(self.merge_bounds(bounds, left_bounds)),
                    None => Some(left_bounds.clone()),
                };
            }
        }
        
        if let Some(ref right) = self.right {
            if let Some(ref right_bounds) = right.bounds {
                self.bounds = match self.bounds {
                    Some(ref bounds) => Some(self.merge_bounds(bounds, right_bounds)),
                    None => Some(right_bounds.clone()),
                };
            }
        }
    }

    /// Merge two bounding boxes
    fn merge_bounds(&self, a: &Aabb, b: &Aabb) -> Aabb {
        crate::spatial::utils::merge_bounds(a, b)
    }

    /// Get the center point of a polygon (centroid of vertices)
    pub fn polygon_center(polygon: &Polygon<S>) -> Point3<Real> {
        let sum = polygon.vertices.iter()
            .fold(Point3::origin(), |acc, vertex| acc + vertex.pos.coords);
        Point3::from(sum.coords / polygon.vertices.len() as Real)
    }

    /// Check if a polygon's center is on the left side of a split
    pub fn is_left_of_split(&self, polygon: &Polygon<S>) -> bool {
        if let (Some(axis), Some(split_value)) = (self.axis, self.split_value) {
            let center = Self::polygon_center(polygon);
            axis.coordinate(&center) <= split_value
        } else {
            false
        }
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
        if let Some(ref left) = self.left {
            size += left.memory_usage();
        }
        if let Some(ref right) = self.right {
            size += right.memory_usage();
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
        self.range_query(bounds)
    }

    fn nearest_neighbor(&self, point: &Point3<Real>) -> Option<&Polygon<S>> {
        self.nearest_neighbor(point)
    }

    fn ray_intersections(&self, ray: &crate::spatial::traits::Ray) -> Vec<crate::spatial::traits::Intersection<S>> {
        self.ray_intersections(ray)
    }

    fn contains_point(&self, point: &Point3<Real>) -> bool {
        // For KD-trees, we check if the point is inside any polygon
        // This is a simplified implementation - more sophisticated methods exist
        if let Some(ref bounds) = self.bounds {
            if !bounds.contains_point(point) {
                return false;
            }
        }

        // Check polygons in this node
        for polygon in &self.polygons {
            if self.point_in_polygon(point, polygon) {
                return true;
            }
        }

        // Recursively check children
        if let Some(ref left) = self.left {
            if left.contains_point(point) {
                return true;
            }
        }
        if let Some(ref right) = self.right {
            if right.contains_point(point) {
                return true;
            }
        }

        false
    }

    fn bounding_box(&self) -> Option<Aabb> {
        self.bounds.clone()
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
    /// Simple point-in-polygon test (2D projection)
    /// This is a simplified implementation for demonstration
    fn point_in_polygon(&self, _point: &Point3<Real>, _polygon: &Polygon<S>) -> bool {
        // TODO: Implement proper 3D point-in-polygon test
        // For now, return false as this is complex and depends on polygon orientation
        false
    }
}

impl<S: Clone + Send + Sync + Debug> Default for Node<S> {
    fn default() -> Self {
        Self::new()
    }
}
