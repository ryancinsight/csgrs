//! Core BSP tree node structure and basic operations

use crate::geometry::{Plane, Polygon};
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

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

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create a new empty BSP node
    pub fn new() -> Self {
        Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        }
    }

    /// Invert all polygons in the BSP tree
    pub fn invert(&mut self) {
        // Flip all polygons and plane in this node
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        // Recursively invert children - parallel when available and both exist
        #[cfg(feature = "parallel")]
        match (&mut self.front, &mut self.back) {
            (Some(front_node), Some(back_node)) => {
                join(|| front_node.invert(), || back_node.invert());
            },
            (Some(front_node), None) => front_node.invert(),
            (None, Some(back_node)) => back_node.invert(),
            (None, None) => {},
        }

        #[cfg(not(feature = "parallel"))]
        {
            if let Some(ref mut front) = self.front {
                front.invert();
            }
            if let Some(ref mut back) = self.back {
                back.invert();
            }
        }

        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// Return all polygons in this BSP tree using an iterative approach.
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

    /// Get the center point of a polygon (centroid of vertices)
    pub fn polygon_center(polygon: &crate::geometry::Polygon<S>) -> nalgebra::Point3<crate::core::float_types::Real> {
        crate::spatial::utils::polygon_center(polygon)
    }

    /// Check if a polygon intersects with a bounding box (simplified implementation)
    fn polygon_intersects_bounds(&self, polygon: &crate::geometry::Polygon<S>, bounds: &crate::spatial::traits::Aabb) -> bool {
        crate::spatial::utils::polygon_intersects_bounds(polygon, bounds)
    }

    /// Simple point-in-polygon test (simplified implementation)
    fn point_in_polygon(&self, point: &nalgebra::Point3<crate::core::float_types::Real>, polygon: &crate::geometry::Polygon<S>) -> bool {
        crate::spatial::utils::point_in_polygon(point, polygon)
    }

    /// Count the total number of nodes in this subtree
    pub fn node_count(&self) -> usize {
        let mut count = 1;
        if let Some(ref front) = self.front {
            count += front.node_count();
        }
        if let Some(ref back) = self.back {
            count += back.node_count();
        }
        count
    }

    /// Get the depth of this subtree
    pub fn depth(&self) -> usize {
        let front_depth = self.front.as_ref().map_or(0, |n| n.depth());
        let back_depth = self.back.as_ref().map_or(0, |n| n.depth());
        1 + front_depth.max(back_depth)
    }

    /// Count the total number of polygons in this subtree
    pub fn polygon_count(&self) -> usize {
        let mut count = self.polygons.len();
        if let Some(ref front) = self.front {
            count += front.polygon_count();
        }
        if let Some(ref back) = self.back {
            count += back.polygon_count();
        }
        count
    }

    /// Count the number of leaf nodes in this subtree
    pub fn leaf_count(&self) -> usize {
        if self.front.is_none() && self.back.is_none() {
            1 // This is a leaf
        } else {
            let mut count = 0;
            if let Some(ref front) = self.front {
                count += front.leaf_count();
            }
            if let Some(ref back) = self.back {
                count += back.leaf_count();
            }
            count
        }
    }

    /// Estimate memory usage of this subtree in bytes
    pub fn memory_usage(&self) -> usize {
        let mut size = std::mem::size_of::<Self>();

        // Add polygon memory
        size += self.polygons.capacity() * std::mem::size_of::<crate::geometry::Polygon<S>>();
        for polygon in &self.polygons {
            size += polygon.vertices.capacity() * std::mem::size_of_val(&polygon.vertices[0]);
        }

        // Add child memory
        if let Some(ref front) = self.front {
            size += front.memory_usage();
        }
        if let Some(ref back) = self.back {
            size += back.memory_usage();
        }

        size
    }

    /// Recursive range query helper method
    fn query_range_recursive<'a>(&'a self, bounds: &crate::spatial::traits::Aabb, results: &mut Vec<&'a crate::geometry::Polygon<S>>) {
        // Check polygons in this node
        for polygon in &self.polygons {
            if self.polygon_intersects_bounds(polygon, bounds) {
                results.push(polygon);
            }
        }

        // Recursively search children
        if let Some(ref front) = self.front {
            front.query_range_recursive(bounds, results);
        }
        if let Some(ref back) = self.back {
            back.query_range_recursive(bounds, results);
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Default for Node<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> crate::spatial::traits::SpatialIndex<S> for Node<S> {
    fn build(polygons: &[crate::geometry::Polygon<S>]) -> Self {
        Self::from_polygons(polygons)
    }

    fn new() -> Self {
        Self::new()
    }

    fn all_polygons(&self) -> Vec<crate::geometry::Polygon<S>> {
        self.all_polygons()
    }

    fn query_range(&self, bounds: &crate::spatial::traits::Aabb) -> Vec<&crate::geometry::Polygon<S>> {
        // BSP trees don't have native range query support
        // This would require implementing a proper BSP traversal with bounds checking
        // For now, return empty vector to indicate this operation is not efficiently supported
        let mut results = Vec::new();
        self.query_range_recursive(bounds, &mut results);
        results
    }

    fn nearest_neighbor(&self, _point: &nalgebra::Point3<crate::core::float_types::Real>) -> Option<&crate::geometry::Polygon<S>> {
        // BSP trees don't have efficient nearest neighbor search
        // This would require significant implementation effort and is not the primary use case for BSP trees
        // Return None to indicate this operation is not supported efficiently
        None
    }

    fn ray_intersections(&self, _ray: &crate::spatial::traits::Ray) -> Vec<crate::spatial::traits::Intersection<S>> {
        // BSP trees don't have native ray intersection support in the current implementation
        // This would require implementing ray-BSP traversal, which is complex
        // Return empty vector to indicate this operation is not implemented yet
        Vec::new()
    }

    fn contains_point(&self, point: &nalgebra::Point3<crate::core::float_types::Real>) -> bool {
        // BSP trees can determine point containment by traversing the tree
        if let Some(ref plane) = self.plane {
            // Calculate signed distance: d = n·p - n·p0 where p0 is a point on the plane
            let normal = plane.normal();
            let offset = plane.offset();
            let signed_distance = normal.dot(&point.coords) - offset;

            if signed_distance > crate::core::EPSILON {
                // Point is in front
                if let Some(ref front) = self.front {
                    return front.contains_point(point);
                }
            } else if signed_distance < -crate::core::EPSILON {
                // Point is in back
                if let Some(ref back) = self.back {
                    return back.contains_point(point);
                }
            }
            // Point is on the plane - check if it's inside any polygon
            for polygon in &self.polygons {
                if self.point_in_polygon(point, polygon) {
                    return true;
                }
            }
        }
        false
    }

    fn bounding_box(&self) -> Option<crate::spatial::traits::Aabb> {
        let all_polys = self.all_polygons();
        crate::spatial::traits::Aabb::from_polygons(&all_polys)
    }

    fn statistics(&self) -> crate::spatial::traits::SpatialStatistics {
        crate::spatial::traits::SpatialStatistics {
            node_count: self.node_count(),
            max_depth: self.depth(),
            polygon_count: self.polygon_count(),
            memory_usage_bytes: self.memory_usage(),
        }
    }
}
