//! Octree spatial query operations

use super::core::Node;
use crate::geometry::Polygon;
use crate::core::float_types::Real;
use crate::spatial::traits::Aabb;
use nalgebra::Point3;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Find all polygons within a given volume (bounding box)
    pub fn volume_query(&self, query_bounds: &Aabb) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.volume_query_recursive(query_bounds, &mut results);
        results
    }

    /// Parallel version of volume query
    #[cfg(feature = "parallel")]
    pub fn volume_query_parallel(&self, query_bounds: &Aabb) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.volume_query_recursive_parallel(query_bounds, &mut results);
        results
    }

    /// Recursive volume query implementation
    fn volume_query_recursive<'a>(&'a self, query_bounds: &Aabb, results: &mut Vec<&'a Polygon<S>>) {
        // Check if this node's bounds intersect with query bounds
        if !self.bounds.intersects(query_bounds) {
            return;
        }

        // Check polygons in this node
        for polygon in &self.polygons {
            if self.polygon_intersects_bounds(polygon, query_bounds) {
                results.push(polygon);
            }
        }

        // Recursively search children
        for child in &self.children {
            if let Some(child_node) = child {
                child_node.volume_query_recursive(query_bounds, results);
            }
        }
    }

    /// Parallel recursive volume query implementation
    #[cfg(feature = "parallel")]
    fn volume_query_recursive_parallel<'a>(&'a self, query_bounds: &Aabb, results: &mut Vec<&'a Polygon<S>>) {
        // Check if this node's bounds intersect with query bounds
        if !self.bounds.intersects(query_bounds) {
            return;
        }

        // Check polygons in this node
        for polygon in &self.polygons {
            if self.polygon_intersects_bounds(polygon, query_bounds) {
                results.push(polygon);
            }
        }

        // For large subtrees, search children in parallel
        if self.polygon_count() > 100 {
            let active_children: Vec<_> = self.children
                .iter()
                .filter_map(|child| child.as_ref())
                .collect();

            if active_children.len() >= 2 {
                // Process children in parallel pairs
                for chunk in active_children.chunks(2) {
                    if chunk.len() == 2 {
                        let (mut left_results, mut right_results) = join(
                            || {
                                let mut res = Vec::new();
                                chunk[0].volume_query_recursive(query_bounds, &mut res);
                                res
                            },
                            || {
                                let mut res = Vec::new();
                                chunk[1].volume_query_recursive(query_bounds, &mut res);
                                res
                            },
                        );
                        results.append(&mut left_results);
                        results.append(&mut right_results);
                    } else {
                        chunk[0].volume_query_recursive(query_bounds, results);
                    }
                }
            } else {
                // Single child or no children
                for child in &self.children {
                    if let Some(child_node) = child {
                        child_node.volume_query_recursive(query_bounds, results);
                    }
                }
            }
        } else {
            // Use sequential for smaller subtrees
            for child in &self.children {
                if let Some(child_node) = child {
                    child_node.volume_query_recursive(query_bounds, results);
                }
            }
        }
    }

    /// Level-of-detail query based on distance from viewpoint
    pub fn level_of_detail(&self, viewpoint: &Point3<Real>, detail_threshold: Real) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.level_of_detail_recursive(viewpoint, detail_threshold, &mut results);
        results
    }

    /// Parallel version of level-of-detail query
    #[cfg(feature = "parallel")]
    pub fn level_of_detail_parallel(&self, viewpoint: &Point3<Real>, detail_threshold: Real) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.level_of_detail_recursive_parallel(viewpoint, detail_threshold, &mut results);
        results
    }

    /// Recursive level-of-detail implementation
    fn level_of_detail_recursive<'a>(
        &'a self,
        viewpoint: &Point3<Real>,
        detail_threshold: Real,
        results: &mut Vec<&'a Polygon<S>>,
    ) {
        // Calculate distance from viewpoint to this node's center
        let node_center = self.bounds.center();
        let distance = (viewpoint - node_center).norm();

        // Calculate appropriate level of detail based on distance and node size
        let node_size = self.bounds.size().norm();
        let detail_ratio = node_size / distance;

        // If we're far enough away, use lower detail (fewer polygons)
        if detail_ratio < detail_threshold && self.is_subdivided {
            // Use a representative subset of polygons from this subtree
            self.collect_representative_polygons(results, (detail_ratio * 10.0) as usize);
            return;
        }

        // Include polygons from this node
        for polygon in &self.polygons {
            results.push(polygon);
        }

        // Recursively process children
        for child in &self.children {
            if let Some(child_node) = child {
                child_node.level_of_detail_recursive(viewpoint, detail_threshold, results);
            }
        }
    }

    /// Parallel recursive level-of-detail implementation
    #[cfg(feature = "parallel")]
    fn level_of_detail_recursive_parallel<'a>(
        &'a self,
        viewpoint: &Point3<Real>,
        detail_threshold: Real,
        results: &mut Vec<&'a Polygon<S>>,
    ) {
        // Calculate distance from viewpoint to this node's center
        let node_center = self.bounds.center();
        let distance = (viewpoint - node_center).norm();

        // Calculate appropriate level of detail based on distance and node size
        let node_size = self.bounds.size().norm();
        let detail_ratio = node_size / distance;

        // If we're far enough away, use lower detail (fewer polygons)
        if detail_ratio < detail_threshold && self.is_subdivided {
            // Use a representative subset of polygons from this subtree
            self.collect_representative_polygons(results, (detail_ratio * 10.0) as usize);
            return;
        }

        // Include polygons from this node
        for polygon in &self.polygons {
            results.push(polygon);
        }

        // Process children in parallel for large subtrees
        if self.polygon_count() > 100 {
            let active_children: Vec<_> = self.children
                .iter()
                .filter_map(|child| child.as_ref())
                .collect();

            if active_children.len() >= 2 {
                for chunk in active_children.chunks(2) {
                    if chunk.len() == 2 {
                        let (mut left_results, mut right_results) = join(
                            || {
                                let mut res = Vec::new();
                                chunk[0].level_of_detail_recursive(viewpoint, detail_threshold, &mut res);
                                res
                            },
                            || {
                                let mut res = Vec::new();
                                chunk[1].level_of_detail_recursive(viewpoint, detail_threshold, &mut res);
                                res
                            },
                        );
                        results.append(&mut left_results);
                        results.append(&mut right_results);
                    } else {
                        chunk[0].level_of_detail_recursive(viewpoint, detail_threshold, results);
                    }
                }
            } else {
                for child in &self.children {
                    if let Some(child_node) = child {
                        child_node.level_of_detail_recursive(viewpoint, detail_threshold, results);
                    }
                }
            }
        } else {
            for child in &self.children {
                if let Some(child_node) = child {
                    child_node.level_of_detail_recursive(viewpoint, detail_threshold, results);
                }
            }
        }
    }

    /// Frustum culling query for rendering optimization
    pub fn frustum_query(&self, frustum_planes: &[crate::geometry::Plane]) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.frustum_query_recursive(frustum_planes, &mut results);
        results
    }

    /// Recursive frustum culling implementation
    fn frustum_query_recursive<'a>(
        &'a self,
        frustum_planes: &[crate::geometry::Plane],
        results: &mut Vec<&'a Polygon<S>>,
    ) {
        // Check if this node's bounds are inside the frustum
        if !self.bounds_in_frustum(&self.bounds, frustum_planes) {
            return;
        }

        // Include polygons from this node
        for polygon in &self.polygons {
            if self.polygon_in_frustum(polygon, frustum_planes) {
                results.push(polygon);
            }
        }

        // Recursively process children
        for child in &self.children {
            if let Some(child_node) = child {
                child_node.frustum_query_recursive(frustum_planes, results);
            }
        }
    }

    /// Collect a representative subset of polygons from this subtree
    fn collect_representative_polygons<'a>(&'a self, results: &mut Vec<&'a Polygon<S>>, max_count: usize) {
        let mut collected = 0;
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Add some polygons from this node
            let take_count = (max_count - collected).min(node.polygons.len());
            for polygon in node.polygons.iter().take(take_count) {
                results.push(polygon);
                collected += 1;
                if collected >= max_count {
                    return;
                }
            }

            // Add children to stack (prioritize by distance or other criteria)
            for child in &node.children {
                if let Some(child_node) = child {
                    stack.push(child_node.as_ref());
                }
            }
        }
    }

    /// Check if a polygon intersects with a bounding box
    fn polygon_intersects_bounds(&self, polygon: &Polygon<S>, bounds: &Aabb) -> bool {
        // Check if any vertex is inside the bounds
        for vertex in &polygon.vertices {
            if bounds.contains_point(&vertex.pos) {
                return true;
            }
        }

        // Check if polygon bounding box intersects query bounds
        if let Some(poly_bounds) = Aabb::from_polygons(&[polygon.clone()]) {
            return poly_bounds.intersects(bounds);
        }

        false
    }

    /// Check if bounding box is inside frustum
    fn bounds_in_frustum(&self, bounds: &Aabb, frustum_planes: &[crate::geometry::Plane]) -> bool {
        // Simplified frustum culling - check if any corner of the bounding box is inside all planes
        let corners = [
            bounds.min,
            Point3::new(bounds.max.x, bounds.min.y, bounds.min.z),
            Point3::new(bounds.min.x, bounds.max.y, bounds.min.z),
            Point3::new(bounds.min.x, bounds.min.y, bounds.max.z),
            Point3::new(bounds.max.x, bounds.max.y, bounds.min.z),
            Point3::new(bounds.max.x, bounds.min.y, bounds.max.z),
            Point3::new(bounds.min.x, bounds.max.y, bounds.max.z),
            bounds.max,
        ];

        for corner in &corners {
            let mut inside_all_planes = true;
            for plane in frustum_planes {
                // Calculate signed distance: d = n·p - n·p0 where p0 is a point on the plane
                let normal = plane.normal();
                let offset = plane.offset();
                let signed_distance = normal.dot(&corner.coords) - offset;
                if signed_distance < 0.0 {
                    inside_all_planes = false;
                    break;
                }
            }
            if inside_all_planes {
                return true;
            }
        }

        false
    }

    /// Check if polygon is inside frustum
    fn polygon_in_frustum(&self, polygon: &Polygon<S>, frustum_planes: &[crate::geometry::Plane]) -> bool {
        // Check if any vertex is inside all frustum planes
        for vertex in &polygon.vertices {
            let mut inside_all_planes = true;
            for plane in frustum_planes {
                // Calculate signed distance: d = n·p - n·p0 where p0 is a point on the plane
                let normal = plane.normal();
                let offset = plane.offset();
                let signed_distance = normal.dot(&vertex.pos.coords) - offset;
                if signed_distance < 0.0 {
                    inside_all_planes = false;
                    break;
                }
            }
            if inside_all_planes {
                return true;
            }
        }

        false
    }
}
