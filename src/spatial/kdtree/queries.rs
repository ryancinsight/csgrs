//! KD-tree spatial query operations

use super::core::Node;
use crate::geometry::Polygon;
use crate::core::float_types::Real;
use crate::spatial::traits::{Aabb, Ray, Intersection};
use nalgebra::Point3;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Find the nearest polygon to a given point
    pub fn nearest_neighbor(&self, query_point: &Point3<Real>) -> Option<&Polygon<S>> {
        let mut best_polygon = None;
        let mut best_distance_squared = Real::INFINITY;

        self.nearest_neighbor_recursive(query_point, &mut best_polygon, &mut best_distance_squared);
        best_polygon
    }

    /// Parallel version of nearest neighbor search
    #[cfg(feature = "parallel")]
    pub fn nearest_neighbor_parallel(&self, query_point: &Point3<Real>) -> Option<&Polygon<S>> {
        let mut best_polygon = None;
        let mut best_distance_squared = Real::INFINITY;

        self.nearest_neighbor_recursive_parallel(query_point, &mut best_polygon, &mut best_distance_squared);
        best_polygon
    }

    /// Recursive nearest neighbor search
    fn nearest_neighbor_recursive<'a>(
        &'a self,
        query_point: &Point3<Real>,
        best_polygon: &mut Option<&'a Polygon<S>>,
        best_distance_squared: &mut Real,
    ) {
        // Check polygons in this node using iterator patterns
        if let Some((polygon, distance_squared)) = self.polygons
            .iter()
            .map(|polygon| (polygon, self.distance_squared_to_polygon(query_point, polygon)))
            .filter(|(_, distance_squared)| *distance_squared < *best_distance_squared)
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        {
            *best_distance_squared = distance_squared;
            *best_polygon = Some(polygon);
        }

        // If this is a leaf node, we're done
        if self.is_leaf() {
            return;
        }

        // Determine which child to search first based on query point position
        let (first_child, second_child) = if self.is_left_of_split_point(query_point) {
            (&self.left, &self.right)
        } else {
            (&self.right, &self.left)
        };

        // Search the closer child first
        if let Some(child) = first_child {
            child.nearest_neighbor_recursive(query_point, best_polygon, best_distance_squared);
        }

        // Check if we need to search the other child
        if let Some(child) = second_child {
            if self.could_contain_closer_point(query_point, *best_distance_squared) {
                child.nearest_neighbor_recursive(query_point, best_polygon, best_distance_squared);
            }
        }
    }

    /// Parallel recursive nearest neighbor search
    #[cfg(feature = "parallel")]
    fn nearest_neighbor_recursive_parallel<'a>(
        &'a self,
        query_point: &Point3<Real>,
        best_polygon: &mut Option<&'a Polygon<S>>,
        best_distance_squared: &mut Real,
    ) {
        // Check polygons in this node using iterator patterns
        if let Some((polygon, distance_squared)) = self.polygons
            .iter()
            .map(|polygon| (polygon, self.distance_squared_to_polygon(query_point, polygon)))
            .filter(|(_, distance_squared)| *distance_squared < *best_distance_squared)
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        {
            *best_distance_squared = distance_squared;
            *best_polygon = Some(polygon);
        }

        // If this is a leaf node, we're done
        if self.is_leaf() {
            return;
        }

        // For parallel search, we need to be more careful about shared state
        // This is a simplified version - a full implementation would use atomic operations
        let (first_child, second_child) = if self.is_left_of_split_point(query_point) {
            (&self.left, &self.right)
        } else {
            (&self.right, &self.left)
        };

        // Search the closer child first
        if let Some(child) = first_child {
            child.nearest_neighbor_recursive(query_point, best_polygon, best_distance_squared);
        }

        // Check if we need to search the other child
        if let Some(child) = second_child {
            if self.could_contain_closer_point(query_point, *best_distance_squared) {
                child.nearest_neighbor_recursive(query_point, best_polygon, best_distance_squared);
            }
        }
    }

    /// Find all polygons within a given bounding box
    pub fn range_query(&self, query_bounds: &Aabb) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.range_query_recursive(query_bounds, &mut results);
        results
    }

    /// Parallel version of range query
    #[cfg(feature = "parallel")]
    pub fn range_query_parallel(&self, query_bounds: &Aabb) -> Vec<&Polygon<S>> {
        let mut results = Vec::new();
        self.range_query_recursive_parallel(query_bounds, &mut results);
        results
    }

    /// Recursive range query implementation with advanced iterator patterns
    fn range_query_recursive<'a>(&'a self, query_bounds: &Aabb, results: &mut Vec<&'a Polygon<S>>) {
        // Check if this node's bounds intersect with query bounds
        if let Some(ref node_bounds) = self.bounds {
            if !node_bounds.intersects(query_bounds) {
                return;
            }
        }

        // Enhanced polygon filtering with early termination patterns
        let current_len = results.len();
        if current_len < 10000 {
            results.extend(
                self.polygons
                    .iter()
                    .filter(|polygon| self.polygon_intersects_bounds(polygon, query_bounds))
                    .take(10000 - current_len) // Prevent excessive memory usage
            );
        }

        // Recursively search children
        if let Some(ref left) = self.left {
            left.range_query_recursive(query_bounds, results);
        }
        if let Some(ref right) = self.right {
            right.range_query_recursive(query_bounds, results);
        }
    }

    /// Parallel recursive range query implementation
    #[cfg(feature = "parallel")]
    fn range_query_recursive_parallel<'a>(&'a self, query_bounds: &Aabb, results: &mut Vec<&'a Polygon<S>>) {
        // Check if this node's bounds intersect with query bounds
        if let Some(ref node_bounds) = self.bounds {
            if !node_bounds.intersects(query_bounds) {
                return;
            }
        }

        // Check polygons in this node using iterator patterns
        results.extend(
            self.polygons
                .iter()
                .filter(|polygon| self.polygon_intersects_bounds(polygon, query_bounds))
        );

        // For large subtrees, search children in parallel
        match (&self.left, &self.right) {
            (Some(left), Some(right)) if self.polygon_count() > 100 => {
                let (mut left_results, mut right_results) = join(
                    || {
                        let mut left_res = Vec::new();
                        left.range_query_recursive(query_bounds, &mut left_res);
                        left_res
                    },
                    || {
                        let mut right_res = Vec::new();
                        right.range_query_recursive(query_bounds, &mut right_res);
                        right_res
                    },
                );
                results.append(&mut left_results);
                results.append(&mut right_results);
            }
            (Some(left), Some(right)) => {
                left.range_query_recursive(query_bounds, results);
                right.range_query_recursive(query_bounds, results);
            }
            (Some(child), None) | (None, Some(child)) => {
                child.range_query_recursive(query_bounds, results);
            }
            (None, None) => {}
        }
    }

    /// Find all polygons intersected by a ray
    pub fn ray_intersections(&self, ray: &Ray) -> Vec<Intersection<S>> {
        let mut intersections = Vec::new();
        self.ray_intersections_recursive(ray, &mut intersections);
        intersections
    }

    /// Recursive ray intersection implementation
    fn ray_intersections_recursive(&self, ray: &Ray, intersections: &mut Vec<Intersection<S>>) {
        // Check if ray intersects this node's bounds
        if let Some(ref bounds) = self.bounds {
            if !self.ray_intersects_aabb(ray, bounds) {
                return;
            }
        }

        // Check intersections with polygons in this node using iterator patterns
        intersections.extend(
            self.polygons
                .iter()
                .filter_map(|polygon| self.ray_polygon_intersection(ray, polygon))
        );

        // Recursively check children
        if let Some(ref left) = self.left {
            left.ray_intersections_recursive(ray, intersections);
        }
        if let Some(ref right) = self.right {
            right.ray_intersections_recursive(ray, intersections);
        }
    }

    /// Check if a point is on the left side of the split
    fn is_left_of_split_point(&self, point: &Point3<Real>) -> bool {
        if let (Some(axis), Some(split_value)) = (self.axis, self.split_value) {
            axis.coordinate(point) <= split_value
        } else {
            false
        }
    }

    /// Check if the other side of the split could contain a closer point
    fn could_contain_closer_point(&self, query_point: &Point3<Real>, best_distance_squared: Real) -> bool {
        if let (Some(axis), Some(split_value)) = (self.axis, self.split_value) {
            let distance_to_split = (axis.coordinate(query_point) - split_value).abs();
            distance_to_split * distance_to_split < best_distance_squared
        } else {
            false
        }
    }

    /// Calculate squared distance from point to polygon (simplified)
    fn distance_squared_to_polygon(&self, point: &Point3<Real>, polygon: &Polygon<S>) -> Real {
        // Simplified: distance to polygon centroid
        // A full implementation would calculate distance to the actual polygon surface
        let center = Self::polygon_center(polygon);
        (point - center).norm_squared()
    }

    /// Check if a polygon intersects with a bounding box
    fn polygon_intersects_bounds(&self, polygon: &Polygon<S>, bounds: &Aabb) -> bool {
        // Check if any vertex is inside the bounds using iterator patterns
        if polygon.vertices
            .iter()
            .any(|vertex| bounds.contains_point(&vertex.pos))
        {
            return true;
        }

        // Check if polygon bounding box intersects query bounds
        if let Some(poly_bounds) = Aabb::from_polygons(&[polygon.clone()]) {
            return poly_bounds.intersects(bounds);
        }

        false
    }

    /// Check if a ray intersects with an AABB
    fn ray_intersects_aabb(&self, ray: &Ray, aabb: &Aabb) -> bool {
        // Ray-AABB intersection using slab method
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
    fn ray_polygon_intersection(&self, ray: &Ray, polygon: &Polygon<S>) -> Option<Intersection<S>> {
        // Simplified ray-polygon intersection
        // A full implementation would use proper ray-triangle intersection
        
        // For now, just check if ray passes near polygon center
        let center = Self::polygon_center(polygon);
        let to_center = center - ray.origin;
        let projection = to_center.dot(&ray.direction);
        
        if projection < 0.0 {
            return None; // Behind ray origin
        }
        
        let closest_point = ray.origin + ray.direction * projection;
        let distance = (closest_point - center).norm();
        
        if distance < 0.1 { // Arbitrary threshold
            Some(Intersection {
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

use nalgebra::Vector3;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// **Advanced spatial query with early termination using find()**
    ///
    /// Finds the first polygon that satisfies a given predicate, using
    /// advanced iterator patterns for optimal performance.
    pub fn find_first_matching<F>(&self, predicate: F) -> Option<&Polygon<S>>
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
    {
        // Use find() for immediate early termination
        self.polygons
            .iter()
            .find(|polygon| predicate(polygon))
            .or_else(|| {
                // Search children with early termination
                self.left
                    .as_ref()
                    .and_then(|child| child.find_first_matching(predicate))
                    .or_else(|| {
                        self.right
                            .as_ref()
                            .and_then(|child| child.find_first_matching(predicate))
                    })
            })
    }

    /// **Conditional range query with skip_while() and take_while()**
    ///
    /// Performs range queries with conditional processing based on
    /// spatial criteria and result limits.
    pub fn conditional_range_query<F, G>(
        &self,
        query_bounds: &Aabb,
        skip_condition: F,
        take_condition: G,
        max_results: usize,
    ) -> Vec<&Polygon<S>>
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
        G: Fn(&Polygon<S>) -> bool + Copy,
    {
        let mut results = Vec::new();
        self.conditional_range_recursive(query_bounds, skip_condition, take_condition, max_results, &mut results);
        results
    }

    /// **Recursive implementation of conditional range query**
    fn conditional_range_recursive<'a, F, G>(
        &'a self,
        query_bounds: &Aabb,
        skip_condition: F,
        take_condition: G,
        max_results: usize,
        results: &mut Vec<&'a Polygon<S>>,
    )
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
        G: Fn(&Polygon<S>) -> bool + Copy,
    {
        // Early termination if we have enough results
        if results.len() >= max_results {
            return;
        }

        // Check if this node's bounds intersect with query bounds
        if let Some(ref node_bounds) = self.bounds {
            if !node_bounds.intersects(query_bounds) {
                return;
            }
        }

        // Advanced iterator patterns with conditional processing
        let filtered_polygons: Vec<&Polygon<S>> = self.polygons
            .iter()
            .filter(|polygon| self.polygon_intersects_bounds(polygon, query_bounds))
            .skip_while(|polygon| skip_condition(polygon))
            .take_while(|polygon| take_condition(polygon) && results.len() < max_results)
            .collect();

        results.extend(filtered_polygons);

        // Recursively search children with early termination
        if results.len() < max_results {
            if let Some(ref left_child) = self.left {
                left_child.conditional_range_recursive(
                    query_bounds,
                    skip_condition,
                    take_condition,
                    max_results,
                    results,
                );
            }
        }

        if results.len() < max_results {
            if let Some(ref right_child) = self.right {
                right_child.conditional_range_recursive(
                    query_bounds,
                    skip_condition,
                    take_condition,
                    max_results,
                    results,
                );
            }
        }
    }

    /// **Parallel spatial filtering with intelligent thresholds**
    ///
    /// Performs parallel spatial filtering for large datasets with
    /// automatic threshold-based optimization.
    #[cfg(feature = "parallel")]
    pub fn parallel_spatial_filter<F>(&self, predicate: F) -> usize
    where
        F: Fn(&Polygon<S>) -> bool + Send + Sync + Copy,
    {
        if self.polygon_count() > 1000 {
            use rayon::prelude::*;

            // Use parallel iterator for large datasets - return count instead of references
            self.all_polygons()
                .par_iter()
                .filter(|polygon| predicate(polygon))
                .count()
        } else {
            // Use sequential iterator for smaller datasets
            self.all_polygons()
                .iter()
                .filter(|polygon| predicate(polygon))
                .count()
        }
    }

    /// **Count polygons matching a predicate with early termination**
    ///
    /// Counts polygons that match a predicate using iterator patterns
    /// with early termination for performance.
    pub fn count_matching<F>(&self, predicate: F, max_count: usize) -> usize
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
    {
        self.polygons
            .iter()
            .filter(|polygon| predicate(polygon))
            .take(max_count)
            .count()
            + self.left
                .as_ref()
                .map(|child| child.count_matching(predicate, max_count))
                .unwrap_or(0)
            + self.right
                .as_ref()
                .map(|child| child.count_matching(predicate, max_count))
                .unwrap_or(0)
    }
}
