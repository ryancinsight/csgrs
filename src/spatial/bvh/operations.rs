//! BVH ray traversal, collision detection, and dynamic update operations
//!
//! This module implements the core BVH algorithms for ray tracing and dynamic
//! object management, optimized for performance and cache efficiency.

use super::core::Node;
use super::config::BVHConfig;
use crate::geometry::Polygon;
use crate::spatial::traits::{Aabb, Ray, Intersection};
use crate::core::float_types::Real;
use nalgebra::{Point3, Vector3};

impl<S: Clone> Node<S> {
    /// Optimized ray traversal using stack-based iteration
    ///
    /// This method provides the fastest ray traversal for ray tracing applications
    /// by avoiding recursion and using a stack for node traversal.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::{bvh::Node, traits::Ray};
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let bvh = Node::from_polygons(&vec![polygon]);
    ///
    /// let ray = Ray {
    ///     origin: Point3::new(-1.0, 0.5, 0.0),
    ///     direction: Vector3::new(1.0, 0.0, 0.0),
    /// };
    ///
    /// let intersections = bvh.ray_traversal_optimized(&ray);
    /// // Process intersections for rendering
    /// ```
    pub fn ray_traversal_optimized(&self, ray: &Ray) -> Vec<Intersection<S>> {
        let mut intersections = Vec::new();
        let mut stack = Vec::with_capacity(64); // Pre-allocate stack for performance
        
        // Start with root node
        stack.push(self);
        
        while let Some(node) = stack.pop() {
            // Early termination if ray doesn't intersect bounding volume
            if let Some(ref bounds) = node.bounding_volume {
                if !self.ray_aabb_intersection_optimized(ray, bounds) {
                    continue;
                }
            }
            
            if node.is_leaf {
                // Test ray against all polygons in leaf
                for polygon in &node.polygons {
                    if let Some(intersection) = self.ray_polygon_intersection(ray, polygon) {
                        intersections.push(intersection);
                    }
                }
            } else if let Some((ref left, ref right)) = node.children {
                // Add children to stack for traversal
                // Order children by ray direction for better cache performance
                if self.should_traverse_left_first(ray, left, right) {
                    stack.push(right);
                    stack.push(left);
                } else {
                    stack.push(left);
                    stack.push(right);
                }
            }
        }
        
        // Sort intersections by distance for closest-hit queries
        intersections.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        intersections
    }
    
    /// Find closest ray intersection (early termination optimization)
    ///
    /// This method is optimized for primary ray casting where only the closest
    /// intersection is needed, allowing early termination for better performance.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::{bvh::Node, traits::Ray};
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let bvh = Node::from_polygons(&vec![polygon]);
    ///
    /// let ray = Ray {
    ///     origin: Point3::new(0.5, 0.3, -1.0),
    ///     direction: Vector3::new(0.0, 0.0, 1.0),
    /// };
    ///
    /// if let Some(closest) = bvh.closest_ray_intersection(&ray) {
    ///     println!("Hit at distance: {}", closest.distance);
    /// }
    /// ```
    pub fn closest_ray_intersection(&self, ray: &Ray) -> Option<Intersection<S>> {
        let mut closest_intersection: Option<Intersection<S>> = None;
        let mut closest_distance = Real::INFINITY;
        
        self.closest_ray_intersection_recursive(ray, &mut closest_intersection, &mut closest_distance);
        closest_intersection
    }
    
    /// Refit bounding volumes after object movement (dynamic scenes)
    ///
    /// This method updates bounding volumes without rebuilding the entire BVH,
    /// which is much faster for dynamic scenes with moving objects.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut bvh: Node<i32> = Node::new();
    /// 
    /// // After objects have moved, refit the BVH
    /// bvh.refit_bounding_volumes();
    /// 
    /// // BVH is now updated for the new object positions
    /// ```
    pub fn refit_bounding_volumes(&mut self) {
        self.refit_count += 1;
        
        if self.is_leaf {
            // Update leaf bounding volume based on current polygons
            self.update_bounding_volume();
        } else if let Some((ref mut left, ref mut right)) = self.children {
            // Recursively refit children first
            left.refit_bounding_volumes();
            right.refit_bounding_volumes();
            
            // Update this node's bounding volume
            self.update_bounding_volume();
        }
        
        // Check if BVH quality has degraded and needs rebuilding
        if self.should_rebuild() {
            self.trigger_rebuild();
        }
    }
    
    /// Remove an object from the BVH (dynamic operation)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut bvh: Node<i32> = Node::new();
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, Some(42));
    ///
    /// bvh.insert_object(polygon.clone());
    /// assert_eq!(bvh.polygon_count(), 1);
    ///
    /// let removed = bvh.remove_object_by_metadata(&Some(42));
    /// assert!(removed);
    /// assert_eq!(bvh.polygon_count(), 0);
    /// ```
    pub fn remove_object_by_metadata(&mut self, metadata: &Option<S>) -> bool
    where
        S: PartialEq,
    {
        if self.is_leaf {
            // Try to find and remove the polygon by metadata
            if let Some(pos) = self.polygons.iter().position(|p| &p.metadata == metadata) {
                self.polygons.remove(pos);
                self.update_bounding_volume();
                return true;
            }
            false
        } else if let Some((ref mut left, ref mut right)) = self.children {
            // Search children
            let removed_left = left.remove_object_by_metadata(metadata);
            let removed_right = right.remove_object_by_metadata(metadata);
            
            if removed_left || removed_right {
                self.update_bounding_volume();
                return true;
            }
            false
        } else {
            false
        }
    }
    
    /// Calculate BVH quality metric for rebuild decisions
    ///
    /// Returns a value between 0.0 (poor quality) and 1.0 (excellent quality).
    /// Quality is based on surface area heuristic and tree balance.
    pub fn calculate_quality(&self) -> Real {
        if self.is_leaf {
            return 1.0; // Leaves are always optimal
        }
        
        if let Some((ref left, ref right)) = self.children {
            let left_sa = left.surface_area;
            let right_sa = right.surface_area;
            let total_sa = self.surface_area;
            
            if total_sa <= 0.0 {
                return 0.0;
            }
            
            // SAH-based quality metric
            let sah_cost = (left_sa / total_sa) * left.polygon_count() as Real +
                          (right_sa / total_sa) * right.polygon_count() as Real;
            
            // Balance factor (prefer balanced trees)
            let left_count = left.polygon_count() as Real;
            let right_count = right.polygon_count() as Real;
            let total_count = left_count + right_count;
            
            let balance_factor = if total_count > 0.0 {
                1.0 - ((left_count - right_count).abs() / total_count)
            } else {
                1.0
            };
            
            // Combine SAH cost and balance factor
            let quality = (1.0 / (1.0 + sah_cost)) * balance_factor;
            
            // Recursively consider children quality
            let child_quality = (left.calculate_quality() + right.calculate_quality()) / 2.0;
            
            (quality + child_quality) / 2.0
        } else {
            0.0
        }
    }
    
    /// Optimized ray-AABB intersection with SIMD hints
    #[inline]
    fn ray_aabb_intersection_optimized(&self, ray: &Ray, aabb: &Aabb) -> bool {
        crate::spatial::utils::ray_intersects_aabb(ray, aabb)
    }
    
    /// Ray-polygon intersection test (placeholder for full implementation)
    fn ray_polygon_intersection(&self, ray: &Ray, polygon: &Polygon<S>) -> Option<Intersection<S>> {
        // Placeholder implementation
        // In a full ray tracer, this would perform precise ray-triangle intersection
        if let Some(bounds) = crate::spatial::utils::polygon_bounds(polygon) {
            if crate::spatial::utils::ray_intersects_aabb(ray, &bounds) {
                return Some(Intersection {
                    polygon: polygon.clone(),
                    distance: 1.0, // Placeholder distance
                    point: ray.origin + ray.direction,
                    normal: Vector3::new(0.0, 0.0, 1.0),
                });
            }
        }
        None
    }
    
    /// Determine optimal child traversal order for ray direction
    fn should_traverse_left_first(&self, ray: &Ray, left: &Node<S>, right: &Node<S>) -> bool {
        // Simple heuristic: traverse child closer to ray origin first
        if let (Some(ref left_bounds), Some(ref right_bounds)) = 
            (&left.bounding_volume, &right.bounding_volume) {
            let left_dist = self.ray_aabb_entry_distance(ray, left_bounds);
            let right_dist = self.ray_aabb_entry_distance(ray, right_bounds);
            left_dist <= right_dist
        } else {
            true // Default to left first
        }
    }
    
    /// Calculate ray entry distance to AABB
    fn ray_aabb_entry_distance(&self, ray: &Ray, aabb: &Aabb) -> Real {
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
        tmin.max(0.0)
    }
    
    /// Recursive helper for closest ray intersection
    fn closest_ray_intersection_recursive(
        &self, 
        ray: &Ray, 
        closest: &mut Option<Intersection<S>>, 
        closest_distance: &mut Real
    ) {
        // Early termination if ray doesn't intersect bounding volume
        if let Some(ref bounds) = self.bounding_volume {
            let entry_dist = self.ray_aabb_entry_distance(ray, bounds);
            if entry_dist >= *closest_distance {
                return; // This subtree can't contain closer intersection
            }
            if !self.ray_aabb_intersection_optimized(ray, bounds) {
                return;
            }
        }
        
        if self.is_leaf {
            // Test ray against all polygons in leaf
            for polygon in &self.polygons {
                if let Some(intersection) = self.ray_polygon_intersection(ray, polygon) {
                    if intersection.distance < *closest_distance {
                        *closest_distance = intersection.distance;
                        *closest = Some(intersection);
                    }
                }
            }
        } else if let Some((ref left, ref right)) = self.children {
            // Traverse children in optimal order
            if self.should_traverse_left_first(ray, left, right) {
                left.closest_ray_intersection_recursive(ray, closest, closest_distance);
                right.closest_ray_intersection_recursive(ray, closest, closest_distance);
            } else {
                right.closest_ray_intersection_recursive(ray, closest, closest_distance);
                left.closest_ray_intersection_recursive(ray, closest, closest_distance);
            }
        }
    }
    
    /// Check if BVH should be rebuilt based on quality metrics
    fn should_rebuild(&self) -> bool {
        if self.refit_count >= self.config.max_refits_before_rebuild {
            return true;
        }
        
        let quality = self.calculate_quality();
        quality < self.config.rebuild_quality_threshold
    }
    
    /// Trigger BVH rebuild (placeholder for full implementation)
    fn trigger_rebuild(&mut self) {
        // Placeholder: In a full implementation, this would:
        // 1. Collect all polygons from the current BVH
        // 2. Rebuild the BVH using the configured construction algorithm
        // 3. Reset refit count and quality metrics
        self.refit_count = 0;
    }
}
