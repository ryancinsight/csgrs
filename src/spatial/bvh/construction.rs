//! BVH construction algorithms optimized for ray tracing performance
//!
//! This module implements various BVH construction algorithms including
//! SAH (Surface Area Heuristic), binned SAH, and median split approaches.

use super::core::Node;
use super::config::{BVHConfig, ConstructionAlgorithm};
use crate::geometry::Polygon;
use crate::spatial::traits::Aabb;
use crate::core::float_types::Real;

/// Primitive wrapper for construction algorithms
#[derive(Debug, Clone)]
struct Primitive<S: Clone> {
    polygon: Polygon<S>,
    bounds: Aabb,
    centroid: nalgebra::Point3<Real>,
}

impl<S: Clone> Node<S> {
    /// Build BVH using the configured construction algorithm
    ///
    /// This is the main entry point for BVH construction, dispatching to
    /// the appropriate algorithm based on configuration.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::{Node, BVHConfig, ConstructionAlgorithm};
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let polygons = vec![polygon];
    ///
    /// let config = BVHConfig {
    ///     construction_algorithm: ConstructionAlgorithm::SAH,
    ///     ..Default::default()
    /// };
    ///
    /// let bvh = Node::build_bvh_with_algorithm(&polygons, &config);
    /// assert_eq!(bvh.polygon_count(), 1);
    /// ```
    pub fn build_bvh_with_algorithm(polygons: &[Polygon<S>], config: &BVHConfig) -> Self {
        if polygons.is_empty() {
            return Self::with_config(config.clone());
        }
        
        // Convert polygons to primitives with precomputed bounds and centroids
        let primitives: Vec<Primitive<S>> = polygons.iter()
            .filter_map(|polygon| {
                if let Some(bounds) = crate::spatial::utils::polygon_bounds(polygon) {
                    let centroid = bounds.center();
                    Some(Primitive {
                        polygon: polygon.clone(),
                        bounds,
                        centroid,
                    })
                } else {
                    None
                }
            })
            .collect();
        
        if primitives.is_empty() {
            return Self::with_config(config.clone());
        }
        
        // Dispatch to appropriate construction algorithm
        match config.construction_algorithm {
            ConstructionAlgorithm::Median => {
                Self::build_median_split(&primitives, config, 0)
            },
            ConstructionAlgorithm::SAH => {
                Self::build_sah(&primitives, config, 0)
            },
            ConstructionAlgorithm::BinnedSAH => {
                Self::build_binned_sah(&primitives, config, 0)
            },
            ConstructionAlgorithm::SpatialSAH => {
                Self::build_spatial_sah(&primitives, config, 0)
            },
        }
    }
    
    /// Build BVH using median split algorithm (fastest construction)
    ///
    /// This algorithm splits primitives at the median along the longest axis,
    /// providing O(n log n) construction time with reasonable quality.
    fn build_median_split(primitives: &[Primitive<S>], config: &BVHConfig, depth: usize) -> Self {
        let mut node = Self::with_config(config.clone());
        node.level = depth;
        
        // Termination criteria
        if primitives.len() <= config.max_polygons_per_leaf || depth >= config.max_depth {
            // Create leaf node
            node.is_leaf = true;
            node.polygons = primitives.iter().map(|p| p.polygon.clone()).collect();
            node.update_bounding_volume();
            return node;
        }
        
        // Calculate bounding box of all primitive centroids
        let centroid_bounds = Self::calculate_centroid_bounds(primitives);
        
        // Find longest axis
        let size = centroid_bounds.max - centroid_bounds.min;
        let axis = if size.x >= size.y && size.x >= size.z {
            0 // X axis
        } else if size.y >= size.z {
            1 // Y axis
        } else {
            2 // Z axis
        };
        
        node.split_axis = Some(axis);
        
        // Sort primitives by centroid along chosen axis
        let mut sorted_primitives = primitives.to_vec();
        sorted_primitives.sort_by(|a, b| {
            let a_coord = match axis {
                0 => a.centroid.x,
                1 => a.centroid.y,
                _ => a.centroid.z,
            };
            let b_coord = match axis {
                0 => b.centroid.x,
                1 => b.centroid.y,
                _ => b.centroid.z,
            };
            a_coord.partial_cmp(&b_coord).unwrap()
        });
        
        // Split at median
        let mid = sorted_primitives.len() / 2;
        let (left_primitives, right_primitives) = sorted_primitives.split_at(mid);
        
        // Recursively build children
        let left_child = Self::build_median_split(left_primitives, config, depth + 1);
        let right_child = Self::build_median_split(right_primitives, config, depth + 1);
        
        node.is_leaf = false;
        node.children = Some((Box::new(left_child), Box::new(right_child)));
        node.update_bounding_volume();
        
        node
    }
    
    /// Build BVH using Surface Area Heuristic (optimal ray tracing quality)
    ///
    /// This algorithm minimizes the expected ray traversal cost by choosing
    /// splits that minimize the SAH cost function.
    fn build_sah(primitives: &[Primitive<S>], config: &BVHConfig, depth: usize) -> Self {
        let mut node = Self::with_config(config.clone());
        node.level = depth;
        
        // Termination criteria
        if primitives.len() <= config.max_polygons_per_leaf || depth >= config.max_depth {
            // Create leaf node
            node.is_leaf = true;
            node.polygons = primitives.iter().map(|p| p.polygon.clone()).collect();
            node.update_bounding_volume();
            return node;
        }
        
        // Find best split using SAH
        let best_split = Self::find_best_sah_split(primitives, config);
        
        if let Some((axis, split_pos)) = best_split {
            node.split_axis = Some(axis);
            
            // Sort primitives by centroid along chosen axis
            let mut sorted_primitives = primitives.to_vec();
            sorted_primitives.sort_by(|a, b| {
                let a_coord = match axis {
                    0 => a.centroid.x,
                    1 => a.centroid.y,
                    _ => a.centroid.z,
                };
                let b_coord = match axis {
                    0 => b.centroid.x,
                    1 => b.centroid.y,
                    _ => b.centroid.z,
                };
                a_coord.partial_cmp(&b_coord).unwrap()
            });
            
            // Split at optimal position
            let (left_primitives, right_primitives) = sorted_primitives.split_at(split_pos);
            
            // Recursively build children
            let left_child = Self::build_sah(left_primitives, config, depth + 1);
            let right_child = Self::build_sah(right_primitives, config, depth + 1);
            
            node.is_leaf = false;
            node.children = Some((Box::new(left_child), Box::new(right_child)));
            node.update_bounding_volume();
        } else {
            // No good split found, create leaf
            node.is_leaf = true;
            node.polygons = primitives.iter().map(|p| p.polygon.clone()).collect();
            node.update_bounding_volume();
        }
        
        node
    }
    
    /// Build BVH using binned SAH (fast approximation of SAH)
    ///
    /// This algorithm approximates SAH by dividing the space into bins
    /// and evaluating splits only at bin boundaries.
    fn build_binned_sah(primitives: &[Primitive<S>], config: &BVHConfig, depth: usize) -> Self {
        let mut node = Self::with_config(config.clone());
        node.level = depth;
        
        // Termination criteria
        if primitives.len() <= config.max_polygons_per_leaf || depth >= config.max_depth {
            // Create leaf node
            node.is_leaf = true;
            node.polygons = primitives.iter().map(|p| p.polygon.clone()).collect();
            node.update_bounding_volume();
            return node;
        }
        
        // Find best split using binned SAH
        let best_split = Self::find_best_binned_sah_split(primitives, config);
        
        if let Some((axis, split_coord)) = best_split {
            node.split_axis = Some(axis);
            
            // Partition primitives based on split coordinate
            let mut left_primitives = Vec::new();
            let mut right_primitives = Vec::new();
            
            for primitive in primitives {
                let coord = match axis {
                    0 => primitive.centroid.x,
                    1 => primitive.centroid.y,
                    _ => primitive.centroid.z,
                };
                
                if coord <= split_coord {
                    left_primitives.push(primitive.clone());
                } else {
                    right_primitives.push(primitive.clone());
                }
            }
            
            // Ensure both sides have primitives
            if left_primitives.is_empty() || right_primitives.is_empty() {
                // Fallback to median split
                return Self::build_median_split(primitives, config, depth);
            }
            
            // Recursively build children
            let left_child = Self::build_binned_sah(&left_primitives, config, depth + 1);
            let right_child = Self::build_binned_sah(&right_primitives, config, depth + 1);
            
            node.is_leaf = false;
            node.children = Some((Box::new(left_child), Box::new(right_child)));
            node.update_bounding_volume();
        } else {
            // No good split found, create leaf
            node.is_leaf = true;
            node.polygons = primitives.iter().map(|p| p.polygon.clone()).collect();
            node.update_bounding_volume();
        }
        
        node
    }
    
    /// Build BVH using spatial SAH with primitive splitting
    ///
    /// This algorithm allows splitting large primitives for highest quality
    /// ray tracing performance at the cost of increased construction time.
    fn build_spatial_sah(primitives: &[Primitive<S>], config: &BVHConfig, depth: usize) -> Self {
        // For now, fall back to regular SAH
        // TODO: Implement spatial splitting for primitives
        Self::build_sah(primitives, config, depth)
    }
    
    /// Calculate bounding box of primitive centroids
    fn calculate_centroid_bounds(primitives: &[Primitive<S>]) -> Aabb {
        if primitives.is_empty() {
            return Aabb::new(
                nalgebra::Point3::new(0.0, 0.0, 0.0),
                nalgebra::Point3::new(0.0, 0.0, 0.0)
            );
        }
        
        let mut min = primitives[0].centroid;
        let mut max = primitives[0].centroid;
        
        for primitive in primitives.iter().skip(1) {
            min.x = min.x.min(primitive.centroid.x);
            min.y = min.y.min(primitive.centroid.y);
            min.z = min.z.min(primitive.centroid.z);
            max.x = max.x.max(primitive.centroid.x);
            max.y = max.y.max(primitive.centroid.y);
            max.z = max.z.max(primitive.centroid.z);
        }
        
        Aabb::new(min, max)
    }
    
    /// Find best split using exact SAH evaluation
    fn find_best_sah_split(primitives: &[Primitive<S>], config: &BVHConfig) -> Option<(usize, usize)> {
        let mut best_cost = Real::INFINITY;
        let mut best_axis = None;
        let mut best_split = None;
        
        // Try each axis
        for axis in 0..3 {
            // Sort primitives by centroid along this axis
            let mut sorted_primitives = primitives.to_vec();
            sorted_primitives.sort_by(|a, b| {
                let a_coord = match axis {
                    0 => a.centroid.x,
                    1 => a.centroid.y,
                    _ => a.centroid.z,
                };
                let b_coord = match axis {
                    0 => b.centroid.x,
                    1 => b.centroid.y,
                    _ => b.centroid.z,
                };
                a_coord.partial_cmp(&b_coord).unwrap()
            });
            
            // Try each possible split position
            for split_pos in 1..sorted_primitives.len() {
                let left_primitives = &sorted_primitives[..split_pos];
                let right_primitives = &sorted_primitives[split_pos..];
                
                let cost = Self::calculate_sah_cost(left_primitives, right_primitives, config);
                
                if cost < best_cost {
                    best_cost = cost;
                    best_axis = Some(axis);
                    best_split = Some(split_pos);
                }
            }
        }
        
        if let (Some(axis), Some(split)) = (best_axis, best_split) {
            Some((axis, split))
        } else {
            None
        }
    }
    
    /// Find best split using binned SAH approximation
    fn find_best_binned_sah_split(primitives: &[Primitive<S>], config: &BVHConfig) -> Option<(usize, Real)> {
        let centroid_bounds = Self::calculate_centroid_bounds(primitives);
        let mut best_cost = Real::INFINITY;
        let mut best_axis = None;
        let mut best_split_coord = None;
        
        // Try each axis
        for axis in 0..3 {
            let axis_size = match axis {
                0 => centroid_bounds.max.x - centroid_bounds.min.x,
                1 => centroid_bounds.max.y - centroid_bounds.min.y,
                _ => centroid_bounds.max.z - centroid_bounds.min.z,
            };
            
            if axis_size <= 0.0 {
                continue; // Skip degenerate axis
            }
            
            // Create bins
            let bin_size = axis_size / config.sah_bins as Real;
            
            // Try each bin boundary as a split
            for bin in 1..config.sah_bins {
                let split_coord = match axis {
                    0 => centroid_bounds.min.x + bin as Real * bin_size,
                    1 => centroid_bounds.min.y + bin as Real * bin_size,
                    _ => centroid_bounds.min.z + bin as Real * bin_size,
                };
                
                // Partition primitives
                let mut left_primitives = Vec::new();
                let mut right_primitives = Vec::new();
                
                for primitive in primitives {
                    let coord = match axis {
                        0 => primitive.centroid.x,
                        1 => primitive.centroid.y,
                        _ => primitive.centroid.z,
                    };
                    
                    if coord <= split_coord {
                        left_primitives.push(primitive.clone());
                    } else {
                        right_primitives.push(primitive.clone());
                    }
                }
                
                if left_primitives.is_empty() || right_primitives.is_empty() {
                    continue; // Skip degenerate splits
                }
                
                let cost = Self::calculate_sah_cost(&left_primitives, &right_primitives, config);
                
                if cost < best_cost {
                    best_cost = cost;
                    best_axis = Some(axis);
                    best_split_coord = Some(split_coord);
                }
            }
        }
        
        if let (Some(axis), Some(split_coord)) = (best_axis, best_split_coord) {
            Some((axis, split_coord))
        } else {
            None
        }
    }
    
    /// Calculate SAH cost for a potential split
    fn calculate_sah_cost(
        left_primitives: &[Primitive<S>], 
        right_primitives: &[Primitive<S>], 
        config: &BVHConfig
    ) -> Real {
        if left_primitives.is_empty() || right_primitives.is_empty() {
            return Real::INFINITY;
        }
        
        // Calculate bounding boxes for left and right partitions
        let left_bounds = Self::calculate_primitive_bounds(left_primitives);
        let right_bounds = Self::calculate_primitive_bounds(right_primitives);
        
        // Calculate surface areas
        let left_sa = Self::surface_area(&left_bounds);
        let right_sa = Self::surface_area(&right_bounds);
        
        // Calculate total surface area (union of left and right)
        let total_bounds = crate::spatial::utils::merge_bounds(&left_bounds, &right_bounds);
        let total_sa = Self::surface_area(&total_bounds);
        
        if total_sa <= 0.0 {
            return Real::INFINITY;
        }
        
        // SAH cost function
        let traversal_cost = config.sah_traversal_cost;
        let intersection_cost = config.sah_intersection_cost;
        
        traversal_cost + 
        intersection_cost * (
            (left_sa / total_sa) * left_primitives.len() as Real +
            (right_sa / total_sa) * right_primitives.len() as Real
        )
    }
    
    /// Calculate bounding box for a set of primitives
    fn calculate_primitive_bounds(primitives: &[Primitive<S>]) -> Aabb {
        if primitives.is_empty() {
            return Aabb::new(
                nalgebra::Point3::new(0.0, 0.0, 0.0),
                nalgebra::Point3::new(0.0, 0.0, 0.0)
            );
        }
        
        let bounds: Vec<Aabb> = primitives.iter().map(|p| p.bounds.clone()).collect();
        crate::spatial::utils::bounds_union(&bounds)
    }
    
    /// Calculate surface area of an AABB
    fn surface_area(bounds: &Aabb) -> Real {
        let size = bounds.max - bounds.min;
        2.0 * (size.x * size.y + size.y * size.z + size.z * size.x)
    }
}
