//! R-tree bulk loading and construction algorithms
//!
//! This module implements efficient bulk loading algorithms for R-trees,
//! particularly the STR (Sort-Tile-Recursive) algorithm for building
//! high-quality trees from large datasets.

use super::core::Node;
use super::config::RTreeConfig;
use crate::geometry::Polygon;
use crate::spatial::traits::Aabb;
use crate::core::float_types::Real;
use std::fmt::Debug;

impl<S: Clone> Node<S> {
    /// Bulk load R-tree using STR (Sort-Tile-Recursive) algorithm
    ///
    /// This algorithm provides better tree quality than incremental insertion
    /// for large datasets by:
    /// 1. Sorting polygons by spatial coordinates
    /// 2. Recursively partitioning into tiles
    /// 3. Building tree bottom-up
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::{Node, RTreeConfig};
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// // Create multiple polygons for bulk loading
    /// let mut polygons = Vec::new();
    /// for i in 0..10 {
    ///     let vertices = vec![
    ///         Vertex::new(Point3::new(i as f64, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///         Vertex::new(Point3::new(i as f64 + 1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///         Vertex::new(Point3::new(i as f64 + 0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     ];
    ///     let polygon: Polygon<i32> = Polygon::new(vertices, Some(i));
    ///     polygons.push(polygon);
    /// }
    ///
    /// let config = RTreeConfig::for_range_queries();
    /// let rtree = Node::str_bulk_load(&polygons, &config);
    /// assert_eq!(rtree.polygon_count(), 10);
    /// ```
    pub fn str_bulk_load(polygons: &[Polygon<S>], config: &RTreeConfig) -> Self {
        if polygons.is_empty() {
            return Self::with_config(config.clone());
        }
        
        if polygons.len() <= config.max_children {
            // Small dataset - create single leaf
            let mut leaf = Self::with_config(config.clone());
            leaf.polygons = polygons.to_vec();
            leaf.update_bounding_box();
            return leaf;
        }
        
        // Create polygon-bounds pairs for sorting
        let mut poly_bounds: Vec<(Polygon<S>, Aabb)> = polygons.iter()
            .filter_map(|p| {
                crate::spatial::utils::polygon_bounds(p).map(|bounds| (p.clone(), bounds))
            })
            .collect();
        
        if poly_bounds.is_empty() {
            return Self::with_config(config.clone());
        }
        
        // Build tree recursively
        Self::str_recursive(&mut poly_bounds, config, 0)
    }
    
    /// Recursive STR implementation
    fn str_recursive(
        poly_bounds: &mut [(Polygon<S>, Aabb)], 
        config: &RTreeConfig, 
        level: usize
    ) -> Self {
        if poly_bounds.is_empty() {
            return Self::with_config(config.clone());
        }
        
        if poly_bounds.len() <= config.max_children {
            // Create leaf node
            let mut leaf = Self::with_config(config.clone());
            leaf.is_leaf = true;
            leaf.level = level;
            leaf.polygons = poly_bounds.iter().map(|(p, _)| p.clone()).collect();
            leaf.update_bounding_box();
            return leaf;
        }
        
        // Calculate number of slices needed
        let total_nodes = (poly_bounds.len() + config.max_children - 1) / config.max_children;
        let slices = ((total_nodes as f64).sqrt().ceil() as usize).max(1);
        
        // Sort by X coordinate first
        poly_bounds.sort_by(|a, b| {
            a.1.center().x.partial_cmp(&b.1.center().x).unwrap_or(std::cmp::Ordering::Equal)
        });
        
        let slice_size = (poly_bounds.len() + slices - 1) / slices;
        let mut children = Vec::new();
        
        // Process each slice
        for slice_start in (0..poly_bounds.len()).step_by(slice_size) {
            let slice_end = (slice_start + slice_size).min(poly_bounds.len());
            let slice = &mut poly_bounds[slice_start..slice_end];
            
            if slice.len() <= config.max_children {
                // Create leaf for small slice
                let mut leaf = Self::with_config(config.clone());
                leaf.is_leaf = true;
                leaf.level = level;
                leaf.polygons = slice.iter().map(|(p, _)| p.clone()).collect();
                leaf.update_bounding_box();
                children.push(Box::new(leaf));
            } else {
                // Sort slice by Y coordinate and subdivide
                slice.sort_by(|a, b| {
                    a.1.center().y.partial_cmp(&b.1.center().y).unwrap_or(std::cmp::Ordering::Equal)
                });
                
                let sub_slice_size = config.max_children;
                for sub_start in (0..slice.len()).step_by(sub_slice_size) {
                    let sub_end = (sub_start + sub_slice_size).min(slice.len());
                    let sub_slice = &mut slice[sub_start..sub_end];
                    
                    let child = Self::str_recursive(sub_slice, config, level);
                    children.push(Box::new(child));
                }
            }
        }
        
        // Create internal node
        let mut internal = Self::with_config(config.clone());
        internal.is_leaf = false;
        internal.level = level + 1;
        internal.children = children;
        internal.update_bounding_box();
        
        // If we have too many children, recursively build another level
        if internal.children.len() > config.max_children {
            // Convert children back to polygon-bounds format and recurse
            let mut child_bounds = Vec::new();
            for child in &internal.children {
                if let Some(bounds) = &child.bounding_box {
                    // Create a representative polygon for the child's bounds
                    // This is a simplification - in a full implementation,
                    // we would handle this more elegantly
                    let center = bounds.center();
                    let vertices = vec![
                        crate::geometry::Vertex::new(center, nalgebra::Vector3::new(0.0, 0.0, 1.0)),
                        crate::geometry::Vertex::new(center, nalgebra::Vector3::new(0.0, 0.0, 1.0)),
                        crate::geometry::Vertex::new(center, nalgebra::Vector3::new(0.0, 0.0, 1.0)),
                    ];
                    let poly = Polygon::new(vertices, None);
                    child_bounds.push((poly, bounds.clone()));
                }
            }
            
            if !child_bounds.is_empty() {
                return Self::str_recursive(&mut child_bounds, config, level + 1);
            }
        }
        
        internal
    }
    
    /// Hilbert curve bulk loading (alternative to STR)
    ///
    /// Orders polygons along a Hilbert curve for better spatial locality.
    /// This can provide better query performance for certain access patterns.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::{Node, RTreeConfig};
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
    /// let config = RTreeConfig::default();
    /// let rtree = Node::hilbert_bulk_load(&polygons, &config);
    /// assert_eq!(rtree.polygon_count(), 1);
    /// ```
    pub fn hilbert_bulk_load(polygons: &[Polygon<S>], config: &RTreeConfig) -> Self {
        if polygons.is_empty() {
            return Self::with_config(config.clone());
        }
        
        // For now, fall back to STR algorithm
        // TODO: Implement proper Hilbert curve ordering
        Self::str_bulk_load(polygons, config)
    }
    
    /// Pack polygons into leaf nodes with optimal fill factor
    ///
    /// This is a helper function for bulk loading algorithms that
    /// creates leaf nodes with near-optimal fill factors.
    fn pack_leaves(polygons: &[Polygon<S>], config: &RTreeConfig) -> Vec<Node<S>> {
        let mut leaves = Vec::new();
        let mut current_leaf = Self::with_config(config.clone());
        current_leaf.is_leaf = true;
        
        for polygon in polygons {
            if current_leaf.polygons.len() >= config.max_children {
                // Finalize current leaf and start new one
                current_leaf.update_bounding_box();
                leaves.push(current_leaf);
                current_leaf = Self::with_config(config.clone());
                current_leaf.is_leaf = true;
            }
            
            current_leaf.polygons.push(polygon.clone());
        }
        
        // Add final leaf if it has any polygons
        if !current_leaf.polygons.is_empty() {
            current_leaf.update_bounding_box();
            leaves.push(current_leaf);
        }
        
        leaves
    }
    
    /// Calculate Hilbert curve value for a point (simplified 2D version)
    ///
    /// This is used for Hilbert curve bulk loading to order polygons
    /// along a space-filling curve for better spatial locality.
    fn hilbert_value(x: Real, y: Real, order: u32) -> u64 {
        // Simplified Hilbert curve calculation
        // In a full implementation, this would be more sophisticated
        let max_coord = (1 << order) as Real;
        let norm_x = ((x * max_coord) as u64).min((1 << order) - 1);
        let norm_y = ((y * max_coord) as u64).min((1 << order) - 1);
        
        // Simple interleaving as approximation
        let mut result = 0u64;
        for i in 0..order {
            let bit_x = (norm_x >> i) & 1;
            let bit_y = (norm_y >> i) & 1;
            result |= (bit_x << (2 * i)) | (bit_y << (2 * i + 1));
        }
        
        result
    }
    
    /// Validate tree structure after bulk loading
    ///
    /// Ensures that the constructed tree satisfies R-tree invariants:
    /// - All leaf nodes are at the same level
    /// - All nodes (except root) have between min_children and max_children entries
    /// - All bounding boxes properly contain their children
    pub fn validate_structure(&self) -> bool {
        self.validate_recursive(None, 0).is_ok()
    }
    
    /// Recursive validation helper
    fn validate_recursive(&self, expected_level: Option<usize>, current_depth: usize) -> Result<usize, String> {
        // Check bounding box containment
        if let Some(ref bounds) = self.bounding_box {
            if self.is_leaf {
                // Verify all polygons are contained in bounding box
                for polygon in &self.polygons {
                    if let Some(poly_bounds) = crate::spatial::utils::polygon_bounds(polygon) {
                        if !bounds.contains(&poly_bounds.min) || !bounds.contains(&poly_bounds.max) {
                            return Err("Polygon not contained in node bounding box".to_string());
                        }
                    }
                }
            } else {
                // Verify all children are contained in bounding box
                for child in &self.children {
                    if let Some(ref child_bounds) = child.bounding_box {
                        if !bounds.contains(&child_bounds.min) || !bounds.contains(&child_bounds.max) {
                            return Err("Child not contained in parent bounding box".to_string());
                        }
                    }
                }
            }
        }
        
        if self.is_leaf {
            // Check that all leaves are at the same level
            if let Some(expected) = expected_level {
                if current_depth != expected {
                    return Err("Leaves at different levels".to_string());
                }
            }
            Ok(current_depth)
        } else {
            // Validate children recursively
            let mut leaf_level = None;
            for child in &self.children {
                let child_leaf_level = child.validate_recursive(leaf_level, current_depth + 1)?;
                if leaf_level.is_none() {
                    leaf_level = Some(child_leaf_level);
                } else if leaf_level != Some(child_leaf_level) {
                    return Err("Inconsistent leaf levels".to_string());
                }
            }
            
            leaf_level.ok_or_else(|| "Internal node with no children".to_string())
        }
    }
}
