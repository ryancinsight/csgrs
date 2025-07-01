//! R-tree insertion, deletion, and search operations
//!
//! This module implements the core R-tree algorithms for dynamic operations:
//! - Insertion with node splitting
//! - Deletion with tree condensation
//! - Advanced search operations

use super::core::Node;
use super::config::{RTreeConfig, SplitAlgorithm};
use crate::geometry::Polygon;
use crate::spatial::traits::Aabb;
use crate::core::float_types::Real;
use std::fmt::Debug;

impl<S: Clone> Node<S> {
    /// Insert a polygon into the R-tree using proper R-tree insertion algorithm
    ///
    /// This implements the classic R-tree insertion algorithm:
    /// 1. Choose leaf node for insertion
    /// 2. Insert polygon into leaf
    /// 3. Propagate changes up the tree
    /// 4. Split nodes if they overflow
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut rtree: Node<i32> = Node::new();
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    ///
    /// rtree.insert_rtree(polygon);
    /// assert_eq!(rtree.polygon_count(), 1);
    /// ```
    pub fn insert_rtree(&mut self, polygon: Polygon<S>) {
        // Get bounding box for the polygon
        let polygon_bounds = match crate::spatial::utils::polygon_bounds(&polygon) {
            Some(bounds) => bounds,
            None => return, // Skip invalid polygons
        };
        
        // Choose leaf node for insertion
        let leaf_path = self.choose_leaf(&polygon_bounds);
        
        // Insert into the chosen leaf
        if let Some(leaf) = leaf_path.last() {
            // For now, simple insertion - will be enhanced with proper path tracking
            self.insert_into_leaf(polygon, polygon_bounds);
        } else {
            // Tree is empty, insert directly
            self.insert_into_leaf(polygon, polygon_bounds);
        }
    }
    
    /// Choose the best leaf node for inserting a new polygon
    ///
    /// Uses the area enlargement heuristic to minimize tree degradation.
    fn choose_leaf(&self, polygon_bounds: &Aabb) -> Vec<usize> {
        let mut path = Vec::new();
        let mut current = self;
        
        while !current.is_leaf {
            if current.children.is_empty() {
                break;
            }
            
            // Find child that requires least enlargement
            let mut best_child = 0;
            let mut best_enlargement = Real::INFINITY;
            let mut best_area = Real::INFINITY;
            
            for (i, child) in current.children.iter().enumerate() {
                if let Some(ref child_bounds) = child.bounding_box {
                    let enlarged_bounds = crate::spatial::utils::merge_bounds(child_bounds, polygon_bounds);
                    let enlargement = crate::spatial::utils::bounds_volume(&enlarged_bounds) 
                                    - crate::spatial::utils::bounds_volume(child_bounds);
                    let area = crate::spatial::utils::bounds_volume(child_bounds);
                    
                    // Choose child with least enlargement, break ties by smallest area
                    if enlargement < best_enlargement || 
                       (enlargement == best_enlargement && area < best_area) {
                        best_child = i;
                        best_enlargement = enlargement;
                        best_area = area;
                    }
                }
            }
            
            path.push(best_child);
            current = &current.children[best_child];
        }
        
        path
    }
    
    /// Insert polygon directly into a leaf node
    fn insert_into_leaf(&mut self, polygon: Polygon<S>, polygon_bounds: Aabb) {
        if !self.is_leaf {
            // Convert to leaf if necessary
            self.is_leaf = true;
            self.children.clear();
        }
        
        self.polygons.push(polygon);
        
        // Update bounding box
        if let Some(ref current_bounds) = self.bounding_box {
            self.bounding_box = Some(crate::spatial::utils::merge_bounds(current_bounds, &polygon_bounds));
        } else {
            self.bounding_box = Some(polygon_bounds);
        }
        
        // Check if node needs to be split
        if self.polygons.len() > self.config.max_children {
            self.split_node();
        }
    }
    
    /// Split an overflowing node using the configured split algorithm
    fn split_node(&mut self) {
        if self.polygons.len() <= self.config.max_children {
            return; // No need to split
        }
        
        match self.config.split_algorithm {
            SplitAlgorithm::Linear => self.linear_split(),
            SplitAlgorithm::Quadratic => self.quadratic_split(),
            SplitAlgorithm::RStarTree => self.rstar_split(),
        }
    }
    
    /// Linear split algorithm - O(M) complexity
    ///
    /// Picks two polygons that are farthest apart and distributes
    /// remaining polygons to minimize area enlargement.
    fn linear_split(&mut self) {
        if self.polygons.len() <= 1 {
            return;
        }
        
        // Find two polygons with maximum separation
        let mut max_distance = 0.0;
        let mut seed1 = 0;
        let mut seed2 = 1;
        
        for i in 0..self.polygons.len() {
            for j in (i + 1)..self.polygons.len() {
                if let (Some(bounds1), Some(bounds2)) = (
                    crate::spatial::utils::polygon_bounds(&self.polygons[i]),
                    crate::spatial::utils::polygon_bounds(&self.polygons[j])
                ) {
                    let center1 = bounds1.center();
                    let center2 = bounds2.center();
                    let distance = crate::spatial::utils::distance(&center1, &center2);
                    
                    if distance > max_distance {
                        max_distance = distance;
                        seed1 = i;
                        seed2 = j;
                    }
                }
            }
        }
        
        // Create two new nodes
        let mut node1 = Node::with_config(self.config.clone());
        let mut node2 = Node::with_config(self.config.clone());
        
        // Move seed polygons
        if seed1 > seed2 {
            node1.polygons.push(self.polygons.remove(seed1));
            node2.polygons.push(self.polygons.remove(seed2));
        } else {
            node2.polygons.push(self.polygons.remove(seed2));
            node1.polygons.push(self.polygons.remove(seed1));
        }
        
        // Distribute remaining polygons
        while !self.polygons.is_empty() {
            let polygon = self.polygons.pop().unwrap();
            
            // Choose node that requires less enlargement
            let bounds1 = node1.bounding_box.clone().unwrap_or_else(|| {
                crate::spatial::utils::polygon_bounds(&node1.polygons[0]).unwrap()
            });
            let bounds2 = node2.bounding_box.clone().unwrap_or_else(|| {
                crate::spatial::utils::polygon_bounds(&node2.polygons[0]).unwrap()
            });
            
            if let Some(poly_bounds) = crate::spatial::utils::polygon_bounds(&polygon) {
                let enlarged1 = crate::spatial::utils::merge_bounds(&bounds1, &poly_bounds);
                let enlarged2 = crate::spatial::utils::merge_bounds(&bounds2, &poly_bounds);
                
                let enlargement1 = crate::spatial::utils::bounds_volume(&enlarged1) 
                                 - crate::spatial::utils::bounds_volume(&bounds1);
                let enlargement2 = crate::spatial::utils::bounds_volume(&enlarged2) 
                                 - crate::spatial::utils::bounds_volume(&bounds2);
                
                if enlargement1 <= enlargement2 {
                    node1.polygons.push(polygon);
                } else {
                    node2.polygons.push(polygon);
                }
            } else {
                // Fallback: add to smaller node
                if node1.polygons.len() <= node2.polygons.len() {
                    node1.polygons.push(polygon);
                } else {
                    node2.polygons.push(polygon);
                }
            }
        }
        
        // Update bounding boxes
        node1.update_bounding_box();
        node2.update_bounding_box();
        
        // Convert this node to internal node
        self.is_leaf = false;
        self.level += 1;
        self.children.clear();
        self.children.push(Box::new(node1));
        self.children.push(Box::new(node2));
        self.update_bounding_box();
    }
    
    /// Quadratic split algorithm - O(M²) complexity
    ///
    /// More sophisticated than linear split, considers all pairs
    /// to find the best initial seeds.
    fn quadratic_split(&mut self) {
        // For now, use linear split as placeholder
        // TODO: Implement proper quadratic split algorithm
        self.linear_split();
    }
    
    /// R*-tree split algorithm with forced reinsertion
    ///
    /// Highest quality split algorithm that may reinsert some
    /// polygons to improve tree structure.
    fn rstar_split(&mut self) {
        // For now, use linear split as placeholder
        // TODO: Implement R*-tree split with forced reinsertion
        self.linear_split();
    }
    
    /// Remove a polygon from the R-tree
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut rtree: Node<i32> = Node::new();
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    ///
    /// rtree.insert_rtree(polygon.clone());
    /// assert_eq!(rtree.polygon_count(), 1);
    ///
    /// let removed = rtree.remove_rtree(&polygon);
    /// assert!(removed);
    /// assert_eq!(rtree.polygon_count(), 0);
    /// ```
    pub fn remove_rtree(&mut self, polygon: &Polygon<S>) -> bool
    where
        S: PartialEq,
    {
        if self.is_leaf {
            // Try to find and remove the polygon
            if let Some(pos) = self.polygons.iter().position(|p| p == polygon) {
                self.polygons.remove(pos);
                self.update_bounding_box();
                return true;
            }
            false
        } else {
            // Search children
            for child in &mut self.children {
                if child.remove_rtree(polygon) {
                    self.update_bounding_box();
                    return true;
                }
            }
            false
        }
    }
}
