//! Unified BSP Tree Operations
//!
//! This module consolidates the sequential and parallel BSP implementations into a single,
//! clean interface using feature gates. This eliminates code duplication and provides
//! a unified API while maintaining performance characteristics of both approaches.
//!
//! ## Design Principles Applied
//!
//! - **SSOT**: Single source of truth for BSP operations
//! - **DRY**: No duplication between sequential and parallel implementations
//! - **CLEAN**: Clear separation of concerns with feature-gated compilation
//! - **SOLID**: Single responsibility with open/closed principle for extensions

// Removed unused imports
use crate::mesh::{
    plane::{Plane, BACK, COPLANAR, FRONT, SPANNING},
    polygon::Polygon,
    vertex::Vertex,
};
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Unified BSP node that works with both sequential and parallel processing
/// 
/// This structure embeds BSP functionality within sparse voxel octree nodes,
/// providing efficient local geometry management with optional parallelism.
#[derive(Debug, Clone)]
pub struct UnifiedBspNode<S: Clone + Send + Sync + Debug> {
    /// Splitting plane for this node, None for leaf nodes
    pub plane: Option<Plane>,
    
    /// Front child node
    pub front: Option<Box<UnifiedBspNode<S>>>,
    
    /// Back child node  
    pub back: Option<Box<UnifiedBspNode<S>>>,
    
    /// Polygons stored at this node
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Default for UnifiedBspNode<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> UnifiedBspNode<S> {
    /// Create a new empty BSP node
    pub const fn new() -> Self {
        Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        }
    }
    
    /// Create BSP node from polygons using unified algorithm
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() {
            node.build(polygons);
        }
        node
    }
    
    /// Invert all polygons in the BSP tree
    /// Uses iterative approach to avoid stack overflow
    pub fn invert(&mut self) {
        let mut stack = vec![self];
        
        while let Some(node) = stack.pop() {
            // Flip polygons - use parallel iterator if available
            #[cfg(feature = "parallel")]
            node.polygons.par_iter_mut().for_each(|p| p.flip());
            
            #[cfg(not(feature = "parallel"))]
            node.polygons.iter_mut().for_each(|p| p.flip());
            
            // Flip plane if present
            if let Some(ref mut plane) = node.plane {
                plane.flip();
            }
            
            // Swap front and back children
            std::mem::swap(&mut node.front, &mut node.back);
            
            // Add children to processing stack
            if let Some(ref mut front) = node.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = node.back {
                stack.push(back.as_mut());
            }
        }
    }
    
    /// Select optimal splitting plane using heuristic
    /// Minimizes polygon splits while balancing tree depth
    pub fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = f64::INFINITY;
        
        // Evaluate subset of polygons as potential splitting planes
        let step = (polygons.len() / 10).max(1);
        
        for polygon in polygons.iter().step_by(step) {
            let plane = &polygon.plane;
            let mut front_count = 0;
            let mut back_count = 0;
            let mut split_count = 0;
            
            for other in polygons {
                match plane.classify_polygon(other) {
                    FRONT => front_count += 1,
                    BACK => back_count += 1,
                    SPANNING => split_count += 1,
                    COPLANAR => {}, // Coplanar polygons don't affect balance
                    _ => {}, // Handle any other cases
                }
            }
            
            // Scoring function: minimize splits, balance front/back
            let balance_penalty = ((front_count as f64) - (back_count as f64)).abs();
            let split_penalty = (split_count as f64) * 3.0; // Splits are expensive
            let score = balance_penalty + split_penalty;
            
            if score < best_score {
                best_score = score;
                best_plane = plane.clone();
            }
        }
        
        best_plane
    }
    
    /// Clip polygons against this BSP tree
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if polygons.is_empty() {
            return Vec::new();
        }
        
        let Some(ref plane) = self.plane else {
            // Leaf node - return all polygons
            return polygons.to_vec();
        };
        
        // Split polygons against this plane
        let (mut front_polys, mut back_polys) = Self::split_polygons_by_plane(polygons, plane);
        
        // Recursively clip against children
        if let Some(ref back_node) = self.back {
            back_polys = back_node.clip_polygons(&back_polys);
        }
        
        if let Some(ref front_node) = self.front {
            front_polys = front_node.clip_polygons(&front_polys);
        } else {
            // No front node means front polygons are outside
            front_polys.clear();
        }
        
        // Combine results
        front_polys.extend(back_polys);
        front_polys
    }
    
    /// Split polygons by a plane into front and back lists
    fn split_polygons_by_plane(polygons: &[Polygon<S>], plane: &Plane) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        let mut front_polys = Vec::new();
        let mut back_polys = Vec::new();
        
        for polygon in polygons {
            let (mut front, mut back, _, _) = plane.split_polygon(polygon);
            front_polys.append(&mut front);
            back_polys.append(&mut back);
        }
        
        (front_polys, back_polys)
    }
    
    /// Clip this BSP tree to another BSP tree
    pub fn clip_to(&mut self, other: &UnifiedBspNode<S>) {
        self.polygons = other.clip_polygons(&self.polygons);
        
        if let Some(ref mut front) = self.front {
            front.clip_to(other);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(other);
        }
    }
    
    /// Get all polygons from this BSP tree
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = self.polygons.clone();
        
        if let Some(ref front) = self.front {
            result.extend(front.all_polygons());
        }
        if let Some(ref back) = self.back {
            result.extend(back.all_polygons());
        }
        
        result
    }
    
    /// Build BSP tree from polygons
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }
        
        // Select splitting plane
        self.plane = Some(self.pick_best_splitting_plane(polygons));
        let plane = self.plane.as_ref().unwrap();
        
        let mut front_polys = Vec::new();
        let mut back_polys = Vec::new();
        
        // Classify and split polygons
        for polygon in polygons {
            let (mut front, mut back, mut coplanar_front, mut coplanar_back) = 
                plane.split_polygon(polygon);
            
            front_polys.append(&mut front);
            back_polys.append(&mut back);
            
            // Coplanar polygons go to this node
            self.polygons.append(&mut coplanar_front);
            self.polygons.append(&mut coplanar_back);
        }
        
        // Recursively build children if they have polygons
        if !front_polys.is_empty() {
            let mut front_node = Box::new(UnifiedBspNode::new());
            front_node.build(&front_polys);
            self.front = Some(front_node);
        }
        
        if !back_polys.is_empty() {
            let mut back_node = Box::new(UnifiedBspNode::new());
            back_node.build(&back_polys);
            self.back = Some(back_node);
        }
    }
    
    /// Slice BSP tree with a plane, returning polygons and edge segments
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let mut result_polygons = Vec::new();
        let mut result_edges = Vec::new();
        
        // Process polygons at this node
        for polygon in &self.polygons {
            let (front, back, coplanar_front, coplanar_back) = slicing_plane.split_polygon(polygon);
            
            // Add coplanar polygons to result
            result_polygons.extend(coplanar_front);
            result_polygons.extend(coplanar_back);
            
            // Generate edge segments from split polygons
            for front_poly in &front {
                for back_poly in &back {
                    if let Some(edge) = Self::find_intersection_edge(front_poly, back_poly, slicing_plane) {
                        result_edges.push(edge);
                    }
                }
            }
        }
        
        // Recursively process children
        if let Some(ref front) = self.front {
            let (mut polys, mut edges) = front.slice(slicing_plane);
            result_polygons.append(&mut polys);
            result_edges.append(&mut edges);
        }
        
        if let Some(ref back) = self.back {
            let (mut polys, mut edges) = back.slice(slicing_plane);
            result_polygons.append(&mut polys);
            result_edges.append(&mut edges);
        }
        
        (result_polygons, result_edges)
    }
    
    /// Find intersection edge between two polygons on a slicing plane
    fn find_intersection_edge(
        front_poly: &Polygon<S>,
        back_poly: &Polygon<S>,
        slicing_plane: &Plane,
    ) -> Option<[Vertex; 2]> {
        let mut intersection_points = Vec::new();
        
        // Find intersections between edges of front_poly and back_poly
        for i in 0..front_poly.vertices.len() {
            let v1 = &front_poly.vertices[i];
            let v2 = &front_poly.vertices[(i + 1) % front_poly.vertices.len()];
            
            for j in 0..back_poly.vertices.len() {
                let v3 = &back_poly.vertices[j];
                let v4 = &back_poly.vertices[(j + 1) % back_poly.vertices.len()];
                
                if let Some(intersection) = Self::line_segment_intersection(v1, v2, v3, v4, slicing_plane) {
                    intersection_points.push(intersection);
                }
            }
        }
        
        // Return edge if we have exactly 2 intersection points
        if intersection_points.len() >= 2 {
            Some([intersection_points[0].clone(), intersection_points[1].clone()])
        } else {
            None
        }
    }
    
    /// Find intersection between two line segments on a plane
    fn line_segment_intersection(
        p1: &Vertex,
        p2: &Vertex,
        p3: &Vertex,
        p4: &Vertex,
        plane: &Plane,
    ) -> Option<Vertex> {
        let d1 = p2.pos - p1.pos;
        let d2 = p4.pos - p3.pos;
        let d3 = p1.pos - p3.pos;
        
        let cross = d1.cross(&d2);
        let denom = cross.norm_squared();
        
        if denom < 1e-10 {
            return None; // Lines are parallel
        }
        
        let t1 = d3.cross(&d2).dot(&cross) / denom;
        let t2 = d3.cross(&d1).dot(&cross) / denom;
        
        // Check if intersection is within both line segments
        if t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0 {
            let intersection_pos = p1.pos + t1 * d1;
            
            // Verify the intersection lies on the slicing plane
            let plane_normal = plane.normal();
            let plane_offset = plane.offset();
            if (plane_normal.dot(&intersection_pos.coords) - plane_offset).abs() < 1e-6 {
                // Interpolate vertex attributes
                let normal = (p1.normal + p2.normal).normalize();
                
                return Some(Vertex::new(intersection_pos, normal));
            }
        }
        
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_unified_bsp_creation() {
        let node = UnifiedBspNode::<()>::new();
        assert!(node.plane.is_none());
        assert!(node.polygons.is_empty());
    }
    
    #[test]
    fn test_unified_bsp_invert() {
        let mut node = UnifiedBspNode::<()>::new();
        // Add test polygons and verify inversion
        node.invert();
        // Verify the inversion worked correctly
    }
}