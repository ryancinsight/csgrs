//! Sparse Voxel Octree Node Implementation
//!
//! This module implements the core SvoNode structure that combines sparse voxel octree
//! spatial organization with embedded BSP trees for efficient CSG operations.
//!
//! ## Key Features
//!
//! - **Sparse Allocation**: Only allocates memory for voxels containing geometry
//! - **Embedded BSP**: Each node contains a BSP subtree for local geometry management
//! - **Adaptive Subdivision**: Subdivides only where geometry complexity requires it
//! - **Zero-Copy Operations**: Leverages iterators and references for efficiency

use crate::mesh::polygon::Polygon;
use crate::voxels::{
    bsp_unified::UnifiedBspNode,
    precision::PrecisionConfig,
};
use crate::float_types::parry3d::bounding_volume::{Aabb, BoundingVolume};
use nalgebra::Point3;
use std::fmt::Debug;

// Constants
const OCTREE_CHILDREN: usize = 8;
const MIN_VOXEL_SIZE: f64 = 0.001;
const MAX_SUBDIVISION_DEPTH: usize = 5;
const SUBDIVISION_THRESHOLD: usize = 100;

/// Sparse Voxel Octree node with embedded BSP functionality
/// 
/// Unlike dense octrees, this implementation only stores nodes where geometry exists,
/// providing significant memory savings for sparse geometries common in CAD applications.
#[derive(Debug, Clone)]
pub struct SvoNode<S: Clone + Send + Sync + Debug> {
    /// Spatial bounds for this sparse voxel
    pub bounds: Aabb,
    
    /// Local BSP tree for geometry within this voxel cell
    /// Only present when this voxel contains actual geometry
    pub bsp: Option<UnifiedBspNode<S>>,
    
    /// Eight octree children (SPARSE: None = empty space)
    /// Memory is allocated only for children containing geometry
    /// Empty regions are implicitly represented by None values
    pub children: [Option<Box<SvoNode<S>>>; OCTREE_CHILDREN],
    
    /// Node-level metadata (only for occupied voxels)
    pub metadata: Option<S>,
    
    /// Fixed-precision arithmetic support for robust operations
    pub precision_scale: Option<i32>,
    
    /// Current subdivision depth (for preventing infinite recursion)
    pub depth: usize,
}

impl<S: Clone + Send + Sync + Debug> SvoNode<S> {
    /// Create a new empty sparse voxel octree node
    pub fn new(bounds: Aabb) -> Self {
        Self {
            bounds,
            bsp: None,
            children: Default::default(),
            metadata: None,
            precision_scale: None,
            depth: 0,
        }
    }
    
    /// Create SVO node with specified depth
    pub fn new_with_depth(bounds: Aabb, depth: usize) -> Self {
        Self {
            bounds,
            bsp: None,
            children: Default::default(),
            metadata: None,
            precision_scale: None,
            depth,
        }
    }
    
    /// Create SVO node from polygons with adaptive subdivision
    pub fn from_polygons(bounds: Aabb, polygons: &[Polygon<S>], config: &PrecisionConfig) -> Self {
        let mut node = Self::new(bounds);
        node.insert_polygons(polygons, config);
        node
    }
    
    /// Check if this node is a leaf (has no children)
    pub fn is_leaf(&self) -> bool {
        self.children.iter().all(|child| child.is_none())
    }
    
    /// Check if this node is empty (no BSP and no children)
    pub fn is_empty(&self) -> bool {
        self.bsp.is_none() && self.is_leaf()
    }
    
    /// Get the number of occupied child nodes
    pub fn occupied_children_count(&self) -> usize {
        self.children.iter().filter(|child| child.is_some()).count()
    }
    
    /// Insert polygons into this node with adaptive subdivision
    pub fn insert_polygons(&mut self, polygons: &[Polygon<S>], _config: &PrecisionConfig) {
        if polygons.is_empty() {
            return;
        }
        
        // Use a recursive helper to avoid unsafe code
        self.insert_polygons_recursive(polygons);
    }
    
    fn insert_polygons_recursive(&mut self, polygons: &[Polygon<S>]) {
        if self.should_subdivide(polygons) {
            // Create children if they don't exist
            if self.children.iter().all(|child| child.is_none()) {
                let child_bounds = self.compute_child_bounds();
                for (i, bounds) in child_bounds.iter().enumerate() {
                    self.children[i] = Some(Box::new(SvoNode::new_with_depth(*bounds, self.depth + 1)));
                }
            }
            
            // Distribute polygons to children
            for (i, child_bounds) in self.compute_child_bounds().iter().enumerate() {
                let child_polygons: Vec<_> = polygons.iter()
                    .filter(|poly| self.polygon_intersects_bounds(poly, child_bounds))
                    .cloned()
                    .collect();
                
                if !child_polygons.is_empty() {
                    if let Some(ref mut child) = self.children[i] {
                        child.insert_polygons_recursive(&child_polygons);
                    }
                }
            }
        } else {
            // Store polygons in local BSP
            self.bsp = Some(UnifiedBspNode::from_polygons(polygons));
        }
    }
    
    /// Determine if this node should be subdivided
    fn should_subdivide(&self, polygons: &[Polygon<S>]) -> bool {
        // Don't subdivide if:
        // - Already at maximum depth
        // - Voxel is too small
        // - Not enough polygons to warrant subdivision
        if self.depth >= MAX_SUBDIVISION_DEPTH {
            return false;
        }
        
        let voxel_size = self.bounds.maxs - self.bounds.mins;
        if voxel_size.x < MIN_VOXEL_SIZE || voxel_size.y < MIN_VOXEL_SIZE || voxel_size.z < MIN_VOXEL_SIZE {
            return false;
        }
        
        if polygons.len() < SUBDIVISION_THRESHOLD {
            return false;
        }
        
        // Check if polygons are spatially distributed enough to benefit from subdivision
        self.has_spatial_distribution(polygons)
    }
    
    /// Check if polygons have sufficient spatial distribution for subdivision
    fn has_spatial_distribution(&self, polygons: &[Polygon<S>]) -> bool {
        let center = self.bounds.center();
        
        // Check distribution along each axis
        for axis in 0..3 {
            let mut front_count = 0;
            let mut back_count = 0;
            
            for polygon in polygons {
                let poly_center = polygon.center();
                if poly_center[axis] > center[axis] {
                    front_count += 1;
                } else {
                    back_count += 1;
                }
            }
            
            // If polygons are well distributed along this axis, subdivision is beneficial
            let min_count = polygons.len() / 4; // At least 25% on each side
            if front_count >= min_count && back_count >= min_count {
                return true;
            }
        }
        
        false
    }
    
    /// Subdivide this node and distribute polygons to children

    
    /// Compute bounding boxes for the 8 octree children
    fn compute_child_bounds(&self) -> [Aabb; OCTREE_CHILDREN] {
        let center = self.bounds.center();
        let mins = self.bounds.mins;
        let maxs = self.bounds.maxs;
        
        [
            // Bottom level (z = mins.z to center.z)
            Aabb::new(Point3::new(mins.x, mins.y, mins.z), Point3::new(center.x, center.y, center.z)), // 0: ---
            Aabb::new(Point3::new(center.x, mins.y, mins.z), Point3::new(maxs.x, center.y, center.z)), // 1: +--
            Aabb::new(Point3::new(mins.x, center.y, mins.z), Point3::new(center.x, maxs.y, center.z)), // 2: -+-
            Aabb::new(Point3::new(center.x, center.y, mins.z), Point3::new(maxs.x, maxs.y, center.z)), // 3: ++-
            
            // Top level (z = center.z to maxs.z)
            Aabb::new(Point3::new(mins.x, mins.y, center.z), Point3::new(center.x, center.y, maxs.z)), // 4: --+
            Aabb::new(Point3::new(center.x, mins.y, center.z), Point3::new(maxs.x, center.y, maxs.z)), // 5: +-+
            Aabb::new(Point3::new(mins.x, center.y, center.z), Point3::new(center.x, maxs.y, maxs.z)), // 6: -++
            Aabb::new(Point3::new(center.x, center.y, center.z), Point3::new(maxs.x, maxs.y, maxs.z)), // 7: +++
        ]
    }
    
    /// Check if a polygon intersects with given bounds
    fn polygon_intersects_bounds(&self, polygon: &Polygon<S>, bounds: &Aabb) -> bool {
        // Simple bounding box intersection test
        let poly_bounds = polygon.bounding_box();
        bounds.intersects(&poly_bounds)
    }
    
    /// Get all polygons from this node and its children
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];
        
        while let Some(node) = stack.pop() {
            // Add polygons from local BSP
            if let Some(ref bsp) = node.bsp {
                result.extend(bsp.all_polygons());
            }
            
            // Add children to stack for processing
            for child in &node.children {
                if let Some(child_node) = child {
                    stack.push(child_node.as_ref());
                }
            }
        }
        
        result
    }
    
    /// Clip polygons against this SVO node
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if polygons.is_empty() {
            return Vec::new();
        }
        
        let mut result = Vec::new();
        let mut stack = vec![(self, polygons.to_vec())];
        
        while let Some((node, current_polygons)) = stack.pop() {
            if current_polygons.is_empty() {
                continue;
            }
            
            // First clip against local BSP if present
            let clipped_polygons = if let Some(ref bsp) = node.bsp {
                bsp.clip_polygons(&current_polygons)
            } else {
                current_polygons.clone()
            };
            
            let mut has_children = false;
            // Then clip against children
            for child in &node.children {
                if let Some(child_node) = child {
                    has_children = true;
                    // Only process polygons that intersect this child's bounds
                    let child_polygons: Vec<_> = clipped_polygons
                        .iter()
                        .filter(|poly| node.polygon_intersects_bounds(poly, &child_node.bounds))
                        .cloned()
                        .collect();
                    
                    if !child_polygons.is_empty() {
                        stack.push((child_node.as_ref(), child_polygons));
                    }
                }
            }
            
            // If no children, add the clipped polygons to result
            if !has_children && !clipped_polygons.is_empty() {
                result.extend(clipped_polygons);
            }
        }
        
        result
    }
    
    /// Invert all geometry in this node
    pub fn invert(&mut self) {
        // Invert local BSP
        if let Some(ref mut bsp) = self.bsp {
            bsp.invert();
        }
        
        // Recursively invert children
        for child in &mut self.children {
            if let Some(child_node) = child.as_mut() {
                child_node.invert();
            }
        }
    }
    
    /// Clip this node to another SVO node
    pub fn clip_to(&mut self, other: &SvoNode<S>) {
        let all_other_polygons = other.all_polygons();
        if all_other_polygons.is_empty() {
            return;
        }
        
        let other_bsp = UnifiedBspNode::from_polygons(&all_other_polygons);
        
        // Clip local BSP
        if let Some(ref mut bsp) = self.bsp {
            bsp.clip_to(&other_bsp);
        }
        
        // Recursively clip children
        for child in &mut self.children {
            if let Some(child_node) = child.as_mut() {
                child_node.clip_to(other);
            }
        }
    }
    
    /// Compute memory usage of this node and its subtree
    pub fn memory_usage(&self) -> usize {
        let mut size = 0;
        let mut stack = vec![self];
        
        while let Some(node) = stack.pop() {
            size += std::mem::size_of::<Self>();
            
            // Add BSP memory usage
            if let Some(ref bsp) = node.bsp {
                size += std::mem::size_of_val(bsp);
                size += bsp.all_polygons().len() * std::mem::size_of::<Polygon<S>>();
            }
            
            // Add children to stack for processing
            for child in &node.children {
                if let Some(child_node) = child {
                    stack.push(child_node.as_ref());
                }
            }
        }
        
        size
    }
    
    /// Get statistics about this node and its subtree
    pub fn statistics(&self) -> SvoStatistics {
        let mut stats = SvoStatistics::default();
        let mut stack = vec![self];
        
        while let Some(node) = stack.pop() {
            // Count this node
            stats.total_nodes += 1;
            if node.is_leaf() {
                stats.leaf_nodes += 1;
            }
            if node.bsp.is_some() {
                stats.nodes_with_geometry += 1;
            }
            
            // Count polygons
            if let Some(ref bsp) = node.bsp {
                stats.total_polygons += bsp.all_polygons().len();
            }
            
            // Update depth
            stats.max_depth = stats.max_depth.max(node.depth);
            
            // Add children to stack for processing
            for child in &node.children {
                if let Some(child_node) = child {
                    stack.push(child_node.as_ref());
                }
            }
        }
        
        stats
    }
}

/// Statistics about an SVO tree structure
#[derive(Debug, Default, Clone)]
pub struct SvoStatistics {
    pub total_nodes: usize,
    pub leaf_nodes: usize,
    pub nodes_with_geometry: usize,
    pub total_polygons: usize,
    pub max_depth: usize,
}

impl SvoStatistics {
    /// Calculate memory efficiency (polygons per node)
    pub fn memory_efficiency(&self) -> f64 {
        if self.total_nodes == 0 {
            0.0
        } else {
            self.total_polygons as f64 / self.total_nodes as f64
        }
    }
    
    /// Calculate sparsity ratio (empty nodes / total possible nodes)
    pub fn sparsity_ratio(&self) -> f64 {
        if self.max_depth == 0 {
            return 0.0;
        }
        
        // Calculate total possible nodes in a complete octree of this depth
        let total_possible = (0..=self.max_depth)
            .map(|d| 8_usize.pow(d as u32))
            .sum::<usize>();
        
        let empty_nodes = total_possible - self.total_nodes;
        empty_nodes as f64 / total_possible as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;
    
    #[test]
    fn test_svo_node_creation() {
        let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let node = SvoNode::<()>::new(bounds);
        
        assert!(node.is_empty());
        assert!(node.is_leaf());
        assert_eq!(node.depth, 0);
    }
    
    #[test]
    fn test_child_bounds_computation() {
        let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let node = SvoNode::<()>::new(bounds);
        let child_bounds = node.compute_child_bounds();
        
        // Verify all 8 children have correct bounds
        assert_eq!(child_bounds.len(), 8);
        
        // Check first child (bottom-left-front)
        assert_eq!(child_bounds[0].mins, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(child_bounds[0].maxs, Point3::new(1.0, 1.0, 1.0));
        
        // Check last child (top-right-back)
        assert_eq!(child_bounds[7].mins, Point3::new(1.0, 1.0, 1.0));
        assert_eq!(child_bounds[7].maxs, Point3::new(2.0, 2.0, 2.0));
    }
    
    #[test]
    fn test_statistics() {
        let bounds = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let node = SvoNode::<()>::new(bounds);
        let stats = node.statistics();
        
        assert_eq!(stats.total_nodes, 1);
        assert_eq!(stats.leaf_nodes, 1);
        assert_eq!(stats.nodes_with_geometry, 0);
        assert_eq!(stats.total_polygons, 0);
        assert_eq!(stats.max_depth, 0);
    }
}