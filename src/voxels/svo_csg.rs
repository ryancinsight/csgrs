//! SVO-based CSG operations working on voxel occupancy
//! 
//! This module implements CSG operations directly on sparse voxel octrees,
//! working with occupancy states rather than polygon geometry.

use crate::float_types::Real;
use crate::voxels::{Svo, SvoNode, Occupancy};

use nalgebra::Point3;
use std::fmt::Debug;

/// SVO-based CSG operations following SOLID principles
pub struct SvoCsg;

impl SvoCsg {
    /// Perform union operation on two SVOs
    pub fn union<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> Svo<S> {
        // Create result SVO with combined bounds
        let combined_bounds = Self::compute_combined_bounds(a, b);
        let max_depth = a.max_depth.max(b.max_depth);
        let mut result = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
        
        Self::union_nodes(
            &mut *result.root,
            &*a.root,
            &*b.root,
            &result.center,
            result.half,
            0,
            max_depth,
        );
        
        result
    }
    
    /// Perform intersection operation on two SVOs
    pub fn intersection<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> Svo<S> {
        let combined_bounds = Self::compute_combined_bounds(a, b);
        let max_depth = a.max_depth.max(b.max_depth);
        let mut result = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
        
        Self::intersection_nodes(
            &mut *result.root,
            &*a.root,
            &*b.root,
            &result.center,
            result.half,
            0,
            max_depth,
        );
        
        result
    }
    
    /// Perform difference operation on two SVOs
    pub fn difference<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> Svo<S> {
        let combined_bounds = Self::compute_combined_bounds(a, b);
        let max_depth = a.max_depth.max(b.max_depth);
        let mut result = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
        
        Self::difference_nodes(
            &mut *result.root,
            &*a.root,
            &*b.root,
            &result.center,
            result.half,
            0,
            max_depth,
        );
        
        result
    }
    
    /// Union operation on SVO nodes
    fn union_nodes<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
    ) {
        // Union truth table for occupancy:
        // Empty ∪ Empty = Empty
        // Empty ∪ Full = Full
        // Empty ∪ Mixed = Mixed
        // Full ∪ * = Full
        // Mixed ∪ Mixed = Mixed
        
        match (a.occupancy, b.occupancy) {
            (Occupancy::Full, _) | (_, Occupancy::Full) => {
                result.occupancy = Occupancy::Full;
                return;
            }
            (Occupancy::Empty, Occupancy::Empty) => {
                result.occupancy = Occupancy::Empty;
                return;
            }
            (Occupancy::Empty, occ) | (occ, Occupancy::Empty) => {
                result.occupancy = occ;
                // Copy the non-empty structure
                Self::copy_node_structure(result, if a.occupancy == Occupancy::Empty { b } else { a });
                return;
            }
            (Occupancy::Mixed, Occupancy::Mixed) => {
                result.occupancy = Occupancy::Mixed;
                // Merge BSP trees for precise surface representation
                Self::merge_bsp_trees_union(result, a, b);
                // Need to recurse for precise union
            }
        }
        
        // Recurse for Mixed cases
        if depth < max_depth {
            for child_idx in 0..8 {
                let a_child = a.get_child(child_idx);
                let b_child = b.get_child(child_idx);
                
                if a_child.is_some() || b_child.is_some() {
                    let result_child = result.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);
                    
                    let empty_node = SvoNode::new();
                    let a_ref = a_child.unwrap_or(&empty_node);
                    let b_ref = b_child.unwrap_or(&empty_node);
                    
                    Self::union_nodes(
                        result_child,
                        a_ref,
                        b_ref,
                        &child_center,
                        half * 0.5,
                        depth + 1,
                        max_depth,
                    );
                }
            }
        }
    }
    
    /// Intersection operation on SVO nodes
    fn intersection_nodes<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
    ) {
        // Intersection truth table:
        // Empty ∩ * = Empty
        // Full ∩ Empty = Empty
        // Full ∩ Full = Full
        // Full ∩ Mixed = Mixed
        // Mixed ∩ Mixed = Mixed (need to recurse)
        
        match (a.occupancy, b.occupancy) {
            (Occupancy::Empty, _) | (_, Occupancy::Empty) => {
                result.occupancy = Occupancy::Empty;
                return;
            }
            (Occupancy::Full, Occupancy::Full) => {
                result.occupancy = Occupancy::Full;
                return;
            }
            (Occupancy::Full, Occupancy::Mixed) => {
                result.occupancy = Occupancy::Mixed;
                Self::copy_node_structure(result, b);
                // Copy BSP from Mixed node
                if let Some(ref bsp) = b.local_bsp {
                    result.local_bsp = Some(bsp.clone());
                }
                return;
            }
            (Occupancy::Mixed, Occupancy::Full) => {
                result.occupancy = Occupancy::Mixed;
                Self::copy_node_structure(result, a);
                // Copy BSP from Mixed node
                if let Some(ref bsp) = a.local_bsp {
                    result.local_bsp = Some(bsp.clone());
                }
                return;
            }
            (Occupancy::Mixed, Occupancy::Mixed) => {
                result.occupancy = Occupancy::Mixed;
                // Merge BSP trees for precise surface representation
                Self::merge_bsp_trees_intersection(result, a, b);
                // Need to recurse for precise intersection
            }
        }
        
        // Recurse for Mixed ∩ Mixed
        if depth < max_depth {
            for child_idx in 0..8 {
                let a_child = a.get_child(child_idx);
                let b_child = b.get_child(child_idx);
                
                if let (Some(a_ref), Some(b_ref)) = (a_child, b_child) {
                    let result_child = result.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);
                    
                    Self::intersection_nodes(
                        result_child,
                        a_ref,
                        b_ref,
                        &child_center,
                        half * 0.5,
                        depth + 1,
                        max_depth,
                    );
                }
            }
        }
    }
    
    /// Difference operation on SVO nodes
    fn difference_nodes<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
    ) {
        // Difference truth table (A - B):
        // Empty - * = Empty
        // Full - Empty = Full
        // Full - Full = Empty
        // Full - Mixed = Mixed
        // Mixed - Empty = Mixed
        // Mixed - Full = Empty
        // Mixed - Mixed = Mixed (need to recurse)
        
        match (a.occupancy, b.occupancy) {
            (Occupancy::Empty, _) => {
                result.occupancy = Occupancy::Empty;
                return;
            }
            (Occupancy::Full, Occupancy::Empty) => {
                result.occupancy = Occupancy::Full;
                return;
            }
            (Occupancy::Full, Occupancy::Full) => {
                result.occupancy = Occupancy::Empty;
                return;
            }
            (Occupancy::Full, Occupancy::Mixed) => {
                result.occupancy = Occupancy::Mixed;
                // Complex case - need BSP operations
                return;
            }
            (Occupancy::Mixed, Occupancy::Empty) => {
                result.occupancy = Occupancy::Mixed;
                Self::copy_node_structure(result, a);
                // Copy BSP from Mixed node
                if let Some(ref bsp) = a.local_bsp {
                    result.local_bsp = Some(bsp.clone());
                }
                return;
            }
            (Occupancy::Mixed, Occupancy::Full) => {
                result.occupancy = Occupancy::Empty;
                return;
            }
            (Occupancy::Mixed, Occupancy::Mixed) => {
                result.occupancy = Occupancy::Mixed;
                // Merge BSP trees for precise surface representation
                Self::merge_bsp_trees_difference(result, a, b);
                // Need to recurse for precise difference
            }
        }
        
        // Recurse for Mixed cases
        if depth < max_depth {
            for child_idx in 0..8 {
                let a_child = a.get_child(child_idx);
                let b_child = b.get_child(child_idx);
                
                if a_child.is_some() {
                    let result_child = result.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);
                    
                    let empty_node = SvoNode::new();
                    let a_ref = a_child.unwrap();
                    let b_ref = b_child.unwrap_or(&empty_node);
                    
                    Self::difference_nodes(
                        result_child,
                        a_ref,
                        b_ref,
                        &child_center,
                        half * 0.5,
                        depth + 1,
                        max_depth,
                    );
                }
            }
        }
    }
    
    /// Copy node structure from source to destination
    fn copy_node_structure<S: Clone + Debug + Send + Sync>(
        dest: &mut SvoNode<S>,
        src: &SvoNode<S>,
    ) {
        dest.occupancy = src.occupancy;
        dest.metadata = src.metadata.clone();
        dest.local_bsp = src.local_bsp.clone();
        
        // Copy children structure
        dest.children_mask = src.children_mask;
        dest.children.clear();
        for child in &src.children {
            dest.children.push(child.clone());
        }
    }
    
    /// Compute combined bounds for two SVOs
    fn compute_combined_bounds<S: Clone>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> (Point3<Real>, Real) {
        let a_aabb = a.aabb();
        let b_aabb = b.aabb();
        
        let min_x = a_aabb.mins.x.min(b_aabb.mins.x);
        let min_y = a_aabb.mins.y.min(b_aabb.mins.y);
        let min_z = a_aabb.mins.z.min(b_aabb.mins.z);
        let max_x = a_aabb.maxs.x.max(b_aabb.maxs.x);
        let max_y = a_aabb.maxs.y.max(b_aabb.maxs.y);
        let max_z = a_aabb.maxs.z.max(b_aabb.maxs.z);
        
        let center = Point3::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        );
        
        let half = ((max_x - min_x) * 0.5)
            .max((max_y - min_y) * 0.5)
            .max((max_z - min_z) * 0.5);
        
        (center, half)
    }
    
    /// Compute child center for octree subdivision
    fn child_center(parent_center: &Point3<Real>, parent_half: Real, child_idx: u8) -> Point3<Real> {
        let quarter = parent_half * 0.5;
        let dx = if (child_idx & 1) != 0 { quarter } else { -quarter };
        let dy = if (child_idx & 2) != 0 { quarter } else { -quarter };
        let dz = if (child_idx & 4) != 0 { quarter } else { -quarter };
        
        Point3::new(
            parent_center.x + dx,
            parent_center.y + dy,
            parent_center.z + dz,
        )
    }

    /// Merge BSP trees for union operation
    fn merge_bsp_trees_union<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
    ) {

        match (&a.local_bsp, &b.local_bsp) {
            (Some(bsp_a), Some(bsp_b)) => {
                // Merge both BSP trees - union combines all polygons
                let mut all_polygons = bsp_a.all_polygons();
                all_polygons.extend(bsp_b.all_polygons());

                if !all_polygons.is_empty() {
                    result.local_bsp = Some(crate::voxels::bsp::Node::from_polygons(&all_polygons));
                }
            }
            (Some(bsp), None) | (None, Some(bsp)) => {
                // Copy the existing BSP tree
                result.local_bsp = Some(bsp.clone());
            }
            (None, None) => {
                // No BSP trees to merge
            }
        }
    }

    /// Merge BSP trees for intersection operation
    fn merge_bsp_trees_intersection<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
    ) {

        match (&a.local_bsp, &b.local_bsp) {
            (Some(bsp_a), Some(bsp_b)) => {
                // For intersection, we need to compute the intersection of surfaces
                // This is complex - for now, use the more detailed BSP
                let polygons_a = bsp_a.all_polygons();
                let polygons_b = bsp_b.all_polygons();

                let selected_bsp = if polygons_a.len() >= polygons_b.len() { bsp_a } else { bsp_b };
                result.local_bsp = Some(selected_bsp.clone());
            }
            (Some(_), None) | (None, Some(_)) => {
                // Intersection with empty - no surface
                result.local_bsp = None;
            }
            (None, None) => {
                // No BSP trees to merge
            }
        }
    }

    /// Merge BSP trees for difference operation
    fn merge_bsp_trees_difference<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
    ) {

        match (&a.local_bsp, &b.local_bsp) {
            (Some(bsp_a), Some(_bsp_b)) => {
                // For difference, the result surface comes primarily from A
                // This is a simplification - proper CSG would compute the actual difference
                result.local_bsp = Some(bsp_a.clone());
            }
            (Some(bsp_a), None) => {
                // A - empty = A
                result.local_bsp = Some(bsp_a.clone());
            }
            (None, Some(_)) => {
                // empty - B = empty (no surface)
                result.local_bsp = None;
            }
            (None, None) => {
                // No BSP trees to merge
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn union_empty_svos() {
        let a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        
        let result = SvoCsg::union(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Empty);
    }
    
    #[test]
    fn intersection_empty_svos() {
        let a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        
        let result = SvoCsg::intersection(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Empty);
    }
}
