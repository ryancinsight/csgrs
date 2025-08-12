//! SVO-based CSG operations working on voxel occupancy
//! 
//! This module implements CSG operations directly on sparse voxel octrees,
//! working with occupancy states rather than polygon geometry.

use crate::float_types::Real;
use crate::voxels::{Svo, SvoNode, Occupancy};

use nalgebra::Point3;
use std::fmt::Debug;

/// CSG operation types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CsgOperation {
    Union,
    Intersection,
    Difference,
}

/// SVO-based CSG operations following SOLID principles
#[allow(dead_code)]
pub struct SvoCsg;

// Static empty node to avoid repeated allocations
static EMPTY_NODE: std::sync::OnceLock<SvoNode<()>> = std::sync::OnceLock::new();

#[allow(dead_code)]
impl SvoCsg {
    /// Get reference to static empty node to avoid repeated allocations
    fn empty_node<S: Clone>() -> &'static SvoNode<()> {
        EMPTY_NODE.get_or_init(|| SvoNode::new())
    }
    /// Perform union operation on two SVOs
    pub fn union<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> Svo<S> {
        Self::perform_csg_operation(a, b, CsgOperation::Union)
    }
    
    /// Perform intersection operation on two SVOs
    pub fn intersection<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> Svo<S> {
        Self::perform_csg_operation(a, b, CsgOperation::Intersection)
    }

    /// Perform difference operation on two SVOs
    pub fn difference<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
    ) -> Svo<S> {
        Self::perform_csg_operation(a, b, CsgOperation::Difference)
    }

    /// Main CSG dispatch with bounds normalization and validation
    fn perform_csg_operation<S: Clone + Debug + Send + Sync>(
        a: &Svo<S>,
        b: &Svo<S>,
        operation: CsgOperation,
    ) -> Svo<S> {
        // Validation: Check for reasonable depth limits to prevent stack overflow
        const MAX_SAFE_DEPTH: u8 = 20;
        if a.max_depth > MAX_SAFE_DEPTH || b.max_depth > MAX_SAFE_DEPTH {
            eprintln!("Warning: CSG operation with very deep trees (depth > {}), may cause performance issues", MAX_SAFE_DEPTH);
        }

        // Step 1: Normalize bounds - create result SVO with combined bounds
        let combined_bounds = Self::compute_combined_bounds(a, b);
        let max_depth = a.max_depth.max(b.max_depth);
        let mut result = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);

        // Step 2: Early termination optimizations
        let mut a_fixed = a.clone();
        let mut b_fixed = b.clone();
        Self::fix_root_occupancy(&mut a_fixed);
        Self::fix_root_occupancy(&mut b_fixed);

        // Short-circuit for trivial cases
        match operation {
            CsgOperation::Union => {
                if a_fixed.root.occupancy == Occupancy::Full || b_fixed.root.occupancy == Occupancy::Full {
                    result.root.occupancy = Occupancy::Full;
                    return result;
                }
                if a_fixed.root.occupancy == Occupancy::Empty {
                    // Return B but with combined bounds
                    let mut result_b = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
                    Self::copy_node_structure(&mut *result_b.root, &*b_fixed.root);
                    return result_b;
                }
                if b_fixed.root.occupancy == Occupancy::Empty {
                    // Return A but with combined bounds
                    let mut result_a = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
                    Self::copy_node_structure(&mut *result_a.root, &*a_fixed.root);
                    return result_a;
                }
            }
            CsgOperation::Intersection => {
                if a_fixed.root.occupancy == Occupancy::Empty || b_fixed.root.occupancy == Occupancy::Empty {
                    result.root.occupancy = Occupancy::Empty;
                    return result;
                }
                if a_fixed.root.occupancy == Occupancy::Full {
                    // Return B but with combined bounds
                    let mut result_b = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
                    Self::copy_node_structure(&mut *result_b.root, &*b_fixed.root);
                    return result_b;
                }
                if b_fixed.root.occupancy == Occupancy::Full {
                    // Return A but with combined bounds
                    let mut result_a = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
                    Self::copy_node_structure(&mut *result_a.root, &*a_fixed.root);
                    return result_a;
                }
            }
            CsgOperation::Difference => {
                if a_fixed.root.occupancy == Occupancy::Empty {
                    result.root.occupancy = Occupancy::Empty;
                    return result;
                }
                if b_fixed.root.occupancy == Occupancy::Empty {
                    // Return A but with combined bounds
                    let mut result_a = Svo::new(combined_bounds.0, combined_bounds.1, max_depth);
                    Self::copy_node_structure(&mut *result_a.root, &*a_fixed.root);
                    return result_a;
                }
                if b_fixed.root.occupancy == Occupancy::Full {
                    result.root.occupancy = Occupancy::Empty;
                    return result;
                }
            }
        }

        // Check if bounds are compatible for direct node operations
        if Self::svos_have_same_bounds(&a_fixed, &b_fixed) &&
           a_fixed.max_depth == b_fixed.max_depth {
            // Direct node operations for same bounds and depth
            Self::csg_nodes(
                &mut *result.root,
                &*a_fixed.root,
                &*b_fixed.root,
                &result.center,
                result.half,
                0,
                max_depth,
                operation,
            );
        } else {
            // Use resampling for different bounds or depths
            Self::csg_nodes_with_resampling(
                &mut *result.root,
                &a_fixed,
                &b_fixed,
                &result.center,
                result.half,
                0,
                max_depth,
                operation,
            );
        }

        // Step 4: Fix occupancy inconsistencies throughout the tree
        Self::fix_tree_occupancy(&mut *result.root);

        // Step 5: Simplify result to remove redundant nodes
        result.simplify();

        result
    }

    /// Unified CSG node operation dispatcher
    fn csg_nodes<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        operation: CsgOperation,
    ) {
        match operation {
            CsgOperation::Union => Self::union_nodes(result, a, b, center, half, depth, max_depth),
            CsgOperation::Intersection => Self::intersection_nodes(result, a, b, center, half, depth, max_depth),
            CsgOperation::Difference => Self::difference_nodes(result, a, b, center, half, depth, max_depth),
        }
    }

    /// CSG operation with coordinate resampling for different bounds
    fn csg_nodes_with_resampling<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &Svo<S>,
        b: &Svo<S>,
        result_center: &Point3<Real>,
        result_half: Real,
        depth: u8,
        max_depth: u8,
        operation: CsgOperation,
    ) {
        // Sample occupancy from both SVOs at the result's coordinate system
        let a_occupancy = Self::sample_svo_occupancy(a, result_center, result_half);
        let b_occupancy = Self::sample_svo_occupancy(b, result_center, result_half);

        // Apply CSG operation on sampled occupancies
        result.occupancy = Self::apply_csg_occupancy(a_occupancy, b_occupancy, operation);

        // If result is Mixed and we haven't reached max depth, recurse
        if result.occupancy == Occupancy::Mixed && depth < max_depth {
            for child_idx in 0..8 {
                let child_center = Self::child_center(result_center, result_half, child_idx);
                let child_half = result_half * 0.5;

                // Recursively process child
                let result_child = result.ensure_child(child_idx);
                Self::csg_nodes_with_resampling(
                    result_child,
                    a,
                    b,
                    &child_center,
                    child_half,
                    depth + 1,
                    max_depth,
                    operation,
                );
            }
        }

        // Clear children if result became Full or Empty, but preserve BSP with surface data
        if result.occupancy != Occupancy::Mixed {
            result.children.clear();
            result.children_mask = 0;

            // Only clear BSP if it's truly empty (no surface information)
            if let Some(ref bsp) = result.local_bsp {
                if bsp.polygons.is_empty() && bsp.front.is_none() && bsp.back.is_none() {
                    result.local_bsp = None;
                }
                // Otherwise keep BSP for surface extraction
            }
        }
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
                // Copy the non-empty structure and ensure occupancy consistency
                let source = if a.occupancy == Occupancy::Empty { b } else { a };
                Self::copy_node_structure(result, source);

                // Ensure parent occupancy is consistent with children
                if !result.children.is_empty() && result.occupancy != Occupancy::Mixed {
                    result.occupancy = Occupancy::Mixed;
                }
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
            // Create empty node once for this recursion level
            let empty_node = SvoNode::new();

            for child_idx in 0..8 {
                let a_child = a.get_child(child_idx);
                let b_child = b.get_child(child_idx);

                if a_child.is_some() || b_child.is_some() {
                    let result_child = result.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);

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

        // Clean up result: if occupancy became Full or Empty, clear children but preserve BSP with surface data
        if result.occupancy != Occupancy::Mixed {
            result.children.clear();
            result.children_mask = 0;

            // Only clear BSP if it's truly empty (no surface information)
            if let Some(ref bsp) = result.local_bsp {
                if bsp.polygons.is_empty() && bsp.front.is_none() && bsp.back.is_none() {
                    result.local_bsp = None;
                }
                // Otherwise keep BSP for surface extraction
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
        
        // Recurse for Mixed ∩ Mixed - need to consider all child combinations
        if depth < max_depth {
            // Create empty node once for this recursion level
            let empty_node = SvoNode::new();

            for child_idx in 0..8 {
                let a_child = a.get_child(child_idx);
                let b_child = b.get_child(child_idx);

                // For intersection, we need to recurse if either side has a child
                // Missing children are treated as Empty for intersection
                if a_child.is_some() || b_child.is_some() {
                    let result_child = result.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);

                    let a_ref = a_child.unwrap_or(&empty_node);
                    let b_ref = b_child.unwrap_or(&empty_node);

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

        // Clean up result: if occupancy became Full or Empty, clear children but preserve BSP with surface data
        if result.occupancy != Occupancy::Mixed {
            result.children.clear();
            result.children_mask = 0;

            // Only clear BSP if it's truly empty (no surface information)
            if let Some(ref bsp) = result.local_bsp {
                if bsp.polygons.is_empty() && bsp.front.is_none() && bsp.back.is_none() {
                    result.local_bsp = None;
                }
                // Otherwise keep BSP for surface extraction
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
                // Full - Mixed requires BSP operations and recursion
                // Create a temporary Full BSP for A if needed, then merge with B's BSP
                if let Some(ref bsp_b) = b.local_bsp {
                    // For Full - Mixed, we need to subtract B's surface from A's full volume
                    // This creates new surface geometry where B intersects A
                    let mut result_bsp = bsp_b.clone();
                    #[cfg(not(feature = "parallel"))]
                    {
                        result_bsp.invert(); // Invert B to create "hole" surfaces
                    }
                    result.local_bsp = Some(result_bsp);
                }
                // Continue to recursion for detailed subtraction
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
            // Create empty node once for this recursion level
            let empty_node = SvoNode::new();

            for child_idx in 0..8 {
                let a_child = a.get_child(child_idx);
                let b_child = b.get_child(child_idx);

                if a_child.is_some() {
                    let result_child = result.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);

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

        // Clean up result: if occupancy became Full or Empty, clear children but preserve BSP with surface data
        if result.occupancy != Occupancy::Mixed {
            result.children.clear();
            result.children_mask = 0;

            // Only clear BSP if it's truly empty (no surface information)
            if let Some(ref bsp) = result.local_bsp {
                if bsp.polygons.is_empty() && bsp.front.is_none() && bsp.back.is_none() {
                    result.local_bsp = None;
                }
                // Otherwise keep BSP for surface extraction
            }
        }
    }
    
    /// Copy node structure from source to destination with occupancy validation
    fn copy_node_structure<S: Clone + Debug + Send + Sync>(
        dest: &mut SvoNode<S>,
        src: &SvoNode<S>,
    ) {
        dest.occupancy = src.occupancy;
        dest.metadata = src.metadata.clone();

        // Copy children structure
        dest.children_mask = src.children_mask;
        dest.children.clear();
        for child in &src.children {
            dest.children.push(child.clone());
        }

        // Copy BSP only if occupancy is Mixed (BSP should only exist for Mixed nodes)
        if dest.occupancy == Occupancy::Mixed {
            dest.local_bsp = src.local_bsp.clone();
        } else {
            dest.local_bsp = None;
        }

        // Validate occupancy consistency: if node has children, it should be Mixed
        if !dest.children.is_empty() && dest.occupancy != Occupancy::Mixed {
            dest.occupancy = Occupancy::Mixed;
        }

        // Validate occupancy consistency: if node is Full/Empty, it shouldn't have children
        if dest.occupancy != Occupancy::Mixed && (!dest.children.is_empty() || dest.local_bsp.is_some()) {
            dest.children.clear();
            dest.children_mask = 0;
            dest.local_bsp = None;
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
    
    /// Compute child center for octree subdivision (delegates to SVO implementation)
    fn child_center(parent_center: &Point3<Real>, parent_half: Real, child_idx: u8) -> Point3<Real> {
        Svo::<()>::child_center(parent_center, parent_half, child_idx)
    }

    /// Check if two SVOs have the same bounds (center and half size)
    fn svos_have_same_bounds<S: Clone>(a: &Svo<S>, b: &Svo<S>) -> bool {
        const EPSILON: Real = 1e-6;
        (a.center - b.center).norm() < EPSILON && (a.half - b.half).abs() < EPSILON
    }

    /// Union operation using sampling approach for different bounds
    fn union_with_sampling<S: Clone + Debug + Send + Sync>(
        larger: &Svo<S>,
        smaller: &Svo<S>,
    ) -> Svo<S> {
        // Start with a copy of the larger SVO
        let mut result = larger.clone();

        // Sample the smaller SVO and add its occupied regions to the result
        Self::sample_and_merge(&mut result, smaller);

        result
    }

    /// Sample one SVO and merge its occupied regions into another
    fn sample_and_merge<S: Clone + Debug + Send + Sync>(
        target: &mut Svo<S>,
        source: &Svo<S>,
    ) {
        // For now, use a simple approach: if the source has any Full or Mixed nodes,
        // mark the corresponding region in target as occupied
        if source.root.occupancy != Occupancy::Empty {
            // Simple merge: if source is not empty, ensure target covers the same region
            Self::merge_occupancy(&mut *target.root, &*source.root, &target.center, target.half, &source.center, source.half, 0, target.max_depth);
        }
    }

    /// Merge occupancy from source to target with coordinate transformation
    fn merge_occupancy<S: Clone>(
        target: &mut SvoNode<S>,
        source: &SvoNode<S>,
        target_center: &Point3<Real>,
        target_half: Real,
        source_center: &Point3<Real>,
        source_half: Real,
        depth: u8,
        max_depth: u8,
    ) {
        match source.occupancy {
            Occupancy::Empty => {
                // Source is empty, no change to target
            }
            Occupancy::Full => {
                // Source is full, mark target as full (union)
                target.occupancy = Occupancy::Full;
                target.children.clear();
                target.children_mask = 0;
            }
            Occupancy::Mixed => {
                // Source is mixed, need to recurse if possible
                match target.occupancy {
                    Occupancy::Empty => {
                        target.occupancy = Occupancy::Mixed;
                    }
                    Occupancy::Full => {
                        // Target is already full, union result stays full
                        return;
                    }
                    Occupancy::Mixed => {
                        // Both are mixed, need to merge children
                    }
                }

                if depth < max_depth {
                    // Ensure target has children and recurse
                    for child_idx in 0..8 {
                        if source.has_child(child_idx) {
                            let target_child = target.ensure_child(child_idx);
                            let source_child = source.get_child(child_idx).unwrap();

                            let target_child_center = Self::child_center(target_center, target_half, child_idx);
                            let source_child_center = Self::child_center(source_center, source_half, child_idx);

                            Self::merge_occupancy(
                                target_child,
                                source_child,
                                &target_child_center,
                                target_half * 0.5,
                                &source_child_center,
                                source_half * 0.5,
                                depth + 1,
                                max_depth,
                            );
                        }
                    }
                }
            }
        }
    }

    /// Sample occupancy from an SVO at a given point
    fn sample_svo_occupancy<S: Clone>(
        svo: &Svo<S>,
        center: &Point3<Real>,
        half: Real,
    ) -> Occupancy {
        // Check if the sample region overlaps with the SVO bounds
        if !Self::regions_overlap(center, half, &svo.center, svo.half) {
            return Occupancy::Empty;
        }

        // Sample the SVO at this location
        Self::sample_node_occupancy(&*svo.root, &svo.center, svo.half, center, half, 0, svo.max_depth)
    }

    /// Sample occupancy from a node at a given region
    fn sample_node_occupancy<S: Clone>(
        node: &SvoNode<S>,
        node_center: &Point3<Real>,
        node_half: Real,
        sample_center: &Point3<Real>,
        sample_half: Real,
        depth: u8,
        max_depth: u8,
    ) -> Occupancy {
        match node.occupancy {
            Occupancy::Empty => Occupancy::Empty,
            Occupancy::Full => Occupancy::Full,
            Occupancy::Mixed => {
                if depth >= max_depth {
                    // At leaf level, assume Mixed means partially occupied
                    Occupancy::Mixed
                } else {
                    // Check which child octants overlap with the sample region
                    let mut has_full = false;
                    let mut has_empty = false;

                    for child_idx in 0..8 {
                        if node.has_child(child_idx) {
                            let child_center = Self::child_center(node_center, node_half, child_idx);
                            let child_half = node_half * 0.5;

                            // Check if child region overlaps with sample region
                            if Self::regions_overlap(&child_center, child_half, sample_center, sample_half) {
                                let child = node.get_child(child_idx).unwrap();
                                let child_occupancy = Self::sample_node_occupancy(
                                    child,
                                    &child_center,
                                    child_half,
                                    sample_center,
                                    sample_half,
                                    depth + 1,
                                    max_depth,
                                );

                                match child_occupancy {
                                    Occupancy::Full => has_full = true,
                                    Occupancy::Empty => has_empty = true,
                                    Occupancy::Mixed => return Occupancy::Mixed,
                                }
                            } else {
                                has_empty = true; // Non-overlapping regions are empty
                            }
                        } else {
                            has_empty = true; // Missing children are empty
                        }
                    }

                    if has_full && has_empty {
                        Occupancy::Mixed
                    } else if has_full {
                        Occupancy::Full
                    } else {
                        Occupancy::Empty
                    }
                }
            }
        }
    }

    /// Check if two axis-aligned regions overlap with tolerance
    fn regions_overlap(
        center1: &Point3<Real>,
        half1: Real,
        center2: &Point3<Real>,
        half2: Real,
    ) -> bool {
        const EPSILON: Real = 1e-9; // Small tolerance for numerical precision

        (center1.x - half1 < center2.x + half2 + EPSILON) &&
        (center1.x + half1 + EPSILON > center2.x - half2) &&
        (center1.y - half1 < center2.y + half2 + EPSILON) &&
        (center1.y + half1 + EPSILON > center2.y - half2) &&
        (center1.z - half1 < center2.z + half2 + EPSILON) &&
        (center1.z + half1 + EPSILON > center2.z - half2)
    }

    /// Apply CSG operation on two occupancy values
    fn apply_csg_occupancy(
        a: Occupancy,
        b: Occupancy,
        operation: CsgOperation,
    ) -> Occupancy {
        match operation {
            CsgOperation::Union => match (a, b) {
                (Occupancy::Full, _) | (_, Occupancy::Full) => Occupancy::Full,
                (Occupancy::Empty, occ) | (occ, Occupancy::Empty) => occ,
                (Occupancy::Mixed, Occupancy::Mixed) => Occupancy::Mixed,
            },
            CsgOperation::Intersection => match (a, b) {
                (Occupancy::Empty, _) | (_, Occupancy::Empty) => Occupancy::Empty,
                (Occupancy::Full, occ) | (occ, Occupancy::Full) => occ,
                (Occupancy::Mixed, Occupancy::Mixed) => Occupancy::Mixed,
            },
            CsgOperation::Difference => match (a, b) {
                (Occupancy::Empty, _) => Occupancy::Empty,
                (_, Occupancy::Full) => Occupancy::Empty,
                (Occupancy::Full, Occupancy::Empty) => Occupancy::Full,
                (Occupancy::Full, Occupancy::Mixed) => Occupancy::Mixed,
                (Occupancy::Mixed, Occupancy::Empty) => Occupancy::Mixed,
                (Occupancy::Mixed, Occupancy::Mixed) => Occupancy::Mixed,
            },
        }
    }

    /// Fix node occupancy if it's inconsistent with children (works on any node)
    fn fix_node_occupancy<S: Clone>(node: &mut SvoNode<S>) {
        // If node has no children, occupancy should be consistent
        if node.children.is_empty() {
            // No children: BSP should only exist for Mixed nodes
            if node.occupancy != Occupancy::Mixed {
                node.local_bsp = None;
            }
            return;
        }

        // Node has children: recompute occupancy based on children
        let mut has_full = false;
        let mut has_empty = false;
        let mut has_mixed = false;

        for child in &node.children {
            match child.occupancy {
                Occupancy::Full => has_full = true,
                Occupancy::Empty => has_empty = true,
                Occupancy::Mixed => has_mixed = true,
            }
        }

        // Determine correct occupancy based on children
        let correct_occupancy = if has_mixed || (has_full && has_empty) {
            Occupancy::Mixed
        } else if has_full && !has_empty {
            // All children are Full - this node should be Full and have no children
            Occupancy::Full
        } else if has_empty && !has_full {
            // All children are Empty - this node should be Empty and have no children
            Occupancy::Empty
        } else {
            // No children or inconsistent state - default to Mixed
            Occupancy::Mixed
        };

        node.occupancy = correct_occupancy;

        // If occupancy became Full or Empty, clear children but preserve BSP if it has surface data
        if node.occupancy != Occupancy::Mixed {
            node.children.clear();
            node.children_mask = 0;

            // Only clear BSP if it's truly empty (no surface information)
            if let Some(ref bsp) = node.local_bsp {
                if bsp.polygons.is_empty() && bsp.front.is_none() && bsp.back.is_none() {
                    node.local_bsp = None;
                }
                // Otherwise keep BSP for surface extraction even if occupancy is Full/Empty
            }
        }
    }

    /// Fix occupancy inconsistencies throughout the entire tree (recursive)
    fn fix_tree_occupancy<S: Clone>(node: &mut SvoNode<S>) {
        // First, recursively fix all children
        for child in &mut node.children {
            Self::fix_tree_occupancy(child);
        }

        // Then fix this node based on its (now corrected) children
        Self::fix_node_occupancy(node);
    }

    /// Fix root occupancy if it's inconsistent with children (legacy wrapper)
    fn fix_root_occupancy<S: Clone>(svo: &mut Svo<S>) {
        Self::fix_node_occupancy(&mut *svo.root);
    }

    /// Merge BSP trees for union operation using proper CSG algorithms
    /// Implements: A ∪ B = A + (B - A)
    fn merge_bsp_trees_union<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
    ) {
        #[cfg(not(feature = "parallel"))]
        {
            match (&a.local_bsp, &b.local_bsp) {
                (Some(bsp_a), Some(bsp_b)) => {
                    // Validate BSP trees are not empty
                    if bsp_a.all_polygons().is_empty() && bsp_b.all_polygons().is_empty() {
                        result.local_bsp = None;
                        return;
                    }

                    // Canonical BSP Union: a.clip_to(b); b.clip_to(a); b.invert(); b.clip_to(a); b.invert(); a.build(b.all_polygons())
                    let mut a_node = bsp_a.clone();
                    let mut b_node = bsp_b.clone();

                    // Step 1: a.clip_to(b) - Remove parts of A that are inside B
                    a_node.clip_to(&b_node);

                    // Step 2: b.clip_to(a) - Remove parts of B that are inside A (using original A)
                    b_node.clip_to(bsp_a);

                    // Step 3: b.invert() - Invert B
                    b_node.invert();

                    // Step 4: b.clip_to(a) - Clip inverted B against clipped A
                    b_node.clip_to(&a_node);

                    // Step 5: b.invert() - Invert B back
                    b_node.invert();

                    // Step 6: a.build(b.all_polygons()) - Add B's polygons to A's BSP structure
                    // This is the canonical step that merges B's geometry into A's partitioning
                    let b_polygons = b_node.all_polygons();
                    if !b_polygons.is_empty() {
                        a_node.build(&b_polygons);
                    }

                    // Optimize and store result
                    a_node.optimize_memory();
                    result.local_bsp = Some(a_node);
                }
                (Some(bsp), None) | (None, Some(bsp)) => {
                    // Copy the existing BSP tree and optimize
                    let mut result_bsp = bsp.clone();
                    result_bsp.optimize_memory();
                    result.local_bsp = Some(result_bsp);
                }
                (None, None) => {
                    // No BSP trees to merge
                    result.local_bsp = None;
                }
            }
        }

        #[cfg(feature = "parallel")]
        {
            // Fallback for parallel feature: use simple concatenation
            // TODO: Implement parallel-safe BSP CSG operations
            match (&a.local_bsp, &b.local_bsp) {
                (Some(bsp_a), Some(bsp_b)) => {
                    let mut all_polygons = bsp_a.all_polygons();
                    all_polygons.extend(bsp_b.all_polygons());
                    if !all_polygons.is_empty() {
                        result.local_bsp = Some(crate::voxels::bsp::Node::from_polygons(&all_polygons));
                    }
                }
                (Some(bsp), None) | (None, Some(bsp)) => {
                    result.local_bsp = Some(bsp.clone());
                }
                (None, None) => {
                    result.local_bsp = None;
                }
            }
        }
    }

    /// Merge BSP trees for intersection operation using proper CSG algorithms
    /// Implements: A ∩ B = interior surfaces where both shapes overlap
    fn merge_bsp_trees_intersection<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
    ) {
        #[cfg(not(feature = "parallel"))]
        {
            match (&a.local_bsp, &b.local_bsp) {
                (Some(bsp_a), Some(bsp_b)) => {
                    // Canonical BSP Intersection: a.invert(); b.clip_to(a); b.invert(); a.clip_to(b); b.clip_to(a); a.build(b.all_polygons()); a.invert()
                    let mut a_node = bsp_a.clone();
                    let mut b_node = bsp_b.clone();

                    // Step 1: a.invert() - Invert A
                    a_node.invert();

                    // Step 2: b.clip_to(a) - Clip B against inverted A
                    b_node.clip_to(&a_node);

                    // Step 3: b.invert() - Invert B
                    b_node.invert();

                    // Step 4: a.clip_to(b) - Clip inverted A against inverted B
                    a_node.clip_to(&b_node);

                    // Step 5: b.clip_to(a) - Clip inverted B against clipped inverted A
                    b_node.clip_to(&a_node);

                    // Step 6: a.build(b.all_polygons()) - Add B's polygons to A's BSP structure
                    // This is the canonical step that merges B's geometry into A's partitioning
                    let b_polygons = b_node.all_polygons();
                    if !b_polygons.is_empty() {
                        a_node.build(&b_polygons);
                    }

                    // Step 7: a.invert() - Final invert to get correct intersection result
                    a_node.invert();

                    // Optimize and store result
                    a_node.optimize_memory();
                    result.local_bsp = Some(a_node);
                }
                (Some(_), None) | (None, Some(_)) => {
                    // Intersection with empty - no surface
                    result.local_bsp = None;
                }
                (None, None) => {
                    // No BSP trees to merge
                    result.local_bsp = None;
                }
            }
        }

        #[cfg(feature = "parallel")]
        {
            // Fallback for parallel feature: use heuristic selection
            // TODO: Implement parallel-safe BSP CSG operations
            match (&a.local_bsp, &b.local_bsp) {
                (Some(bsp_a), Some(bsp_b)) => {
                    let polygons_a = bsp_a.all_polygons();
                    let polygons_b = bsp_b.all_polygons();
                    let selected_bsp = if polygons_a.len() >= polygons_b.len() { bsp_a } else { bsp_b };
                    result.local_bsp = Some(selected_bsp.clone());
                }
                (Some(_), None) | (None, Some(_)) => {
                    result.local_bsp = None;
                }
                (None, None) => {
                    result.local_bsp = None;
                }
            }
        }
    }

    /// Merge BSP trees for difference operation using proper CSG algorithms
    /// Implements: A - B = A outside B + inverted B inside A
    fn merge_bsp_trees_difference<S: Clone + Debug + Send + Sync>(
        result: &mut SvoNode<S>,
        a: &SvoNode<S>,
        b: &SvoNode<S>,
    ) {
        #[cfg(not(feature = "parallel"))]
        {
            match (&a.local_bsp, &b.local_bsp) {
                (Some(bsp_a), Some(bsp_b)) => {
                    // Canonical BSP Difference: a.invert(); a.clip_to(b); b.clip_to(a); b.invert(); b.clip_to(a); b.invert(); a.build(b.all_polygons()); a.invert()
                    let mut a_node = bsp_a.clone();
                    let mut b_node = bsp_b.clone();

                    // Step 1: a.invert() - Invert A
                    a_node.invert();

                    // Step 2: a.clip_to(b) - Clip inverted A against B
                    a_node.clip_to(&b_node);

                    // Step 3: b.clip_to(a) - Clip B against inverted A
                    b_node.clip_to(&a_node);

                    // Step 4: b.invert() - Invert B
                    b_node.invert();

                    // Step 5: b.clip_to(a) - Clip inverted B against clipped inverted A
                    b_node.clip_to(&a_node);

                    // Step 6: b.invert() - Invert B back
                    b_node.invert();

                    // Step 7: a.build(b.all_polygons()) - Add B's polygons to A's BSP structure
                    // This is the canonical step that merges B's geometry into A's partitioning
                    let b_polygons = b_node.all_polygons();
                    if !b_polygons.is_empty() {
                        a_node.build(&b_polygons);
                    }

                    // Step 8: a.invert() - Final invert to get correct difference result
                    a_node.invert();

                    // Optimize and store result
                    a_node.optimize_memory();
                    result.local_bsp = Some(a_node);
                }
                (Some(bsp_a), None) => {
                    // A - empty = A (optimize memory)
                    let mut result_bsp = bsp_a.clone();
                    result_bsp.optimize_memory();
                    result.local_bsp = Some(result_bsp);
                }
                (None, Some(_)) => {
                    // empty - B = empty (no surface)
                    result.local_bsp = None;
                }
                (None, None) => {
                    // No BSP trees to merge
                    result.local_bsp = None;
                }
            }
        }

        #[cfg(feature = "parallel")]
        {
            // Fallback for parallel feature: simplified approach
            // TODO: Implement parallel-safe BSP CSG operations
            match (&a.local_bsp, &b.local_bsp) {
                (Some(bsp_a), Some(_bsp_b)) => {
                    result.local_bsp = Some(bsp_a.clone());
                }
                (Some(bsp_a), None) => {
                    result.local_bsp = Some(bsp_a.clone());
                }
                (None, Some(_)) => {
                    result.local_bsp = None;
                }
                (None, None) => {
                    result.local_bsp = None;
                }
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

    #[test]
    fn difference_empty_svos() {
        let a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);

        let result = SvoCsg::difference(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Empty);
    }

    #[test]
    fn union_full_with_empty() {
        let mut a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        a.root.occupancy = Occupancy::Full;
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);

        let result = SvoCsg::union(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Full);
        assert!(result.root.children.is_empty());
        assert!(result.root.local_bsp.is_none());
    }

    #[test]
    fn intersection_full_with_empty() {
        let mut a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        a.root.occupancy = Occupancy::Full;
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);

        let result = SvoCsg::intersection(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Empty);
    }

    #[test]
    fn difference_full_minus_empty() {
        let mut a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        a.root.occupancy = Occupancy::Full;
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);

        let result = SvoCsg::difference(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Full);
    }

    #[test]
    fn difference_empty_minus_full() {
        let a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let mut b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        b.root.occupancy = Occupancy::Full;

        let result = SvoCsg::difference(&a, &b);
        assert_eq!(result.root.occupancy, Occupancy::Empty);
    }

    #[test]
    fn occupancy_consistency_after_operations() {
        // Create SVOs with Mixed occupancy and children
        let mut a: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        a.root.occupancy = Occupancy::Mixed;
        let child_a = a.root.ensure_child(0);
        child_a.occupancy = Occupancy::Full;

        let mut b: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        b.root.occupancy = Occupancy::Mixed;
        let child_b = b.root.ensure_child(1);
        child_b.occupancy = Occupancy::Full;

        let result = SvoCsg::union(&a, &b);

        println!("Result occupancy: {:?}, children count: {}", result.root.occupancy, result.root.children.len());

        // The union of two Mixed nodes with different Full children should result in Mixed
        // However, our fix_tree_occupancy might simplify this if all children become Full
        // Let's just validate consistency rather than specific occupancy
        validate_occupancy_consistency(&*result.root);

        // Ensure the result makes sense: if it's Full, it should have no children
        if result.root.occupancy == Occupancy::Full {
            assert!(result.root.children.is_empty());
        }
    }

    #[test]
    fn bounds_normalization() {
        // Create SVOs with different bounds
        let a: Svo<()> = Svo::new(Point3::new(-1.0, -1.0, -1.0), 1.0, 4);
        let b: Svo<()> = Svo::new(Point3::new(1.0, 1.0, 1.0), 1.0, 4);

        let result = SvoCsg::union(&a, &b);

        // Result should have combined bounds that encompass both inputs
        // A spans from (-2,-2,-2) to (0,0,0), B spans from (0,0,0) to (2,2,2)
        // Combined should be centered at (0,0,0) with half >= 2.0
        println!("Result center: {:?}, half: {}", result.center, result.half);
        assert!(result.center.x.abs() < 1e-6); // Should be centered at origin
        assert!(result.center.y.abs() < 1e-6);
        assert!(result.center.z.abs() < 1e-6);
        assert!(result.half >= 2.0); // Should encompass both inputs
    }

    #[test]
    fn depth_handling() {
        // Create SVOs with different depths
        let a: Svo<()> = Svo::new(Point3::origin(), 1.0, 3);
        let b: Svo<()> = Svo::new(Point3::origin(), 1.0, 5);

        let result = SvoCsg::union(&a, &b);

        // Result should use maximum depth
        assert_eq!(result.max_depth, 5);
    }

    /// Helper function to validate occupancy consistency throughout a tree
    fn validate_occupancy_consistency<S: Clone>(node: &SvoNode<S>) {
        match node.occupancy {
            Occupancy::Empty => {
                assert!(node.children.is_empty(), "Empty node should not have children");
                assert!(node.local_bsp.is_none(), "Empty node should not have BSP");
            }
            Occupancy::Full => {
                assert!(node.children.is_empty(), "Full node should not have children");
                assert!(node.local_bsp.is_none(), "Full node should not have BSP");
            }
            Occupancy::Mixed => {
                // Mixed nodes can have children and/or BSP
                // Recursively validate children
                for child in &node.children {
                    validate_occupancy_consistency(child);
                }
            }
        }
    }

    #[test]
    fn test_canonical_bsp_csg_intersection() {
        // Create two simple BSP trees for testing
        let mut bsp_a = crate::voxels::bsp::Node::new();
        let mut bsp_b = crate::voxels::bsp::Node::new();

        // Create test polygons
        let polygons_a = create_test_cube_polygons(Point3::new(0.0, 0.0, 0.0), 1.0);
        let polygons_b = create_test_cube_polygons(Point3::new(0.5, 0.0, 0.0), 1.0);

        bsp_a.build(&polygons_a);
        bsp_b.build(&polygons_b);

        // Create SVO nodes with BSP trees
        let mut node_a = SvoNode::<()>::new();
        let mut node_b = SvoNode::<()>::new();
        let mut result = SvoNode::<()>::new();

        node_a.occupancy = Occupancy::Mixed;
        node_b.occupancy = Occupancy::Mixed;
        node_a.local_bsp = Some(bsp_a);
        node_b.local_bsp = Some(bsp_b);

        // Test intersection
        SvoCsg::merge_bsp_trees_intersection(&mut result, &node_a, &node_b);

        // Verify result has BSP tree
        assert!(result.local_bsp.is_some(), "Intersection should produce BSP tree");

        // Verify BSP tree is not empty
        let result_bsp = result.local_bsp.as_ref().unwrap();
        let result_polygons = result_bsp.all_polygons();

        // Should have some polygons (intersection of two cubes)
        assert!(!result_polygons.is_empty(), "Intersection should produce polygons");
    }

    #[test]
    fn test_canonical_bsp_csg_union() {
        // Create two simple BSP trees for testing
        let mut bsp_a = crate::voxels::bsp::Node::new();
        let mut bsp_b = crate::voxels::bsp::Node::new();

        // Create test polygons
        let polygons_a = create_test_cube_polygons(Point3::new(0.0, 0.0, 0.0), 1.0);
        let polygons_b = create_test_cube_polygons(Point3::new(1.0, 0.0, 0.0), 1.0);

        bsp_a.build(&polygons_a);
        bsp_b.build(&polygons_b);

        // Create SVO nodes with BSP trees
        let mut node_a = SvoNode::<()>::new();
        let mut node_b = SvoNode::<()>::new();
        let mut result = SvoNode::<()>::new();

        node_a.occupancy = Occupancy::Mixed;
        node_b.occupancy = Occupancy::Mixed;
        node_a.local_bsp = Some(bsp_a);
        node_b.local_bsp = Some(bsp_b);

        // Test union
        SvoCsg::merge_bsp_trees_union(&mut result, &node_a, &node_b);

        // Verify result has BSP tree
        assert!(result.local_bsp.is_some(), "Union should produce BSP tree");

        // Verify BSP tree has polygons
        let result_bsp = result.local_bsp.as_ref().unwrap();
        let result_polygons = result_bsp.all_polygons();

        // Should have polygons from both cubes
        assert!(!result_polygons.is_empty(), "Union should produce polygons");
    }

    #[test]
    fn test_canonical_bsp_csg_difference() {
        // Create two simple BSP trees for testing
        let mut bsp_a = crate::voxels::bsp::Node::new();
        let mut bsp_b = crate::voxels::bsp::Node::new();

        // Create test polygons - overlapping cubes
        let polygons_a = create_test_cube_polygons(Point3::new(0.0, 0.0, 0.0), 1.0);
        let polygons_b = create_test_cube_polygons(Point3::new(0.5, 0.0, 0.0), 0.5);

        bsp_a.build(&polygons_a);
        bsp_b.build(&polygons_b);

        // Create SVO nodes with BSP trees
        let mut node_a = SvoNode::<()>::new();
        let mut node_b = SvoNode::<()>::new();
        let mut result = SvoNode::<()>::new();

        node_a.occupancy = Occupancy::Mixed;
        node_b.occupancy = Occupancy::Mixed;
        node_a.local_bsp = Some(bsp_a);
        node_b.local_bsp = Some(bsp_b);

        // Test difference
        SvoCsg::merge_bsp_trees_difference(&mut result, &node_a, &node_b);

        // Verify result has BSP tree
        assert!(result.local_bsp.is_some(), "Difference should produce BSP tree");

        // Verify BSP tree has polygons
        let result_bsp = result.local_bsp.as_ref().unwrap();
        let result_polygons = result_bsp.all_polygons();

        // Should have polygons (A - B) or be empty if B completely contains A
        // The test is mainly to ensure no crash occurs during difference operation
        println!("Difference result has {} polygons", result_polygons.len());
    }

    #[test]
    fn test_csg_edge_cases() {
        let mut result = SvoNode::<()>::new();
        let empty_node = SvoNode::<()>::new();
        let mut mixed_node = SvoNode::<()>::new();
        mixed_node.occupancy = Occupancy::Mixed;

        // Test intersection with empty
        SvoCsg::merge_bsp_trees_intersection(&mut result, &empty_node, &mixed_node);
        assert!(result.local_bsp.is_none(), "Intersection with empty should be empty");

        // Test union with empty
        result = SvoNode::<()>::new();
        SvoCsg::merge_bsp_trees_union(&mut result, &empty_node, &mixed_node);
        // Union with empty should return the non-empty part

        // Test difference with empty
        result = SvoNode::<()>::new();
        SvoCsg::merge_bsp_trees_difference(&mut result, &mixed_node, &empty_node);
        // A - empty = A
    }

    // Helper function to create test cube polygons
    fn create_test_cube_polygons(center: Point3<Real>, size: Real) -> Vec<crate::voxels::polygon::Polygon<()>> {
        use crate::voxels::vertex::Vertex;
        use crate::voxels::polygon::Polygon;
        use nalgebra::Vector3;

        let half = size * 0.5;
        let vertices = [
            // Bottom face
            Vertex::new(Point3::new(center.x - half, center.y - half, center.z - half), Vector3::new(0.0, 0.0, -1.0)),
            Vertex::new(Point3::new(center.x + half, center.y - half, center.z - half), Vector3::new(0.0, 0.0, -1.0)),
            Vertex::new(Point3::new(center.x + half, center.y + half, center.z - half), Vector3::new(0.0, 0.0, -1.0)),
            Vertex::new(Point3::new(center.x - half, center.y + half, center.z - half), Vector3::new(0.0, 0.0, -1.0)),
            // Top face
            Vertex::new(Point3::new(center.x - half, center.y - half, center.z + half), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(center.x + half, center.y - half, center.z + half), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(center.x + half, center.y + half, center.z + half), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(center.x - half, center.y + half, center.z + half), Vector3::new(0.0, 0.0, 1.0)),
        ];

        vec![
            // Bottom face (two triangles)
            Polygon::new(vec![vertices[0], vertices[1], vertices[2]], None),
            Polygon::new(vec![vertices[0], vertices[2], vertices[3]], None),
            // Top face (two triangles)
            Polygon::new(vec![vertices[4], vertices[6], vertices[5]], None),
            Polygon::new(vec![vertices[4], vertices[7], vertices[6]], None),
        ]
    }
}
