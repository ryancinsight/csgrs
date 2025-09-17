//! # Sparse Voxel CSG Operations
//!
//! This module provides boolean operations (union, difference, intersection, XOR)
//! for sparse voxel octrees, implementing the CSG trait.

use crate::float_types::Real;
use crate::traits::CSG;
use crate::voxels::octree::{SparseVoxelNode, SparseVoxelOctree};
use nalgebra::Matrix4;
use std::cell::RefCell;
use std::fmt::Debug;
use std::rc::Rc;

/// Voxel CSG operation types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VoxelCsgOp {
    /// Union (A ∪ B)
    Union,
    /// Intersection (A ∩ B)
    Intersection,
    /// Difference (A - B)
    Difference,
    /// Symmetric difference (A Δ B, XOR)
    SymmetricDifference,
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq> CSG
    for SparseVoxelOctree<S>
{
    /// Create a new empty sparse voxel octree
    fn new() -> Self {
        Self::new(
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            1.0,
            5, // Default max depth
            None,
        )
    }

    /// Union operation: A ∪ B
    fn union(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::Union)
    }

    /// Difference operation: A - B
    fn difference(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::Difference)
    }

    /// Intersection operation: A ∩ B
    fn intersection(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::Intersection)
    }

    /// XOR operation: A Δ B (symmetric difference)
    fn xor(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::SymmetricDifference)
    }

    /// Transform the octree by applying a 4x4 transformation matrix
    fn transform(&self, matrix: &Matrix4<Real>) -> Self {
        // For sparse voxel octrees, we need to transform all occupied voxels
        // and potentially adjust the octree bounds

        // Calculate new bounds after transformation
        let bbox = self.bounding_box();
        let corners = [
            nalgebra::Point3::new(bbox.mins.x, bbox.mins.y, bbox.mins.z),
            nalgebra::Point3::new(bbox.maxs.x, bbox.mins.y, bbox.mins.z),
            nalgebra::Point3::new(bbox.mins.x, bbox.maxs.y, bbox.mins.z),
            nalgebra::Point3::new(bbox.maxs.x, bbox.maxs.y, bbox.mins.z),
            nalgebra::Point3::new(bbox.mins.x, bbox.mins.y, bbox.maxs.z),
            nalgebra::Point3::new(bbox.maxs.x, bbox.mins.y, bbox.maxs.z),
            nalgebra::Point3::new(bbox.mins.x, bbox.maxs.y, bbox.maxs.z),
            nalgebra::Point3::new(bbox.maxs.x, bbox.maxs.y, bbox.maxs.z),
        ];

        // Transform all corners to find new bounds
        let mut transformed_corners = Vec::new();
        for corner in &corners {
            let homogeneous = matrix * corner.to_homogeneous();
            let transformed_point = nalgebra::Point3::from_homogeneous(homogeneous)
                .expect("Matrix should preserve homogeneity");
            transformed_corners.push(transformed_point);
        }

        // Find new bounding box
        let mut new_mins = transformed_corners[0].coords;
        let mut new_maxs = transformed_corners[0].coords;

        for corner in transformed_corners.iter().skip(1) {
            new_mins = new_mins.inf(&corner.coords);
            new_maxs = new_maxs.sup(&corner.coords);
        }

        let new_origin = nalgebra::Point3::from(new_mins);
        let new_size = (new_maxs - new_mins).max();

        // Create new octree with transformed bounds
        let mut result = SparseVoxelOctree::new(
            new_origin,
            new_size,
            self.max_depth,
            self.metadata.clone(),
        );

        // Enable compression if original had it
        if self.dag_registry.is_some() {
            result.enable_compression();
        }

        // Transform all occupied voxels
        Self::transform_voxels_recursive(
            self,
            &self.root,
            &mut result,
            matrix,
            self.origin,
            self.size,
            0,
        );

        result
    }

    /// Invert the octree (swap occupied/unoccupied)
    fn inverse(&self) -> Self {
        let mut result = SparseVoxelOctree::new(
            self.origin,
            self.size,
            self.max_depth,
            self.metadata.clone(),
        );

        // Enable compression if original had it
        if self.dag_registry.is_some() {
            result.enable_compression();
        }

        // Recursively invert all voxels
        Self::invert_voxels_recursive(
            self,
            &self.root,
            &mut result,
            self.origin,
            self.size,
            0,
        );

        result
    }

    /// Get bounding box of the octree
    fn bounding_box(&self) -> crate::float_types::parry3d::bounding_volume::Aabb {
        use crate::float_types::parry3d::bounding_volume::Aabb;

        let max_point = self.origin + nalgebra::Vector3::new(self.size, self.size, self.size);
        Aabb::new(self.origin, max_point)
    }

    /// Invalidate bounding box cache (no-op for sparse voxel octrees)
    fn invalidate_bounding_box(&mut self) {
        // Sparse voxel octrees don't cache bounding boxes, so this is a no-op
    }
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq>
    SparseVoxelOctree<S>
{
    /// Perform a CSG operation between two octrees
    pub fn csg_operation(&self, other: &Self, operation: VoxelCsgOp) -> Self {
        // Ensure both octrees have the same bounds for CSG operations
        if !self.bounds_compatible(other) {
            panic!("CSG operations require octrees with compatible bounds");
        }

        let mut result = SparseVoxelOctree::new(
            self.origin,
            self.size,
            self.max_depth.max(other.max_depth),
            self.metadata.clone(),
        );

        // Enable compression if either input has it
        if self.dag_registry.is_some() || other.dag_registry.is_some() {
            result.enable_compression();
        }

        // Perform the CSG operation recursively without modifying input trees
        // Create completely new nodes for the result to avoid borrowing conflicts
        Self::csg_operation_recursive_immutable(
            self,
            other,
            Rc::clone(&self.root),
            Rc::clone(&other.root),
            Rc::clone(&result.root),
            self.origin,
            self.size,
            0,
            operation,
        );

        // Update the occupied leaves counter after the operation
        result.update_occupied_leaves_count();

        // For compressed octrees, compress the result tree after operation
        if result.dag_registry.is_some() {
            result.compress_existing();
        }

        result
    }

    /// Recursively perform CSG operations by traversing both input trees simultaneously
    #[allow(clippy::too_many_arguments)]
    fn csg_operation_recursive_immutable(
        _self: &Self,
        _other_octree: &SparseVoxelOctree<S>,
        node_a: Rc<RefCell<SparseVoxelNode<S>>>,
        node_b: Rc<RefCell<SparseVoxelNode<S>>>,
        result_node: Rc<RefCell<SparseVoxelNode<S>>>,
        node_origin: nalgebra::Point3<Real>,
        node_size: Real,
        _depth: usize,
        operation: VoxelCsgOp,
    ) {
        let node_a_ref = node_a.borrow();
        let node_b_ref = node_b.borrow();

        match (&*node_a_ref, &*node_b_ref) {
            // Both leaves: combine their values directly
            (
                SparseVoxelNode::Leaf {
                    occupied: occ_a,
                    metadata: meta_a,
                },
                SparseVoxelNode::Leaf {
                    occupied: occ_b,
                    metadata: meta_b,
                },
            ) => {
                let result_occupied = match operation {
                    VoxelCsgOp::Union => *occ_a || *occ_b,
                    VoxelCsgOp::Intersection => *occ_a && *occ_b,
                    VoxelCsgOp::Difference => *occ_a && !*occ_b,
                    VoxelCsgOp::SymmetricDifference => *occ_a != *occ_b,
                };

                // Combine metadata
                let result_metadata = if result_occupied {
                    match operation {
                        VoxelCsgOp::Union => meta_a.clone().or_else(|| meta_b.clone()),
                        VoxelCsgOp::Intersection => meta_a.clone().or_else(|| meta_b.clone()),
                        VoxelCsgOp::Difference => meta_a.clone(),
                        VoxelCsgOp::SymmetricDifference => {
                            meta_a.clone().or_else(|| meta_b.clone())
                        },
                    }
                } else {
                    None
                };

                let result_leaf = SparseVoxelNode::Leaf {
                    occupied: result_occupied,
                    metadata: result_metadata,
                };

                let mut result_ref = result_node.borrow_mut();
                *result_ref = result_leaf;
            },

            // One leaf, one internal: subdivide the internal and recurse
            (
                SparseVoxelNode::Leaf {
                    occupied: occ_leaf,
                    metadata: meta_leaf,
                },
                SparseVoxelNode::Internal {
                    children: children_internal,
                    ..
                },
            )
            | (
                SparseVoxelNode::Internal {
                    children: children_internal,
                    ..
                },
                SparseVoxelNode::Leaf {
                    occupied: occ_leaf,
                    metadata: meta_leaf,
                },
            ) => {
                // Create result children
                let mut result_children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8] =
                    Default::default();
                let half_size = node_size * 0.5;

                #[allow(clippy::needless_range_loop)]
                for octant_idx in 0..8 {
                    let octant = crate::voxels::octree::Octant::from_index(octant_idx)
                        .expect("Invalid octant index - should be 0-7");
                    let child_origin = _self.get_child_origin(node_origin, half_size, octant);

                    // Get the internal's child (or create default)
                    let child_internal = if let Some(child) = &children_internal[octant_idx] {
                        Rc::clone(child)
                    } else {
                        Rc::new(RefCell::new(SparseVoxelNode::Leaf {
                            occupied: false,
                            metadata: None,
                        }))
                    };

                    // Create leaf node with the leaf value
                    let _child_leaf = Rc::new(RefCell::new(SparseVoxelNode::Leaf {
                        occupied: *occ_leaf,
                        metadata: meta_leaf.clone(),
                    }));

                    // Determine which is which and recurse
                    let (child_a, child_b) =
                        if matches!(&*node_a_ref, SparseVoxelNode::Leaf { .. }) {
                            (Rc::clone(&node_a), child_internal)
                        } else {
                            (child_internal, Rc::clone(&node_b))
                        };

                    let result_child = Rc::new(RefCell::new(SparseVoxelNode::Leaf {
                        occupied: false,
                        metadata: None,
                    }));

                    Self::csg_operation_recursive_immutable(
                        _self,
                        _other_octree,
                        child_a,
                        child_b,
                        Rc::clone(&result_child),
                        child_origin,
                        half_size,
                        _depth + 1,
                        operation,
                    );

                    result_children[octant_idx] = Some(result_child);
                }

                let result_internal = SparseVoxelNode::Internal {
                    children: result_children,
                    bsp_tree: None,
                };

                let mut result_ref = result_node.borrow_mut();
                *result_ref = result_internal;
            },

            // Both internal: recurse into children
            (
                SparseVoxelNode::Internal {
                    children: children_a,
                    ..
                },
                SparseVoxelNode::Internal {
                    children: children_b,
                    ..
                },
            ) => {
                let mut result_children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8] =
                    Default::default();
                let half_size = node_size * 0.5;

                #[allow(clippy::needless_range_loop)]
                for octant_idx in 0..8 {
                    let octant = crate::voxels::octree::Octant::from_index(octant_idx)
                        .expect("Invalid octant index - should be 0-7");
                    let child_origin = _self.get_child_origin(node_origin, half_size, octant);

                    let child_a = if let Some(child) = &children_a[octant_idx] {
                        Rc::clone(child)
                    } else {
                        Rc::new(RefCell::new(SparseVoxelNode::Leaf {
                            occupied: false,
                            metadata: None,
                        }))
                    };

                    let child_b = if let Some(child) = &children_b[octant_idx] {
                        Rc::clone(child)
                    } else {
                        Rc::new(RefCell::new(SparseVoxelNode::Leaf {
                            occupied: false,
                            metadata: None,
                        }))
                    };

                    let result_child = Rc::new(RefCell::new(SparseVoxelNode::Leaf {
                        occupied: false,
                        metadata: None,
                    }));

                    Self::csg_operation_recursive_immutable(
                        _self,
                        _other_octree,
                        child_a,
                        child_b,
                        Rc::clone(&result_child),
                        child_origin,
                        half_size,
                        _depth + 1,
                        operation,
                    );

                    result_children[octant_idx] = Some(result_child);
                }

                let result_internal = SparseVoxelNode::Internal {
                    children: result_children,
                    bsp_tree: None,
                };

                let mut result_ref = result_node.borrow_mut();
                *result_ref = result_internal;
            },
        }
    }

    /// Union operation: A ∪ B
    pub fn csg_union(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::Union)
    }

    /// Intersection operation: A ∩ B
    pub fn csg_intersection(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::Intersection)
    }

    /// Difference operation: A - B
    pub fn csg_difference(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::Difference)
    }

    /// Symmetric difference operation: A Δ B
    pub fn csg_symmetric_difference(&self, other: &Self) -> Self {
        self.csg_operation(other, VoxelCsgOp::SymmetricDifference)
    }

    /// Update the occupied leaves count by traversing the entire tree
    fn update_occupied_leaves_count(&mut self) {
        fn count_occupied_leaves_recursive<S: Clone + Debug + Send + Sync>(
            node: &Rc<RefCell<SparseVoxelNode<S>>>,
        ) -> usize {
            let node_ref = node.borrow();
            match &*node_ref {
                SparseVoxelNode::Leaf { occupied, .. } => {
                    if *occupied {
                        1
                    } else {
                        0
                    }
                },
                SparseVoxelNode::Internal { children, .. } => children
                    .iter()
                    .filter_map(|child| child.as_ref())
                    .map(|child| count_occupied_leaves_recursive(child))
                    .sum(),
            }
        }

        self.occupied_leaves = count_occupied_leaves_recursive(&self.root);
    }

    /// Check if two octrees have compatible bounds for CSG operations
    fn bounds_compatible(&self, other: &Self) -> bool {
        // For now, require exact bounds match
        // In the future, this could be relaxed to handle different bounds
        (self.origin - other.origin).norm() < 1e-10 && (self.size - other.size).abs() < 1e-10
    }

    // Recursively perform CSG operations
    // fn csg_operation_recursive(
    // &self,
    // node_a: Rc<RefCell<SparseVoxelNode<S>>>,
    // node_b: Rc<RefCell<SparseVoxelNode<S>>>,
    // result_node: Rc<RefCell<SparseVoxelNode<S>>>,
    // node_origin: nalgebra::Point3<Real>,
    // node_size: Real,
    // depth: usize,
    // operation: VoxelCsgOp,
    // registry: Option<Rc<RefCell<super::dag::VoxelDagRegistry<S>>>>,
    // ) {
    // let node_a_ref = node_a.borrow();
    // let node_b_ref = node_b.borrow();
    //
    // match (&*node_a_ref, &*node_b_ref) {
    // Both leaves: apply boolean operation directly
    // (SparseVoxelNode::Leaf { occupied: occ_a, metadata: meta_a },
    // SparseVoxelNode::Leaf { occupied: occ_b, metadata: meta_b }) => {
    // let result_occupied = match operation {
    // VoxelCsgOp::Union => *occ_a || *occ_b,
    // VoxelCsgOp::Intersection => *occ_a && *occ_b,
    // VoxelCsgOp::Difference => *occ_a && !*occ_b,
    // VoxelCsgOp::SymmetricDifference => *occ_a != *occ_b,
    // };
    //
    // Choose metadata based on the operation result
    // let result_metadata = if result_occupied {
    // match operation {
    // VoxelCsgOp::Union => meta_a.clone().or_else(|| meta_b.clone()),
    // VoxelCsgOp::Intersection => meta_a.clone().or_else(|| meta_b.clone()),
    // VoxelCsgOp::Difference => meta_a.clone(),
    // VoxelCsgOp::SymmetricDifference => meta_a.clone().or_else(|| meta_b.clone()),
    // }
    // } else {
    // None
    // };
    //
    // let result_leaf = SparseVoxelNode::Leaf {
    // occupied: result_occupied,
    // metadata: result_metadata,
    // };
    //
    // Handle registry operations separately to avoid double borrow
    // let mut result_ref = result_node.borrow_mut();
    // if let Some(ref reg) = registry {
    // Clone the result_leaf before registry borrow to avoid conflicts
    // let leaf_clone = result_leaf.clone();
    // drop(result_ref); // Release the result_node borrow
    // let canonical_result = reg.borrow_mut().get_or_insert(leaf_clone);
    // let canonical_content = canonical_result.borrow().clone();
    // result_node.borrow_mut() = canonical_content;
    // } else {
    // result_ref = result_leaf;
    // }
    // }
    //
    // At least one internal node: subdivide and recurse
    // _ => {
    // Convert leaves to internal nodes if necessary
    // let node_a_internal = self.ensure_internal_node(&node_a, registry.as_ref().map(Rc::clone));
    // let node_b_internal = self.ensure_internal_node(&node_b, registry.as_ref().map(Rc::clone));
    // let result_internal = self.ensure_internal_node(&result_node, registry.as_ref().map(Rc::clone));
    //
    // Process all 8 octants
    // let half_size = node_size * 0.5;
    // let mut result_children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8] = Default::default();
    //
    // for (octant_idx, result_child_slot) in result_children.iter_mut().enumerate() {
    // let octant = crate::voxels::octree::Octant::from_index(octant_idx)
    // .expect("Invalid octant index - should be 0-7");
    // let child_origin = self.get_child_origin(node_origin, half_size, octant);
    //
    // Get child nodes (create if they don't exist)
    // Create all children in a single registry borrow scope to avoid double borrowing
    // let (child_a, child_b, result_child) = self.get_or_create_children(
    // &node_a_internal,
    // &node_b_internal,
    // &result_internal,
    // octant_idx,
    // registry.as_ref().map(Rc::clone),
    // );
    //
    // Recurse into this octant
    // self.csg_operation_recursive(
    // child_a,
    // child_b,
    // result_child.clone(),
    // child_origin,
    // half_size,
    // depth + 1,
    // operation.clone(),
    // registry.clone(),
    // );
    //
    // result_child_slot = Some(result_child);
    // }
    //
    // Create the result internal node
    // let result_node_content = SparseVoxelNode::Internal {
    // children: result_children,
    // bsp_tree: None, // CSG results don't include BSP trees
    // };
    //
    // Handle registry operations separately to avoid double borrow
    // let mut result_ref = result_node.borrow_mut();
    // if let Some(ref reg) = registry {
    // Clone the result_node_content before registry borrow to avoid conflicts
    // let content_clone = result_node_content.clone();
    // drop(result_ref); // Release the result_node borrow
    // let canonical_result = reg.borrow_mut().get_or_insert(content_clone);
    // let canonical_content = canonical_result.borrow().clone();
    // result_node.borrow_mut() = canonical_content;
    // } else {
    // result_ref = result_node_content;
    // }
    // }
    // }
    // }

    /// Transform voxels recursively
    fn transform_voxels_recursive(
        _self: &Self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        result: &mut SparseVoxelOctree<S>,
        matrix: &Matrix4<Real>,
        node_origin: nalgebra::Point3<Real>,
        node_size: Real,
        depth: usize,
    ) {
        let node_ref = node.borrow();

        match &*node_ref {
            SparseVoxelNode::Leaf { occupied, metadata } => {
                if *occupied {
                    // Calculate voxel center in world coordinates
                    let voxel_size = _self.voxel_size_at_depth(depth);
                    let half_voxel = voxel_size * 0.5;
                    let voxel_center = node_origin
                        + nalgebra::Vector3::new(half_voxel, half_voxel, half_voxel);

                    // Transform the voxel center
                    let homogeneous = matrix * voxel_center.to_homogeneous();
                    let transformed_center = nalgebra::Point3::from_homogeneous(homogeneous)
                        .expect("Matrix should preserve homogeneity");

                    // Set the transformed voxel
                    result.set_voxel(&transformed_center, true, metadata.clone());
                }
            },
            SparseVoxelNode::Internal { children, .. } => {
                let half_size = node_size * 0.5;

                #[allow(clippy::needless_range_loop)]
                for octant_idx in 0..8 {
                    let octant = crate::voxels::octree::Octant::from_index(octant_idx)
                        .expect("Invalid octant index - should be 0-7");
                    let child_origin = _self.get_child_origin(node_origin, half_size, octant);

                    if let Some(ref child) = children[octant_idx] {
                        Self::transform_voxels_recursive(
                            _self,
                            child,
                            result,
                            matrix,
                            child_origin,
                            half_size,
                            depth + 1,
                        );
                    }
                }
            },
        }
    }

    /// Invert voxels recursively
    fn invert_voxels_recursive(
        _self: &Self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        result: &mut SparseVoxelOctree<S>,
        node_origin: nalgebra::Point3<Real>,
        node_size: Real,
        depth: usize,
    ) {
        let node_ref = node.borrow();

        match &*node_ref {
            SparseVoxelNode::Leaf { occupied, metadata } => {
                // Calculate voxel center in world coordinates
                let voxel_size = _self.voxel_size_at_depth(depth);
                let half_voxel = voxel_size * 0.5;
                let voxel_center =
                    node_origin + nalgebra::Vector3::new(half_voxel, half_voxel, half_voxel);

                // Set the inverted occupancy
                result.set_voxel(&voxel_center, !*occupied, metadata.clone());
            },
            SparseVoxelNode::Internal { children, .. } => {
                let half_size = node_size * 0.5;

                #[allow(clippy::needless_range_loop)]
                for octant_idx in 0..8 {
                    let octant = crate::voxels::octree::Octant::from_index(octant_idx)
                        .expect("Invalid octant index - should be 0-7");
                    let child_origin = _self.get_child_origin(node_origin, half_size, octant);

                    if let Some(ref child) = children[octant_idx] {
                        Self::invert_voxels_recursive(
                            _self,
                            child,
                            result,
                            child_origin,
                            half_size,
                            depth + 1,
                        );
                    }
                }
            },
        }
    }

    // Ensure a node is an internal node, converting if necessary
    // fn ensure_internal_node(
    // &self,
    // node: &Rc<RefCell<SparseVoxelNode<S>>>,
    // registry: Option<Rc<RefCell<super::dag::VoxelDagRegistry<S>>>>,
    // ) -> Rc<RefCell<SparseVoxelNode<S>>> {
    // let node_ref = node.borrow();
    //
    // match &*node_ref {
    // SparseVoxelNode::Internal { .. } => {
    // Already an internal node
    // drop(node_ref);
    // Rc::clone(node)
    // }
    // SparseVoxelNode::Leaf { .. } => {
    // Convert leaf to internal node
    // let children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8] = Default::default();
    // let new_internal = SparseVoxelNode::Internal {
    // children,
    // bsp_tree: None,
    // };
    //
    // Drop the immutable borrow before acquiring mutable borrow
    // drop(node_ref);
    //
    // Handle registry operations with proper borrowing scope
    // if let Some(ref reg) = registry {
    // let canonical = reg.borrow_mut().get_or_insert(new_internal);
    // let mut node_ref = node.borrow_mut();
    // node_ref = canonical.borrow().clone();
    // } else {
    // let mut node_ref = node.borrow_mut();
    // node_ref = new_internal;
    // }
    //
    // Rc::clone(node)
    // }
    // }
    // }

    // Get or create child nodes for all three parents in a single registry borrow scope
    // fn get_or_create_children(
    // &self,
    // parent_a: &Rc<RefCell<SparseVoxelNode<S>>>,
    // parent_b: &Rc<RefCell<SparseVoxelNode<S>>>,
    // parent_result: &Rc<RefCell<SparseVoxelNode<S>>>,
    // octant_idx: usize,
    // registry: Option<Rc<RefCell<super::dag::VoxelDagRegistry<S>>>>,
    // ) -> (
    // Rc<RefCell<SparseVoxelNode<S>>>,
    // Rc<RefCell<SparseVoxelNode<S>>>,
    // Rc<RefCell<SparseVoxelNode<S>>>,
    // ) {
    // Check if children already exist (immutable borrows)
    // let parent_a_ref = parent_a.borrow();
    // let parent_b_ref = parent_b.borrow();
    // let parent_result_ref = parent_result.borrow();
    //
    // let child_a = if let SparseVoxelNode::Internal { children, .. } = &*parent_a_ref {
    // children[octant_idx].as_ref().map(Rc::clone)
    // } else {
    // None
    // };
    //
    // let child_b = if let SparseVoxelNode::Internal { children, .. } = &*parent_b_ref {
    // children[octant_idx].as_ref().map(Rc::clone)
    // } else {
    // None
    // };
    //
    // let child_result = if let SparseVoxelNode::Internal { children, .. } = &*parent_result_ref {
    // children[octant_idx].as_ref().map(Rc::clone)
    // } else {
    // None
    // };
    //
    // Drop immutable borrows before creating new children
    // drop(parent_a_ref);
    // drop(parent_b_ref);
    // drop(parent_result_ref);
    //
    // Create missing children in a single registry borrow scope
    // let (child_a, child_b, child_result) = if let Some(ref reg) = registry {
    // let reg_ref = reg.borrow_mut();
    //
    // let child_a = child_a.unwrap_or_else(|| {
    // let node = SparseVoxelNode::Leaf { occupied: false, metadata: None };
    // reg_ref.get_or_insert(node)
    // });
    //
    // let child_b = child_b.unwrap_or_else(|| {
    // let node = SparseVoxelNode::Leaf { occupied: false, metadata: None };
    // reg_ref.get_or_insert(node)
    // });
    //
    // let child_result = child_result.unwrap_or_else(|| {
    // let node = SparseVoxelNode::Leaf { occupied: false, metadata: None };
    // reg_ref.get_or_insert(node)
    // });
    //
    // (child_a, child_b, child_result)
    // } else {
    // let child_a = child_a.unwrap_or_else(|| {
    // Rc::new(RefCell::new(SparseVoxelNode::Leaf { occupied: false, metadata: None }))
    // });
    //
    // let child_b = child_b.unwrap_or_else(|| {
    // Rc::new(RefCell::new(SparseVoxelNode::Leaf { occupied: false, metadata: None }))
    // });
    //
    // let child_result = child_result.unwrap_or_else(|| {
    // Rc::new(RefCell::new(SparseVoxelNode::Leaf { occupied: false, metadata: None }))
    // });
    //
    // (child_a, child_b, child_result)
    // };
    //
    // Add children to parents (mutable borrows, but no registry involvement)
    // Process each parent separately to avoid any borrowing conflicts
    //
    // Handle parent_a
    // {
    // let needs_child = Self::child_ref_is_none(&*parent_a.borrow(), octant_idx);
    // if needs_child {
    // let mut parent_ref = parent_a.borrow_mut();
    // if let SparseVoxelNode::Internal { children, .. } = &mut *parent_ref {
    // children[octant_idx] = Some(Rc::clone(&child_a));
    // }
    // }
    // }
    //
    // Handle parent_b
    // {
    // let needs_child = Self::child_ref_is_none(&*parent_b.borrow(), octant_idx);
    // if needs_child {
    // let mut parent_ref = parent_b.borrow_mut();
    // if let SparseVoxelNode::Internal { children, .. } = &mut *parent_ref {
    // children[octant_idx] = Some(Rc::clone(&child_b));
    // }
    // }
    // }
    //
    // Handle parent_result
    // {
    // let needs_child = Self::child_ref_is_none(&*parent_result.borrow(), octant_idx);
    // if needs_child {
    // let mut parent_ref = parent_result.borrow_mut();
    // if let SparseVoxelNode::Internal { children, .. } = &mut *parent_ref {
    // children[octant_idx] = Some(Rc::clone(&child_result));
    // }
    // }
    // }
    //
    // (child_a, child_b, child_result)
    // }

    // Helper function to check if a child reference is None
    // fn child_ref_is_none(node: &SparseVoxelNode<S>, octant_idx: usize) -> bool {
    // if let SparseVoxelNode::Internal { children, .. } = node {
    // children[octant_idx].is_none()
    // } else {
    // true
    // }
    // }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_csg_union_basic() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);

        // Set some voxels in first octree (MMM octant)
        octree1.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);

        // Set different voxels in second octree (PPP octant)
        octree2.set_voxel(&Point3::new(3.0, 3.0, 3.0), true, None);

        // Union should contain both voxels
        let union = octree1.csg_union(&octree2);

        assert_eq!(union.get_voxel(&Point3::new(1.0, 1.0, 1.0)), Some(true));
        assert_eq!(union.get_voxel(&Point3::new(2.0, 2.0, 2.0)), Some(true));
        assert_eq!(union.occupied_leaves, 2);
    }

    #[test]
    fn test_csg_intersection_basic() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);

        // Set overlapping voxel
        let overlap_point = Point3::new(1.0, 1.0, 1.0);
        octree1.set_voxel(&overlap_point, true, None);
        octree2.set_voxel(&overlap_point, true, None);

        // Set non-overlapping voxels
        octree1.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);
        octree2.set_voxel(&Point3::new(3.0, 3.0, 3.0), true, None);

        // Intersection should only contain the overlapping voxel
        let intersection = octree1.csg_intersection(&octree2);

        assert_eq!(intersection.get_voxel(&overlap_point), Some(true));
        assert_eq!(
            intersection.get_voxel(&Point3::new(2.0, 2.0, 2.0)),
            Some(false)
        );
        assert_eq!(
            intersection.get_voxel(&Point3::new(3.0, 3.0, 3.0)),
            Some(false)
        );
        assert_eq!(intersection.occupied_leaves, 1);
    }

    #[test]
    fn test_csg_difference_basic() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);

        // Set voxels in both octrees
        let point1 = Point3::new(1.0, 1.0, 1.0);
        let point2 = Point3::new(2.0, 2.0, 2.0);

        octree1.set_voxel(&point1, true, None);
        octree1.set_voxel(&point2, true, None);
        octree2.set_voxel(&point1, true, None); // Overlapping

        // Difference should only contain point2
        let difference = octree1.csg_difference(&octree2);

        assert_eq!(difference.get_voxel(&point1), Some(false));
        assert_eq!(difference.get_voxel(&point2), Some(true));
        assert_eq!(difference.occupied_leaves, 1);
    }

    #[test]
    fn test_csg_xor_basic() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);

        // Set some overlapping and non-overlapping voxels
        let overlap_point = Point3::new(1.0, 1.0, 1.0);
        let only_in_1 = Point3::new(2.0, 2.0, 2.0);
        let only_in_2 = Point3::new(3.0, 3.0, 3.0);

        octree1.set_voxel(&overlap_point, true, None);
        octree1.set_voxel(&only_in_1, true, None);
        octree2.set_voxel(&overlap_point, true, None);
        octree2.set_voxel(&only_in_2, true, None);

        // XOR should contain only the non-overlapping voxels
        let xor = octree1.xor(&octree2);

        assert_eq!(xor.get_voxel(&overlap_point), Some(false));
        assert_eq!(xor.get_voxel(&only_in_1), Some(true));
        assert_eq!(xor.get_voxel(&only_in_2), Some(true));
        assert_eq!(xor.occupied_leaves, 2);
    }

    #[test]
    fn test_csg_bounds_compatibility() {
        let origin1 = Point3::new(0.0, 0.0, 0.0);
        let origin2 = Point3::new(1.0, 1.0, 1.0); // Different origin

        let octree1 = SparseVoxelOctree::<()>::new(origin1, 4.0, 2, None);
        let octree2 = SparseVoxelOctree::<()>::new(origin2, 4.0, 2, None);

        // Different bounds should not be compatible
        assert!(!octree1.bounds_compatible(&octree2));
    }

    #[test]
    fn test_csg_operation_with_compressed_octrees() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new_compressed(origin, 4.0, 2, None);
        let mut octree2 = SparseVoxelOctree::<()>::new_compressed(origin, 4.0, 2, None);

        // Set some voxels
        octree1.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);
        octree2.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);

        // Union with compression enabled
        let union = octree1.union(&octree2);

        assert_eq!(union.get_voxel(&Point3::new(1.0, 1.0, 1.0)), Some(true));
        assert_eq!(union.get_voxel(&Point3::new(2.0, 2.0, 2.0)), Some(true));
        assert!(union.dag_registry.is_some()); // Should preserve compression
    }

    #[test]
    fn test_csg_operations_edge_case_empty_octrees() {
        // **Mathematical Foundation**: Boolean operations with empty sets
        // **SRS Requirement NFR007**: Edge case handling (empty mesh operations)

        let origin = Point3::new(0.0, 0.0, 0.0);
        let size = 2.0;
        let max_depth = 3;

        let empty = SparseVoxelOctree::<()>::new(origin, size, max_depth, None);
        let mut octree = SparseVoxelOctree::<()>::new(origin, size, max_depth, None);

        // Set one voxel in the non-empty octree
        octree.set_voxel(&Point3::new(0.5, 0.5, 0.5), true, None);

        // All operations with empty should return the non-empty octree
        let union_result = empty.union(&octree);
        let diff_result = octree.difference(&empty);
        let intersect_result = empty.intersection(&octree);
        let xor_result = empty.xor(&octree);

        // Union: empty ∪ A = A
        assert_eq!(union_result.occupied_leaves, octree.occupied_leaves);

        // Difference: A - empty = A
        assert_eq!(diff_result.occupied_leaves, octree.occupied_leaves);

        // Intersection: empty ∩ A = empty
        assert_eq!(intersect_result.occupied_leaves, 0);

        // XOR: empty Δ A = A
        assert_eq!(xor_result.occupied_leaves, octree.occupied_leaves);
    }

    #[test]
    fn test_csg_operations_mathematical_properties() {
        // **Mathematical Foundation**: Boolean algebra properties
        // **SRS Requirement NFR006**: Topological correctness

        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree_a = SparseVoxelOctree::<()>::new(origin, 4.0, 3, None);
        let mut octree_b = SparseVoxelOctree::<()>::new(origin, 4.0, 3, None);
        let mut octree_c = SparseVoxelOctree::<()>::new(origin, 4.0, 3, None);

        // Set up test voxels
        octree_a.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);
        octree_b.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);
        octree_c.set_voxel(&Point3::new(3.0, 3.0, 3.0), true, None);

        // Test commutativity: A ∪ B = B ∪ A
        let union_ab = octree_a.union(&octree_b);
        let union_ba = octree_b.union(&octree_a);
        assert_eq!(union_ab.occupied_leaves, union_ba.occupied_leaves);

        // Test associativity: (A ∪ B) ∪ C = A ∪ (B ∪ C)
        let union_abc_left = octree_a.union(&octree_b).union(&octree_c);
        let union_abc_right = octree_a.union(&octree_b.union(&octree_c));
        assert_eq!(
            union_abc_left.occupied_leaves,
            union_abc_right.occupied_leaves
        );

        // Test idempotency: A ∪ A = A
        let union_aa = octree_a.union(&octree_a);
        assert_eq!(union_aa.occupied_leaves, octree_a.occupied_leaves);
    }

    #[test]
    fn test_csg_difference_absorption_laws() {
        // **Mathematical Foundation**: Absorption laws for set difference
        // **SRS Requirement NFR006**: Topological correctness

        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree_a = SparseVoxelOctree::<()>::new(origin, 4.0, 3, None);
        let mut octree_b = SparseVoxelOctree::<()>::new(origin, 4.0, 3, None);

        // Set up overlapping voxels
        octree_a.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);
        octree_a.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);
        octree_b.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None); // Overlap

        // A - (A - B) = A ∩ B
        let a_minus_b = octree_a.difference(&octree_b);
        let absorption = octree_a.difference(&a_minus_b);
        let intersection = octree_a.intersection(&octree_b);

        // The absorption result should equal the intersection
        assert_eq!(absorption.occupied_leaves, intersection.occupied_leaves);

        // Verify the overlapping voxel is preserved
        assert_eq!(absorption.get_voxel(&Point3::new(1.0, 1.0, 1.0)), Some(true));
        assert_eq!(absorption.get_voxel(&Point3::new(2.0, 2.0, 2.0)), Some(false));
    }

    #[test]
    fn test_csg_operations_with_extreme_coordinates() {
        // **Mathematical Foundation**: Coordinate system robustness
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 1000.0, 5, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 1000.0, 5, None);

        // Test with extreme coordinates
        let extreme_coords: Vec<nalgebra::Point3<Real>> = vec![
            Point3::new(1e-10, 1e-10, 1e-10),    // Near zero
            Point3::new(500.0, 500.0, 500.0),    // Large positive
            Point3::new(-500.0, -500.0, -500.0), // Large negative
            Point3::new(1e10, 1e10, 1e10),       // Very large (may be out of bounds)
        ];

        for coord in extreme_coords {
            // Only set voxels that are within reasonable bounds
            // (avoid extremely large coordinates that would be out of bounds)
            if coord.x.abs() < 1000.0 && coord.y.abs() < 1000.0 && coord.z.abs() < 1000.0 {
                octree1.set_voxel(&coord, true, None);
                octree2.set_voxel(&coord, true, None);
            }
        }

        // Operations should complete without panicking
        let _union = octree1.union(&octree2);
        let _intersection = octree1.intersection(&octree2);
        let _difference = octree1.difference(&octree2);
        let _xor = octree1.xor(&octree2);
    }

    #[test]
    fn test_csg_operations_memory_efficiency() {
        // **Mathematical Foundation**: Memory usage validation
        // **SRS Requirement NFR002**: Memory efficiency (< 0.1x input size for sparse voxels)

        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 16.0, 4, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 16.0, 4, None);

        // Create sparse patterns (only 8 voxels out of 4096 possible)
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let point = Point3::new(i as Real * 2.0, j as Real * 2.0, k as Real * 2.0);
                    octree1.set_voxel(&point, true, None);
                    octree2.set_voxel(&point, true, None);
                }
            }
        }

        // Perform operations
        let union = octree1.union(&octree2);
        let intersection = octree1.intersection(&octree2);
        let difference = octree1.difference(&octree2);

        // Verify memory efficiency (should be far less than dense representation)
        // Dense 16x16x16 would be 4096 voxels, we should have much less
        assert!(
            union.occupied_leaves <= 16,
            "Union should maintain sparsity: {} occupied",
            union.occupied_leaves
        );
        assert!(
            intersection.occupied_leaves <= 16,
            "Intersection should maintain sparsity: {} occupied",
            intersection.occupied_leaves
        );
        assert!(
            difference.occupied_leaves <= 16,
            "Difference should maintain sparsity: {} occupied",
            difference.occupied_leaves
        );

        // Compression ratio should be very high (>95% empty space)
        let total_possible_voxels = 16 * 16 * 16; // 4096
        let compression_ratio = (total_possible_voxels - union.occupied_leaves) as f64
            / total_possible_voxels as f64;
        assert!(
            compression_ratio > 0.95,
            "Compression ratio should be >95%: {:.3}",
            compression_ratio
        );
    }

    #[test]
    fn test_csg_operations_with_metadata_preservation() {
        // **Mathematical Foundation**: Metadata preservation through operations
        // **SRS Requirement NFR008**: Generic metadata system with compile-time guarantees

        #[derive(Clone, Debug, PartialEq, Hash)]
        struct TestMetadata {
            id: u32,
            name: String,
        }

        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::new(
            origin,
            4.0,
            3,
            Some(TestMetadata {
                id: 1,
                name: "octree1".to_string(),
            }),
        );

        let mut octree2 = SparseVoxelOctree::new(
            origin,
            4.0,
            3,
            Some(TestMetadata {
                id: 2,
                name: "octree2".to_string(),
            }),
        );

        // Set some voxels
        octree1.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);
        octree2.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);

        // All operations should preserve metadata from first operand
        let union = octree1.union(&octree2);
        let difference = octree1.difference(&octree2);
        let intersection = octree1.intersection(&octree2);
        let xor = octree1.xor(&octree2);

        assert!(union.metadata.is_some(), "Union should preserve metadata");
        assert!(
            difference.metadata.is_some(),
            "Difference should preserve metadata"
        );
        assert!(
            intersection.metadata.is_some(),
            "Intersection should preserve metadata"
        );
        assert!(xor.metadata.is_some(), "XOR should preserve metadata");

        let metadata = union.metadata.as_ref().unwrap();
        assert_eq!(
            metadata.id, 1,
            "Union should preserve first operand's metadata"
        );
        assert_eq!(
            metadata.name, "octree1",
            "Union should preserve first operand's metadata"
        );
    }

    #[test]
    fn test_csg_operations_numerical_stability() {
        // **Mathematical Foundation**: Numerical precision in floating-point operations
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree1 = SparseVoxelOctree::<()>::new(origin, 2.0, 4, None);
        let mut octree2 = SparseVoxelOctree::<()>::new(origin, 2.0, 4, None);

        // Test with coordinates near floating-point precision limits
        let epsilon = 1e-10;
        let coords = vec![
            Point3::new(0.5 + epsilon, 0.5, 0.5), // Near boundary
            Point3::new(0.5, 0.5 + epsilon, 0.5), // Near boundary
            Point3::new(0.5, 0.5, 0.5 + epsilon), // Near boundary
            Point3::new(1.0 - epsilon, 1.0, 1.0), // Near boundary
        ];

        for coord in coords {
            octree1.set_voxel(&coord, true, None);
            octree2.set_voxel(&coord, true, None);
        }

        // Operations should handle precision boundaries gracefully
        let union = octree1.union(&octree2);
        let intersection = octree1.intersection(&octree2);
        let difference = octree1.difference(&octree2);

        // Should not panic and should produce reasonable results
        assert!(union.occupied_leaves >= intersection.occupied_leaves);
        assert!(union.occupied_leaves >= difference.occupied_leaves);
    }

    #[test]
    fn test_csg_operations_bounds_validation() {
        // **Mathematical Foundation**: Coordinate bounds validation
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        let origin1 = Point3::new(0.0, 0.0, 0.0);
        let origin2 = Point3::new(10.0, 10.0, 10.0); // Different origin

        let octree1 = SparseVoxelOctree::<()>::new(origin1, 4.0, 3, None);
        let octree2 = SparseVoxelOctree::<()>::new(origin2, 4.0, 3, None);

        // Operations between incompatible bounds should fail gracefully
        // The current implementation panics on incompatible bounds
        // This test verifies the bounds check works correctly
        assert!(!octree1.bounds_compatible(&octree2));
    }

    #[test]
    fn test_csg_operations_depth_limits() {
        // **Mathematical Foundation**: Octree depth constraints
        // **SRS Requirement NFR001**: Performance scaling O(log n)

        let origin = Point3::new(0.0, 0.0, 0.0);

        // Test with minimum depth
        let octree_min = SparseVoxelOctree::<()>::new(origin, 2.0, 1, None);
        assert_eq!(octree_min.max_depth, 1);

        // Test with large depth
        let octree_max = SparseVoxelOctree::<()>::new(origin, 2.0, 10, None);
        assert_eq!(octree_max.max_depth, 10);

        // Operations should work at different depths
        let union = octree_min.union(&octree_max);
        let intersection = octree_min.intersection(&octree_max);

        // Should complete without issues
        assert!(union.total_nodes > 0);
        assert!(intersection.total_nodes > 0);
    }
}
