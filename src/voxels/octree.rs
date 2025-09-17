//! # Sparse Voxel Octree System
//!
//! This module provides sparse voxel octree representations for efficient volume-based processing.
//! Sparse octrees use hierarchical decomposition to achieve massive memory savings for sparse volumes.

use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use nalgebra::{Point3, Vector3};
use std::cell::RefCell;
use std::fmt::Debug;
use std::rc::Rc;

/// Sparse voxel node representing either a leaf or internal node
#[derive(Debug, Clone)]
pub enum SparseVoxelNode<S: Clone + Debug + Send + Sync> {
    /// Leaf node with occupancy and metadata
    Leaf { occupied: bool, metadata: Option<S> },
    /// Internal node with up to 8 children
    Internal {
        children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8],
        bsp_tree: Option<Rc<RefCell<crate::mesh::bsp::Node<S>>>>,
    },
}

/// Octant indices for octree children (0-7)
/// Convention: 0=---, 1=+--, 2=-+-, 3=++-, 4=--+-, 5=+-+-, 6=-++, 7=+++
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Octant {
    /// 0: (-x, -y, -z)
    MMM = 0,
    /// 1: (+x, -y, -z)
    PMM = 1,
    /// 2: (-x, +y, -z)
    MPM = 2,
    /// 3: (+x, +y, -z)
    PPM = 3,
    /// 4: (-x, -y, +z)
    MMP = 4,
    /// 5: (+x, -y, +z)
    PMP = 5,
    /// 6: (-x, +y, +z)
    MPP = 6,
    /// 7: (+x, +y, +z)
    PPP = 7,
}

impl Octant {
    /// Convert octant index to Octant enum
    pub const fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Octant::MMM),
            1 => Some(Octant::PMM),
            2 => Some(Octant::MPM),
            3 => Some(Octant::PPM),
            4 => Some(Octant::MMP),
            5 => Some(Octant::PMP),
            6 => Some(Octant::MPP),
            7 => Some(Octant::PPP),
            _ => None,
        }
    }

    /// Convert Octant enum to index
    pub const fn to_index(self) -> usize {
        self as usize
    }

    /// Get the offset vector for this octant
    pub const fn offset(&self) -> Vector3<Real> {
        match self {
            Octant::MMM => Vector3::new(0.0, 0.0, 0.0),
            Octant::PMM => Vector3::new(0.5, 0.0, 0.0),
            Octant::MPM => Vector3::new(0.0, 0.5, 0.0),
            Octant::PPM => Vector3::new(0.5, 0.5, 0.0),
            Octant::MMP => Vector3::new(0.0, 0.0, 0.5),
            Octant::PMP => Vector3::new(0.5, 0.0, 0.5),
            Octant::MPP => Vector3::new(0.0, 0.5, 0.5),
            Octant::PPP => Vector3::new(0.5, 0.5, 0.5),
        }
    }
}

/// Sparse voxel octree for efficient volume representation
#[derive(Debug, Clone)]
pub struct SparseVoxelOctree<
    S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
> {
    /// Root node of the octree
    pub root: Rc<RefCell<SparseVoxelNode<S>>>,
    /// Origin point (minimum corner of root volume)
    pub origin: Point3<Real>,
    /// Size of the root volume (all dimensions equal for octree)
    pub size: Real,
    /// Maximum depth of the octree (determines minimum voxel size)
    pub max_depth: usize,
    /// Current number of occupied leaf nodes
    pub occupied_leaves: usize,
    /// Total number of nodes in the octree (for statistics)
    pub total_nodes: usize,
    /// Optional metadata for the entire octree
    pub metadata: Option<S>,
    /// DAG registry for node deduplication (optional, enables compression)
    pub dag_registry: Option<Rc<RefCell<super::dag::VoxelDagRegistry<S>>>>,
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq>
    SparseVoxelOctree<S>
{
    /// Create a new sparse voxel octree
    pub fn new(
        origin: Point3<Real>,
        size: Real,
        max_depth: usize,
        metadata: Option<S>,
    ) -> Self {
        let root = Rc::new(RefCell::new(SparseVoxelNode::Leaf {
            occupied: false,
            metadata: None,
        }));

        Self {
            root,
            origin,
            size,
            max_depth,
            occupied_leaves: 0,
            total_nodes: 1,
            metadata,
            dag_registry: None,
        }
    }

    /// Create a new sparse voxel octree with compression enabled
    pub fn new_compressed(
        origin: Point3<Real>,
        size: Real,
        max_depth: usize,
        metadata: Option<S>,
    ) -> Self {
        let root = Rc::new(RefCell::new(SparseVoxelNode::Leaf {
            occupied: false,
            metadata: None,
        }));

        let dag_registry = Some(Rc::new(RefCell::new(super::dag::VoxelDagRegistry::new())));

        Self {
            root,
            origin,
            size,
            max_depth,
            occupied_leaves: 0,
            total_nodes: 1,
            metadata,
            dag_registry,
        }
    }

    /// Enable compression on existing octree
    pub fn enable_compression(&mut self) {
        if self.dag_registry.is_none() {
            self.dag_registry =
                Some(Rc::new(RefCell::new(super::dag::VoxelDagRegistry::new())));
        }
    }

    /// Compress existing octree structure using DAG deduplication
    pub fn compress_existing(&mut self) {
        if let Some(registry_rc) = &self.dag_registry {
            // Compress the tree structure recursively
            Self::compress_node_recursive(Rc::clone(&self.root), Rc::clone(registry_rc));

            // Update the octree root to point to the canonical version
            let root_content = (*self.root.borrow()).clone();
            let canonical_root = registry_rc.borrow_mut().get_or_insert(root_content);
            self.root = canonical_root;
        }
    }

    /// Recursively compress nodes using the DAG registry
    fn compress_node_recursive(
        node: Rc<RefCell<SparseVoxelNode<S>>>,
        registry: Rc<RefCell<super::dag::VoxelDagRegistry<S>>>,
    ) {
        let node_content = (*node.borrow()).clone(); // Clone the content

        match node_content {
            SparseVoxelNode::Leaf { .. } => {
                // Leaf nodes are already as compressed as possible
            },
            SparseVoxelNode::Internal { mut children, .. } => {
                // Compress all children first and update children array with canonical versions
                for child in children.iter_mut().flatten() {
                    Self::compress_node_recursive(Rc::clone(child), Rc::clone(&registry));

                    // Get the canonical version of this child and update the reference
                    let child_content = (*child.borrow()).clone();
                    let canonical_child = registry.borrow_mut().get_or_insert(child_content);
                    *child = canonical_child;
                }

                // Now create canonical version of the parent with canonical children
                let canonical =
                    registry
                        .borrow_mut()
                        .get_or_insert(SparseVoxelNode::Internal {
                            children,
                            bsp_tree: None, // CSG results don't include BSP trees
                        });

                // Update the node to point to the canonical version
                let mut node_ref = node.borrow_mut();
                *node_ref = canonical.borrow().clone();
            },
        }
    }

    /// Get the depth of the octree (number of levels)
    pub fn depth(&self) -> usize {
        Self::calculate_depth_recursive(self, &self.root, 0)
    }

    /// Recursively calculate the depth of a subtree
    fn calculate_depth_recursive(
        _self: &Self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        current_depth: usize,
    ) -> usize {
        let node_ref = node.borrow();

        match &*node_ref {
            SparseVoxelNode::Leaf { .. } => current_depth,
            SparseVoxelNode::Internal { children, .. } => {
                let mut max_child_depth = current_depth;
                for child in children.iter().flatten() {
                    let child_depth =
                        Self::calculate_depth_recursive(_self, child, current_depth + 1);
                    max_child_depth = max_child_depth.max(child_depth);
                }
                max_child_depth
            },
        }
    }

    /// Get voxel size at a specific depth
    pub fn voxel_size_at_depth(&self, depth: usize) -> Real {
        self.size / (1 << depth) as Real
    }

    /// Convert world coordinates to octree coordinates at a specific depth
    pub fn world_to_octree_coords(
        &self,
        point: &Point3<Real>,
        depth: usize,
    ) -> (usize, usize, usize) {
        let scale = (1 << depth) as Real / self.size; // Use 2^depth for number of intervals
        let local_point = point - self.origin;

        let x = (local_point.x * scale).floor() as usize;
        let y = (local_point.y * scale).floor() as usize;
        let z = (local_point.z * scale).floor() as usize;

        // Clamp to valid range (0 to 2^depth - 1)
        let max_coord = (1 << depth) - 1;
        let x = x.min(max_coord);
        let y = y.min(max_coord);
        let z = z.min(max_coord);

        (x, y, z)
    }

    /// Convert octree coordinates to world coordinates at a specific depth
    pub fn octree_to_world_coords(
        &self,
        x: usize,
        y: usize,
        z: usize,
        depth: usize,
    ) -> Point3<Real> {
        let voxel_size = self.voxel_size_at_depth(depth);

        self.origin
            + Vector3::new(
                (x as Real + 0.5) * voxel_size,
                (y as Real + 0.5) * voxel_size,
                (z as Real + 0.5) * voxel_size,
            )
    }

    /// Get the octant index for a point within a node
    pub fn get_octant_index(&self, local_point: &Point3<Real>, half_size: Real) -> Octant {
        let center = Vector3::new(half_size, half_size, half_size);

        let x_octant = if local_point.x >= center.x { 1 } else { 0 };
        let y_octant = if local_point.y >= center.y { 2 } else { 0 };
        let z_octant = if local_point.z >= center.z { 4 } else { 0 };

        match x_octant + y_octant + z_octant {
            0 => Octant::MMM, // ---
            1 => Octant::PMM, // +--
            2 => Octant::MPM, // -+-
            3 => Octant::PPM, // ++-
            4 => Octant::MMP, // --+
            5 => Octant::PMP, // +-+
            6 => Octant::MPP, // -++
            7 => Octant::PPP, // +++
            _ => unreachable!(),
        }
    }

    /// Set a voxel at the given world coordinates
    pub fn set_voxel(&mut self, point: &Point3<Real>, occupied: bool, metadata: Option<S>) {
        let origin = self.origin;
        let size = self.size;
        let root_clone = Rc::clone(&self.root);
        self.set_voxel_recursive(&root_clone, point, occupied, metadata, origin, size, 0);
    }

    /// Recursively set a voxel in the octree
    #[allow(clippy::too_many_arguments)]
    fn set_voxel_recursive(
        &mut self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        point: &Point3<Real>,
        occupied: bool,
        metadata: Option<S>,
        node_origin: Point3<Real>,
        node_size: Real,
        depth: usize,
    ) {
        // Check if point is within this node's bounds
        if !self.point_in_node(point, node_origin, node_size) {
            return;
        }

        let mut node_ref = node.borrow_mut();

        match &mut *node_ref {
            SparseVoxelNode::Leaf {
                occupied: current_occupied,
                metadata: current_metadata,
            } => {
                if depth >= self.max_depth {
                    // At maximum depth, update this leaf
                    let was_occupied = *current_occupied;
                    *current_occupied = occupied;
                    *current_metadata = metadata;
                    if occupied && !was_occupied {
                        self.occupied_leaves += 1;
                    } else if !occupied && was_occupied {
                        self.occupied_leaves = self.occupied_leaves.saturating_sub(1);
                    }
                } else {
                    // Need to subdivide this leaf into an internal node
                    let children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8] =
                        Default::default();
                    *node_ref = SparseVoxelNode::Internal {
                        children,
                        bsp_tree: None,
                    };
                    drop(node_ref); // Release borrow before recursive call

                    // Now recurse to set the voxel in the appropriate child
                    // Since we converted to Internal, we need to continue with the same node but increment depth
                    self.set_voxel_recursive(
                        node,
                        point,
                        occupied,
                        metadata,
                        node_origin,
                        node_size,
                        depth + 1,
                    );
                }
            },
            SparseVoxelNode::Internal { .. } => {
                // Need to release the borrow before modifying children
                drop(node_ref);

                let half_size = node_size * 0.5;
                let local_point = point - node_origin;
                let octant = self.get_octant_index(&Point3::from(local_point), half_size);
                let octant_idx = octant.to_index();

                let child_origin = self.get_child_origin(node_origin, half_size, octant);

                // Get or create the child node (need to borrow again)
                let needs_child = {
                    let node_ref = node.borrow();
                    if let SparseVoxelNode::Internal { children, .. } = &*node_ref {
                        children[octant_idx].is_none()
                    } else {
                        false
                    }
                };

                if needs_child {
                    let mut node_ref = node.borrow_mut();
                    if let SparseVoxelNode::Internal { children, .. } = &mut *node_ref {
                        // Create appropriate node type based on whether we'll reach max_depth
                        let child_node = if depth + 1 >= self.max_depth {
                            SparseVoxelNode::Leaf {
                                occupied: false,
                                metadata: None,
                            }
                        } else {
                            SparseVoxelNode::Internal {
                                children: Default::default(),
                                bsp_tree: None,
                            }
                        };

                        children[octant_idx] = Some(Rc::new(RefCell::new(child_node)));
                        self.total_nodes += 1;
                    }
                }

                // Get the child and recurse
                let child = {
                    let node_ref = node.borrow();
                    if let SparseVoxelNode::Internal { children, .. } = &*node_ref {
                        let child_ref = children[octant_idx].as_ref();
                        child_ref.map(Rc::clone)
                    } else {
                        None
                    }
                };

                if let Some(child) = child {
                    self.set_voxel_recursive(
                        &child,
                        point,
                        occupied,
                        metadata,
                        child_origin,
                        half_size,
                        depth + 1,
                    );
                }
            },
        }
    }

    /// Set a voxel using compressed operations (if registry is available)
    #[allow(dead_code, clippy::too_many_arguments)]
    fn set_voxel_compressed_recursive(
        &mut self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        point: &Point3<Real>,
        occupied: bool,
        metadata: Option<S>,
        node_origin: Point3<Real>,
        node_size: Real,
        depth: usize,
    ) {
        // Check if point is within this node's bounds
        if !self.point_in_node(point, node_origin, node_size) {
            return;
        }

        let mut node_ref = node.borrow_mut();

        match &mut *node_ref {
            SparseVoxelNode::Leaf {
                occupied: current_occupied,
                metadata: current_metadata,
            } => {
                if depth >= self.max_depth {
                    // At maximum depth, update this leaf
                    let was_occupied = *current_occupied;
                    *current_occupied = occupied;
                    *current_metadata = metadata;
                    if occupied && !was_occupied {
                        self.occupied_leaves += 1;
                    } else if !occupied && was_occupied {
                        self.occupied_leaves = self.occupied_leaves.saturating_sub(1);
                    }
                } else {
                    // Need to subdivide this leaf into an internal node
                    let children: [Option<Rc<RefCell<SparseVoxelNode<S>>>>; 8] =
                        Default::default();
                    *node_ref = SparseVoxelNode::Internal {
                        children,
                        bsp_tree: None,
                    };
                    drop(node_ref); // Release borrow before recursive call

                    // Now recurse to set the voxel in the appropriate child
                    self.set_voxel_compressed_recursive(
                        node,
                        point,
                        occupied,
                        metadata,
                        node_origin,
                        node_size,
                        depth,
                    );
                }
            },
            SparseVoxelNode::Internal { children, .. } => {
                // Borrow will be released when node_ref goes out of scope

                let half_size = node_size * 0.5;
                let local_point = point - node_origin;
                let octant = self.get_octant_index(&Point3::from(local_point), half_size);
                let octant_idx = octant.to_index();

                let child_origin = self.get_child_origin(node_origin, half_size, octant);

                // Get or create the child node (with registry if available)
                if children[octant_idx].is_none() {
                    let child_node = SparseVoxelNode::Leaf {
                        occupied: false,
                        metadata: None,
                    };

                    let canonical_child = if let Some(ref reg) = self.dag_registry {
                        reg.borrow_mut().get_or_insert(child_node)
                    } else {
                        Rc::new(RefCell::new(child_node))
                    };

                    children[octant_idx] = Some(canonical_child);
                    self.total_nodes += 1;
                }

                if let Some(ref child) = children[octant_idx] {
                    self.set_voxel_compressed_recursive(
                        child,
                        point,
                        occupied,
                        metadata,
                        child_origin,
                        half_size,
                        depth + 1,
                    );
                }
            },
        }
    }

    /// Check if a point is within a node's bounds
    fn point_in_node(
        &self,
        point: &Point3<Real>,
        node_origin: Point3<Real>,
        node_size: Real,
    ) -> bool {
        let max_point = node_origin + Vector3::new(node_size, node_size, node_size);
        point.x >= node_origin.x
            && point.x <= max_point.x
            && point.y >= node_origin.y
            && point.y <= max_point.y
            && point.z >= node_origin.z
            && point.z <= max_point.z
    }

    /// Get the voxel value at the given world coordinates
    pub fn get_voxel(&self, point: &Point3<Real>) -> Option<bool> {
        self.get_voxel_recursive(&self.root, point, self.origin, self.size, 0)
    }

    /// Recursively get a voxel value from the octree
    fn get_voxel_recursive(
        &self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        point: &Point3<Real>,
        node_origin: Point3<Real>,
        node_size: Real,
        _depth: usize,
    ) -> Option<bool> {
        // Check if point is within this node's bounds
        let in_bounds = self.point_in_node(point, node_origin, node_size);
        if !in_bounds {
            return None;
        }

        let node_ref = node.borrow();

        match &*node_ref {
            SparseVoxelNode::Leaf { occupied, .. } => Some(*occupied),
            SparseVoxelNode::Internal { children, .. } => {
                let half_size = node_size * 0.5;
                let local_point = point - node_origin;
                let octant = self.get_octant_index(&Point3::from(local_point), half_size);
                let octant_idx = octant.to_index();

                let child_origin = self.get_child_origin(node_origin, half_size, octant);

                if let Some(ref child) = children[octant_idx] {
                    self.get_voxel_recursive(child, point, child_origin, half_size, _depth + 1)
                } else {
                    Some(false) // Empty child means unoccupied
                }
            },
        }
    }

    /// Get the origin of a child node
    pub fn get_child_origin(
        &self,
        parent_origin: Point3<Real>,
        half_size: Real,
        octant: Octant,
    ) -> Point3<Real> {
        parent_origin + octant.offset() * half_size * 2.0
    }

    /// Get memory statistics for the octree
    pub fn memory_stats(&self) -> super::utils::VoxelMemoryStats {
        let (node_count, occupied_leaves_count) = self.count_nodes_and_occupied();
        let total_nodes = self.total_nodes;

        // For compressed octrees, calculate actual memory usage based on canonical nodes
        let actual_memory_usage = if let Some(ref registry) = self.dag_registry {
            let (_registry_size, canonical_count) = registry.borrow().stats();
            canonical_count * std::mem::size_of::<SparseVoxelNode<S>>()
        } else {
            total_nodes * std::mem::size_of::<SparseVoxelNode<S>>()
        };

        super::utils::VoxelMemoryStats {
            node_count,
            occupied_leaves: occupied_leaves_count,
            total_nodes,
            memory_usage_bytes: actual_memory_usage,
            compression_ratio: if let Some(ref registry) = self.dag_registry {
                let (_registry_size, canonical_count) = registry.borrow().stats();
                if canonical_count > 0 {
                    Some(total_nodes as Real / canonical_count as Real)
                } else {
                    None
                }
            } else {
                None
            },
        }
    }

    /// Count nodes and occupied leaves recursively
    fn count_nodes_and_occupied(&self) -> (usize, usize) {
        Self::count_nodes_recursive(self, &self.root, &mut 0, &mut 0)
    }

    /// Recursively count nodes and occupied leaves
    fn count_nodes_recursive(
        _self: &Self,
        node: &Rc<RefCell<SparseVoxelNode<S>>>,
        node_count: &mut usize,
        occupied_count: &mut usize,
    ) -> (usize, usize) {
        *node_count += 1;

        let node_ref = node.borrow();

        match &*node_ref {
            SparseVoxelNode::Leaf { occupied, .. } => {
                if *occupied {
                    *occupied_count += 1;
                }
            },
            SparseVoxelNode::Internal { children, .. } => {
                for child in children.iter().flatten() {
                    Self::count_nodes_recursive(_self, child, node_count, occupied_count);
                }
            },
        }

        (*node_count, *occupied_count)
    }

    /// Get bounding box of the octree
    pub fn bounding_box(&self) -> Aabb {
        let max_point = self.origin + Vector3::new(self.size, self.size, self.size);
        Aabb::new(self.origin, max_point)
    }
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq>
    SparseVoxelNode<S>
{
    /// Create a new leaf node
    pub const fn new_leaf(occupied: bool, metadata: Option<S>) -> Self {
        SparseVoxelNode::Leaf { occupied, metadata }
    }

    /// Create a new internal node with empty children
    pub const fn new_internal() -> Self {
        SparseVoxelNode::Internal {
            children: [None, None, None, None, None, None, None, None],
            bsp_tree: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_sparse_octree_creation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let octree = SparseVoxelOctree::<()>::new(origin, 10.0, 5, None);

        assert_eq!(octree.origin, origin);
        assert_eq!(octree.size, 10.0);
        assert_eq!(octree.max_depth, 5);
        assert_eq!(octree.occupied_leaves, 0);
        assert_eq!(octree.total_nodes, 1);
    }

    #[test]
    fn test_sparse_node_creation() {
        test_sparse_node_creation_impl::<()>();
    }

    fn test_sparse_node_creation_impl<
        S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
    >() {
        let leaf = SparseVoxelNode::new_leaf(true, Some(()));
        match leaf {
            SparseVoxelNode::Leaf { occupied, metadata } => {
                assert!(occupied);
                assert!(metadata.is_some());
            },
            _ => panic!("Expected leaf node"),
        }

        let internal: SparseVoxelNode<S> = SparseVoxelNode::new_internal();
        match internal {
            SparseVoxelNode::Internal { children, bsp_tree } => {
                assert!(children.iter().all(|c| c.is_none()));
                assert!(bsp_tree.is_none());
            },
            _ => panic!("Expected internal node"),
        }
    }

    #[test]
    fn test_octree_coordinate_conversion() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let octree = SparseVoxelOctree::<()>::new(origin, 8.0, 3, None);

        // At depth 0, the whole octree is one voxel
        let (x, y, z) = octree.world_to_octree_coords(&Point3::new(4.0, 4.0, 4.0), 0);
        assert_eq!((x, y, z), (0, 0, 0));

        // At depth 1, should be divided into 2x2x2 = 8 voxels
        let (x, y, z) = octree.world_to_octree_coords(&Point3::new(3.0, 3.0, 3.0), 1);
        assert_eq!((x, y, z), (0, 0, 0)); // First octant

        let (x, y, z) = octree.world_to_octree_coords(&Point3::new(5.0, 5.0, 5.0), 1);
        assert_eq!((x, y, z), (1, 1, 1)); // Last octant
    }

    #[test]
    fn test_octant_calculation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let octree = SparseVoxelOctree::<()>::new(origin, 8.0, 3, None);

        // Test octant calculation for different points
        let local_point = Point3::new(2.0, 2.0, 2.0); // First octant (MMM)
        let octant = octree.get_octant_index(&local_point, 4.0);
        assert_eq!(octant, Octant::MMM);

        let local_point = Point3::new(6.0, 6.0, 6.0); // Last octant (PPP)
        let octant = octree.get_octant_index(&local_point, 4.0);
        assert_eq!(octant, Octant::PPP);
    }

    #[test]
    fn test_octant_conversion() {
        // Test round-trip conversion
        for i in 0..8 {
            let octant =
                Octant::from_index(i).expect("Invalid octant index in bounds calculation");
            assert_eq!(octant.to_index(), i);
        }

        // Test that all octants are covered
        assert_eq!(Octant::MMM.to_index(), 0);
        assert_eq!(Octant::PPP.to_index(), 7);
    }

    #[test]
    fn test_octree_bounding_box() {
        let origin = Point3::new(-5.0, -5.0, -5.0);
        let octree = SparseVoxelOctree::<()>::new(origin, 10.0, 3, None);

        let bbox = octree.bounding_box();

        assert!((bbox.mins.x - (-5.0)).abs() < 1e-10);
        assert!((bbox.mins.y - (-5.0)).abs() < 1e-10);
        assert!((bbox.mins.z - (-5.0)).abs() < 1e-10);
        assert!((bbox.maxs.x - 5.0).abs() < 1e-10);
        assert!((bbox.maxs.y - 5.0).abs() < 1e-10);
        assert!((bbox.maxs.z - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_memory_stats() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);

        // Set some voxels
        octree.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);
        octree.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);

        let stats = octree.memory_stats();

        assert!(stats.node_count > 0);
        assert_eq!(stats.occupied_leaves, 2);
        assert!(stats.memory_usage_bytes > 0);
        assert!(stats.compression_ratio.is_none()); // No compression enabled
    }

    #[test]
    fn test_point_in_node() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let octree = SparseVoxelOctree::<()>::new(origin, 10.0, 3, None);

        // Point inside node
        assert!(octree.point_in_node(&Point3::new(5.0, 5.0, 5.0), origin, 10.0));

        // Point outside node
        assert!(!octree.point_in_node(&Point3::new(15.0, 5.0, 5.0), origin, 10.0));
        assert!(!octree.point_in_node(&Point3::new(-5.0, 5.0, 5.0), origin, 10.0));
    }

    #[test]
    fn test_sparse_voxel_set_get_basic() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 10.0, 3, None);

        let point = Point3::new(2.0, 2.0, 2.0);

        // Initially unoccupied
        let initial_value = octree.get_voxel(&point);

        assert_eq!(initial_value, Some(false));

        // Set as occupied

        octree.set_voxel(&point, true, None);

        // Debug: Check octree state

        let value_after_set = octree.get_voxel(&point);

        assert_eq!(value_after_set, Some(true));

        // Set as unoccupied
        octree.set_voxel(&point, false, None);
        assert_eq!(octree.get_voxel(&point), Some(false));
    }

    #[test]
    fn test_sparse_voxel_multiple_points() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 10.0, 3, None);

        let points = [
            Point3::new(1.0, 1.0, 1.0), // MMM octant
            Point3::new(6.0, 1.0, 1.0), // PMM octant
            Point3::new(1.0, 6.0, 1.0), // MPM octant
        ];

        // Set multiple points
        for point in &points {
            octree.set_voxel(point, true, None);
        }

        // Verify all points are set
        for point in &points {
            assert_eq!(octree.get_voxel(point), Some(true));
        }

        // Check that a point outside the set is still unoccupied
        assert_eq!(octree.get_voxel(&Point3::new(5.0, 5.0, 5.0)), Some(false));
    }

    #[test]
    fn test_sparse_voxel_subdivision() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 8.0, 2, None);

        // Initially just one node
        assert_eq!(octree.total_nodes, 1);

        // Setting a voxel should cause subdivision
        println!(
            "Before set_voxel: occupied_leaves = {}, total_nodes = {}",
            octree.occupied_leaves, octree.total_nodes
        );
        octree.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);
        println!(
            "After set_voxel: occupied_leaves = {}, total_nodes = {}",
            octree.occupied_leaves, octree.total_nodes
        );

        // Should now have more nodes (root + at least one child)
        assert!(octree.total_nodes > 1);
        assert_eq!(octree.occupied_leaves, 1);
    }

    #[test]
    fn test_sparse_voxel_metadata() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 10.0, 3, None);

        let point = Point3::new(2.0, 2.0, 2.0);

        // Set voxel with metadata
        octree.set_voxel(&point, true, Some(()));

        // Verify voxel is occupied
        assert_eq!(octree.get_voxel(&point), Some(true));
    }

    #[test]
    fn test_sparse_voxel_depth_limits() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 8.0, 1, None); // Max depth 1

        // At depth 1, each dimension is divided into 2, so voxel size is 4.0
        assert!((octree.voxel_size_at_depth(1) - 4.0).abs() < 1e-10);

        // Setting a voxel should respect the depth limit
        octree.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);

        // Should not subdivide beyond max depth
        assert!(octree.depth() <= 1);
    }

    #[test]
    fn test_sparse_voxel_boundary_conditions() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 10.0, 3, None);

        // Test boundary points
        let boundary_points = [
            Point3::new(0.0, 0.0, 0.0),    // Minimum corner
            Point3::new(10.0, 10.0, 10.0), // Maximum corner
            Point3::new(5.0, 5.0, 5.0),    // Center
        ];

        for point in &boundary_points {
            octree.set_voxel(point, true, None);
            assert_eq!(octree.get_voxel(point), Some(true));
        }

        // Points outside bounds should return None
        assert_eq!(octree.get_voxel(&Point3::new(-1.0, 5.0, 5.0)), None);
        assert_eq!(octree.get_voxel(&Point3::new(11.0, 5.0, 5.0)), None);
    }
}
