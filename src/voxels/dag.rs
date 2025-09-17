//! # Sparse Voxel DAG Compression System
//!
//! This module provides DAG (Directed Acyclic Graph) compression for sparse voxel octrees.
//! DAG compression eliminates redundant subtree storage through reference counting and deduplication.

use crate::voxels::octree::SparseVoxelNode;
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt::Debug;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

/// Internal registry state to avoid double borrowing
#[derive(Debug)]
struct VoxelDagRegistryState<
    S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
> {
    /// Map from node hash to canonical node instance
    registry: HashMap<u64, Rc<RefCell<SparseVoxelNode<S>>>>,
    /// Total number of canonical nodes stored
    canonical_count: usize,
}

/// DAG registry for sparse voxel node deduplication
#[derive(Debug)]
pub struct VoxelDagRegistry<
    S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq,
> {
    /// Internal state wrapped in single RefCell to avoid double borrowing
    state: RefCell<VoxelDagRegistryState<S>>,
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq> Default
    for VoxelDagRegistry<S>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq>
    VoxelDagRegistry<S>
{
    /// Create a new empty DAG registry
    pub fn new() -> Self {
        Self {
            state: RefCell::new(VoxelDagRegistryState {
                registry: HashMap::new(),
                canonical_count: 0,
            }),
        }
    }

    /// Get or create a canonical instance of a node
    pub fn get_or_insert(&self, node: SparseVoxelNode<S>) -> Rc<RefCell<SparseVoxelNode<S>>> {
        let hash = self.hash_node(&node);

        let mut state_ref = self.state.borrow_mut();

        if let Some(existing) = state_ref.registry.get(&hash) {
            // Clone the Rc first to avoid borrowing conflicts
            let existing_rc = Rc::clone(existing);

            // Check if nodes are truly identical (handle hash collisions)
            let existing_content = (*existing_rc.borrow()).clone();
            if self.nodes_equal(&existing_content, &node) {
                return existing_rc;
            }
        }

        // Create new canonical instance
        let canonical = Rc::new(RefCell::new(node));
        state_ref.registry.insert(hash, Rc::clone(&canonical));
        state_ref.canonical_count += 1;
        canonical
    }

    /// Compute hash of a sparse voxel node
    fn hash_node(&self, node: &SparseVoxelNode<S>) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        self.hash_node_structural(node, &mut hasher);
        hasher.finish()
    }

    /// Compute structural hash without borrowing RefCells
    #[allow(clippy::only_used_in_recursion)]
    fn hash_node_structural(&self, _node: &SparseVoxelNode<S>, hasher: &mut impl Hasher) {
        match _node {
            SparseVoxelNode::Leaf { occupied, metadata } => {
                0u8.hash(hasher); // Discriminant for Leaf
                occupied.hash(hasher);
                metadata.hash(hasher);
            },
            SparseVoxelNode::Internal { children, bsp_tree } => {
                1u8.hash(hasher); // Discriminant for Internal

                // Hash children (None vs Some with recursive hash)
                for child in children.iter() {
                    match child {
                        None => {
                            0u8.hash(hasher); // None discriminant
                        },
                        Some(child_rc) => {
                            1u8.hash(hasher); // Some discriminant
                            // Hash the child's content without borrowing
                            let child_ref = child_rc.borrow();
                            self.hash_node_structural(&*child_ref, hasher);
                        },
                    }
                }

                // Hash BSP tree presence (simplified)
                bsp_tree.is_some().hash(hasher);
            },
        }
    }

    /// Check if two nodes are structurally equal
    #[allow(clippy::only_used_in_recursion)]
    fn nodes_equal(&self, _a: &SparseVoxelNode<S>, b: &SparseVoxelNode<S>) -> bool {
        match (_a, b) {
            (
                SparseVoxelNode::Leaf {
                    occupied: occ_a,
                    metadata: meta_a,
                },
                SparseVoxelNode::Leaf {
                    occupied: occ_b,
                    metadata: meta_b,
                },
            ) => occ_a == occ_b && meta_a == meta_b,

            (
                SparseVoxelNode::Internal {
                    children: children_a,
                    bsp_tree: bsp_a,
                },
                SparseVoxelNode::Internal {
                    children: children_b,
                    bsp_tree: bsp_b,
                },
            ) => {
                // Compare children recursively
                for (child_a, child_b) in children_a.iter().zip(children_b.iter()) {
                    match (child_a, child_b) {
                        (None, None) => continue,
                        (Some(rc_a), Some(rc_b)) => {
                            let content_a = rc_a.borrow();
                            let content_b = rc_b.borrow();
                            if !self.nodes_equal(&*content_a, &*content_b) {
                                return false;
                            }
                        },
                        _ => return false,
                    }
                }

                // Compare BSP tree presence (simplified comparison)
                bsp_a.is_some() == bsp_b.is_some()
            },

            _ => false, // Different node types
        }
    }

    /// Get the number of canonical nodes stored
    pub fn stats(&self) -> (usize, usize) {
        let state_ref = self.state.borrow();
        (state_ref.registry.len(), state_ref.canonical_count)
    }

    /// Clear the registry
    pub fn clear(&mut self) {
        let mut state_ref = self.state.borrow_mut();
        state_ref.registry.clear();
        state_ref.canonical_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::float_types::Real;
    use crate::voxels::octree::SparseVoxelOctree;
    use nalgebra::Point3;

    #[test]
    fn test_dag_registry_creation() {
        let registry = VoxelDagRegistry::<()>::new();
        let (registry_size, canonical_count) = registry.stats();
        assert_eq!(registry_size, 0);
        assert_eq!(canonical_count, 0);
    }

    #[test]
    fn test_dag_registry_deduplication() {
        let registry = VoxelDagRegistry::<()>::new();

        // Create identical leaf nodes
        let leaf1 = SparseVoxelNode::Leaf {
            occupied: true,
            metadata: Some(()),
        };
        let leaf2 = SparseVoxelNode::Leaf {
            occupied: true,
            metadata: Some(()),
        };

        // Insert both - should get the same canonical instance
        let canonical1 = registry.get_or_insert(leaf1);
        let canonical2 = registry.get_or_insert(leaf2);

        // Should be the same Rc (deduplicated)
        assert!(Rc::ptr_eq(&canonical1, &canonical2));

        let (registry_size, canonical_count) = registry.stats();
        assert_eq!(registry_size, 1);
        assert_eq!(canonical_count, 1);
    }

    #[test]
    fn test_dag_compression_basic() {
        let registry = VoxelDagRegistry::<()>::new();

        // Create an internal node with some children
        let children: [Option<Rc<RefCell<SparseVoxelNode<()>>>>; 8] = Default::default();
        let internal = SparseVoxelNode::Internal {
            children,
            bsp_tree: None,
        };

        let canonical = registry.get_or_insert(internal);

        match &*canonical.borrow() {
            SparseVoxelNode::Internal { children, .. } => {
                assert!(children.iter().all(|c| c.is_none()));
            },
            _ => panic!("Expected internal node"),
        }

        let (registry_size, canonical_count) = registry.stats();
        assert_eq!(registry_size, 1);
        assert_eq!(canonical_count, 1);
    }

    #[test]
    fn test_dag_registry_clear() {
        let mut registry = VoxelDagRegistry::<()>::new();

        // Add some nodes
        let leaf = SparseVoxelNode::Leaf {
            occupied: false,
            metadata: None,
        };
        registry.get_or_insert(leaf);

        assert_eq!(registry.stats().0, 1);

        // Clear registry
        registry.clear();
        assert_eq!(registry.stats().0, 0);
        assert_eq!(registry.stats().1, 0);
    }

    #[test]
    fn test_dag_compression_with_metadata() {
        let registry = VoxelDagRegistry::<String>::new();

        // Create leaf nodes with different metadata
        let leaf1 = SparseVoxelNode::Leaf {
            occupied: true,
            metadata: Some("material1".to_string()),
        };
        let leaf2 = SparseVoxelNode::Leaf {
            occupied: true,
            metadata: Some("material1".to_string()),
        };
        let leaf3 = SparseVoxelNode::Leaf {
            occupied: true,
            metadata: Some("material2".to_string()),
        };

        // Insert nodes
        let canonical1 = registry.get_or_insert(leaf1);
        let canonical2 = registry.get_or_insert(leaf2);
        let canonical3 = registry.get_or_insert(leaf3);

        // First two should be deduplicated (same metadata)
        assert!(Rc::ptr_eq(&canonical1, &canonical2));

        // Third should be different (different metadata)
        assert!(!Rc::ptr_eq(&canonical1, &canonical3));

        let (registry_size, canonical_count) = registry.stats();
        assert_eq!(registry_size, 2); // Two unique nodes
        assert_eq!(canonical_count, 2);
    }

    #[test]
    fn test_enable_compression_on_existing_octree() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new(origin, 4.0, 2, None);

        // Add some voxels before enabling compression
        octree.set_voxel(&Point3::new(1.0, 1.0, 1.0), true, None);
        octree.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);

        // Enable compression
        octree.enable_compression();

        // Verify compression is enabled
        assert!(octree.dag_registry.is_some());

        // Compress existing structure
        octree.compress_existing();

        // Should still work correctly
        assert_eq!(octree.get_voxel(&Point3::new(1.0, 1.0, 1.0)), Some(true));
        assert_eq!(octree.get_voxel(&Point3::new(2.0, 2.0, 2.0)), Some(true));
    }

    #[test]
    fn test_compression_ratio_calculation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut octree = SparseVoxelOctree::<()>::new_compressed(origin, 8.0, 3, None);

        // Add voxels to create some structure
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    if (i + j + k) % 2 == 0 {
                        let point = Point3::new(i as Real, j as Real, k as Real);
                        octree.set_voxel(&point, true, None);
                    }
                }
            }
        }

        // Compress the octree to populate the DAG registry
        octree.compress_existing();

        let stats = octree.memory_stats();

        // Should have compression ratio calculated
        assert!(stats.compression_ratio.is_some());
        assert!(stats.compression_ratio.unwrap() >= 1.0);
    }

    #[test]
    fn test_default_registry() {
        let registry: VoxelDagRegistry<()> = Default::default();
        let (registry_size, canonical_count) = registry.stats();
        assert_eq!(registry_size, 0);
        assert_eq!(canonical_count, 0);
    }
}
