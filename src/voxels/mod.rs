//! Sparse Voxel Octree (SVO) with optional embedded BSP payloads
//!
//! This module provides a minimal, allocation-conscious Sparse Voxel Octree (SVO)
//! data structure intended to spatially index surface/solid geometry with the option
//! to attach a small BSP (Binary Space Partitioning) representation at mixed cells.
//!
//! Design goals:
//! - Sparse: children exist only when needed (bitmask + compact Vec storage)
//! - Minimal allocations: no per-node AABBs; compute child bounds on the fly
//! - Embedded payload: store a tiny BSP (mesh::bsp::Node) at Mixed nodes when desired
//! - Separation of concerns: SVO is generic over metadata S
//!
//! Notes:
//! - The actual “voxelization”/rasterization from polygonal meshes is intentionally
//!   left to higher-level builders to keep this module focused on the SVO container.
//! - You can attach a BSP payload after detecting that a node is Mixed.
pub mod plane;
pub mod vertex;
pub mod polygon;
pub mod bsp;
pub mod bsp_parallel;
pub mod connectivity;
pub mod sdf;
pub mod tpms;
pub mod shapes;
pub mod smoothing;
pub mod metaballs;
pub mod quality;
pub mod convex_hull;
pub mod flatten_slice;
pub mod manifold;
pub mod surface_extraction;
pub mod marching_cubes;
pub mod bsp_integration;
pub mod svo_csg;
pub mod csg;

use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use crate::voxels::bsp::Node as BspNode;
use nalgebra::Point3;
use std::fmt::Debug;

/// Occupancy state of an octree cell
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Occupancy {
    /// No geometry inside this cell
    Empty,
    /// Fully occupied (solid) cell
    Full,
    /// Partially occupied: surface crosses this cell or content is unknown
    Mixed,
}

/// Sparse Voxel Octree node with an optional embedded BSP payload.
#[derive(Debug, Clone)]
pub struct SvoNode<S: Clone> {
    /// Occupancy classification for this cell
    pub occupancy: Occupancy,
    /// Bitmask of present children. Bit i corresponds to child index i in [0,7].
    pub children_mask: u8,
    /// Compact child storage. The i-th set bit in children_mask maps to children[rank(i)].
    pub children: Vec<Box<SvoNode<S>>>,
    /// Optional embedded BSP payload for Mixed cells
    pub local_bsp: Option<BspNode<S>>,
    /// Optional metadata propagated from the source mesh or application layer
    pub metadata: Option<S>,
}

impl<S: Clone> Default for SvoNode<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone> SvoNode<S> {
    /// Create an empty node (Empty occupancy, no children)
    pub const fn new() -> Self {
        Self {
            occupancy: Occupancy::Empty,
            children_mask: 0,
            children: Vec::new(),
            local_bsp: None,
            metadata: None,
        }
    }

    /// Returns true if a child at `child_idx` exists
    #[inline]
    pub const fn has_child(&self, child_idx: u8) -> bool {
        (self.children_mask & (1 << child_idx)) != 0
    }

    /// Compute the position within `children` array for logical child index `child_idx`.
    /// Precondition: has_child(child_idx) == true
    #[inline]
    fn child_rank(&self, child_idx: u8) -> usize {
        let mask = self.children_mask & ((1u8 << child_idx) - 1);
        mask.count_ones() as usize
    }

    /// Get immutable reference to a child if present
    /// Get immutable reference to child at index
    pub fn get_child(&self, child_idx: u8) -> Option<&SvoNode<S>> {
        if !self.has_child(child_idx) {
            return None;
        }
        let rank = self.child_rank(child_idx);
        Some(&self.children[rank])
    }

    /// Get mutable reference to a child if present
    pub fn get_child_mut(&mut self, child_idx: u8) -> Option<&mut SvoNode<S>> {
        if !self.has_child(child_idx) {
            return None;
        }
        let rank = self.child_rank(child_idx);
        Some(&mut self.children[rank])
    }

    /// Ensure a child exists at `child_idx` and return a mutable reference to it.
    pub fn ensure_child(&mut self, child_idx: u8) -> &mut SvoNode<S> {
        if self.has_child(child_idx) {
            let rank = self.child_rank(child_idx);
            return &mut self.children[rank];
        }
        // Insert a new child in rank order to keep children Vec consistent with mask ordering
        let insert_mask = 1u8 << child_idx;
        let rank = (self.children_mask & (insert_mask - 1)).count_ones() as usize;
        self.children_mask |= insert_mask;
        self.children.insert(rank, Box::new(SvoNode::new()));
        &mut self.children[rank]
    }

    /// Remove and return a child node at logical index `child_idx` if present
    pub fn take_child(&mut self, child_idx: u8) -> Option<Box<SvoNode<S>>> {
        if !self.has_child(child_idx) {
            return None;
        }
        let rank = self.child_rank(child_idx);
        self.children_mask &= !(1u8 << child_idx);
        Some(self.children.remove(rank))
    }

    /// Returns true if this node is a leaf (no children)
    #[inline]
    pub const fn is_leaf(&self) -> bool {
        self.children_mask == 0
    }

    /// Optimize memory usage by compacting the node structure
    pub fn optimize_memory(&mut self)
    where
        S: Debug + Send + Sync,
    {
        // Shrink children vector to fit
        self.children.shrink_to_fit();

        // Recursively optimize children
        for child in &mut self.children {
            child.optimize_memory();
        }

        // Optimize BSP tree if present
        if let Some(ref mut bsp) = self.local_bsp {
            bsp.optimize_memory();
        }
    }

    /// Get memory usage statistics for this node
    pub fn memory_usage(&self) -> usize
    where
        S: Debug + Send + Sync,
    {
        let mut size = std::mem::size_of::<Self>();

        // Add children memory
        size += self.children.capacity() * std::mem::size_of::<Box<SvoNode<S>>>();
        for child in &self.children {
            size += child.memory_usage();
        }

        // Add BSP memory if present
        if let Some(ref bsp) = self.local_bsp {
            size += bsp.memory_usage();
        }

        size
    }

    /// Check if this node can be simplified (all children have same occupancy)
    pub fn can_simplify(&self) -> bool {
        if self.children.is_empty() {
            return false;
        }

        let first_occupancy = self.children[0].occupancy;
        self.children.iter().all(|child| {
            child.occupancy == first_occupancy && child.is_leaf()
        })
    }

    /// Simplify node by removing redundant children
    pub fn simplify(&mut self) {
        if self.can_simplify() && !self.children.is_empty() {
            let unified_occupancy = self.children[0].occupancy;
            self.occupancy = unified_occupancy;
            self.children.clear();
            self.children_mask = 0;
            // Keep BSP if it exists and we're Mixed
            if unified_occupancy != Occupancy::Mixed {
                self.local_bsp = None;
            }
        }

        // Recursively simplify children
        for child in &mut self.children {
            child.simplify();
        }
    }
}

/// Sparse Voxel Octree container
#[derive(Debug, Clone)]
pub struct Svo<S: Clone> {
    /// Root node
    pub root: Box<SvoNode<S>>,
    /// Center of the root cube
    pub center: Point3<Real>,
    /// Half-size (extent) of the root cube
    pub half: Real,
    /// Maximum tree depth (root at depth 0)
    pub max_depth: u8,
}

impl<S: Clone> Svo<S> {
    /// Create an empty SVO that spans a cube centered at `center` with half size `half`.
    pub fn new(center: Point3<Real>, half: Real, max_depth: u8) -> Self {
        Self { root: Box::new(SvoNode::new()), center, half, max_depth }
    }

    /// AABB of the root cube
    pub fn aabb(&self) -> Aabb {
        let mins = Point3::new(self.center.x - self.half, self.center.y - self.half, self.center.z - self.half);
        let maxs = Point3::new(self.center.x + self.half, self.center.y + self.half, self.center.z + self.half);
        Aabb::new(mins, maxs)
    }

    /// Compute child cube center for a given child index in [0,7]
    #[inline]
    pub fn child_center(center: &Point3<Real>, half: Real, child_idx: u8) -> Point3<Real> {
        let q = half * 0.5;
        let dx = if (child_idx & 1) != 0 { q } else { -q };
        let dy = if (child_idx & 2) != 0 { q } else { -q };
        let dz = if (child_idx & 4) != 0 { q } else { -q };
        Point3::new(center.x + dx, center.y + dy, center.z + dz)
    }

    /// Compute which child octant a point belongs to
    #[inline]
    pub fn octant_for_point(center: &Point3<Real>, p: &Point3<Real>) -> u8 {
        let mut idx = 0u8;
        if p.x >= center.x { idx |= 1; }
        if p.y >= center.y { idx |= 2; }
        if p.z >= center.z { idx |= 4; }
        idx
    }

    /// Insert a point as occupied. This is a basic example useful for tests and
    /// density-field seeding. For surface mesh voxelization, prefer a dedicated builder.
    pub fn insert_point(&mut self, p: &Point3<Real>) {
        Self::insert_point_impl(&mut self.root, &self.center, self.half, 0, self.max_depth, p);
    }

    fn insert_point_impl(
        node: &mut SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        p: &Point3<Real>,
    ) {
        if depth == max_depth {
            node.occupancy = Occupancy::Full;
            node.children_mask = 0;
            node.children.clear();
            return;
        }
        node.occupancy = Occupancy::Mixed; // inserting points makes the path Mixed
        let cidx = Self::octant_for_point(center, p);
        let child_center = Self::child_center(center, half, cidx);
        let child = node.ensure_child(cidx);
        Self::insert_point_impl(child, &child_center, half * 0.5, depth + 1, max_depth, p);
    }

    /// Query occupied state at the leaf that would contain `p`.
    pub fn query_point(&self, p: &Point3<Real>) -> Occupancy {
        Self::query_point_impl(&self.root, &self.center, self.half, 0, self.max_depth, p)
    }

    fn query_point_impl(
        node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        p: &Point3<Real>,
    ) -> Occupancy {
        match node.occupancy {
            Occupancy::Empty => return Occupancy::Empty,
            Occupancy::Full => return Occupancy::Full,
            Occupancy::Mixed => {},
        }

        if depth == max_depth || node.is_leaf() {
            return node.occupancy;
        }

        let cidx = Self::octant_for_point(center, p);
        if let Some(child) = node.get_child(cidx) {
            let child_center = Self::child_center(center, half, cidx);
            return Self::query_point_impl(child, &child_center, half * 0.5, depth + 1, max_depth, p);
        }
        // Mixed but no child => conservatively treat as Mixed
        Occupancy::Mixed
    }

    /// Iterate all leaf cells, calling `f(center, half, occupancy, &node)`.
    pub fn visit_leaves<F: FnMut(&Point3<Real>, Real, Occupancy, &SvoNode<S>)>(&self, mut f: F) {
        Self::visit_leaves_impl(&self.root, &self.center, self.half, &mut f);
    }

    fn visit_leaves_impl<F: FnMut(&Point3<Real>, Real, Occupancy, &SvoNode<S>)>(
        node: &SvoNode<S>, center: &Point3<Real>, half: Real, f: &mut F,
    ) {
        if node.is_leaf() {
            f(center, half, node.occupancy, node);
            return;
        }
        // Visit all present children
        let mut bit = 0u8;
        while bit < 8 {
            if node.has_child(bit) {
                let rank = node.child_rank(bit);
                let child_center = Self::child_center(center, half, bit);
                Self::visit_leaves_impl(&node.children[rank], &child_center, half * 0.5, f);
            }
            bit += 1;
        }
    }

    /// Attach a BSP payload to the node at `path` (sequence of child indices) creating nodes as needed.
    /// Useful for embedding a micro-BSP at Mixed cells during construction.
    pub fn attach_bsp_at_path(&mut self, path: &[u8], bsp: BspNode<S>, metadata: Option<S>) {
        let mut node: &mut SvoNode<S> = &mut self.root;
        let mut center = self.center;
        let mut half = self.half;
        for &idx in path {
            node.occupancy = Occupancy::Mixed;
            let c = Self::child_center(&center, half, idx);
            node = node.ensure_child(idx);
            center = c;
            half *= 0.5;
        }
        node.local_bsp = Some(bsp);
        node.metadata = metadata;
        if node.children_mask == 0 { node.occupancy = Occupancy::Mixed; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_query_point() {
        let mut svo: Svo<()> = Svo::new(Point3::new(0.0, 0.0, 0.0), 1.0, 5);
        let p = Point3::new(0.25, 0.25, 0.25);
        assert_eq!(svo.query_point(&p), Occupancy::Empty);
        svo.insert_point(&p);
        let occ = svo.query_point(&p);
        assert!(occ == Occupancy::Mixed || occ == Occupancy::Full);
    }

    #[test]
    fn attach_bsp_payload() {
        let mut svo: Svo<()> = Svo::new(Point3::new(0.0, 0.0, 0.0), 2.0, 4);
        // Path to (+x,+y,+z) child then (-x,+y,-z)
        let path = [0b111u8, 0b010u8];
        let bsp: BspNode<()> = BspNode::new();
        svo.attach_bsp_at_path(&path, bsp, None);
        // Verify structure exists
        let mut node = &*svo.root;
        assert!(matches!(node.occupancy, Occupancy::Mixed | Occupancy::Full | Occupancy::Empty));
        let c0 = node.get_child(0b111).expect("child 0b111");
        let c1 = c0.get_child(0b010).expect("child 0b010");
        assert!(c1.local_bsp.is_some());
    }
}

impl<S: Clone + Debug + Send + Sync> Svo<S> {
    /// Optimize memory usage across the entire SVO
    pub fn optimize_memory(&mut self) {
        self.root.optimize_memory();
    }

    /// Get total memory usage of the SVO
    pub fn memory_usage(&self) -> usize {
        std::mem::size_of::<Self>() + self.root.memory_usage()
    }

    /// Simplify the SVO by removing redundant nodes
    pub fn simplify(&mut self) {
        self.root.simplify();
    }

    /// Get statistics about the SVO structure
    pub fn statistics(&self) -> SvoStatistics {
        let mut stats = SvoStatistics::default();
        self.collect_statistics(&*self.root, 0, &mut stats);
        stats
    }

    /// Collect statistics recursively
    fn collect_statistics(&self, node: &SvoNode<S>, depth: u8, stats: &mut SvoStatistics) {
        stats.total_nodes += 1;
        stats.max_depth = stats.max_depth.max(depth);

        match node.occupancy {
            Occupancy::Empty => stats.empty_nodes += 1,
            Occupancy::Full => stats.full_nodes += 1,
            Occupancy::Mixed => {
                stats.mixed_nodes += 1;
                if node.local_bsp.is_some() {
                    stats.nodes_with_bsp += 1;
                }
            }
        }

        if node.is_leaf() {
            stats.leaf_nodes += 1;
        }

        // Recursively collect from children
        for child in &node.children {
            self.collect_statistics(child, depth + 1, stats);
        }
    }
}

/// Statistics about SVO structure for performance analysis
#[derive(Debug, Default, Clone)]
pub struct SvoStatistics {
    pub total_nodes: usize,
    pub empty_nodes: usize,
    pub full_nodes: usize,
    pub mixed_nodes: usize,
    pub leaf_nodes: usize,
    pub nodes_with_bsp: usize,
    pub max_depth: u8,
}

impl SvoStatistics {
    /// Calculate memory efficiency ratio
    pub fn memory_efficiency(&self) -> f64 {
        if self.total_nodes == 0 {
            return 0.0;
        }

        // Efficiency = (leaf nodes + nodes with BSP) / total nodes
        // Higher ratio means more efficient use of memory
        (self.leaf_nodes + self.nodes_with_bsp) as f64 / self.total_nodes as f64
    }

    /// Calculate surface detail ratio
    pub fn surface_detail_ratio(&self) -> f64 {
        if self.mixed_nodes == 0 {
            return 0.0;
        }

        // Ratio of Mixed nodes with BSP trees (detailed surface representation)
        self.nodes_with_bsp as f64 / self.mixed_nodes as f64
    }
}

