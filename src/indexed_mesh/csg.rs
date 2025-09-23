//! CSG trait implementation for IndexedMesh
//!
//! This module contains the CSG (Constructive Solid Geometry) trait implementation
//! for IndexedMesh, providing all boolean operations and transformations.

use super::core::IndexedMesh;
use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use crate::geometry;
use crate::traits::CSG;
use nalgebra::{Matrix4, Point3};
use std::{fmt::Debug, sync::OnceLock};

impl<S: Clone + Send + Sync + Debug> CSG for IndexedMesh<S> {
    /// Returns a new empty IndexedMesh
    fn new() -> Self {
        IndexedMesh::new()
    }

    /// Return a new IndexedMesh representing union of the two IndexedMeshes
    fn union(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        super::operations::union(self, other)
    }

    /// Return a new IndexedMesh representing difference of the two IndexedMeshes
    fn difference(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        super::operations::difference(self, other)
    }

    /// Return a new IndexedMesh representing intersection of the two IndexedMeshes
    fn intersection(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        super::operations::intersection(self, other)
    }

    /// Return a new IndexedMesh representing XOR of the two IndexedMeshes
    fn xor(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        super::operations::xor(self, other)
    }

    /// Apply transformation to IndexedMesh
    fn transform(&self, matrix: &Matrix4<Real>) -> IndexedMesh<S> {
        super::operations::transform(self, matrix)
    }

    /// Invert the IndexedMesh (flip inside vs. outside)
    fn inverse(&self) -> IndexedMesh<S> {
        super::operations::inverse(self)
    }

    /// Returns bounding box of the IndexedMesh
    ///
    /// # Mathematical Foundation
    /// Uses the shared geometry utilities for consistent bounding box computation
    /// across all geometric primitives in the library.
    ///
    /// # Algorithm
    /// - Uses vertex positions directly from the deduplicated vertex array
    /// - Computes min/max bounds using O(n) linear scan
    /// - Leverages shared geometry utilities for numerical stability
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Use shared geometry utilities for consistent bounding box computation
            let (mins, maxs) = geometry::compute_bounding_box_from_points(
                &self.vertices.iter().map(|v| v.pos).collect::<Vec<_>>()
            );

            // Handle degenerate case where no vertices exist
            if mins == maxs {
                Aabb::new(Point3::origin(), Point3::origin())
            } else {
                Aabb::new(mins, maxs)
            }
        })
    }

    /// Invalidate cached bounding box
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
        self.adjacency = OnceLock::new(); // Adjacency may also be invalid
    }
}
