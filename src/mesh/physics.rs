//! Physics integration with Rapier and Parry

use crate::float_types::{
    Real,
    parry3d::shape::{Shape, TriMesh},
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape,
    },
};
use nalgebra::{Point3, Quaternion, Unit};
use std::fmt::Debug;

use super::Mesh;

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Convert the polygons in this Mesh to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_rapier_shape(&self) -> Option<SharedShape> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok().map(SharedShape::new)
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`.
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_trimesh(&self) -> Option<TriMesh> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok()
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> Option<(Real, Point3<Real>, Unit<Quaternion<Real>>)> {
        self.to_trimesh().map(|trimesh| {
            let mp = trimesh.mass_properties(density);
            (
                mp.mass(),
                mp.local_com,                     // a Point3<Real>
                mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
            )
        })
    }

    /// Create a Rapier rigid body + collider from this Mesh, using
    /// an axis-angle `rotation` in 3D (the vector's length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: nalgebra::Vector3<Real>,
        rotation: nalgebra::Vector3<Real>, // rotation axis scaled by angle (radians)
        density: Real,
    ) -> Option<RigidBodyHandle> {
        self.to_rapier_shape().map(|shape| {
            // Build a Rapier RigidBody
            let rb = RigidBodyBuilder::dynamic()
                .translation(translation)
                // Now `rotation(...)` expects an axis-angle Vector3.
                .rotation(rotation)
                .build();
            let rb_handle = rb_set.insert(rb);

            // Build the collider
            let coll = ColliderBuilder::new(shape).density(density).build();
            co_set.insert_with_parent(coll, rb_handle, rb_set);

            rb_handle
        })
    }
}
