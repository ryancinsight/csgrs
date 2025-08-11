//! Vertex type for voxel-aware geometry pipelines.
//!
//! This mirrors mesh::vertex::Vertex with the same API surface where sensible,
//! keeping code cohesive for SVO+BSP workflows without pulling mesh::* directly.
//! It is intentionally minimal and reuses the same math semantics.

use crate::float_types::Real;
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone, PartialEq, Copy)]
pub struct Vertex {
    pub pos: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl Vertex {
    #[inline]
    pub fn new(mut pos: Point3<Real>, mut normal: Vector3<Real>) -> Self {
        for c in pos.coords.iter_mut() { if !c.is_finite() { *c = 0.0; } }
        for c in normal.iter_mut() { if !c.is_finite() { *c = 0.0; } }
        Vertex { pos, normal }
    }

    #[inline]
    pub fn flip(&mut self) { self.normal = -self.normal; }

    pub fn transform(&self, matrix: &nalgebra::Matrix4<Real>) -> Self {
        let pos_h = matrix * self.pos.to_homogeneous();
        let new_pos = Point3::from(pos_h.xyz());
        let normal_matrix = matrix.fixed_view::<3, 3>(0, 0);
        let new_normal = normal_matrix * self.normal;
        Vertex::new(new_pos, new_normal)
    }

    pub fn interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        let new_pos = self.pos + (other.pos - self.pos) * t;
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_pos, new_normal)
    }

    pub fn slerp_interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        let new_pos = self.pos + (other.pos - self.pos) * t;
        let n0 = self.normal.normalize();
        let n1 = other.normal.normalize();
        let dot = n0.dot(&n1).clamp(-1.0, 1.0);
        if (dot.abs() - 1.0).abs() < Real::EPSILON {
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return Vertex::new(new_pos, new_normal);
        }
        let omega = dot.acos();
        let sin_omega = omega.sin();
        if sin_omega.abs() < Real::EPSILON {
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return Vertex::new(new_pos, new_normal);
        }
        let a = ((1.0 - t) * omega).sin() / sin_omega;
        let b = (t * omega).sin() / sin_omega;
        let new_normal = (a * n0 + b * n1).normalize();
        Vertex::new(new_pos, new_normal)
    }

    pub fn distance_to(&self, other: &Vertex) -> Real { (self.pos - other.pos).norm() }
    pub fn distance_squared_to(&self, other: &Vertex) -> Real { (self.pos - other.pos).norm_squared() }

    pub fn normal_angle_to(&self, other: &Vertex) -> Real {
        let n1 = self.normal.normalize();
        let n2 = other.normal.normalize();
        let cos_angle = n1.dot(&n2).clamp(-1.0, 1.0);
        cos_angle.acos()
    }

    pub fn barycentric_interpolate(v1: &Vertex, v2: &Vertex, v3: &Vertex, u: Real, v: Real, w: Real) -> Vertex {
        let total = u + v + w;
        let (u, v, w) = if total.abs() > Real::EPSILON { (u/total, v/total, w/total) } else { (1.0/3.0, 1.0/3.0, 1.0/3.0) };
        let new_pos = Point3::from(u * v1.pos.coords + v * v2.pos.coords + w * v3.pos.coords);
        let new_normal = (u * v1.normal + v * v2.normal + w * v3.normal).normalize();
        Vertex::new(new_pos, new_normal)
    }
}

// Conversion from mesh::vertex::Vertex to voxels::vertex::Vertex
impl From<crate::mesh::vertex::Vertex> for Vertex {
    fn from(v: crate::mesh::vertex::Vertex) -> Self { Vertex::new(v.pos, v.normal) }
}


