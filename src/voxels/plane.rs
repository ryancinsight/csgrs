//! Plane operations tailored for voxel/BSP hybrid workflows.
//!
//! Mirrors mesh::plane::Plane and related constants, but decoupled so voxel
//! pipelines can operate without depending on mesh module directly.

use crate::float_types::{EPSILON, Real};
use crate::voxels::vertex::Vertex;
use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Translation3, Vector3};
use robust::{Coord3D, orient3d};

pub const COPLANAR: i8 = 0;
pub const FRONT: i8 = 1;
pub const BACK: i8 = 2;
pub const SPANNING: i8 = 3;

#[derive(Debug, Clone)]
pub struct Plane {
    pub point_a: Point3<Real>,
    pub point_b: Point3<Real>,
    pub point_c: Point3<Real>,
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        if self.point_a == other.point_a && self.point_b == other.point_b && self.point_c == other.point_c {
            return true;
        }
        // co-planarity check via robust predicates
        orient3d(to_c3(self.point_a), to_c3(self.point_b), to_c3(self.point_c), to_c3(other.point_a)) == 0.0
            && orient3d(to_c3(self.point_a), to_c3(self.point_b), to_c3(self.point_c), to_c3(other.point_b)) == 0.0
            && orient3d(to_c3(self.point_a), to_c3(self.point_b), to_c3(self.point_c), to_c3(other.point_c)) == 0.0
    }
}

#[inline]
fn to_c3(p: Point3<Real>) -> Coord3D<Real> { Coord3D { x: p.x, y: p.y, z: p.z } }

impl Plane {
    /// Create a plane from three points
    ///
    /// **Mathematical Foundation**: A plane is uniquely defined by three non-collinear points.
    /// The plane equation is derived from the cross product of two edge vectors.
    pub fn from_points(point_a: Point3<Real>, point_b: Point3<Real>, point_c: Point3<Real>) -> Plane {
        Plane { point_a, point_b, point_c }
    }

    pub fn from_vertices(vertices: Vec<Vertex>) -> Plane {
        let n = vertices.len();
        let reference = Plane { point_a: vertices[0].pos, point_b: vertices[1].pos, point_c: vertices[2].pos };
        if n == 3 { return reference; }

        // longest chord
        let mut i0 = 0usize; let mut i1 = 1usize; let mut best = 0.0;
        for i in 0..n { for j in i+1..n {
            let d2 = (vertices[i].pos - vertices[j].pos).norm_squared();
            if d2 > best { best = d2; i0 = i; i1 = j; }
        }}
        if best < EPSILON*EPSILON { return reference; }

        let p0 = vertices[i0].pos; let p1 = vertices[i1].pos; let dir = p1 - p0;
        if dir.norm_squared() < EPSILON*EPSILON { return reference; }

        // farthest from line
        let mut i2 = 0usize; let mut a2_best = -1.0;
        for (k, v) in vertices.iter().enumerate() {
            if k == i0 || k == i1 { continue; }
            let a2 = (v.pos - p0).cross(&dir).norm_squared();
            if a2 > a2_best { a2_best = a2; i2 = k; }
        }
        if a2_best <= EPSILON*EPSILON { return reference; }
        let p2 = vertices[i2].pos;

        let mut plane = Plane { point_a: p0, point_b: p1, point_c: p2 };
        // orient to polygon winding via Newell's method
        let ref_n = vertices.iter().zip(vertices.iter().cycle().skip(1)).fold(Vector3::zeros(), |acc, (a,b)| acc + (a.pos - Point3::origin()).cross(&(b.pos - Point3::origin())));
        if plane.normal().dot(&ref_n) < 0.0 { plane.flip(); }
        plane
    }

    pub fn from_normal(normal: Vector3<Real>, offset: Real) -> Self {
        let n2 = normal.norm_squared();
        if n2 < EPSILON*EPSILON { panic!(); }
        let p0 = Point3::from(normal * (offset / n2));
        let mut u = if normal.z.abs() > normal.x.abs() || normal.z.abs() > normal.y.abs() { Vector3::x().cross(&normal) } else { Vector3::z().cross(&normal) };
        u.normalize_mut();
        let v = normal.cross(&u).normalize();
        Self { point_a: p0, point_b: p0 + u, point_c: p0 + v }
    }

    #[inline]
    pub fn orient_plane(&self, other: &Plane) -> i8 { let test = other.point_a + other.normal(); self.orient_point(&test) }

    #[inline]
    pub fn orient_point(&self, p: &Point3<Real>) -> i8 {
        let sign = orient3d(to_c3(self.point_a), to_c3(self.point_b), to_c3(self.point_c), to_c3(*p));
        if sign > EPSILON.into() { BACK } else if sign < (-EPSILON).into() { FRONT } else { COPLANAR }
    }

    #[inline]
    pub fn normal(&self) -> Vector3<Real> {
        let n = (self.point_b - self.point_a).cross(&(self.point_c - self.point_a));
        let len = n.norm(); if len < EPSILON { Vector3::zeros() } else { n/len }
    }

    #[inline]
    pub fn offset(&self) -> Real { self.normal().dot(&self.point_a.coords) }

    pub const fn flip(&mut self) { let tmp = self.point_a; self.point_a = self.point_b; self.point_b = tmp; }

    /// Classify a polygon with respect to the plane. Returns bitmask of COPLANAR/FRONT/BACK.
    pub fn classify_polygon<S: Clone>(&self, polygon: &crate::voxels::polygon::Polygon<S>) -> i8 {
        polygon.vertices.iter().fold(0, |acc, v| acc | self.orient_point(&v.pos))
    }

    pub fn split_polygon<S: Clone + Send + Sync>(&self, polygon: &crate::voxels::polygon::Polygon<S>) -> (Vec<crate::voxels::polygon::Polygon<S>>, Vec<crate::voxels::polygon::Polygon<S>>, Vec<crate::voxels::polygon::Polygon<S>>, Vec<crate::voxels::polygon::Polygon<S>>) {
        use crate::voxels::polygon::Polygon;
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();

        let normal = self.normal();
        let types: Vec<i8> = polygon.vertices.iter().map(|v| self.orient_point(&v.pos)).collect();
        let polygon_type = types.iter().fold(0, |acc, &t| acc | t);

        match polygon_type {
            COPLANAR => {
                if normal.dot(&polygon.plane.normal()) > 0.0 { coplanar_front.push(polygon.clone()); } else { coplanar_back.push(polygon.clone()); }
            }
            FRONT => front.push(polygon.clone()),
            BACK => back.push(polygon.clone()),
            _ => {
                let mut split_front = Vec::<Vertex>::new();
                let mut split_back = Vec::<Vertex>::new();
                for i in 0..polygon.vertices.len() {
                    let j = (i + 1) % polygon.vertices.len();
                    let ti = types[i]; let tj = types[j];
                    let vi = &polygon.vertices[i]; let vj = &polygon.vertices[j];
                    if ti != BACK { split_front.push(vi.clone()); }
                    if ti != FRONT { split_back.push(vi.clone()); }
                    if (ti | tj) == SPANNING {
                        let denom = normal.dot(&(vj.pos - vi.pos));
                        if denom.abs() > EPSILON {
                            let t = (self.offset() - normal.dot(&vi.pos.coords)) / denom;
                            let vn = vi.interpolate(vj, t);
                            split_front.push(vn.clone());
                            split_back.push(vn);
                        }
                    }
                }
                if split_front.len() >= 3 { front.push(Polygon::new(split_front, polygon.metadata.clone())); }
                if split_back.len() >= 3 { back.push(Polygon::new(split_back, polygon.metadata.clone())); }
            }
        }
        (coplanar_front, coplanar_back, front, back)
    }

    pub fn to_xy_transform(&self) -> (Matrix4<Real>, Matrix4<Real>) {
        let n = self.normal();
        let n_len = n.norm();
        if n_len < EPSILON { return (Matrix4::identity(), Matrix4::identity()); }
        let norm_dir = n / n_len;
        let rot = Rotation3::rotation_between(&norm_dir, &Vector3::z()).unwrap_or_else(Rotation3::identity);
        let iso_rot = Isometry3::from_parts(Translation3::identity(), rot.into());
        let denom = n.dot(&n);
        let p0_3d = norm_dir * (self.offset() / denom);
        let p0_rot = iso_rot.transform_point(&Point3::from(p0_3d));
        let shift_z = -p0_rot.z;
        let iso_trans = Translation3::new(0.0, 0.0, shift_z);
        let to_xy = iso_trans.to_homogeneous() * iso_rot.to_homogeneous();
        let from_xy = to_xy.try_inverse().unwrap_or_else(Matrix4::identity);
        (to_xy, from_xy)
    }
}

// Conversions to/from mesh::plane::Plane
impl From<crate::mesh::plane::Plane> for Plane {
    fn from(p: crate::mesh::plane::Plane) -> Self {
        Plane { point_a: p.point_a, point_b: p.point_b, point_c: p.point_c }
    }
}

impl From<Plane> for crate::mesh::plane::Plane {
    fn from(p: Plane) -> Self { crate::mesh::plane::Plane { point_a: p.point_a, point_b: p.point_b, point_c: p.point_c } }
}


