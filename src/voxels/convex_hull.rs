//! Convex hull and Minkowski sum for Voxels, mirroring mesh::convex_hull

use crate::float_types::Real;
use crate::voxels::csg::Voxels;
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use chull::ConvexHullWrapper;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    pub fn convex_hull(&self) -> Voxels<S> {
        let points: Vec<Point3<Real>> = self.polygons().iter().flat_map(|poly| poly.vertices.iter().map(|v| v.pos)).collect();
        let points_for_hull: Vec<Vec<Real>> = points.iter().map(|p| vec![p.x, p.y, p.z]).collect();
        let hull = match ConvexHullWrapper::try_new(&points_for_hull, None) { Ok(h) => h, Err(_) => return Voxels::new() };
        let (verts, indices) = hull.vertices_indices();
        let mut polygons = Vec::new();
        for tri in indices.chunks(3) {
            let v0 = &verts[tri[0]]; let v1 = &verts[tri[1]]; let v2 = &verts[tri[2]];
            let vv0 = Vertex::new(Point3::new(v0[0], v0[1], v0[2]), Vector3::zeros());
            let vv1 = Vertex::new(Point3::new(v1[0], v1[1], v1[2]), Vector3::zeros());
            let vv2 = Vertex::new(Point3::new(v2[0], v2[1], v2[2]), Vector3::zeros());
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], None));
        }
        Voxels::from_polygons(&polygons, self.metadata.clone())
    }

    pub fn minkowski_sum(&self, other: &Voxels<S>) -> Voxels<S> {
        let verts_a: Vec<Point3<Real>> = self.polygons().iter().flat_map(|poly| poly.vertices.iter().map(|v| v.pos)).collect();
        let verts_b: Vec<Point3<Real>> = other.polygons().iter().flat_map(|poly| poly.vertices.iter().map(|v| v.pos)).collect();
        if verts_a.is_empty() || verts_b.is_empty() { return Voxels::new(); }
        let sum_points: Vec<_> = verts_a.iter().flat_map(|a| verts_b.iter().map(move |b| a + b.coords)).map(|v| vec![v.x, v.y, v.z]).collect();
        if sum_points.is_empty() { return Voxels::new(); }
        let hull = match ConvexHullWrapper::try_new(&sum_points, None) { Ok(h) => h, Err(_) => return Voxels::new() };
        let (verts, indices) = hull.vertices_indices();
        let polygons: Vec<Polygon<S>> = indices.chunks_exact(3).filter_map(|tri| {
            let v0 = &verts[tri[0]]; let v1 = &verts[tri[1]]; let v2 = &verts[tri[2]];
            let p0 = Point3::new(v0[0], v0[1], v0[2]); let p1 = Point3::new(v1[0], v1[1], v1[2]); let p2 = Point3::new(v2[0], v2[1], v2[2]);
            let normal = (p1 - p0).cross(&(p2 - p0)); if normal.norm_squared() <= Real::EPSILON { return None; }
            let n = normal.normalize();
            let vv0 = Vertex::new(p0, n); let vv1 = Vertex::new(p1, n); let vv2 = Vertex::new(p2, n);
            Some(Polygon::new(vec![vv0, vv1, vv2], None))
        }).collect();
        Voxels::from_polygons(&polygons, self.metadata.clone())
    }
}

