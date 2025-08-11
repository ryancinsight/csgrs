//! Polygon type for voxel/BSP hybrid use, mirroring mesh::polygon::Polygon with
//! minimal changes, parameterized by metadata S. Uses voxels::{vertex, plane}.

use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use crate::voxels::plane::Plane;
use crate::voxels::vertex::Vertex;
use geo::{LineString, Polygon as GeoPolygon, coord};
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;

#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
    pub bounding_box: OnceLock<Aabb>,
    pub metadata: Option<S>,
}

impl<S: Clone + PartialEq> PartialEq for Polygon<S> {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices && self.plane == other.plane && self.metadata == other.metadata
    }
}

impl<S: Clone + Send + Sync> Polygon<S> {
    pub fn new(vertices: Vec<Vertex>, metadata: Option<S>) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");
        let plane = Plane::from_vertices(vertices.clone());
        Polygon { vertices, plane, bounding_box: OnceLock::new(), metadata }
    }

    pub fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            let mut mins = Point3::new(Real::MAX, Real::MAX, Real::MAX);
            let mut maxs = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);
            for v in &self.vertices {
                mins.x = mins.x.min(v.pos.x);
                mins.y = mins.y.min(v.pos.y);
                mins.z = mins.z.min(v.pos.z);
                maxs.x = maxs.x.max(v.pos.x);
                maxs.y = maxs.y.max(v.pos.y);
                maxs.z = maxs.z.max(v.pos.z);
            }
            Aabb::new(mins, maxs)
        })
    }

    pub fn flip(&mut self) {
        self.vertices.reverse();
        for v in &mut self.vertices { v.flip(); }
        self.plane.flip();
    }

    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> { self.vertices.iter().zip(self.vertices.iter().cycle().skip(1)) }

    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        if self.vertices.len() < 3 { return Vec::new(); }
        if self.vertices.len() == 3 { return vec![[self.vertices[0], self.vertices[1], self.vertices[2]]]; }
        let normal_3d = self.plane.normal().normalize();
        let (u, v) = super::polygon::build_orthonormal_basis(normal_3d);
        let origin_3d = self.vertices[0].pos;

        // Use earcut when enabled (same as mesh::polygon). Here we replicate earcut path.
        #[cfg(feature = "earcut")]
        {
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                all_vertices_2d.push(coord! {x: x, y: y});
            }
            use geo::TriangulateEarcut;
            let triangulation = GeoPolygon::new(LineString::new(all_vertices_2d), Vec::new()).earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;
            let mut triangles = Vec::with_capacity(triangle_indices.len() / 3);
            for tri_chunk in triangle_indices.chunks_exact(3) {
                let mut tri_vertices = [Vertex::new(Point3::origin(), Vector3::zeros()); 3];
                for (k, &idx) in tri_chunk.iter().enumerate() {
                    let base = idx * 2; let x = vertices[base]; let y = vertices[base + 1];
                    let pos_3d = origin_3d.coords + (x * u) + (y * v);
                    tri_vertices[k] = Vertex::new(Point3::from(pos_3d), normal_3d);
                }
                triangles.push(tri_vertices);
            }
            return triangles;
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;
            #[allow(clippy::excessive_precision)]
            const MIN_ALLOWED_VALUE: Real = 1.793662034335766e-43;
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u); let x_clamped = if x.abs() < MIN_ALLOWED_VALUE { 0.0 } else { x };
                let y = offset.dot(&v); let y_clamped = if y.abs() < MIN_ALLOWED_VALUE { 0.0 } else { y };
                if !(x.is_finite() && y.is_finite() && x_clamped.is_finite() && y_clamped.is_finite()) { continue; }
                all_vertices_2d.push(coord! {x: x_clamped, y: y_clamped});
            }
            let polygon_2d = GeoPolygon::new(LineString::new(all_vertices_2d), Vec::new());
            let Ok(tris) = polygon_2d.constrained_triangulation(Default::default()) else { return Vec::new(); };
            let mut final_triangles = Vec::with_capacity(tris.len());
            for tri2d in tris {
                let [a,b,c] = [tri2d.0, tri2d.1, tri2d.2];
                let pa = origin_3d.coords + a.x * u + a.y * v;
                let pb = origin_3d.coords + b.x * u + b.y * v;
                let pc = origin_3d.coords + c.x * u + c.y * v;
                final_triangles.push([
                    Vertex::new(Point3::from(pa), normal_3d),
                    Vertex::new(Point3::from(pb), normal_3d),
                    Vertex::new(Point3::from(pc), normal_3d),
                ]);
            }
            return final_triangles;
        }

        // If neither triangulation feature is enabled, fall through to empty result when both earcut and delaunay are disabled.
        #[allow(unreachable_code)]
        { Vec::new() }
    }

    pub fn subdivide_triangles(&self, subdivisions: core::num::NonZeroU32) -> Vec<[Vertex; 3]> {
        let base = self.triangulate();
        let mut result = Vec::new();
        for tri in base {
            let mut queue = vec![tri];
            for _ in 0..subdivisions.get() {
                let mut next = Vec::new();
                for t in queue { next.extend(super::polygon::subdivide_triangle(t)); }
                queue = next;
            }
            result.extend(queue);
        }
        result
    }

    pub fn center(&self) -> Point3<Real> {
        let sum = self.vertices.iter().fold(Vector3::zeros(), |acc, v| acc + v.pos.coords);
        Point3::from(sum / self.vertices.len() as Real)
    }

    pub fn transform(&self, m: &nalgebra::Matrix4<Real>) -> Self {
        let transformed_vertices = self.vertices.iter().map(|v| v.transform(m)).collect();
        Self::new(transformed_vertices, self.metadata.clone())
    }

    /// Recompute this polygon's normal from all vertices, then set all vertices' normals to match (flat shading).
    pub fn set_new_normal(&mut self) {
        let normal = self.plane.normal();
        for v in &mut self.vertices { v.normal = normal; }
    }
}

pub fn build_orthonormal_basis(n: Vector3<Real>) -> (Vector3<Real>, Vector3<Real>) {
    let n = n.normalize();
    let other = if n.x.abs() < n.y.abs() && n.x.abs() < n.z.abs() { Vector3::x() } else if n.y.abs() < n.z.abs() { Vector3::y() } else { Vector3::z() };
    let v = n.cross(&other).normalize();
    let u = v.cross(&n).normalize();
    (u, v)
}

pub fn subdivide_triangle(tri: [Vertex; 3]) -> Vec<[Vertex; 3]> {
    let v01 = tri[0].interpolate(&tri[1], 0.5);
    let v12 = tri[1].interpolate(&tri[2], 0.5);
    let v20 = tri[2].interpolate(&tri[0], 0.5);
    vec![
        [tri[0], v01, v20],
        [v01, tri[1], v12],
        [v20, v12, tri[2]],
        [v01, v12, v20],
    ]
}

// Conversions to/from mesh::polygon::Polygon
impl<S: Clone> From<crate::mesh::polygon::Polygon<S>> for Polygon<S> {
    fn from(p: crate::mesh::polygon::Polygon<S>) -> Self {
        let verts: Vec<crate::voxels::vertex::Vertex> = p.vertices.into_iter().map(Into::into).collect();
        let plane: crate::voxels::plane::Plane = p.plane.into();
        let poly = Polygon { vertices: verts, plane, bounding_box: OnceLock::new(), metadata: p.metadata };
        poly
    }
}


