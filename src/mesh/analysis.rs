//! Geometric analysis and ray intersection functionality

use crate::float_types::{
    Real,
    parry3d::query::{Ray, RayCast},
    parry3d::shape::Triangle,
};
use nalgebra::{Isometry3, Point3, Vector3};
use std::fmt::Debug;

use super::Mesh;

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Extracts vertices and indices from the Mesh's tessellated polygons.
    pub(crate) fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        let tri_csg = self.triangulate();
        let vertices = tri_csg
            .polygons
            .iter()
            .flat_map(|p| [p.vertices[0].pos, p.vertices[1].pos, p.vertices[2].pos])
            .collect();

        let indices = (0..tri_csg.polygons.len())
            .map(|i| {
                let offset = i as u32 * 3;
                [offset, offset + 1, offset + 2]
            })
            .collect();

        (vertices, indices)
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this Mesh and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray's start point.
    /// - `direction`: The ray's direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3<Real>, Real)` where:
    /// - `Point3<Real>` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<Real>,
        direction: &Vector3<Real>,
    ) -> Vec<(Point3<Real>, Real)> {
        let ray = Ray::new(*origin, *direction);
        let iso = Isometry3::identity(); // No transformation on the triangles themselves.

        let mut hits: Vec<_> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .filter_map(|tri| {
                let a = tri[0].pos;
                let b = tri[1].pos;
                let c = tri[2].pos;
                let triangle = Triangle::new(a, b, c);
                triangle
                    .cast_ray_and_get_normal(&iso, &ray, Real::MAX, true)
                    .map(|hit| {
                        let point_on_ray = ray.point_at(hit.time_of_impact);
                        (Point3::from(point_on_ray.coords), hit.time_of_impact)
                    })
            })
            .collect();

        // 4) Sort hits by ascending distance (toi):
        // Use total_cmp for robust floating-point comparison (available in Rust 1.85+)
        hits.sort_by(|a, b| a.1.total_cmp(&b.1));
        // 5) remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < crate::float_types::EPSILON);

        hits
    }

    /// Uses Parry to check if a point is inside a `Mesh`'s as a `TriMesh`.
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```
    /// # use csgrs::mesh::Mesh;
    /// # use nalgebra::Point3;
    /// # use nalgebra::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, None).expect("Failed to create cube");
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3<Real>) -> bool {
        self.ray_intersections(point, &Vector3::new(1.0, 1.0, 1.0))
            .len()
            % 2
            == 1
    }
}
