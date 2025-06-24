use super::CSG;
use crate::core::float_types::parry3d::{
    bounding_volume::Aabb,
    query::{Ray, RayCast},
    shape::Triangle,
};
use crate::core::float_types::{EPSILON, Real};
use geo::BoundingRect;
use nalgebra::{Isometry3, Point3, Vector3, partial_max, partial_min};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this CSG and returns a list of (intersection_point, distance),
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
            .flat_map(|poly| poly.tessellate())
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
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        // 5) remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < EPSILON);

        hits
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] by merging:
    /// 1. The 3D bounds of all `polygons`.
    /// 2. The 2D bounding rectangle of `self.geometry`, interpreted at z=0.
    ///
    /// [`parry3d::bounding_volume::Aabb`]: crate::core::float_types::parry3d::bounding_volume::Aabb
    pub fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons and the 2D geometry's bounding_rect.
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for v in &poly.vertices {
                    min_x = *partial_min(&min_x, &v.pos.x).unwrap();
                    min_y = *partial_min(&min_y, &v.pos.y).unwrap();
                    min_z = *partial_min(&min_z, &v.pos.z).unwrap();

                    max_x = *partial_max(&max_x, &v.pos.x).unwrap();
                    max_y = *partial_max(&max_y, &v.pos.y).unwrap();
                    max_z = *partial_max(&max_z, &v.pos.z).unwrap();
                }
            }

            // 2) Gather from the 2D geometry using `geo::BoundingRect`
            //    This gives us (min_x, min_y) / (max_x, max_y) in 2D. For 3D, treat z=0.
            //    Explicitly capture the result of `.bounding_rect()` as an Option<Rect<Real>>
            let maybe_rect: Option<geo::Rect<Real>> = self.geometry.bounding_rect();

            if let Some(rect) = maybe_rect {
                let min_pt = rect.min();
                let max_pt = rect.max();

                // Merge the 2D bounds into our existing min/max, forcing z=0 for 2D geometry.
                min_x = *partial_min(&min_x, &min_pt.x).unwrap();
                min_y = *partial_min(&min_y, &min_pt.y).unwrap();
                min_z = *partial_min(&min_z, &0.0).unwrap();

                max_x = *partial_max(&max_x, &max_pt.x).unwrap();
                max_y = *partial_max(&max_y, &max_pt.y).unwrap();
                max_z = *partial_max(&max_z, &0.0).unwrap();
            }

            // If still uninitialized (e.g., no polygons or geometry), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }
} 
