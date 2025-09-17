//! `Sketch` struct and implementations of the `CSGOps` trait for `Sketch`

use crate::float_types::Real;
use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::mesh::Mesh;
use crate::traits::CSG;
use geo::algorithm::winding_order::Winding;
use geo::{
    AffineOps, AffineTransform, BooleanOps as GeoBooleanOps, BoundingRect, Coord, Geometry,
    GeometryCollection, LineString, MultiPolygon, Orient, Polygon as GeoPolygon, Rect,
    orient::Direction,
};
use nalgebra::{Matrix4, Point3, partial_max, partial_min};
use std::fmt::Debug;
use std::sync::OnceLock;

pub mod extrudes;
pub mod shapes;

#[cfg(feature = "hershey-text")]
pub mod hershey;

#[cfg(feature = "image-io")]
pub mod image;

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(feature = "offset")]
pub mod offset;

#[cfg(feature = "truetype-text")]
pub mod truetype;

#[derive(Clone, Debug)]
pub struct Sketch<S> {
    /// 2D points, lines, polylines, polygons, and multipolygons
    pub geometry: GeometryCollection<Real>,

    /// Lazily calculated AABB that spans `geometry`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> Sketch<S> {
    /// Take the [`geo::Polygon`]'s from the `CSG`'s geometry collection
    pub fn to_multipolygon(&self) -> MultiPolygon<Real> {
        let polygons = self
            .geometry
            .iter()
            .flat_map(|geom| match geom {
                Geometry::Polygon(poly) => vec![poly.clone()],
                Geometry::MultiPolygon(mp) => mp.0.clone(),
                _ => vec![],
            })
            .collect();

        MultiPolygon(polygons)
    }

    /// Create a Sketch from a `geo::GeometryCollection`.
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: Option<S>) -> Sketch<S> {
        let mut new_sketch = Sketch::new();
        new_sketch.geometry = geometry;
        new_sketch.metadata = metadata;
        new_sketch
    }

    /// Triangulate this polygon into a list of triangles, each triangle is [v0, v1, v2].
    #[must_use]
    pub fn triangulate_2d(
        outer: &[[Real; 2]],
        holes: &[&[[Real; 2]]],
    ) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> =
            outer.iter().map(|&[x, y]| Coord { x, y }).collect();

        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> =
                    hole.iter().map(|&[x, y]| Coord { x, y }).collect();
                LineString::new(coords)
            })
            .collect();

        // Ear-cut triangulation on the polygon (outer + holes)
        let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);

        #[cfg(all(feature = "earcut", not(feature = "delaunay")))]
        {
            use geo::TriangulateEarcut;
            let triangulation = polygon.earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert the 2D result (x,y) into 3D triangles with z=0
            let mut result = Vec::with_capacity(triangle_indices.len() / 3);
            for tri in triangle_indices.chunks_exact(3) {
                let pts = [
                    Point3::new(vertices[2 * tri[0]], vertices[2 * tri[0] + 1], 0.0),
                    Point3::new(vertices[2 * tri[1]], vertices[2 * tri[1] + 1], 0.0),
                    Point3::new(vertices[2 * tri[2]], vertices[2 * tri[2] + 1], 0.0),
                ];
                result.push(pts);
            }
            result
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;
            // We want polygons with holes => constrained triangulation.
            // For safety, handle the Result the trait returns:
            let Ok(tris) = polygon.constrained_triangulation(Default::default()) else {
                // If a triangulation error is a possibility,
                // pick the error-handling you want here:
                return Vec::new();
            };

            let mut result = Vec::with_capacity(tris.len());
            for triangle in tris {
                // Each triangle is a geo_types::Triangle with fields .0, .1, .2
                // containing the 2D coordinates. We'll embed them at z=0.
                let [a, b, c] = [triangle.0, triangle.1, triangle.2];
                result.push([
                    Point3::new(a.x, a.y, 0.0),
                    Point3::new(b.x, b.y, 0.0),
                    Point3::new(c.x, c.y, 0.0),
                ]);
            }
            result
        }

        #[cfg(not(any(feature = "delaunay", feature = "earcut")))]
        {
            // Fallback when neither triangulation feature is enabled
            // This should return an empty result as triangulation is not available
            Vec::new()
        }
    }

    /// Return a copy of this `Sketch` whose polygons are normalised so that
    /// exterior rings wind counter-clockwise and interior rings clockwise.
    #[must_use]
    pub fn renormalize(&self) -> Sketch<S> {
        // Re-build the collection, orienting only what’s supported.
        let oriented_geoms: Vec<Geometry<Real>> = self
            .geometry
            .iter()
            .map(|geom| match geom {
                Geometry::Polygon(p) => {
                    Geometry::Polygon(p.clone().orient(Direction::Default))
                },
                Geometry::MultiPolygon(mp) => {
                    Geometry::MultiPolygon(mp.clone().orient(Direction::Default))
                },
                // Everything else keeps its original orientation.
                _ => geom.clone(),
            })
            .collect();

        Sketch {
            geometry: GeometryCollection(oriented_geoms),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for Sketch<S> {
    /// Returns a new empty Sketch
    fn new() -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Return a new Sketch representing union of the two Sketches.
    ///
    /// ```text
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Sketch<S>) -> Sketch<S> {
        // Extract multipolygon from geometry
        let polys1 = self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform union on those multipolygons
        let unioned = polys1.union(polys2); // This is valid if each is a MultiPolygon
        let oriented = unioned.orient(Direction::Default);

        // Wrap the unioned multipolygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // re-insert lines & points from both sets:
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Sketch representing diffarence of the two Sketches.
    ///
    /// ```text
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Sketch<S>) -> Sketch<S> {
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform difference on those multipolygons
        let differenced = polys1.difference(polys2);
        let oriented = differenced.orient(Direction::Default);

        // Wrap the differenced multipolygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // Re-insert lines & points from self only
        // (If you need to exclude lines/points that lie inside other, you'd need more checks here.)
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Sketch representing intersection of the two Sketches.
    ///
    /// ```text
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Sketch<S>) -> Sketch<S> {
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform intersection on those multipolygons
        let intersected = polys1.intersection(polys2);
        let oriented = intersected.orient(Direction::Default);

        // Wrap the intersected multipolygons + lines/points into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // For lines and points: keep them only if they intersect in both sets
        // todo: detect intersection of non-polygons
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Sketch representing space in this Sketch excluding the space in the
    /// other Sketch plus the space in the other Sketch excluding the space in this Sketch.
    ///
    /// ```text
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Sketch<S>) -> Sketch<S> {
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform symmetric difference (XOR)
        let xored = polys1.xor(polys2);
        let oriented = xored.orient(Direction::Default);

        // Wrap in a new GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // Re-insert lines & points from both sets
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by ignoring the resulting z after transform.
    fn transform(&self, mat: &Matrix4<Real>) -> Sketch<S> {
        let mut sketch = self.clone();

        // Convert the top-left 2×2 submatrix + translation of a 4×4 into a geo::AffineTransform
        // The 4x4 looks like:
        //  [ m11  m12  m13  m14 ]
        //  [ m21  m22  m23  m24 ]
        //  [ m31  m32  m33  m34 ]
        //  [ m41  m42  m43  m44 ]
        //
        // For 2D, we use the sub-block:
        //   a = m11,  b = m12,
        //   d = m21,  e = m22,
        //   xoff = m14,
        //   yoff = m24,
        // ignoring anything in z.
        //
        // So the final affine transform in 2D has matrix:
        //   [a   b   xoff]
        //   [d   e   yoff]
        //   [0   0    1  ]
        let a = mat[(0, 0)];
        let b = mat[(0, 1)];
        let xoff = mat[(0, 3)];
        let d = mat[(1, 0)];
        let e = mat[(1, 1)];
        let yoff = mat[(1, 3)];

        let affine2 = AffineTransform::new(a, b, xoff, d, e, yoff);

        // Transform sketch.geometry (the GeometryCollection) in 2D
        sketch.geometry = sketch.geometry.affine_transform(&affine2);

        // invalidate the old cached bounding box
        sketch.bounding_box = OnceLock::new();

        sketch
    }

    /// Returns an [`Aabb`] containing:
    /// The 2D bounding rectangle of `self.geometry`, interpreted at z=0.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons and the 2D geometry’s bounding_rect.
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // Gather from the 2D geometry using `geo::BoundingRect`
            // This gives us (min_x, min_y) / (max_x, max_y)
            // Explicitly capture the result of `.bounding_rect()` as an Option<Rect<Real>>
            let maybe_rect: Option<Rect<Real>> = self.geometry.bounding_rect();

            if let Some(rect) = maybe_rect {
                let min_pt = rect.min();
                let max_pt = rect.max();

                // Merge the 2D bounds into our existing min/max, forcing z=0 for 2D geometry.
                if let Some(&new_min_x) = partial_min(&min_x, &min_pt.x) {
                    min_x = new_min_x;
                }
                if let Some(&new_min_y) = partial_min(&min_y, &min_pt.y) {
                    min_y = new_min_y;
                }
                if let Some(&new_min_z) = partial_min(&min_z, &0.0) {
                    min_z = new_min_z;
                }

                if let Some(&new_max_x) = partial_max(&max_x, &max_pt.x) {
                    max_x = new_max_x;
                }
                if let Some(&new_max_y) = partial_max(&max_y, &max_pt.y) {
                    max_y = new_max_y;
                }
                if let Some(&new_max_z) = partial_max(&max_z, &0.0) {
                    max_z = new_max_z;
                }
            }

            // If still uninitialized (e.g., no geometry), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Sketch (flip inside vs. outside)
    fn inverse(&self) -> Sketch<S> {
        // Re-build the collection, orienting only what’s supported.
        let oriented_geoms: Vec<Geometry<Real>> = self
            .geometry
            .iter()
            .map(|geom| match geom {
                Geometry::Polygon(p) => {
                    let flipped = if p.exterior().is_ccw() {
                        p.clone().orient(Direction::Reversed)
                    } else {
                        p.clone().orient(Direction::Default)
                    };
                    Geometry::Polygon(flipped)
                },
                Geometry::MultiPolygon(mp) => {
                    // Loop over every polygon inside and apply the same rule.
                    let flipped_polys: Vec<GeoPolygon<Real>> =
                        mp.0.iter()
                            .map(|p| {
                                if p.exterior().is_ccw() {
                                    p.clone().orient(Direction::Reversed)
                                } else {
                                    p.clone().orient(Direction::Default)
                                }
                            })
                            .collect();

                    Geometry::MultiPolygon(MultiPolygon(flipped_polys))
                },
                // Everything else keeps its original orientation.
                _ => geom.clone(),
            })
            .collect();

        Sketch {
            geometry: GeometryCollection(oriented_geoms),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> From<Mesh<S>> for Sketch<S> {
    fn from(mesh: Mesh<S>) -> Self {
        // If mesh is empty, return empty Sketch
        if mesh.polygons.is_empty() {
            return Sketch::new();
        }

        // Convert mesh into a collection of 2D polygons
        let mut flattened_3d = Vec::new(); // will store geo::Polygon<Real>

        for poly in &mesh.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.triangulate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let ring = vec![
                    (tri[0].pos.x, tri[0].pos.y),
                    (tri[1].pos.x, tri[1].pos.y),
                    (tri[2].pos.x, tri[2].pos.y),
                    (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
                ];
                let polygon_2d = geo::Polygon::new(LineString::from(ring), vec![]);
                flattened_3d.push(polygon_2d);
            }
        }

        // Union all these polygons together into one MultiPolygon
        // (We could chain them in a fold-based union.)
        let unioned_from_3d = if flattened_3d.is_empty() {
            MultiPolygon::new(Vec::new())
        } else {
            // Start with the first polygon as a MultiPolygon
            let mut mp_acc = MultiPolygon(vec![flattened_3d[0].clone()]);
            // Union in the rest
            for p in flattened_3d.iter().skip(1) {
                mp_acc = mp_acc.union(&MultiPolygon(vec![p.clone()]));
            }
            mp_acc
        };

        // Ensure consistent orientation (CCW for exteriors):
        let oriented = unioned_from_3d.orient(Direction::Default);

        // Store final polygons as a MultiPolygon in a new GeometryCollection
        let mut new_gc = GeometryCollection::default();
        new_gc.0.push(Geometry::MultiPolygon(oriented));

        Sketch {
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traits::CSG;
    use geo::Geometry;
    use nalgebra::Vector3;

    #[test]
    fn test_sketch_new_creates_empty_geometry() {
        // **Mathematical Foundation**: Empty sketch initialization
        // **SRS Requirement NFR007**: Edge case handling (empty mesh operations)

        let sketch = Sketch::<()>::new();
        assert!(
            sketch.geometry.0.is_empty(),
            "New sketch should have empty geometry"
        );
        assert!(
            sketch.metadata.is_none(),
            "New sketch should have no metadata"
        );
    }

    #[test]
    fn test_sketch_rectangle_construction() {
        // **Mathematical Foundation**: Axis-aligned rectangle construction
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        let rect = Sketch::<()>::rectangle(4.0, 6.0, None);

        // Verify the geometry contains exactly one polygon
        let polygons: Vec<_> = rect
            .geometry
            .0
            .iter()
            .filter_map(|g| match g {
                Geometry::Polygon(p) => Some(p),
                _ => None,
            })
            .collect();

        assert_eq!(
            polygons.len(),
            1,
            "Rectangle should contain exactly one polygon"
        );

        // Verify bounding box dimensions
        let bbox = rect.bounding_box();
        let width = bbox.maxs.x - bbox.mins.x;
        let height = bbox.maxs.y - bbox.mins.y;

        assert!(
            (width - 4.0).abs() < 1e-8,
            "Width should be 4.0, got {}",
            width
        );
        assert!(
            (height - 6.0).abs() < 1e-8,
            "Height should be 6.0, got {}",
            height
        );

        // Verify rectangle goes from (0,0) to (width,length)
        assert!(
            (bbox.mins.x - 0.0).abs() < 1e-8,
            "Should start at X=0, got {}",
            bbox.mins.x
        );
        assert!(
            (bbox.mins.y - 0.0).abs() < 1e-8,
            "Should start at Y=0, got {}",
            bbox.mins.y
        );
        assert!(
            (bbox.mins.z - bbox.maxs.z).abs() < 1e-10,
            "Z should be zero thickness"
        );
    }

    #[test]
    fn test_sketch_circle_construction() {
        // **Mathematical Foundation**: Circular polygon approximation
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        let segments = 32;
        let radius = 2.5;
        let circle = Sketch::<()>::circle(radius, segments, None);

        // Verify the geometry contains exactly one polygon
        let polygons: Vec<_> = circle
            .geometry
            .0
            .iter()
            .filter_map(|g| match g {
                Geometry::Polygon(p) => Some(p),
                _ => None,
            })
            .collect();

        assert_eq!(polygons.len(), 1, "Circle should contain exactly one polygon");

        // Verify bounding box is approximately square
        let bbox = circle.bounding_box();
        let width = bbox.maxs.x - bbox.mins.x;
        let height = bbox.maxs.y - bbox.mins.y;

        // Circle bounding box should be approximately 2*radius x 2*radius
        let expected_size = 2.0 * radius;
        assert!(
            (width - expected_size).abs() < 0.1,
            "Width should be ~{}, got {}",
            expected_size,
            width
        );
        assert!(
            (height - expected_size).abs() < 0.1,
            "Height should be ~{}, got {}",
            expected_size,
            height
        );

        // Should be centered at origin
        assert!(
            (bbox.mins.x + bbox.maxs.x).abs() < 1e-8,
            "Should be centered at X=0"
        );
        assert!(
            (bbox.mins.y + bbox.maxs.y).abs() < 1e-8,
            "Should be centered at Y=0"
        );
    }

    #[test]
    fn test_cag_union_basic_functionality() {
        // **Mathematical Foundation**: Boolean union operation on 2D geometries
        // **SRS Requirement FR001**: Union operations on geometric objects

        let rect1 = Sketch::<()>::rectangle(2.0, 2.0, None);
        let rect2 = Sketch::<()>::rectangle(2.0, 2.0, None).translate(1.0, 1.0, 0.0);

        let union = rect1.union(&rect2);

        // Union should contain polygons
        let polygons: Vec<_> = union
            .geometry
            .0
            .iter()
            .filter_map(|g| match g {
                Geometry::MultiPolygon(mp) => Some(mp),
                _ => None,
            })
            .collect();

        assert!(!polygons.is_empty(), "Union should contain geometry");

        // Verify bounding box spans both rectangles
        let bbox = union.bounding_box();

        // Union should contain both original rectangles
        let rect1_bbox = rect1.bounding_box();
        let rect2_bbox = rect2.bounding_box();

        // Union bounding box should encompass both input bounding boxes
        assert!(
            bbox.mins.x <= rect1_bbox.mins.x.min(rect2_bbox.mins.x),
            "Union should encompass both inputs"
        );
        assert!(
            bbox.mins.y <= rect1_bbox.mins.y.min(rect2_bbox.mins.y),
            "Union should encompass both inputs"
        );
        assert!(
            bbox.maxs.x >= rect1_bbox.maxs.x.max(rect2_bbox.maxs.x),
            "Union should encompass both inputs"
        );
        assert!(
            bbox.maxs.y >= rect1_bbox.maxs.y.max(rect2_bbox.maxs.y),
            "Union should encompass both inputs"
        );

        // Metadata should be preserved from first operand
        assert!(
            union.metadata.is_none(),
            "Union should preserve None metadata"
        );
    }

    #[test]
    fn test_cag_difference_basic_functionality() {
        // **Mathematical Foundation**: Boolean difference operation on 2D geometries
        // **SRS Requirement FR002**: Difference operations (minuend - subtrahend)

        let rect1 = Sketch::<()>::rectangle(4.0, 4.0, None);
        let rect2 = Sketch::<()>::rectangle(2.0, 2.0, None);

        let difference = rect1.difference(&rect2);

        // Difference should contain geometry
        let polygons: Vec<_> = difference
            .geometry
            .0
            .iter()
            .filter_map(|g| match g {
                Geometry::MultiPolygon(mp) => Some(mp),
                _ => None,
            })
            .collect();

        assert!(!polygons.is_empty(), "Difference should contain geometry");

        // Verify bounding box is contained within the original rectangle
        let bbox = difference.bounding_box();
        let original_bbox = rect1.bounding_box();

        // Difference result should be contained within the first rectangle
        assert!(
            bbox.mins.x >= original_bbox.mins.x - 1e-6,
            "Difference should be within first rectangle bounds"
        );
        assert!(
            bbox.mins.y >= original_bbox.mins.y - 1e-6,
            "Difference should be within first rectangle bounds"
        );
        assert!(
            bbox.maxs.x <= original_bbox.maxs.x + 1e-6,
            "Difference should be within first rectangle bounds"
        );
        assert!(
            bbox.maxs.y <= original_bbox.maxs.y + 1e-6,
            "Difference should be within first rectangle bounds"
        );
    }

    #[test]
    fn test_cag_intersection_basic_functionality() {
        // **Mathematical Foundation**: Boolean intersection operation on 2D geometries
        // **SRS Requirement FR003**: Intersection operations (overlapping regions)

        let rect1 = Sketch::<()>::rectangle(3.0, 3.0, None);
        let rect2 = Sketch::<()>::rectangle(3.0, 3.0, None).translate(1.0, 1.0, 0.0);

        let intersection = rect1.intersection(&rect2);

        // Intersection should contain geometry
        let polygons: Vec<_> = intersection
            .geometry
            .0
            .iter()
            .filter_map(|g| match g {
                Geometry::MultiPolygon(mp) => Some(mp),
                _ => None,
            })
            .collect();

        assert!(!polygons.is_empty(), "Intersection should contain geometry");

        // Verify bounding box represents overlapping region
        let bbox = intersection.bounding_box();

        // Intersection should be contained within both input rectangles
        let rect1_bbox = rect1.bounding_box();
        let rect2_bbox = rect2.bounding_box();

        // Intersection bounds should be within the overlap region
        let overlap_min_x = rect1_bbox.mins.x.max(rect2_bbox.mins.x);
        let overlap_min_y = rect1_bbox.mins.y.max(rect2_bbox.mins.y);
        let overlap_max_x = rect1_bbox.maxs.x.min(rect2_bbox.maxs.x);
        let overlap_max_y = rect1_bbox.maxs.y.min(rect2_bbox.maxs.y);

        assert!(
            bbox.mins.x >= overlap_min_x - 1e-6,
            "Intersection should be within overlap region"
        );
        assert!(
            bbox.mins.y >= overlap_min_y - 1e-6,
            "Intersection should be within overlap region"
        );
        assert!(
            bbox.maxs.x <= overlap_max_x + 1e-6,
            "Intersection should be within overlap region"
        );
        assert!(
            bbox.maxs.y <= overlap_max_y + 1e-6,
            "Intersection should be within overlap region"
        );
    }

    #[test]
    fn test_cag_xor_basic_functionality() {
        // **Mathematical Foundation**: Symmetric difference (XOR) operation
        // **SRS Requirement FR004**: XOR operations (symmetric difference)

        let rect1 = Sketch::<()>::rectangle(2.0, 2.0, None);
        let rect2 = Sketch::<()>::rectangle(2.0, 2.0, None).translate(1.0, 0.0, 0.0);

        let xor = rect1.xor(&rect2);

        // XOR should contain geometry
        let polygons: Vec<_> = xor
            .geometry
            .0
            .iter()
            .filter_map(|g| match g {
                Geometry::MultiPolygon(mp) => Some(mp),
                _ => None,
            })
            .collect();

        assert!(!polygons.is_empty(), "XOR should contain geometry");

        // Verify bounding box spans non-overlapping regions
        let bbox = xor.bounding_box();
        let rect1_bbox = rect1.bounding_box();
        let rect2_bbox = rect2.bounding_box();

        // XOR should encompass the non-overlapping parts of both rectangles
        let expected_min_x = rect1_bbox.mins.x.min(rect2_bbox.mins.x);
        let expected_max_x = rect1_bbox.maxs.x.max(rect2_bbox.maxs.x);

        assert!(
            bbox.mins.x <= expected_min_x + 1e-6,
            "XOR should encompass both rectangles, min_x={}",
            bbox.mins.x
        );
        assert!(
            bbox.maxs.x >= expected_max_x - 1e-6,
            "XOR should encompass both rectangles, max_x={}",
            bbox.maxs.x
        );
    }

    #[test]
    fn test_cag_union_edge_case_empty_sketches() {
        // **Mathematical Foundation**: Union with empty geometries
        // **SRS Requirement NFR007**: Edge case handling (empty mesh operations)

        let empty1 = Sketch::<()>::new();
        let rect = Sketch::<()>::rectangle(2.0, 2.0, None);

        let union1 = empty1.union(&rect);
        let union2 = rect.union(&empty1);

        // Both unions should be equivalent to the non-empty sketch
        let bbox1 = union1.bounding_box();
        let bbox2 = union2.bounding_box();
        let original_bbox = rect.bounding_box();

        assert!(
            (bbox1.mins.x - original_bbox.mins.x).abs() < 1e-8,
            "Union with empty should preserve bounds"
        );
        assert!(
            (bbox2.mins.x - original_bbox.mins.x).abs() < 1e-8,
            "Union with empty should preserve bounds"
        );
    }

    #[test]
    fn test_cag_difference_edge_case_empty_sketches() {
        // **Mathematical Foundation**: Difference with empty geometries
        // **SRS Requirement NFR007**: Edge case handling (empty mesh operations)

        let empty = Sketch::<()>::new();
        let rect = Sketch::<()>::rectangle(2.0, 2.0, None);

        let difference1 = rect.difference(&empty);
        let difference2 = empty.difference(&rect);

        // Rectangle minus empty should equal rectangle
        let bbox1 = difference1.bounding_box();
        let original_bbox = rect.bounding_box();
        assert!(
            (bbox1.mins.x - original_bbox.mins.x).abs() < 1e-8,
            "Rectangle minus empty should preserve bounds"
        );

        // Empty minus rectangle should be empty
        let bbox2 = difference2.bounding_box();
        assert!(
            (bbox2.mins.x - bbox2.maxs.x).abs() < 1e-10,
            "Empty minus rectangle should be degenerate"
        );
    }

    #[test]
    fn test_cag_intersection_edge_case_empty_sketches() {
        // **Mathematical Foundation**: Intersection with empty geometries
        // **SRS Requirement NFR007**: Edge case handling (empty mesh operations)

        let empty = Sketch::<()>::new();
        let rect = Sketch::<()>::rectangle(2.0, 2.0, None);

        let intersection1 = rect.intersection(&empty);
        let intersection2 = empty.intersection(&rect);

        // Both intersections with empty should be empty
        let bbox1 = intersection1.bounding_box();
        let bbox2 = intersection2.bounding_box();

        assert!(
            (bbox1.mins.x - bbox1.maxs.x).abs() < 1e-10,
            "Intersection with empty should be degenerate"
        );
        assert!(
            (bbox2.mins.x - bbox2.maxs.x).abs() < 1e-10,
            "Intersection with empty should be degenerate"
        );
    }

    #[test]
    fn test_cag_xor_edge_case_empty_sketches() {
        // **Mathematical Foundation**: XOR with empty geometries
        // **SRS Requirement NFR007**: Edge case handling (empty mesh operations)

        let empty = Sketch::<()>::new();
        let rect = Sketch::<()>::rectangle(2.0, 2.0, None);

        let xor1 = rect.xor(&empty);
        let xor2 = empty.xor(&rect);

        // Rectangle XOR empty should equal rectangle
        let bbox1 = xor1.bounding_box();
        let original_bbox = rect.bounding_box();
        assert!(
            (bbox1.mins.x - original_bbox.mins.x).abs() < 1e-8,
            "Rectangle XOR empty should preserve bounds"
        );

        // Empty XOR rectangle should equal rectangle
        let bbox2 = xor2.bounding_box();
        assert!(
            (bbox2.mins.x - original_bbox.mins.x).abs() < 1e-8,
            "Empty XOR rectangle should equal rectangle"
        );
    }

    #[test]
    fn test_cag_operations_mathematical_properties() {
        // **Mathematical Foundation**: Boolean operation algebraic properties
        // **SRS Requirement NFR006**: Topological correctness

        let a = Sketch::<()>::rectangle(2.0, 2.0, None);
        let b = Sketch::<()>::rectangle(2.0, 2.0, None).translate(1.0, 0.0, 0.0);

        // Test commutativity: A ∪ B = B ∪ A
        let union_ab = a.union(&b);
        let union_ba = b.union(&a);

        let bbox_ab = union_ab.bounding_box();
        let bbox_ba = union_ba.bounding_box();

        assert!(
            (bbox_ab.mins.x - bbox_ba.mins.x).abs() < 1e-8,
            "Union should be commutative"
        );
        assert!(
            (bbox_ab.maxs.x - bbox_ba.maxs.x).abs() < 1e-8,
            "Union should be commutative"
        );

        // Test associativity: (A ∪ B) ∪ C = A ∪ (B ∪ C)
        let c = Sketch::<()>::rectangle(2.0, 2.0, None).translate(0.5, 1.0, 0.0);
        let union_abc_left = a.union(&b).union(&c);
        let union_abc_right = a.union(&b.union(&c));

        let bbox_left = union_abc_left.bounding_box();
        let bbox_right = union_abc_right.bounding_box();

        assert!(
            (bbox_left.mins.x - bbox_right.mins.x).abs() < 1e-8,
            "Union should be associative"
        );
        assert!(
            (bbox_left.maxs.x - bbox_right.maxs.x).abs() < 1e-8,
            "Union should be associative"
        );
    }

    #[test]
    fn test_cag_difference_absorption() {
        // **Mathematical Foundation**: Absorption law for difference
        // **SRS Requirement NFR006**: Topological correctness

        let a = Sketch::<()>::rectangle(3.0, 3.0, None);
        let b = Sketch::<()>::rectangle(2.0, 2.0, None);

        // A - (A - B) = A ∩ B
        let a_minus_b = a.difference(&b);
        let absorption = a.difference(&a_minus_b);
        let intersection = a.intersection(&b);

        let bbox_absorption = absorption.bounding_box();
        let bbox_intersection = intersection.bounding_box();

        // Both should have similar bounds (the overlapping region)
        assert!(
            (bbox_absorption.mins.x - bbox_intersection.mins.x).abs() < 0.1,
            "Absorption law should hold"
        );
        assert!(
            (bbox_absorption.maxs.x - bbox_intersection.maxs.x).abs() < 0.1,
            "Absorption law should hold"
        );
    }

    #[test]
    fn test_cag_operations_with_metadata() {
        // **Mathematical Foundation**: Metadata preservation through operations
        // **SRS Requirement NFR008**: Generic metadata system with compile-time guarantees

        #[derive(Clone, Debug, PartialEq)]
        struct TestMetadata {
            id: u32,
            name: String,
        }

        let rect1 = Sketch::rectangle(
            2.0,
            2.0,
            Some(TestMetadata {
                id: 1,
                name: "rect1".to_string(),
            }),
        );

        let rect2 = Sketch::rectangle(
            2.0,
            2.0,
            Some(TestMetadata {
                id: 2,
                name: "rect2".to_string(),
            }),
        );

        let union = rect1.union(&rect2);
        let difference = rect1.difference(&rect2);
        let intersection = rect1.intersection(&rect2);
        let xor = rect1.xor(&rect2);

        // All operations should preserve metadata from first operand
        assert!(union.metadata.is_some(), "Union should preserve metadata");
        assert!(
            difference.metadata.is_some(),
            "Difference should preserve metadata"
        );
        assert!(
            intersection.metadata.is_some(),
            "Intersection should preserve metadata"
        );
        assert!(xor.metadata.is_some(), "XOR should preserve metadata");

        let metadata = union.metadata.as_ref().unwrap();
        assert_eq!(
            metadata.id, 1,
            "Union should preserve first operand's metadata"
        );
        assert_eq!(
            metadata.name, "rect1",
            "Union should preserve first operand's metadata"
        );
    }

    #[test]
    fn test_cag_operations_numerical_stability() {
        // **Mathematical Foundation**: Numerical stability under floating-point operations
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        let rect1 = Sketch::<()>::rectangle(1.0, 1.0, None);
        let rect2 = Sketch::<()>::rectangle(1.0, 1.0, None).translate(0.5, 0.5, 0.0);

        let intersection = rect1.intersection(&rect2);

        // The intersection should be a 0.5x0.5 square
        let bbox = intersection.bounding_box();
        let width = bbox.maxs.x - bbox.mins.x;
        let height = bbox.maxs.y - bbox.mins.y;

        // Allow for small numerical errors in geometric computations
        let epsilon = 1e-3;
        assert!(
            (width - 0.5).abs() < epsilon,
            "Intersection width should be 0.5, got {}",
            width
        );
        assert!(
            (height - 0.5).abs() < epsilon,
            "Intersection height should be 0.5, got {}",
            height
        );
    }

    #[test]
    fn test_cag_transform_translation() {
        // **Mathematical Foundation**: Affine translation transformation
        // **SRS Requirement FR011**: Affine transformations (translation, rotation, scaling)

        let original = Sketch::<()>::circle(1.0, 16, None);
        let translated = original.translate(3.0, 4.0, 0.0);

        let original_bbox = original.bounding_box();
        let translated_bbox = translated.bounding_box();

        // Translation should shift the bounding box by the translation vector
        let dx = translated_bbox.mins.x - original_bbox.mins.x;
        let dy = translated_bbox.mins.y - original_bbox.mins.y;

        assert!(
            (dx - 3.0).abs() < 1e-8,
            "X translation should be 3.0, got {}",
            dx
        );
        assert!(
            (dy - 4.0).abs() < 1e-8,
            "Y translation should be 4.0, got {}",
            dy
        );

        // Z should remain zero
        assert!((translated_bbox.mins.z).abs() < 1e-10, "Z should remain 0");
    }

    #[test]
    fn test_cag_transform_scaling() {
        // **Mathematical Foundation**: Affine scaling transformation
        // **SRS Requirement FR011**: Affine transformations with composition

        use nalgebra::Matrix4;
        let original = Sketch::<()>::rectangle(2.0, 3.0, None);

        // Scale by factor of 2 in X and 0.5 in Y
        let scale_matrix = Matrix4::new_nonuniform_scaling(&Vector3::new(2.0, 0.5, 1.0));
        let scaled = original.transform(&scale_matrix);

        let original_bbox = original.bounding_box();
        let scaled_bbox = scaled.bounding_box();

        // Scaling should multiply the dimensions
        let original_width = original_bbox.maxs.x - original_bbox.mins.x;
        let original_height = original_bbox.maxs.y - original_bbox.mins.y;
        let scaled_width = scaled_bbox.maxs.x - scaled_bbox.mins.x;
        let scaled_height = scaled_bbox.maxs.y - scaled_bbox.mins.y;

        assert!(
            (scaled_width - original_width * 2.0).abs() < 1e-8,
            "Width should be scaled by 2.0"
        );
        assert!(
            (scaled_height - original_height * 0.5).abs() < 1e-8,
            "Height should be scaled by 0.5"
        );
    }

    #[test]
    fn test_cag_inverse_operation() {
        // **Mathematical Foundation**: Region inversion (inside/outside orientation flip)
        // **SRS Requirement FR011**: Orientation transformations

        let rect = Sketch::<()>::rectangle(2.0, 2.0, None);
        let inverted = rect.inverse();

        // Inverted region should have same bounding box but reversed orientation
        let original_bbox = rect.bounding_box();
        let inverted_bbox = inverted.bounding_box();

        let original_width = original_bbox.maxs.x - original_bbox.mins.x;
        let original_height = original_bbox.maxs.y - original_bbox.mins.y;
        let inverted_width = inverted_bbox.maxs.x - inverted_bbox.mins.x;
        let inverted_height = inverted_bbox.maxs.y - inverted_bbox.mins.y;

        assert!(
            (original_width - inverted_width).abs() < 1e-8,
            "Inverted width should match original"
        );
        assert!(
            (original_height - inverted_height).abs() < 1e-8,
            "Inverted height should match original"
        );
    }

    #[test]
    fn test_cag_bounding_box_caching() {
        // **Mathematical Foundation**: Lazy bounding box computation
        // **SRS Requirement NFR003**: Efficient connectivity queries with caching

        let mut sketch = Sketch::<()>::rectangle(2.0, 2.0, None);

        // First call should compute bounding box
        let bbox1 = sketch.bounding_box();

        // Second call should return cached result
        let bbox2 = sketch.bounding_box();

        // Results should be identical
        assert!(
            (bbox1.mins.x - bbox2.mins.x).abs() < 1e-15,
            "Cached bbox should match computed bbox"
        );
        assert!(
            (bbox1.mins.y - bbox2.mins.y).abs() < 1e-15,
            "Cached bbox should match computed bbox"
        );
        assert!(
            (bbox1.maxs.x - bbox2.maxs.x).abs() < 1e-15,
            "Cached bbox should match computed bbox"
        );
        assert!(
            (bbox1.maxs.y - bbox2.maxs.y).abs() < 1e-15,
            "Cached bbox should match computed bbox"
        );

        // Invalidation should clear cache
        sketch.invalidate_bounding_box();
        let bbox3 = sketch.bounding_box();

        // Should still be the same mathematically
        assert!(
            (bbox1.mins.x - bbox3.mins.x).abs() < 1e-15,
            "Invalidated bbox should match original"
        );
    }

    #[test]
    fn test_cag_precision_boundary_cases() {
        // **Mathematical Foundation**: Precision boundary testing
        // **SRS Requirement NFR005**: Geometric precision with configurable epsilon

        // Test with very small geometries
        let tiny_rect = Sketch::<()>::rectangle(1e-6, 1e-6, None);
        let bbox = tiny_rect.bounding_box();
        assert!(
            bbox.maxs.x > bbox.mins.x,
            "Tiny rectangle should have valid bounds"
        );

        // Test with very large geometries
        let large_rect = Sketch::<()>::rectangle(1e6, 1e6, None);
        let bbox = large_rect.bounding_box();
        assert!(
            bbox.maxs.x > bbox.mins.x,
            "Large rectangle should have valid bounds"
        );

        // Test precision near machine epsilon
        let epsilon_rect = Sketch::<()>::rectangle(1e-15, 1e-15, None);
        let bbox = epsilon_rect.bounding_box();
        // Should handle gracefully even if degenerate
        let width = bbox.maxs.x - bbox.mins.x;
        assert!(
            width >= 0.0,
            "Width should be non-negative even for tiny geometries"
        );
    }
}
