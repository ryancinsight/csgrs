//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::float_types::{
    Real,
    parry3d::bounding_volume::{Aabb, BoundingVolume},
};
use crate::mesh::{bsp::Node, plane::Plane, polygon::Polygon, vertex::Vertex};
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{CoordsIter, Geometry, Polygon as GeoPolygon};
use nalgebra::{Matrix4, Point3, Vector3, partial_max, partial_min};
use std::{cmp::PartialEq, fmt::Debug, sync::OnceLock};

pub mod bsp;
pub mod bsp_parallel;

#[cfg(feature = "chull")]
pub mod convex_hull;
pub mod flatten_slice;

#[cfg(feature = "metaballs")]
pub mod metaballs;
pub mod plane;
pub mod polygon;

pub mod analysis;
pub mod connectivity;
pub mod conversion;
pub mod manifold;
pub mod operations;
pub mod physics;
pub mod quality;
#[cfg(feature = "sdf")]
pub mod sdf;
pub mod shapes;
pub mod smoothing;
#[cfg(feature = "sdf")]
pub mod tpms;
pub mod vertex;

#[derive(Clone, Debug)]
pub struct Mesh<S: Clone + Send + Sync + Debug> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug + PartialEq> Mesh<S> {
    /// Compare just the `metadata` fields of two meshes
    #[inline]
    pub fn same_metadata(&self, other: &Self) -> bool {
        self.metadata == other.metadata
    }

    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &S) -> Mesh<S> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| p.metadata.as_ref() == Some(needle))
            .cloned()
            .collect();

        Mesh {
            polygons: polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Split polygons into (may_touch, cannot_touch) using bounding-box tests
    fn partition_polys(
        polys: &[Polygon<S>],
        other_bb: &Aabb,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        polys
            .iter()
            .cloned()
            .partition(|p| p.bounding_box().intersects(other_bb))
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for Mesh<S> {
    /// Returns a new empty Mesh
    fn new() -> Self {
        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Return a new Mesh representing union of the two Meshes.
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
    fn union(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);
        final_polys.extend(b_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
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
    fn difference(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // propagate self.metadata to new polygons by overwriting intersecting
        // polygon.metadata in other.
        let b_clip_retagged: Vec<Polygon<S>> = b_clip
            .iter()
            .map(|poly| {
                // Avoid full polygon clone by creating new polygon with same geometry but different metadata
                Polygon::new(poly.vertices.clone(), self.metadata.clone())
            })
            .collect();

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip_retagged);

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing intersection of the two CSG's.
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
    fn intersection(&self, other: &Mesh<S>) -> Mesh<S> {
        let mut a = Node::from_polygons(&self.polygons);
        let mut b = Node::from_polygons(&other.polygons);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        Mesh {
            polygons: a.all_polygons(),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
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
    fn xor(&self, other: &Mesh<S>) -> Mesh<S> {
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to Mesh.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D
    fn transform(&self, mat: &Matrix4<Real>) -> Mesh<S> {
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match mat.try_inverse() {
            Some(inv) => inv.transpose(),
            None => {
                eprintln!(
                    "Warning: Transformation matrix is not invertible, using identity for normals"
                );
                Matrix4::identity()
            },
        };

        let mut mesh = self.clone();

        for poly in &mut mesh.polygons {
            for vert in &mut poly.vertices {
                // Transform position using homogeneous coordinates
                let hom_pos = mat * vert.pos.to_homogeneous();
                match Point3::from_homogeneous(hom_pos) {
                    Some(transformed_pos) => vert.pos = transformed_pos,
                    None => {
                        eprintln!(
                            "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                        );
                        continue;
                    },
                }

                // Transform normal using inverse transpose rule
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }

            // Reconstruct plane from transformed vertices for consistency
            poly.plane = Plane::from_vertices(poly.vertices.clone());

            // Invalidate the polygon's bounding box
            poly.bounding_box = OnceLock::new();
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();

        mesh
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] indicating the 3D bounds of all `polygons`.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for v in &poly.vertices {
                    // Handle potential NaN values gracefully
                    if let Some(new_min_x) = partial_min(&min_x, &v.pos.x) {
                        min_x = *new_min_x;
                    }
                    if let Some(new_min_y) = partial_min(&min_y, &v.pos.y) {
                        min_y = *new_min_y;
                    }
                    if let Some(new_min_z) = partial_min(&min_z, &v.pos.z) {
                        min_z = *new_min_z;
                    }

                    if let Some(new_max_x) = partial_max(&max_x, &v.pos.x) {
                        max_x = *new_max_x;
                    }
                    if let Some(new_max_y) = partial_max(&max_y, &v.pos.y) {
                        max_y = *new_max_y;
                    }
                    if let Some(new_max_z) = partial_max(&max_z, &v.pos.z) {
                        max_z = *new_max_z;
                    }
                }
            }

            // If still uninitialized (e.g., no polygons), return a trivial AABB at origin
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

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<S> {
        // Create new mesh with flipped polygons to avoid full mesh clone
        let flipped_polygons = self
            .polygons
            .iter()
            .map(|poly| {
                let mut flipped = poly.clone();
                flipped.flip();
                flipped
            })
            .collect();

        Mesh {
            polygons: flipped_polygons,
            bounding_box: OnceLock::new(), // Invalidate cached bounding box
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> From<Sketch<S>> for Mesh<S> {
    /// Convert a Sketch into a Mesh.
    fn from(sketch: Sketch<S>) -> Self {
        /// Helper function to convert a geo::Polygon to a Vec<crate::mesh::polygon::Polygon>
        fn geo_poly_to_csg_polys<S: Clone + Debug + Send + Sync>(
            poly2d: &GeoPolygon<Real>,
            metadata: &Option<S>,
        ) -> Vec<Polygon<S>> {
            let mut all_polygons = Vec::new();

            // Handle the exterior ring
            let outer_vertices_3d: Vec<_> = poly2d
                .exterior()
                .coords_iter()
                .map(|c| Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()))
                .collect();

            if outer_vertices_3d.len() >= 3 {
                all_polygons.push(Polygon::new(outer_vertices_3d, metadata.clone()));
            }

            // Handle interior rings (holes)
            for ring in poly2d.interiors() {
                let hole_vertices_3d: Vec<_> = ring
                    .coords_iter()
                    .map(|c| Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()))
                    .collect();
                if hole_vertices_3d.len() >= 3 {
                    all_polygons.push(Polygon::new(hole_vertices_3d, metadata.clone()));
                }
            }
            all_polygons
        }

        let final_polygons = sketch
            .geometry
            .iter()
            .flat_map(|geom| -> Vec<Polygon<S>> {
                match geom {
                    Geometry::Polygon(poly2d) => {
                        geo_poly_to_csg_polys(poly2d, &sketch.metadata)
                    },
                    Geometry::MultiPolygon(multipoly) => multipoly
                        .iter()
                        .flat_map(|poly2d| geo_poly_to_csg_polys(poly2d, &sketch.metadata))
                        .collect(),
                    _ => vec![],
                }
            })
            .collect();

        Mesh {
            polygons: final_polygons,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}
