use super::CSG;
use crate::core::float_types::parry3d::bounding_volume::{Aabb, BoundingVolume};
use crate::spatial::bsp::Node;
// TODO: Re-enable when spatial integration is complete
// use crate::spatial::{SpatialStructureFactory, QueryType, SpatialIndex};
use crate::geometry::Polygon;
use geo::{BooleanOps, Geometry, GeometryCollection, Orient, orient::Direction};
use std::fmt::Debug;
use std::sync::OnceLock;

/// Enum for coplanar polygon boolean operations
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
enum CoplanarOperation {
    Difference,
    Union,
    Intersection,
    Xor,
}

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Split polygons into (may_touch, cannot_touch) using bounding‑box tests
    fn partition_polys(
        polys: &[Polygon<S>],
        other_bb: &Aabb,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        polys
            .iter()
            .cloned()
            .partition(|p| p.bounding_box().intersects(other_bb))
    }

    /// Check if two polygons are coplanar within tolerance
    #[allow(dead_code)]
    fn are_coplanar(poly1: &Polygon<S>, poly2: &Polygon<S>) -> bool {
        use crate::core::float_types::EPSILON;

        // Check if planes are parallel (normals are parallel)
        let n1 = poly1.plane.normal();
        let n2 = poly2.plane.normal();
        let cross = n1.cross(&n2);

        // If cross product is near zero, normals are parallel
        // Use much tighter tolerance to avoid false positives
        if cross.norm() > EPSILON {
            return false;
        }

        // Check if planes are at the same distance from origin
        let d1 = poly1.plane.offset();
        let d2 = poly2.plane.offset();

        // Account for opposite normal directions
        let distance_diff = if n1.dot(&n2) > 0.0 {
            (d1 - d2).abs()
        } else {
            (d1 + d2).abs()
        };

        // Use much tighter tolerance for distance
        distance_diff < EPSILON
    }

    /// Check if two coplanar polygons actually overlap in 2D space
    #[allow(dead_code)]
    fn polygons_overlap_2d(poly1: &Polygon<S>, poly2: &Polygon<S>) -> bool {
        use crate::core::float_types::EPSILON;

        // Quick bounding box check first
        let bb1 = poly1.bounding_box();
        let bb2 = poly2.bounding_box();

        // If bounding boxes don't overlap, polygons don't overlap
        if !bb1.intersects(&bb2) {
            return false;
        }

        // Check if bounding boxes have substantial overlap (not just touching)
        let overlap_x = (bb1.maxs.x.min(bb2.maxs.x) - bb1.mins.x.max(bb2.mins.x)).max(0.0);
        let overlap_y = (bb1.maxs.y.min(bb2.maxs.y) - bb1.mins.y.max(bb2.mins.y)).max(0.0);
        let overlap_z = (bb1.maxs.z.min(bb2.maxs.z) - bb1.mins.z.max(bb2.mins.z)).max(0.0);

        // For 2D polygons (like squares), one dimension might be very small (near zero)
        // Count how many dimensions have substantial overlap
        let substantial_overlaps = [overlap_x, overlap_y, overlap_z]
            .iter()
            .filter(|&&overlap| overlap > EPSILON * 10.0)
            .count();

        // For true 2D overlapping polygons, we expect at least 2 dimensions to overlap substantially
        // This filters out adjacent 3D faces that only touch at edges or share a single dimension
        substantial_overlaps >= 2
    }

    /// Handle boolean operations between coplanar polygons using 2D geometry
    #[allow(dead_code)]
    fn handle_coplanar_boolean_operation(
        polys_a: &[Polygon<S>],
        polys_b: &[Polygon<S>],
        operation: CoplanarOperation,
    ) -> Vec<Polygon<S>> {
        use geo::{BooleanOps, MultiPolygon, Polygon as GeoPolygon};
        use geo::LineString;
        use geo::Coord;

        if polys_a.is_empty() || polys_b.is_empty() {
            return match operation {
                CoplanarOperation::Difference => polys_a.to_vec(),
                CoplanarOperation::Union => {
                    let mut result = polys_a.to_vec();
                    result.extend(polys_b.iter().cloned());
                    result
                },
                CoplanarOperation::Intersection => Vec::new(),
                CoplanarOperation::Xor => {
                    let mut result = polys_a.to_vec();
                    result.extend(polys_b.iter().cloned());
                    result
                },
            };
        }

        // Use the first polygon to establish the plane
        let reference_plane = &polys_a[0].plane;
        let normal = reference_plane.normal();

        // Create orthonormal basis for 2D projection
        let (u, v) = crate::geometry::polygon::build_orthonormal_basis(&normal);
        let origin = polys_a[0].vertices[0].pos;

        // Convert 3D polygons to 2D for boolean operations
        let convert_to_2d = |polys: &[Polygon<S>]| -> MultiPolygon<crate::core::float_types::Real> {
            let mut geo_polygons = Vec::new();

            for poly in polys {
                let mut coords_2d = Vec::new();
                for vertex in &poly.vertices {
                    let offset = vertex.pos.coords - origin.coords;
                    let x = offset.dot(&u);
                    let y = offset.dot(&v);
                    coords_2d.push(Coord { x, y });
                }

                // Close the polygon if not already closed
                if let (Some(first), Some(last)) = (coords_2d.first(), coords_2d.last()) {
                    if (first.x - last.x).abs() > crate::core::float_types::EPSILON ||
                       (first.y - last.y).abs() > crate::core::float_types::EPSILON {
                        coords_2d.push(*first);
                    }
                }

                if coords_2d.len() >= 4 { // Need at least 3 unique points + closure
                    let line_string = LineString::new(coords_2d);
                    let geo_poly = GeoPolygon::new(line_string, vec![]);
                    geo_polygons.push(geo_poly);
                }
            }

            MultiPolygon::new(geo_polygons)
        };

        let multi_a = convert_to_2d(polys_a);
        let multi_b = convert_to_2d(polys_b);

        // Perform 2D boolean operation
        let result_2d = match operation {
            CoplanarOperation::Difference => multi_a.difference(&multi_b),
            CoplanarOperation::Union => multi_a.union(&multi_b),
            CoplanarOperation::Intersection => multi_a.intersection(&multi_b),
            CoplanarOperation::Xor => {
                // XOR = (A - B) ∪ (B - A)
                let a_minus_b = multi_a.difference(&multi_b);
                let b_minus_a = multi_b.difference(&multi_a);
                a_minus_b.union(&b_minus_a)
            },
        };

        // Convert back to 3D polygons
        let mut result_3d = Vec::new();
        for geo_poly in result_2d.iter() {
            let exterior = geo_poly.exterior();
            if exterior.coords().count() >= 4 { // At least 3 unique points + closure
                let mut vertices_3d = Vec::new();

                for coord in exterior.coords().take(exterior.coords().count() - 1) { // Skip closure
                    let pos_3d = origin.coords + coord.x * u + coord.y * v;
                    let vertex = crate::geometry::Vertex::new(
                        nalgebra::Point3::from(pos_3d),
                        normal,
                    );
                    vertices_3d.push(vertex);
                }

                if vertices_3d.len() >= 3 {
                    // Use metadata from the first input polygon
                    let metadata = polys_a.first().and_then(|p| p.metadata.clone());
                    result_3d.push(Polygon::new(vertices_3d, metadata));
                }
            }
        }

        result_3d
    }

    /// Future enhancement: Create optimal spatial structure for CSG operations
    /// This will provide transparent spatial acceleration while maintaining API compatibility
    /// For now, this is a placeholder that returns the original polygons
    fn _prepare_polygons_for_optimization(polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // TODO: Integrate SpatialStructureFactory::create_optimal() when lifetime constraints are resolved
        // This will enable automatic selection of optimal spatial structures:
        // - BSP trees for boolean operations
        // - BVH for ray tracing
        // - KD-trees for point location
        // - R-trees for range queries
        polygons.to_vec()
    }

    /// Return a new CSG representing union of the two CSG's.
    ///
    /// ```no_run
    /// # use csgrs::CSG;
    /// # fn main() {
    /// # let a: CSG<()> = CSG::new();
    /// # let b: CSG<()> = CSG::new();
    /// let c = a.union(&b);
    /// # }
    /// //    +-------+            +-------+
    /// //    |       |            |       |
    /// //    |   a   |            |   c   |
    /// //    |    +--+----+   =   |       +----+
    /// //    +----+--+    |       +----+       |
    /// //         |   b   |            |   c   |
    /// //         |       |            |       |
    /// //         +-------+            +-------+
    /// ```
    #[must_use = "Use new CSG representing space in both CSG's"]
    pub fn union(&self, other: &CSG<S>) -> CSG<S> {
        // 3D union:
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // TODO: Future enhancement - use intelligent spatial structure selection
        // let optimized_a = Self::_prepare_polygons_for_optimization(&a_clip);
        // let optimized_b = Self::_prepare_polygons_for_optimization(&b_clip);

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

        // 2D union:
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

        CSG {
            polygons: final_polys,
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing diffarence of the two CSG's.
    ///
    /// ```no_run
    /// # use csgrs::CSG;
    /// # fn main() {
    /// # let a: CSG<()> = CSG::new();
    /// # let b: CSG<()> = CSG::new();
    /// let c = a.difference(&b);
    /// # }
    /// //    +-------+            +-------+
    /// //    |       |            |       |
    /// //    |   a   |            |   c   |
    /// //    |    +--+----+   =   |    +--+
    /// //    +----+--+    |       +----+
    /// //         |   b   |
    /// //         |       |
    /// //         +-------+
    /// ```
    #[must_use = "Use new CSG"]
    pub fn difference(&self, other: &CSG<S>) -> CSG<S> {
        // 3D difference:
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // DISABLE coplanar polygon handling for difference operations for now
        // The coplanar handling is causing issues with vertex containment tests
        // TODO: Re-enable with better detection criteria
        let coplanar_result: Vec<Polygon<S>> = Vec::new();
        let non_coplanar_a = a_clip;
        let non_coplanar_b = b_clip;

        // TODO: Future enhancement - use intelligent spatial structure selection
        // let optimized_a = Self::_prepare_polygons_for_optimization(&non_coplanar_a);
        // let optimized_b = Self::_prepare_polygons_for_optimization(&non_coplanar_b);

        let mut a = Node::from_polygons(&non_coplanar_a);
        let mut b = Node::from_polygons(&non_coplanar_b);

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
        final_polys.extend(coplanar_result);

        // 2D difference:
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

        CSG {
            polygons: final_polys,
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```no_run
    /// # use csgrs::CSG;
    /// # fn main() {
    /// # let a: CSG<()> = CSG::new();
    /// # let b: CSG<()> = CSG::new();
    /// let c = a.intersection(&b);
    /// # }
    /// //    +-------+
    /// //    |       |
    /// //    |   a   |
    /// //    |    +--+----+   =   +--+
    /// //    +----+--+    |       +--+
    /// //         |   b   |
    /// //         |       |
    /// //         +-------+
    /// ```
    pub fn intersection(&self, other: &CSG<S>) -> CSG<S> {
        // 3D intersection:
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, _a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // TODO: Future enhancement - use intelligent spatial structure selection
        // let optimized_a = Self::_prepare_polygons_for_optimization(&a_clip);
        // let optimized_b = Self::_prepare_polygons_for_optimization(&b_clip);

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        // 2D intersection:
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

        CSG {
            polygons: a.all_polygons(),
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```no_run
    /// # use csgrs::CSG;
    /// # fn main() {
    /// # let a: CSG<()> = CSG::new();
    /// # let b: CSG<()> = CSG::new();
    /// let c = a.xor(&b);
    /// # }
    /// //    +-------+            +-------+
    /// //    |       |            |       |
    /// //    |   a   |            |   a   |
    /// //    |    +--+----+   =   |    +--+----+
    /// //    +----+--+    |       +----+--+    |
    /// //         |   b   |            |       |
    /// //         |       |            |       |
    /// //         +-------+            +-------+
    /// ```
    pub fn xor(&self, other: &CSG<S>) -> CSG<S> {
        // Use traditional BSP approach for XOR
        // The coplanar polygon handling is too complex and causes issues with 3D geometry
        // For now, stick with the proven BSP tree approach

        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)
    }
}
