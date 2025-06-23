use super::CSG;
use crate::csg::bsp::Node;
use crate::float_types::parry3d::bounding_volume::{Aabb, BoundingVolume};
use crate::polygon::Polygon;
use geo::{BooleanOps, Geometry, GeometryCollection, Orient, orient::Direction};
use std::fmt::Debug;
use std::sync::OnceLock;

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

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

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
                }
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                }
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

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

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
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
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
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

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
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
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
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)
    }
}
