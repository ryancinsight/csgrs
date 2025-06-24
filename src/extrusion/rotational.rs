use crate::csg::CSG;
use crate::core::float_types::{EPSILON, Real};
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use geo::{Area, LineString, GeometryCollection};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Rotate-extrude (revolve) only the Polygon and MultiPolygon geometry in `self.geometry`
    /// around the Y-axis from 0..`angle_degs` in `segments` steps, producing side walls
    /// in an orientation consistent with the polygon's winding.
    ///
    /// - Ignores `self.polygons`.
    /// - Returns a new CSG containing **only** the newly extruded side polygons (no end caps).
    /// - `angle_degs`: how far to revolve, in degrees (e.g. 360 for a full revolve).
    /// - `segments`: number of subdivisions around the revolve.
    ///
    /// # Key Points
    /// - Only 2D geometry in `self.geometry` is used. Any `self.polygons` are ignored.
    /// - Axis of revolution: **Y-axis**. We treat each ring's (x,y) -> revolve_around_y(x,y,theta).
    /// - Exterior rings (CCW in Geo) produce outward-facing side polygons.
    /// - Interior rings (CW) produce inward-facing side polygons ("holes").
    /// - If `angle_degs < 360`, we add **two caps**: one at angle=0, one at angle=angle_degs.
    ///   - Cap orientation is set so that normals face outward, consistent with a solid.
    /// - Returns a new CSG with `.polygons` containing only the side walls + any caps.
    ///   The `.geometry` is empty, i.e. `GeometryCollection::default()`.
    pub fn rotate_extrude(&self, angle_degs: Real, segments: usize) -> CSG<S> {
        if segments < 2 {
            panic!("rotate_extrude requires at least 2 segments.");
        }

        let angle_radians = angle_degs.to_radians();
        let mut new_polygons = Vec::new();

        // Pre-calculate sin/cos for each segment angle
        let step = angle_radians / (segments as Real);
        let trig_table: Vec<_> = (0..=segments)
            .map(|s| {
                let theta = s as Real * step;
                (theta.sin(), theta.cos())
            })
            .collect();

        //----------------------------------------------------------------------
        // 2) Iterate over each geometry (Polygon or MultiPolygon),
        //    revolve the side walls, and possibly add caps if angle_degs < 360.
        //----------------------------------------------------------------------
        let full_revolve = (angle_degs - 360.0).abs() < EPSILON; // or angle_degs >= 359.999..., etc.
        let do_caps = !full_revolve && (angle_degs > 0.0);

        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    revolve_and_cap_polygon(
                        poly2d,
                        do_caps,
                        angle_radians,
                        &trig_table,
                        &self.metadata,
                        &mut new_polygons,
                    );
                }

                geo::Geometry::MultiPolygon(mpoly) => {
                    // Each Polygon inside
                    for poly2d in &mpoly.0 {
                        revolve_and_cap_polygon(
                            poly2d,
                            do_caps,
                            angle_radians,
                            &trig_table,
                            &self.metadata,
                            &mut new_polygons,
                        );
                    }
                }

                // Ignore lines, points, etc.
                _ => {}
            }
        }

        //----------------------------------------------------------------------
        // 3) Return the new CSG:
        //----------------------------------------------------------------------
        CSG {
            polygons: new_polygons,
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

/// Helper to process a single 2D polygon: revolve its exterior and interior rings,
/// and add start/end caps if it's a partial revolve.
fn revolve_and_cap_polygon<S: Clone + Debug + Send + Sync>(
    poly2d: &geo::Polygon<Real>,
    do_caps: bool,
    angle_radians: Real,
    trig_table: &[(Real, Real)],
    metadata: &Option<S>,
    out_polygons: &mut Vec<Polygon<S>>,
) {
    // Exterior ring
    let ext_ring = poly2d.exterior();
    let ext_ccw = is_ccw(ext_ring);

    // (A) side walls
    revolve_ring(
        &ext_ring.0,
        ext_ccw,
        trig_table,
        metadata,
        out_polygons,
    );

    // (B) cap(s) if partial revolve
    if do_caps {
        // start-cap at angle=0
        //   flip if ext_ccw == true
        if let Some(cap) = build_cap_polygon(
            &ext_ring.0,
            0.0,
            ext_ccw, // exterior ring => flip the start cap
            metadata,
        ) {
            out_polygons.push(cap);
        }

        // end-cap at angle= angle_radians
        //   flip if ext_ccw == false
        if let Some(cap) = build_cap_polygon(
            &ext_ring.0,
            angle_radians,
            !ext_ccw, // exterior ring => keep normal orientation for end
            metadata,
        ) {
            out_polygons.push(cap);
        }
    }

    // Interior rings (holes)
    for hole in poly2d.interiors() {
        let hole_ccw = is_ccw(hole);
        revolve_ring(
            &hole.0,
            hole_ccw,
            trig_table,
            metadata,
            out_polygons,
        );
    }
}

/// A small helper to revolve a point (x,y) in the XY plane around the Y-axis by theta.
/// The output is a 3D point (X, Y, Z).
fn revolve_around_y(x: Real, y: Real, sin_t: Real, cos_t: Real) -> Point3<Real> {
    // Map (x, y, 0) => ( x*cos θ, y, x*sin θ )
    Point3::new(x * cos_t, y, x * sin_t)
}

/// Another helper to determine if a ring (LineString) is CCW or CW in Geo.
/// In `geo`, ring.exterior() is CCW for an outer boundary, CW for holes.
/// If the signed area > 0 => CCW; < 0 => CW.
fn is_ccw(ring: &LineString<Real>) -> bool {
    ring.signed_area() > 0.0
}

/// A helper to extrude one ring of coordinates (including the last->first if needed),
/// pushing its side polygons into `out_polygons`.
/// - `ring_coords`: The ring's sequence of points. Usually closed (last=first).
/// - `ring_is_ccw`: true if it's an exterior ring, false if interior/hole.
/// - `trig_table`: precomputed (sin, cos) values for each segment.
/// - `metadata`: user metadata to attach to side polygons.
fn revolve_ring<S: Clone + Send + Sync>(
    ring_coords: &[geo::Coord<Real>],
    ring_is_ccw: bool,
    trig_table: &[(Real, Real)],
    metadata: &Option<S>,
    out_polygons: &mut Vec<Polygon<S>>,
) {
    if ring_coords.len() < 2 {
        return;
    }

    let segments = trig_table.len() - 1;

    // For each edge in the ring:
    for edge in ring_coords.windows(2) {
        let c_i = edge[0];
        let c_j = edge[1];

        // If these two points are the same, skip degenerate edge
        if (c_i.x - c_j.x).abs() < EPSILON && (c_i.y - c_j.y).abs() < EPSILON {
            continue;
        }

        // For each revolve slice j..j+1
        for s in 0..segments {
            let (sin0, cos0) = trig_table[s];
            let (sin1, cos1) = trig_table[s + 1];

            // revolve bottom edge endpoints at angle th0
            let b_i = revolve_around_y(c_i.x, c_i.y, sin0, cos0);
            let b_j = revolve_around_y(c_j.x, c_j.y, sin0, cos0);
            // revolve top edge endpoints at angle th1
            let t_i = revolve_around_y(c_i.x, c_i.y, sin1, cos1);
            let t_j = revolve_around_y(c_j.x, c_j.y, sin1, cos1);

            // Build a 4-vertex side polygon for the ring edge.
            // The orientation depends on ring_is_ccw:
            //    If CCW => outward walls -> [b_i, b_j, t_j, t_i]
            //    If CW  => reverse it -> [b_j, b_i, t_i, t_j]
            let quad_verts = if ring_is_ccw {
                [b_i, b_j, t_j, t_i]
            } else {
                [b_j, b_i, t_i, t_j]
            };

            out_polygons.push(Polygon::new(
                quad_verts
                    .iter()
                    .map(|&pos| Vertex::new(pos, Vector3::zeros()))
                    .collect(),
                metadata.clone(),
            ));
        }
    }
}

/// Build a single "cap" polygon from ring_coords at a given angle (0 or angle_radians).
///  - revolve each 2D point by `angle`, produce a 3D ring
///  - if `flip` is true, reverse the ring so the normal is inverted
fn build_cap_polygon<S: Clone + Send + Sync>(
    ring_coords: &[geo::Coord<Real>],
    angle: Real,
    flip: bool,
    metadata: &Option<S>,
) -> Option<Polygon<S>> {
    if ring_coords.len() < 3 {
        return None;
    }
    let (sin_a, cos_a) = (angle.sin(), angle.cos());
    // revolve each coordinate at the given angle
    let mut pts_3d: Vec<_> = ring_coords
        .iter()
        .map(|c| revolve_around_y(c.x, c.y, sin_a, cos_a))
        .collect();

    // ensure closed if the ring wasn't strictly closed
    // (the last point in a Geo ring is typically the same as the first)
    let last = pts_3d.last().unwrap();
    let first = pts_3d.first().unwrap();
    if (last.x - first.x).abs() > EPSILON
        || (last.y - first.y).abs() > EPSILON
        || (last.z - first.z).abs() > EPSILON
    {
        pts_3d.push(*first);
    }

    // Turn into Vertex
    let mut verts: Vec<_> = pts_3d
        .into_iter()
        .map(|p3| Vertex::new(p3, Vector3::zeros()))
        .collect();

    // If flip == true, reverse them and flip each vertex
    if flip {
        verts.reverse();
        for v in &mut verts {
            v.flip();
        }
    }

    // Build the polygon
    let poly = Polygon::new(verts, metadata.clone());
    Some(poly)
} 
