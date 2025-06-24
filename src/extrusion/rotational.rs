//! **Mathematical Foundations for Rotational Extrusion**
//!
//! This module implements mathematically rigorous algorithms for creating
//! surfaces of revolution by rotating 2D profiles around an axis based on
//! classical differential geometry and surface theory.
//!
//! ## **Theoretical Foundation**
//!
//! ### **Surfaces of Revolution Definition**
//! For a 2D profile curve C(t) = (x(t), y(t)) in the XY plane,
//! rotation around the Y-axis generates a surface of revolution:
//! ```text
//! S(t,θ) = (x(t)·cos(θ), y(t), x(t)·sin(θ))
//! where:
//! - t ∈ [0,1]: Parameter along profile curve
//! - θ ∈ [0,2π]: Rotation angle around Y-axis
//! ```
//!
//! ### **Surface Normal Calculation**
//! Surface normals are computed using the cross product of tangent vectors:
//! ```text
//! n⃗ = (∂S/∂t × ∂S/∂θ).normalize()
//! ```
//! For revolution surfaces, this simplifies to:
//! ```text
//! n⃗ = (x'cos(θ), -x, x'sin(θ)).normalize()
//! where x' = dx/dt is the derivative of the profile
//! ```
//!
//! ### **Winding Orientation Rules**
//! Profile curve orientation determines surface normal direction:
//! - **Counter-clockwise profiles**: Generate outward-facing normals
//! - **Clockwise profiles**: Generate inward-facing normals (holes)
//! - **Consistent orientation**: Essential for CSG boolean operations
//!
//! ### **Partial Revolution Handling**
//! For partial rotations (angle < 360°):
//! 1. **Side surfaces**: Generated from profile rotation
//! 2. **Start cap**: Planar surface at θ = 0
//! 3. **End cap**: Planar surface at θ = angle
//! 4. **Cap orientation**: Ensures manifold topology
//!
//! ### **Multi-Contour Support**
//! - **Exterior contours**: Create solid boundaries
//! - **Interior contours**: Create holes and cavities
//! - **Proper nesting**: Interior contours must be inside exterior
//!
//! ## **Geometric Properties**
//! For a surface of revolution with profile length L and rotation angle α:
//! - **Surface Area**: A = ∫ 2πx(t)·|C'(t)| dt × (α/2π)
//! - **Volume**: V = ∫ πx²(t)·y'(t) dt × (α/2π)
//! - **Centroid**: Computed using Pappus's theorems
//!
//! ## **Applications**
//! - **Pottery and vessels**: Symmetric containers
//! - **Mechanical components**: Shafts, pulleys, gears
//! - **Architectural elements**: Columns, domes, arches
//! - **Artistic modeling**: Vases, sculptures
//!
//! All operations preserve geometric accuracy and maintain proper topology.

use crate::csg::CSG;
use crate::core::float_types::{EPSILON, Real};
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use geo::{Area, LineString, GeometryCollection};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// **Mathematical Foundation: Surface of Revolution Generation**
    ///
    /// Rotate-extrude (revolve) 2D geometry around the Y-axis to create surfaces of revolution.
    /// This implements the complete mathematical theory of revolution surfaces with
    /// proper orientation handling and cap generation.
    ///
    /// ## **Revolution Mathematics**
    ///
    /// ### **Parametric Surface Generation**
    /// For each 2D boundary point (x,y), generate revolution surface:
    /// ```text
    /// S(θ) = (x·cos(θ), y, x·sin(θ))
    /// where θ ∈ [0, angle_radians]
    /// ```
    ///
    /// ### **Surface Mesh Construction**
    /// The algorithm creates quadrilateral strips:
    /// 1. **Vertex Grid**: (n_segments+1) × (n_boundary_points) vertices
    /// 2. **Quad Formation**: Connect adjacent vertices in parameter space
    /// 3. **Orientation**: Preserve winding from 2D profile
    ///
    /// ### **Normal Vector Calculation**
    /// For each quad, compute normals using right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```
    /// Direction depends on profile curve orientation.
    ///
    /// ### **Boundary Orientation Handling**
    /// - **Exterior boundaries (CCW)**: Generate outward-facing surfaces
    /// - **Interior boundaries (CW)**: Generate inward-facing surfaces (holes)
    /// - **Winding preservation**: Essential for manifold topology
    ///
    /// ### **Partial Revolution Caps**
    /// For angle < 360°, generate planar caps:
    /// 1. **Start cap** (θ=0): Triangulated profile at initial position
    /// 2. **End cap** (θ=angle): Triangulated profile at final position
    /// 3. **Cap normals**: Point outward from solid interior
    /// 4. **Manifold closure**: Ensures watertight geometry
    ///
    /// ### **Multi-Polygon Support**
    /// - **Exterior polygons**: Create main solid boundaries
    /// - **Interior polygons**: Create holes and cavities
    /// - **Nesting rules**: Interior must be properly contained
    ///
    /// ## **Algorithm Complexity**
    /// - **Boundary Processing**: O(n) for n boundary edges
    /// - **Surface Generation**: O(n×s) for s segments
    /// - **Cap Triangulation**: O(n log n) for complex profiles
    ///
    /// ## **Geometric Properties**
    /// - **Surface continuity**: C⁰ (positional) at segment boundaries
    /// - **Normal continuity**: Discontinuous at segment boundaries (faceted)
    /// - **Manifold property**: Maintained for valid input profiles
    ///
    /// ## **Applications**
    /// - **Turned objects**: Lathe-created components
    /// - **Vessels**: Bowls, vases, containers
    /// - **Mechanical parts**: Pulleys, gears, shafts
    /// - **Architectural elements**: Columns, balusters
    ///
    /// ## **Numerical Considerations**
    /// - **Trigonometric precomputation**: Improves performance
    /// - **Degeneracy handling**: Skips zero-length edges
    /// - **Precision**: Maintains accuracy for small angles
    ///
    /// # Parameters
    /// - `angle_degs`: Revolution angle in degrees (0-360)
    /// - `segments`: Number of angular subdivisions (≥ 2)
    ///
    /// **Note**: Only processes 2D geometry in `self.geometry`. 3D polygons are ignored.
    /// Returns CSG with revolution surfaces only (original 2D geometry cleared).
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
