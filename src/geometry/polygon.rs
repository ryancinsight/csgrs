use crate::core::float_types::parry3d::bounding_volume::Aabb;
use crate::core::float_types::{PI, Real};
use crate::geometry::Plane;
use crate::geometry::Vertex;

use nalgebra::{Point2, Point3, Vector3};
use std::sync::OnceLock;
use geo::{LineString, Polygon as GeoPolygon, coord};

/// A polygon, defined by a list of vertices.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    /// Vertices defining the Polygon's shape
    pub vertices: Vec<Vertex>,

    /// The plane on which this Polygon lies, used for splitting
    pub plane: Plane,

    /// Lazily‑computed axis‑aligned bounding box of the Polygon
    pub bounding_box: OnceLock<Aabb>,

    /// Generic metadata associated with the Polygon
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync> Polygon<S> {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: Option<S>) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");

        let plane = Plane::from_vertices(vertices.clone());

        Polygon {
            vertices,
            plane,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Axis aligned bounding box of this Polygon (cached after first call)
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

    /// Invalidate the cached bounding box, forcing it to be recomputed on next access
    pub fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }
}

/// Build an orthonormal basis from a normal vector for 2D projection
pub fn build_orthonormal_basis(normal: &Vector3<Real>) -> (Vector3<Real>, Vector3<Real>) {
    // Choose a vector that's not parallel to the normal
    let temp = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };

    // Create first basis vector perpendicular to normal
    let u = normal.cross(&temp).normalize();

    // Create second basis vector perpendicular to both normal and u
    let v = normal.cross(&u).normalize();

    (u, v)
}

impl<S: Clone + Send + Sync> Polygon<S> {
    /// Reverses winding order, flips vertices normals, and flips the plane normal
    pub fn flip(&mut self) {
        // 1) reverse vertices
        self.vertices.reverse();
        // 2) flip all vertex normals
        for v in &mut self.vertices {
            v.flip();
        }
        // 3) flip the cached plane too
        self.plane.flip();
        // Note: flip() doesn't change vertex positions, so bounding box remains valid
    }

    /// Return an iterator over paired vertices each forming an edge of the polygon
    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> {
        self.vertices.iter().zip(self.vertices.iter().cycle().skip(1))
    }

    /// **Mathematical Foundation: Polygon Triangulation**
    ///
    /// Triangulate this polygon into a list of triangles, each triangle is [v0, v1, v2].
    /// This implements robust 2D triangulation algorithms for 3D planar polygons.
    ///
    /// ## **Algorithmic Approaches**
    ///
    /// ### **Ear Clipping (Earcut)**
    /// **Algorithm**: Based on the "ear removal" theorem:
    /// - **Ear Definition**: A triangle formed by three consecutive vertices with no other vertices inside
    /// - **Theorem**: Every simple polygon with n > 3 vertices has at least two ears
    /// - **Complexity**: O(n²) worst case, O(n) for most practical polygons
    /// - **Robustness**: Handles arbitrary simple polygons including concave shapes
    ///
    /// ### **Delaunay Triangulation (Spade)**
    /// **Algorithm**: Based on maximizing minimum angles:
    /// - **Delaunay Property**: No vertex lies inside circumcircle of any triangle
    /// - **Complexity**: O(n log n) expected time
    /// - **Quality**: Produces well-shaped triangles, avoids slivers
    /// - **Constraints**: Maintains polygon boundary as constraint edges
    ///
    /// ## **3D to 2D Projection**
    /// The algorithm projects the 3D planar polygon to 2D:
    /// 1. **Orthonormal Basis**: Compute basis vectors {u⃗, v⃗} in the plane
    /// 2. **Projection**: For each vertex pᵢ: (x,y) = ((pᵢ-p₀)·u⃗, (pᵢ-p₀)·v⃗)
    /// 3. **Triangulation**: Apply 2D algorithm to projected coordinates
    /// 4. **Reconstruction**: Map 2D triangles back to 3D using inverse projection
    ///
    /// ## **Numerical Considerations**
    /// - **Degeneracy Handling**: Filters out near-zero coordinates for stability
    /// - **Precision Limits**: Spade enforces minimum coordinate values
    /// - **Normal Preservation**: All output triangles maintain original plane normal
    ///
    /// The choice between algorithms depends on build features:
    /// - `earcut`: Fast for simple polygons, handles concave shapes
    /// - `delaunay`: Better triangle quality, more robust for complex geometry
    pub fn tessellate(&self) -> Vec<[Vertex; 3]> {
        // If polygon has fewer than 3 vertices, nothing to tessellate
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        let normal_3d = self.plane.normal().normalize();
        let (u, v) = build_orthonormal_basis(&normal_3d);
        let origin_3d = self.vertices[0].pos;

        #[cfg(feature = "earcut")]
        {
            // Flatten each vertex to 2D
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                all_vertices_2d.push(coord! {x: x, y: y});
            }

            use geo::TriangulateEarcut;
            let triangulation = GeoPolygon::new(LineString::new(all_vertices_2d), Vec::new())
                .earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert back into 3D triangles using iterator combinators
            triangle_indices
                .chunks_exact(3)
                .map(|tri_chunk| {
                    // Convert each triangle's indices to 3D vertices
                    let tri_vertices: [Vertex; 3] = tri_chunk
                        .iter()
                        .map(|&idx| {
                            let base = idx * 2;
                            let x = vertices[base];
                            let y = vertices[base + 1];
                            let pos_3d = origin_3d.coords + (x * u) + (y * v);
                            Vertex::new(Point3::from(pos_3d), normal_3d)
                        })
                        .collect::<Vec<_>>()
                        .try_into()
                        .expect("chunks_exact(3) guarantees exactly 3 elements");
                    tri_vertices
                })
                .collect()
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;

            // Flatten each vertex to 2D
            // Here we clamp values within spade's minimum allowed value of  0.0 to 0.0
            // because spade refuses to triangulate with values within it's minimum:
            const MIN_ALLOWED_VALUE: Real = 1.793662034335766e-43; // 1.0 * 2^-142
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let x_clamped = if x.abs() < MIN_ALLOWED_VALUE { 0.0 } else { x };
                let y = offset.dot(&v);
                let y_clamped = if y.abs() < MIN_ALLOWED_VALUE { 0.0 } else { y };

                // test for NaN/±∞
                if !(x.is_finite()
                    && y.is_finite()
                    && x_clamped.is_finite()
                    && y_clamped.is_finite())
                {
                    // at least one coordinate was NaN/±∞ – ignore this triangle
                    continue;
                }
                all_vertices_2d.push(coord! {x: x_clamped, y: y_clamped});
            }

            let polygon_2d = GeoPolygon::new(
                LineString::new(all_vertices_2d),
                // no holes if your polygon is always simple
                Vec::new(),
            );
            let Ok(tris) = polygon_2d.constrained_triangulation(Default::default()) else {
                return Vec::new();
            };

            let mut final_triangles = Vec::with_capacity(tris.len());
            for tri2d in tris {
                // tri2d is a geo::Triangle in 2D
                // Convert each corner from (x,y) to 3D again
                let [coord_a, coord_b, coord_c] = [tri2d.0, tri2d.1, tri2d.2];
                let pos_a_3d = origin_3d.coords + coord_a.x * u + coord_a.y * v;
                let pos_b_3d = origin_3d.coords + coord_b.x * u + coord_b.y * v;
                let pos_c_3d = origin_3d.coords + coord_c.x * u + coord_c.y * v;

                final_triangles.push([
                    Vertex::new(Point3::from(pos_a_3d), normal_3d),
                    Vertex::new(Point3::from(pos_b_3d), normal_3d),
                    Vertex::new(Point3::from(pos_c_3d), normal_3d),
                ]);
            }
            final_triangles
        }
    }

    /// **Mathematical Foundation: Triangle Subdivision for Mesh Refinement**
    ///
    /// Subdivide this polygon into smaller triangles using recursive triangle splitting.
    /// This implements the mathematical theory of uniform mesh refinement:
    ///
    /// ## **Subdivision Algorithm**
    ///
    /// ### **Base Triangulation**
    /// 1. **Initial Tessellation**: Convert polygon to base triangles using tessellate()
    /// 2. **Triangle Count**: n base triangles from polygon
    ///
    /// ### **Recursive Subdivision**
    /// For each subdivision level, each triangle T is split into 4 smaller triangles:
    /// ```text
    /// Original Triangle:     Subdivided Triangle:
    ///        A                        A
    ///       /\                      /\ \
    ///      /  \                    /  \ \
    ///     /____\                  M₁___M₂ \
    ///    B      C                /\    /\ \
    ///                           /  \  /  \ \
    ///                          /____\/____\
    ///                         B     M₃     C
    /// ```
    ///
    /// ### **Midpoint Calculation**
    /// For triangle vertices (A, B, C):
    /// - **M₁ = midpoint(A,B)**: Linear interpolation at t=0.5
    /// - **M₂ = midpoint(A,C)**: Linear interpolation at t=0.5  
    /// - **M₃ = midpoint(B,C)**: Linear interpolation at t=0.5
    ///
    /// ### **Subdivision Pattern**
    /// Creates 4 congruent triangles:
    /// 1. **Corner triangles**: (A,M₁,M₂), (M₁,B,M₃), (M₂,M₃,C)
    /// 2. **Center triangle**: (M₁,M₂,M₃)
    ///
    /// ## **Mathematical Properties**
    /// - **Area Preservation**: Total area remains constant
    /// - **Similarity**: All subtriangles are similar to original
    /// - **Scaling Factor**: Each subtriangle has 1/4 the area
    /// - **Growth Rate**: Triangle count × 4ᵏ after k subdivisions
    /// - **Smoothness**: C¹ continuity maintained across edges
    ///
    /// ## **Applications**
    /// - **Level of Detail**: Adaptive mesh resolution
    /// - **Smooth Surfaces**: Approximating curved surfaces with flat triangles
    /// - **Numerical Methods**: Finite element mesh refinement
    /// - **Rendering**: Progressive mesh detail for distance-based LOD
    ///
    /// Returns a list of refined triangles (each is a [Vertex; 3]).
    /// For polygon applications, these can be converted back to triangular polygons.
    pub fn subdivide_triangles(
        &self,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<[Vertex; 3]> {
        // 1) Triangulate the polygon as it is.
        let base_tris = self.tessellate();

        // 2) For each triangle, subdivide 'subdivisions' times using iterator combinators.
        base_tris
            .into_iter()
            .flat_map(|tri| {
                // Use fold to iteratively subdivide each triangle
                (0..subdivisions.get())
                    .fold(vec![tri], |current_triangles, _| {
                        // For each subdivision level, flat_map all current triangles
                        current_triangles
                            .into_iter()
                            .flat_map(subdivide_triangle)
                            .collect()
                    })
            })
            .collect()
    }

    /// Convert subdivision triangles back to polygons for CSG operations
    /// Each triangle becomes a triangular polygon with the same metadata
    pub fn subdivide_to_polygons(
        &self,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<Polygon<S>> {
        self.subdivide_triangles(subdivisions)
            .into_iter()
            .map(|tri| {
                let vertices = tri.to_vec();
                Polygon::new(vertices, self.metadata.clone())
            })
            .collect()
    }

    /// return a normal calculated from all polygon vertices
    pub fn calculate_new_normal(&self) -> Vector3<Real> {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        let mut points = Vec::new();
        for vertex in &self.vertices {
            points.push(vertex.pos);
        }
        let mut normal = Vector3::zeros();

        // Loop over each edge of the polygon.
        for i in 0..n {
            let current = points[i];
            let next = points[(i + 1) % n]; // wrap around using modulo
            normal.x += (current.y - next.y) * (current.z + next.z);
            normal.y += (current.z - next.z) * (current.x + next.x);
            normal.z += (current.x - next.x) * (current.y + next.y);
        }

        // Normalize the computed normal.
        let mut poly_normal = normal.normalize();

        // Ensure the computed normal is in the same direction as the given normal.
        if poly_normal.dot(&self.plane.normal()) < 0.0 {
            poly_normal = -poly_normal;
        }

        poly_normal
    }

    /// Recompute this polygon's normal from all vertices, then set all vertices' normals to match (flat shading).
    pub fn set_new_normal(&mut self) {
        // Assign each vertex's normal to match the plane
        let new_normal = self.calculate_new_normal();
        for v in &mut self.vertices {
            v.normal = new_normal;
        }
    }

    /// Returns a reference to the metadata, if any.
    pub fn metadata(&self) -> Option<&S> {
        self.metadata.as_ref()
    }

    /// Returns a mutable reference to the metadata, if any.
    pub fn metadata_mut(&mut self) -> Option<&mut S> {
        self.metadata.as_mut()
    }

    /// Sets the metadata to the given value.
    pub fn set_metadata(&mut self, data: S) {
        self.metadata = Some(data);
    }
}




// Helper function to subdivide a triangle
pub fn subdivide_triangle(tri: [Vertex; 3]) -> Vec<[Vertex; 3]> {
    let v01 = tri[0].interpolate(&tri[1], 0.5);
    let v12 = tri[1].interpolate(&tri[2], 0.5);
    let v20 = tri[2].interpolate(&tri[0], 0.5);

    vec![
        [tri[0].clone(), v01.clone(), v20.clone()],
        [v01.clone(), tri[1].clone(), v12.clone()],
        [v20.clone(), v12.clone(), tri[2].clone()],
        [v01, v12, v20],
    ]
}

/// Helper to normalize angles into (-π, π].
const fn normalize_angle(mut a: Real) -> Real {
    while a <= -PI {
        a += 2.0 * PI;
    }
    while a > PI {
        a -= 2.0 * PI;
    }
    a
}

/// Compute an initial guess of the circle center through three points p1, p2, p3
/// (this is used purely as an initial guess).
///
/// This is a direct port of your snippet's `centre(p1, p2, p3)`, but
/// returning a `Point2<Real>` from nalgebra.
fn naive_circle_center(
    p1: &Point2<Real>,
    p2: &Point2<Real>,
    p3: &Point2<Real>,
) -> Point2<Real> {
    // Coordinates
    let (x1, y1) = (p1.x, p1.y);
    let (x2, y2) = (p2.x, p2.y);
    let (x3, y3) = (p3.x, p3.y);

    let x12 = x1 - x2;
    let x13 = x1 - x3;
    let y12 = y1 - y2;
    let y13 = y1 - y3;

    let y31 = y3 - y1;
    let y21 = y2 - y1;
    let x31 = x3 - x1;
    let x21 = x2 - x1;

    let sx13 = x1.powi(2) - x3.powi(2);
    let sy13 = y1.powi(2) - y3.powi(2);
    let sx21 = x2.powi(2) - x1.powi(2);
    let sy21 = y2.powi(2) - y1.powi(2);

    let xden = 2.0 * (x31 * y12 - x21 * y13);
    let yden = 2.0 * (y31 * x12 - y21 * x13);

    if xden.abs() < 1e-14 || yden.abs() < 1e-14 {
        // fallback => just average the points
        let cx = (x1 + x2 + x3) / 3.0;
        let cy = (y1 + y2 + y3) / 3.0;
        return Point2::new(cx, cy);
    }

    let g = (sx13 * y12 + sy13 * y12 + sx21 * y13 + sy21 * y13) / xden;
    let f = (sx13 * x12 + sy13 * x12 + sx21 * x13 + sy21 * x13) / yden;

    // Return the center as a Point2
    Point2::new(-g, -f)
}

/// **Mathematical Foundation: Circle Fitting by Least Squares Optimization**
///
/// Fit a circle to the points `[pt_c, intermediates..., pt_n]` by adjusting an offset `d` from
/// the midpoint. This implements a sophisticated numerical optimization approach:
///
/// ## **Theoretical Foundation**
///
/// ### **Parametric Circle Representation**
/// Given two endpoints A and B, we parameterize possible circles as:
/// - **Midpoint**: M = (A + B) / 2  
/// - **Perpendicular direction**: v⊥ = rotate_90°(B - A)
/// - **Center**: C(d) = M + d · v⊥ / |B - A|
/// - **Radius**: R(d) = √(d² + |B - A|²/4)
///
/// ### **Optimization Problem**
/// Find parameter d that minimizes the sum of squared distances:
/// ```text
/// minimize: Σᵢ (|Pᵢ - C(d)| - R(d))²
/// ```
///
/// ### **Numerical Method: Secant Iteration**
/// 1. **Initial Guess**: Use circumcenter of three well-spaced points
/// 2. **Derivative Estimation**: Finite difference: f'(d) ≈ (f(d+h) - f(d-h))/(2h)
/// 3. **Secant Update**: dₙ₊₁ = dₙ - f'(dₙ) · (dₙ - dₙ₋₁) / (f'(dₙ) - f'(dₙ₋₁))
/// 4. **Convergence**: Typically 10-15 iterations achieve machine precision
///
/// ### **Orientation Determination**
/// Arc direction (clockwise vs counterclockwise) determined by:
/// - **Angular sweep**: θ = normalize_angle(θₑₙd - θₛₜₐᵣₜ)
/// - **CW if θ < 0**, **CCW if θ > 0**
///
/// This method is numerically stable and handles near-linear configurations gracefully.
///
/// # Returns
///
/// `(center, radius, cw, rms)`:
/// - `center`: fitted circle center (Point2),
/// - `radius`: circle radius,
/// - `cw`: `true` if the arc is clockwise, `false` if ccw,
/// - `rms`: root‐mean‐square error of the fit.
pub fn fit_circle_arcfinder(
    pt_c: &Point2<Real>,
    pt_n: &Point2<Real>,
    intermediates: &[Point2<Real>],
) -> (Point2<Real>, Real, bool, Real) {
    // 1) Distance between pt_c and pt_n, plus midpoint
    let k = (pt_c - pt_n).norm();
    if k < 1e-14 {
        // Degenerate case => no unique circle
        let center = *pt_c;
        return (center, 0.0, false, 9999.0);
    }
    let mid = Point2::new(0.5 * (pt_c.x + pt_n.x), 0.5 * (pt_c.y + pt_n.y));

    // 2) Pre‐compute the direction used for the offset:
    //    This is the 2D +90 rotation of (pt_n - pt_c).
    //    i.e. rotate( dx, dy ) => (dy, -dx ) or similar.
    let vec_cn = pt_n - pt_c; // a Vector2
    let rx = vec_cn.y; // +90 deg
    let ry = -vec_cn.x; // ...

    // collect all points in one array for the mismatch
    let mut all_points = Vec::with_capacity(intermediates.len() + 2);
    all_points.push(*pt_c);
    all_points.extend_from_slice(intermediates);
    all_points.push(*pt_n);

    // The mismatch function g(d)
    let g = |d: Real| -> Real {
        let r_desired = (d * d + 0.25 * k * k).sqrt();
        // circle center
        let cx = mid.x + (d / k) * rx;
        let cy = mid.y + (d / k) * ry;
        let mut sum_sq = 0.0;
        for p in &all_points {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dist = (dx * dx + dy * dy).sqrt();
            let diff = dist - r_desired;
            sum_sq += diff * diff;
        }
        sum_sq
    };

    // derivative dg(d) => we'll do a small finite difference
    let dg = |d: Real| -> Real {
        let h = 1e-6;
        let g_p = g(d + h);
        let g_m = g(d - h);
        (g_p - g_m) / (2.0 * h)
    };

    // 3) choose an initial guess for d
    let mut d_est = 0.0; // fallback
    if !intermediates.is_empty() {
        // pick p3 ~ the middle of intermediates
        let mididx = intermediates.len() / 2;
        let p3 = intermediates[mididx];
        let c_est = naive_circle_center(pt_c, pt_n, &p3);
        // project c_est - mid onto (rx, ry)/k => that is d
        let dx = c_est.x - mid.x;
        let dy = c_est.y - mid.y;
        let dot = dx * (rx / k) + dy * (ry / k);
        d_est = dot;
    }

    // 4) small secant iteration for ~10 steps
    let mut d0 = d_est - 0.1 * k;
    let mut d1 = d_est;
    let mut dg0 = dg(d0);
    let mut dg1 = dg(d1);

    for _ in 0..10 {
        if (dg1 - dg0).abs() < 1e-14 {
            break;
        }
        let temp = d1;
        d1 = d1 - dg1 * (d1 - d0) / (dg1 - dg0);
        d0 = temp;
        dg0 = dg1;
        dg1 = dg(d1);
    }

    let d_opt = d1;
    let cx = mid.x + (d_opt / k) * rx;
    let cy = mid.y + (d_opt / k) * ry;
    let center = Point2::new(cx, cy);
    let radius_opt = (d_opt * d_opt + 0.25 * k * k).sqrt();

    // sum of squares at d_opt
    let sum_sq = g(d_opt);
    let n_pts = all_points.len() as Real;
    let rms = (sum_sq / n_pts).sqrt();

    // 5) determine cw vs ccw
    let dx0 = pt_c.x - cx;
    let dy0 = pt_c.y - cy;
    let dx1 = pt_n.x - cx;
    let dy1 = pt_n.y - cy;
    let angle0 = dy0.atan2(dx0);
    let angle1 = dy1.atan2(dx1);
    let total_sweep = normalize_angle(angle1 - angle0);
    let cw = total_sweep < 0.0;

    (center, radius_opt, cw, rms)
}
