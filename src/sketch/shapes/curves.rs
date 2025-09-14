//! Curve-based geometric shapes for Sketch
//!
//! This module provides parametric curve implementations including
//! Bézier curves and B-splines with comprehensive mathematical foundations.

use crate::float_types::Real;
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{Geometry, GeometryCollection, LineString, Polygon as GeoPolygon};

use std::fmt::Debug;

/// Curve-based geometric shape implementations
impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Sample an arbitrary-degree Bézier curve (de Casteljau).
    /// Returns a poly-line (closed if the first = last point).
    ///
    /// * `control`: list of 2-D control points
    /// * `segments`: number of straight-line segments used for the tessellation
    pub fn bezier(control: &[[Real; 2]], segments: usize, metadata: Option<S>) -> Self {
        if control.len() < 2 || segments < 1 {
            return Sketch::new();
        }

        /// Evaluates a Bézier curve at a given parameter `t` using de Casteljau's algorithm.
        fn de_casteljau(control: &[[Real; 2]], t: Real) -> (Real, Real) {
            let mut points = control.to_vec();
            let n = points.len();

            for k in 1..n {
                for i in 0..(n - k) {
                    points[i][0] = (1.0 - t) * points[i][0] + t * points[i + 1][0];
                    points[i][1] = (1.0 - t) * points[i][1] + t * points[i + 1][1];
                }
            }
            (points[0][0], points[0][1])
        }

        let pts: Vec<(Real, Real)> = (0..=segments)
            .map(|i| {
                let t = i as Real / segments as Real;
                de_casteljau(control, t)
            })
            .collect();

        let is_closed = {
            let first = pts[0];
            let last = pts[segments];
            (first.0 - last.0).abs() < crate::float_types::EPSILON
                && (first.1 - last.1).abs() < crate::float_types::EPSILON
        };

        let geometry = if is_closed {
            let ring: LineString<Real> = pts.into();
            Geometry::Polygon(GeoPolygon::new(ring, vec![]))
        } else {
            Geometry::LineString(pts.into())
        };

        Sketch::from_geo(GeometryCollection(vec![geometry]), metadata)
    }

    /// Sample an open-uniform B-spline of arbitrary degree (`p`) using the
    /// Cox-de Boor recursion. Returns a poly-line (or a filled region if closed).
    ///
    /// * `control`: control points
    /// * `p`:       spline degree (e.g. 3 for a cubic)
    /// * `segments_per_span`: tessellation resolution inside every knot span
    pub fn bspline(
        control: &[[Real; 2]],
        p: usize,
        segments_per_span: usize,
        metadata: Option<S>,
    ) -> Self {
        if control.len() < p + 1 || segments_per_span < 1 {
            return Sketch::new();
        }

        let n = control.len() - 1;
        let m = n + p + 1; // knot count
        // open-uniform knot vector: 0,0,…,0,1,2,…,n-p-1,(n-p),…,(n-p)
        let mut knot = Vec::<Real>::with_capacity(m + 1);
        for i in 0..=m {
            if i <= p {
                knot.push(0.0);
            } else if i >= m - p {
                knot.push((n - p) as Real);
            } else {
                knot.push((i - p) as Real);
            }
        }

        // Cox-de Boor basis evaluation
        fn basis(i: usize, p: usize, u: Real, knot: &[Real]) -> Real {
            if p == 0 {
                return if u >= knot[i] && u < knot[i + 1] {
                    1.0
                } else {
                    0.0
                };
            }
            let denom1 = knot[i + p] - knot[i];
            let denom2 = knot[i + p + 1] - knot[i + 1];
            let term1 = if denom1.abs() < crate::float_types::EPSILON {
                0.0
            } else {
                (u - knot[i]) / denom1 * basis(i, p - 1, u, knot)
            };
            let term2 = if denom2.abs() < crate::float_types::EPSILON {
                0.0
            } else {
                (knot[i + p + 1] - u) / denom2 * basis(i + 1, p - 1, u, knot)
            };
            term1 + term2
        }

        let mut pts: Vec<(Real, Real)> = Vec::new();

        // Evaluate spline at parameter values
        for i in p..=n {
            let u_start = knot[i];
            let u_end = knot[i + 1];
            let span_length = u_end - u_start;

            if span_length > 0.0 {
                for j in 0..segments_per_span {
                    let t = j as Real / (segments_per_span - 1) as Real;
                    let u = u_start + t * span_length;

                    let mut x = 0.0;
                    let mut y = 0.0;
                    for (k, point) in control.iter().enumerate().take(n + 1) {
                        let b = basis(k, p, u, &knot);
                        x += b * point[0];
                        y += b * point[1];
                    }
                    pts.push((x, y));
                }
            }
        }

        let geometry = Geometry::LineString(pts.into());
        Sketch::from_geo(GeometryCollection(vec![geometry]), metadata)
    }
}
