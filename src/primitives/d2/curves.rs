//! Parametric curves and complex 2D curve generation
//!
//! This module provides functions for generating sophisticated 2D curves including
//! NACA airfoils, Bézier curves, and B-splines.

use crate::csg::CSG;
use crate::core::float_types::{EPSILON, Real};
use geo::{
    Geometry, GeometryCollection, LineString, Orient,
    Polygon as GeoPolygon, orient::Direction,
};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Generate a NACA 4-digit series airfoil profile.
    /// 
    /// # Parameters
    /// - `code`: 4-digit NACA code (e.g., "2412")
    /// - `chord`: chord length of the airfoil
    /// - `samples`: number of sample points per surface (upper/lower)
    /// - `metadata`: optional metadata
    /// 
    /// # Example
    /// ```
    /// use csgrs::CSG;
    /// let airfoil: CSG<()> = CSG::airfoil("2412", 1.0, 50, None);
    /// ```
    pub fn airfoil(code: &str, chord: Real, samples: usize, metadata: Option<S>) -> CSG<S>
    where
        S: Clone + Send + Sync,
    {
        assert!(
            code.len() == 4 && code.chars().all(|c| c.is_ascii_digit()),
            "NACA code must be exactly 4 digits"
        );
        assert!(samples >= 10, "Need at least 10 points per surface");

        // --- decode code -------------------------------------------------------
        let m = code[0..1].parse::<Real>().unwrap() / 100.0; // max-camber %
        let p = code[1..2].parse::<Real>().unwrap() / 10.0; // camber-pos
        let tt = code[2..4].parse::<Real>().unwrap() / 100.0; // thickness %

        // thickness half-profile -----------------------------------------------
        let yt = |x: Real| -> Real {
            5.0 * tt
                * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x
                    - 0.1015 * x * x * x * x)
        };

        // mean-camber line & slope ---------------------------------------------
        let camber = |x: Real| -> (Real, Real) {
            if x < p {
                let yc = m / (p * p) * (2.0 * p * x - x * x);
                let dy = 2.0 * m / (p * p) * (p - x);
                (yc, dy)
            } else {
                let yc = m / ((1.0 - p).powi(2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x);
                let dy = 2.0 * m / ((1.0 - p).powi(2)) * (p - x);
                (yc, dy)
            }
        };

        // --- sample upper & lower surfaces ------------------------------------
        let n = samples as Real;
        let mut coords: Vec<(Real, Real)> = Vec::with_capacity(2 * samples + 1);

        // leading-edge → trailing-edge (upper)
        for i in 0..=samples {
            let xc = i as Real / n; // 0–1
            let x = xc * chord; // physical
            let t = yt(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xu = x - t * theta.sin();
            let yu = chord * (yc_val + t * theta.cos());
            coords.push((xu, yu));
        }

        // trailing-edge → leading-edge (lower)
        for i in (1..samples).rev() {
            let xc = i as Real / n;
            let x = xc * chord;
            let t = yt(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xl = x + t * theta.sin();
            let yl = chord * (yc_val - t * theta.cos());
            coords.push((xl, yl));
        }

        coords.push(coords[0]); // close

        let polygon_2d =
            GeoPolygon::new(LineString::from(coords), vec![]).orient(Direction::Default);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Sample an arbitrary-degree Bézier curve using de Casteljau's algorithm.
    /// Returns a poly-line (closed if the first = last point).
    ///
    /// # Parameters
    /// - `control`: list of 2-D control points
    /// - `segments`: number of straight-line segments used for the tessellation
    /// - `metadata`: optional metadata
    pub fn bezier(control: &[[Real; 2]], segments: usize, metadata: Option<S>) -> Self {
        if control.len() < 2 || segments < 1 {
            return CSG::new();
        }

        // de Casteljau evaluator -------------------------------------------------
        fn de_casteljau(ctrl: &[[Real; 2]], t: Real, tmp: &mut Vec<(Real, Real)>) -> (Real, Real) {
            tmp.clear();
            tmp.extend(ctrl.iter().map(|&[x, y]| (x, y)));
            let n = tmp.len();
            for k in 1..n {
                for i in 0..(n - k) {
                    tmp[i].0 = (1.0 - t) * tmp[i].0 + t * tmp[i + 1].0;
                    tmp[i].1 = (1.0 - t) * tmp[i].1 + t * tmp[i + 1].1;
                }
            }
            tmp[0]
        }

        let mut pts = Vec::<(Real, Real)>::with_capacity(segments + 1);
        let mut tmp = Vec::with_capacity(control.len());
        for i in 0..=segments {
            let t = i as Real / segments as Real;
            pts.push(de_casteljau(control, t, &mut tmp));
        }

        // If the curve happens to be closed, make sure the polygon ring closes.
        let closed = (pts.first().unwrap().0 - pts.last().unwrap().0).abs() < EPSILON
            && (pts.first().unwrap().1 - pts.last().unwrap().1).abs() < EPSILON;
        if !closed {
            // open curve → produce a LineString geometry, *not* a filled polygon
            let ls: LineString<Real> = pts.into();
            let mut gc = GeometryCollection::default();
            gc.0.push(Geometry::LineString(ls));
            return CSG::from_geo(gc, metadata);
        }

        // closed curve → create a filled polygon
        let poly_2d = GeoPolygon::new(LineString::from(pts), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(poly_2d)]), metadata)
    }

    /// Sample an open-uniform B-spline of arbitrary degree using the Cox-de Boor recursion.
    /// Returns a poly-line (or a filled region if closed).
    ///
    /// # Parameters
    /// - `control`: control points  
    /// - `p`: spline degree (e.g. 3 for a cubic)  
    /// - `segments_per_span`: tessellation resolution inside every knot span
    /// - `metadata`: optional metadata
    pub fn bspline(
        control: &[[Real; 2]],
        p: usize,
        segments_per_span: usize,
        metadata: Option<S>,
    ) -> Self {
        if control.len() < p + 1 || segments_per_span < 1 {
            return CSG::new();
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

        // Cox-de Boor basis evaluation ------------------------------------------
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
            let term1 = if denom1.abs() < EPSILON {
                0.0
            } else {
                (u - knot[i]) / denom1 * basis(i, p - 1, u, knot)
            };
            let term2 = if denom2.abs() < EPSILON {
                0.0
            } else {
                (knot[i + p + 1] - u) / denom2 * basis(i + 1, p - 1, u, knot)
            };
            term1 + term2
        }

        let span_count = n - p; // #inner knot spans
        let _max_u = span_count as Real; // parametric upper bound
        let dt = 1.0 / segments_per_span as Real; // step in local span coords

        let mut pts = Vec::<(Real, Real)>::new();
        for span in 0..=span_count {
            for s in 0..=segments_per_span {
                if span == span_count && s == segments_per_span {
                    // avoid duplicating final knot value
                    continue;
                }
                let u = span as Real + s as Real * dt; // global param
                let mut x = 0.0;
                let mut y = 0.0;
                for (idx, &[px, py]) in control.iter().enumerate() {
                    let b = basis(idx, p, u, &knot);
                    x += b * px;
                    y += b * py;
                }
                pts.push((x, y));
            }
        }

        let closed = (pts.first().unwrap().0 - pts.last().unwrap().0).abs() < EPSILON
            && (pts.first().unwrap().1 - pts.last().unwrap().1).abs() < EPSILON;
        if !closed {
            let ls: LineString<Real> = pts.into();
            let mut gc = GeometryCollection::default();
            gc.0.push(Geometry::LineString(ls));
            return CSG::from_geo(gc, metadata);
        }

        let poly_2d = GeoPolygon::new(LineString::from(pts), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(poly_2d)]), metadata)
    }
} 
