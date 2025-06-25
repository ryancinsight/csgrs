//! Decorative 2D shapes with aesthetic appeal
//!
//! This module contains shapes commonly used for decorative purposes,
//! artistic designs, and visual elements in user interfaces.

use crate::core::float_types::{EPSILON, PI, Real, TAU};
use crate::csg::CSG;
use geo::{Geometry, GeometryCollection, LineString, Polygon as GeoPolygon};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Star shape (typical "spiky star") with `num_points`, outer_radius, inner_radius.
    /// The star is centered at (0,0).
    pub fn star(
        num_points: usize,
        outer_radius: Real,
        inner_radius: Real,
        metadata: Option<S>,
    ) -> Self {
        if num_points < 2 {
            return CSG::new();
        }
        let step = TAU / (num_points as Real);
        let mut coords: Vec<(Real, Real)> = (0..num_points)
            .flat_map(|i| {
                let theta_out = i as Real * step;
                let outer_point =
                    (outer_radius * theta_out.cos(), outer_radius * theta_out.sin());

                let theta_in = theta_out + 0.5 * step;
                let inner_point =
                    (inner_radius * theta_in.cos(), inner_radius * theta_in.sin());

                [outer_point, inner_point]
            })
            .collect();

        // close
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Teardrop shape. A simple approach:
    /// - a circle arc for the "round" top
    /// - it tapers down to a cusp at bottom.
    /// This is just one of many possible "teardrop" definitions.
    pub fn teardrop_outline(
        width: Real,
        length: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 2 || width < EPSILON || length < EPSILON {
            return CSG::new();
        }
        let r = 0.5 * width;
        let center_y = length - r;
        let half_seg = segments / 2;

        let mut coords = vec![(0.0, 0.0)]; // Start at the tip
        coords.extend((0..=half_seg).map(|i| {
            let t = PI * (i as Real / half_seg as Real); // Corrected angle for semi-circle
            let x = -r * t.cos();
            let y = center_y + r * t.sin();
            (x, y)
        }));
        coords.push((0.0, 0.0)); // Close path to the tip

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Egg outline. Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation. It creates a closed "egg-ish" outline around the origin.
    pub fn egg_outline(
        width: Real,
        length: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * length;
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = TAU * (i as Real) / (segments as Real);
                // toy distortion approach
                let distort = 1.0 + 0.2 * theta.cos();
                let x = rx * theta.sin();
                let y = ry * theta.cos() * distort * 0.8;
                (-x, y) // mirrored
            })
            .collect();
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// 2-D heart outline (closed polygon) sized to `width` × `height`.
    ///
    /// `segments` controls smoothness (≥ 8 recommended).
    pub fn heart(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 8 {
            return Self::new();
        }

        let step = TAU / segments as Real;

        let mut pts: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let t = i as Real * step;
                let x = 16.0 * (t.sin().powi(3));
                let y = 13.0 * t.cos()
                    - 5.0 * (2.0 * t).cos()
                    - 2.0 * (3.0 * t).cos()
                    - (4.0 * t).cos();
                (x, y)
            })
            .collect();
        pts.push(pts[0]); // close

        // normalise & scale to desired bounding box
        let (min_x, max_x) = pts.iter().fold((Real::MAX, -Real::MAX), |(lo, hi), &(x, _)| {
            (lo.min(x), hi.max(x))
        });
        let (min_y, max_y) = pts.iter().fold((Real::MAX, -Real::MAX), |(lo, hi), &(_, y)| {
            (lo.min(y), hi.max(y))
        });
        let s_x = width / (max_x - min_x);
        let s_y = height / (max_y - min_y);

        let coords: Vec<(Real, Real)> = pts
            .into_iter()
            .map(|(x, y)| ((x - min_x) * s_x, (y - min_y) * s_y))
            .collect();

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Self::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// 2-D crescent obtained by subtracting a displaced smaller circle
    /// from a larger one.
    /// `segments` controls circle smoothness.
    ///
    /// # Example
    /// ```
    /// use csgrs::CSG;
    /// let cres: CSG<()> = CSG::crescent(2.0, 1.4, 0.8, 64, None);
    /// ```
    pub fn crescent(
        outer_r: Real,
        inner_r: Real,
        offset: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Self {
        if outer_r <= inner_r + EPSILON || segments < 6 {
            return Self::new();
        }

        let big = Self::circle(outer_r, segments, metadata.clone());
        let small =
            Self::circle(inner_r, segments, metadata.clone()).translate(offset, 0.0, 0.0);

        big.difference(&small)
    }
}
