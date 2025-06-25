//! Functional 2D shapes for practical applications
//!
//! This module contains shapes commonly used in mechanical design,
//! engineering applications, and functional components.

use crate::core::float_types::{EPSILON, FRAC_PI_2, PI, Real};
use crate::csg::CSG;
use geo::{Geometry, GeometryCollection, LineString, Polygon as GeoPolygon};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Rounded rectangle in XY plane, from (0,0) to (width,height) with radius for corners.
    /// `corner_segments` controls the smoothness of each rounded corner.
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        let r = corner_radius.min(width * 0.5).min(height * 0.5);
        if r <= EPSILON {
            return Self::rectangle(width, height, metadata);
        }
        let step = FRAC_PI_2 / corner_segments as Real;

        let corner = |cx, cy, start_angle| {
            (0..=corner_segments).map(move |i| {
                let angle: Real = start_angle + (i as Real) * step;
                (cx + r * angle.cos(), cy + r * angle.sin())
            })
        };

        let mut coords: Vec<(Real, Real)> = corner(r, r, PI) // Bottom-left
            .chain(corner(width - r, r, 1.5 * PI)) // Bottom-right
            .chain(corner(width - r, height - r, 0.0)) // Top-right
            .chain(corner(r, height - r, 0.5 * PI)) // Top-left
            .collect();

        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Keyhole shape (simple version): a large circle + a rectangle "handle".
    /// This does *not* have a hole. If you want a literal hole, you'd do difference ops.
    /// Here we do union of a circle and a rectangle.
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        // 1) Circle
        let circle = CSG::circle(circle_radius, segments, metadata.clone());

        // 2) Rectangle (handle)
        let handle = CSG::rectangle(handle_width, handle_height, metadata).translate(
            -handle_width * 0.5,
            0.0,
            0.0,
        );

        // 3) Union them
        circle.union(&handle)
    }

    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    pub fn ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        if id <= 0.0 || thickness <= 0.0 || segments < 3 {
            return CSG::new();
        }
        let inner_radius = 0.5 * id;
        let outer_radius = inner_radius + thickness;

        let outer_circle = CSG::circle(outer_radius, segments, metadata.clone());
        let inner_circle = CSG::circle(inner_radius, segments, metadata);

        outer_circle.difference(&inner_circle)
    }

    /// Creates a 2D circle with a rectangular keyway slot cut out on the +X side.
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());

        // 2. Construct the keyway rectangle
        let key_rect = CSG::rectangle(key_depth, key_width, metadata.clone()).translate(
            radius - key_depth,
            -key_width * 0.5,
            0.0,
        );

        circle.difference(&key_rect)
    }

    /// Creates a 2D "D" shape (circle with one flat chord).
    /// `radius` is the circle radius,
    /// `flat_dist` is how far from the center the flat chord is placed.
    pub fn circle_with_flat(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());

        // 2. Build a large rectangle that cuts off everything below y = -flat_dist
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::rectangle(2.0 * radius, cutter_height, metadata.clone())
            .translate(-radius, -cutter_height, 0.0) // put its bottom near "negative infinity"
            .translate(0.0, -flat_dist, 0.0); // now top edge is at y = -flat_dist

        // 3. Subtract to produce the flat chord
        circle.difference(&rect_cutter)
    }

    /// Circle with two parallel flat chords on opposing sides (e.g., "double D" shape).
    /// `radius`   => circle radius
    /// `segments` => how many segments in the circle approximation
    /// `flat_dist` => half-distance between flats measured from the center.
    pub fn circle_with_two_flats(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());

        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let cutter_height = 9999.0;
        let top_rect = CSG::rectangle(2.0 * radius, cutter_height, metadata.clone())
            // place bottom at y=flat_dist
            .translate(-radius, flat_dist, 0.0);

        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let bottom_rect = CSG::rectangle(2.0 * radius, cutter_height, metadata.clone())
            // place top at y=-flat_dist => bottom extends downward
            .translate(-radius, -cutter_height - flat_dist, 0.0);

        // 4. Subtract both
        let with_top_flat = circle.difference(&top_rect);
        let with_both_flats = with_top_flat.difference(&bottom_rect);

        with_both_flats
    }
}
