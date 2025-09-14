//! Complex geometric shapes for Sketch
//!
//! This module provides advanced 2D geometric shapes with mathematical rigor
//! and comprehensive documentation for specialized geometric constructions.

use crate::float_types::Real;
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{Geometry, GeometryCollection, LineString, Polygon as GeoPolygon};

use std::fmt::Debug;

/// Complex geometric shape implementations
impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Creates an ellipse in the XY plane.
    ///
    /// # Parameters
    /// - `width`: Full width (diameter) along x-axis
    /// - `height`: Full height (diameter) along y-axis
    /// - `segments`: Number of polygon edges (minimum 3)
    /// - `metadata`: Optional metadata
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return Sketch::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = crate::float_types::TAU * (i as Real) / (segments as Real);
                (rx * theta.cos(), ry * theta.sin())
            })
            .collect();
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// **Mathematical Foundation: Regular Polygon Construction**
    ///
    /// Creates a regular n-gon inscribed in a circle of given radius.
    /// This implements the classical construction of regular polygons using
    /// uniform angular division of the circumscribed circle.
    ///
    /// ## **Regular Polygon Mathematics**
    ///
    /// ### **Vertex Construction**
    /// For a regular n-gon inscribed in a circle of radius r:
    /// ```text
    /// Vertex_i = (r·cos(2πi/n), r·sin(2πi/n))
    /// where i ∈ {0, 1, ..., n-1}
    /// ```
    ///
    /// ### **Geometric Properties**
    /// - **Interior angle**: α = (n-2)π/n = π - 2π/n
    /// - **Central angle**: β = 2π/n
    /// - **Exterior angle**: γ = 2π/n
    /// - **Side length**: s = 2r·sin(π/n)
    /// - **Apothem** (distance from center to side): a = r·cos(π/n)
    /// - **Area**: A = (n·s·a)/2 = (n·r²·sin(2π/n))/2
    ///
    /// ### **Special Cases**
    /// - **n = 3**: Equilateral triangle (α = 60°)
    /// - **n = 4**: Square (α = 90°)
    /// - **n = 5**: Regular pentagon (α = 108°)
    /// - **n = 6**: Regular hexagon (α = 120°)
    /// - **n → ∞**: Approaches circle (lim α = 180°)
    ///
    /// ### **Constructibility Theorem**
    /// A regular n-gon is constructible with compass and straightedge if and only if:
    /// ```text
    /// n = 2^k · p₁ · p₂ · ... · pₘ
    /// ```
    /// where k ≥ 0 and pᵢ are distinct Fermat primes (3, 5, 17, 257, 65537).
    ///
    /// ### **Approximation to Circle**
    /// As n increases, the regular n-gon converges to a circle:
    /// - **Perimeter convergence**: P_n = n·s → 2πr as n → ∞
    /// - **Area convergence**: A_n → πr² as n → ∞
    /// - **Error bound**: |A_circle - A_n| ≤ πr³/(3n²) for large n
    ///
    /// ## **Numerical Considerations**
    /// - Uses crate::float_types::TAU for precise angular calculations
    /// - Explicit closure for geometric validity
    /// - Minimum n = 3 to avoid degenerate cases
    ///
    /// # Parameters
    /// - `sides`: Number of polygon edges (≥ 3)
    /// - `radius`: Circumscribed circle radius
    /// - `metadata`: Optional metadata
    pub fn regular_ngon(sides: usize, radius: Real, metadata: Option<S>) -> Self {
        if sides < 3 {
            return Sketch::new();
        }
        let mut coords: Vec<(Real, Real)> = (0..sides)
            .map(|i| {
                let theta = crate::float_types::TAU * (i as Real) / (sides as Real);
                (radius * theta.cos(), radius * theta.sin())
            })
            .collect();
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Trapezoid from (0,0) -> (bottom_width,0) -> (top_width+top_offset,height) -> (top_offset,height)
    /// Note: this is a simple shape that can represent many trapezoids or parallelograms.
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
        metadata: Option<S>,
    ) -> Self {
        let coords = vec![
            (0.0, 0.0),
            (bottom_width, 0.0),
            (top_width + top_offset, height),
            (top_offset, height),
            (0.0, 0.0), // close
        ];
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Star shape (typical "spiky star") with `num_points`, outer_radius, inner_radius.
    /// The star is centered at (0,0).
    pub fn star(
        num_points: usize,
        outer_radius: Real,
        inner_radius: Real,
        metadata: Option<S>,
    ) -> Self {
        if num_points < 2 {
            return Sketch::new();
        }
        let step = crate::float_types::TAU / (num_points as Real);
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
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// **Mathematical Foundation: Egg Shape Parametric Equation**
    ///
    /// Creates an egg-shaped curve using parametric equations.
    /// The egg shape is generated using a combination of sine and cosine functions
    /// with asymmetric scaling to create the characteristic egg profile.
    ///
    /// ## **Egg Shape Parametric Equations**
    /// ```text
    /// x(θ) = width · sin(θ) · (1 + 0.3·cos(θ))
    /// y(θ) = length · cos(θ) · (1 + 0.2·sin(θ))
    /// where θ ∈ [0, 2π]
    /// ```
    ///
    /// # Parameters
    /// - `width`: Maximum width of the egg
    /// - `length`: Maximum height of the egg
    /// - `segments`: Number of polygon edges (minimum 3)
    /// - `metadata`: Optional metadata
    pub fn egg(width: Real, length: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return Sketch::new();
        }

        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = crate::float_types::TAU * (i as Real) / (segments as Real);
                let x = width * theta.sin() * (1.0 + 0.3 * theta.cos());
                let y = length * theta.cos() * (1.0 + 0.2 * theta.sin());
                (x, y)
            })
            .collect();
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Teardrop shape for 3D printing and fluid dynamics applications.
    /// Combines a semicircle with a tapering triangle for optimal flow characteristics.
    ///
    /// # Parameters
    /// - `width`: Maximum width of the teardrop
    /// - `length`: Maximum height of the teardrop
    /// - `segments`: Number of segments for the curved portion (minimum 3)
    /// - `metadata`: Optional metadata
    pub fn teardrop(width: Real, length: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return Sketch::new();
        }

        let radius = width / 2.0;
        let mut coords: Vec<(Real, Real)> = Vec::new();

        // Generate the curved top half (semicircle)
        for i in 0..=segments {
            let theta = crate::float_types::PI * (i as Real) / (segments as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin() + (length - radius);
            coords.push((x, y));
        }

        // Add the tapering bottom point
        coords.push((0.0, 0.0));

        // Close the shape
        coords.push((0.0, length));

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Rounded rectangle with configurable corner radius.
    /// Useful for creating smooth-edged rectangular shapes.
    ///
    /// # Parameters
    /// - `width`: Total width of the rectangle
    /// - `height`: Total height of the rectangle
    /// - `radius`: Corner radius (must be ≤ min(width/2, height/2))
    /// - `segments`: Segments per corner (minimum 1)
    /// - `metadata`: Optional metadata
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        radius: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Self {
        if segments < 1 || radius <= 0.0 {
            return Self::rectangle(width, height, metadata);
        }

        let half_width = width / 2.0;
        let half_height = height / 2.0;
        let max_radius = half_width.min(half_height);
        let radius = radius.min(max_radius);

        let mut coords: Vec<(Real, Real)> = Vec::new();

        // Top-right corner
        for i in 0..=segments {
            let theta = crate::float_types::PI / 2.0 * (i as Real) / (segments as Real);
            let x = half_width - radius + radius * theta.cos();
            let y = half_height - radius + radius * theta.sin();
            coords.push((x, y));
        }

        // Bottom-right corner
        for i in 0..=segments {
            let theta = crate::float_types::PI / 2.0
                + crate::float_types::PI / 2.0 * (i as Real) / (segments as Real);
            let x = half_width - radius + radius * theta.cos();
            let y = -half_height + radius + radius * theta.sin();
            coords.push((x, y));
        }

        // Bottom-left corner
        for i in 0..=segments {
            let theta = crate::float_types::PI
                + crate::float_types::PI / 2.0 * (i as Real) / (segments as Real);
            let x = -half_width + radius + radius * theta.cos();
            let y = -half_height + radius + radius * theta.sin();
            coords.push((x, y));
        }

        // Top-left corner
        for i in 0..=segments {
            let theta = 3.0 * crate::float_types::PI / 2.0
                + crate::float_types::PI / 2.0 * (i as Real) / (segments as Real);
            let x = -half_width + radius + radius * theta.cos();
            let y = half_height - radius + radius * theta.sin();
            coords.push((x, y));
        }

        // Close the shape
        if let Some(&first) = coords.first() {
            coords.push(first);
        }

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }
}
