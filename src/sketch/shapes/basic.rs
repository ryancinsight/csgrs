//! Basic geometric shapes for Sketch
//!
//! This module provides fundamental 2D geometric shapes with comprehensive
//! mathematical documentation and robust error handling.

use crate::float_types::Real;
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{Geometry, GeometryCollection, LineString, Polygon as GeoPolygon, line_string};

use std::fmt::Debug;

/// Basic geometric shape implementations
impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Creates a 2D rectangle in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the rectangle
    /// - `length`: the height of the rectangle
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// ```
    /// use csgrs::sketch::Sketch;
    /// let sq2 = Sketch::<()>::rectangle(2.0, 3.0, None);
    /// ```
    pub fn rectangle(width: Real, length: Real, metadata: Option<S>) -> Self {
        // In geo, a Polygon is basically (outer: LineString, Vec<LineString> for holes).
        let outer = line_string![
            (x: 0.0,     y: 0.0),
            (x: width,   y: 0.0),
            (x: width,   y: length),
            (x: 0.0,     y: length),
            (x: 0.0,     y: 0.0),  // close explicitly
        ];
        let polygon_2d = GeoPolygon::new(outer, vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width=length of the square
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// let sq2 = Sketch::square(2.0, None);
    pub fn square(width: Real, metadata: Option<S>) -> Self {
        Self::rectangle(width, width, metadata)
    }

    /// **Mathematical Foundation: Parametric Circle Discretization**
    ///
    /// Creates a 2D circle in the XY plane using parametric equations.
    /// This implements the standard circle parameterization with uniform angular sampling.
    ///
    /// ## **Circle Mathematics**
    ///
    /// ### **Parametric Representation**
    /// For a circle of radius r centered at origin:
    /// ```text
    /// x(θ) = r·cos(θ)
    /// y(θ) = r·sin(θ)
    /// where θ ∈ [0, 2π]
    /// ```
    ///
    /// ### **Discretization Algorithm**
    /// For n segments, sample at angles:
    /// ```text
    /// θᵢ = 2πi/n, i ∈ {0, 1, ..., n-1}
    /// ```
    /// This produces n vertices uniformly distributed around the circle.
    ///
    /// ### **Approximation Error**
    /// The polygonal approximation has:
    /// - **Maximum radial error**: r(1 - cos(π/n)) ≈ r(π/n)²/8 for large n
    /// - **Perimeter error**: 2πr - n·r·sin(π/n) ≈ πr/3n² for large n
    /// - **Area error**: πr² - (nr²sin(2π/n))/2 ≈ πr³/6n² for large n
    ///
    /// ### **Numerical Stability**
    /// - Uses Real::crate::float_types::TAU (2π) constant for better floating-point precision
    /// - Explicit closure ensures geometric validity
    /// - Minimum 3 segments to avoid degenerate polygons
    ///
    /// ## **Applications**
    /// - **Geometric modeling**: Base shape for 3D extrusion
    /// - **Collision detection**: Circular boundaries
    /// - **Numerical integration**: Circular domains
    ///
    /// # Parameters
    /// - `radius`: Circle radius (must be > 0)
    /// - `segments`: Number of polygon edges (minimum 3 for valid geometry)
    /// - `metadata`: Optional metadata attached to the shape
    pub fn circle(radius: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return Sketch::new();
        }
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = 2.0 * crate::float_types::PI * (i as Real) / (segments as Real);
                (radius * theta.cos(), radius * theta.sin())
            })
            .collect();
        // close it
        coords.push((coords[0].0, coords[0].1));
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Creates a 2D right triangle in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the triangle (base)
    /// - `height`: the height of the triangle
    /// - `metadata`: optional metadata
    pub fn right_triangle(width: Real, height: Real, metadata: Option<S>) -> Self {
        let outer = line_string![
            (x: 0.0,   y: 0.0),
            (x: width, y: 0.0),
            (x: 0.0,   y: height),
            (x: 0.0,   y: 0.0),  // close explicitly
        ];
        let polygon_2d = GeoPolygon::new(outer, vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Creates a 2D polygon from a list of points.
    ///
    /// # Parameters
    ///
    /// - `points`: Array of [x, y] coordinate pairs
    /// - `metadata`: optional metadata
    pub fn polygon(points: &[[Real; 2]], metadata: Option<S>) -> Self {
        if points.len() < 3 {
            return Sketch::new();
        }

        let coords: Vec<(Real, Real)> = points.iter().map(|&[x, y]| (x, y)).collect();

        let mut coords = coords;
        // Ensure polygon is closed
        if coords.first() != coords.last() {
            if let Some(&first) = coords.first() {
                coords.push(first);
            }
        }

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }
}
