//! **Mathematical Foundations for 2D Primitive Generation**
//!
//! This module implements mathematically rigorous algorithms for generating
//! fundamental 2D geometric primitives based on analytic geometry and
//! computational geometry principles.
//!
//! ## **Theoretical Foundations**
//!
//! ### **Parametric Circle Generation**
//! A circle of radius r centered at origin is parameterized as:
//! ```text
//! x(θ) = r·cos(θ)
//! y(θ) = r·sin(θ)
//! where θ ∈ [0, 2π]
//! ```
//! **Discretization**: For n segments, θᵢ = 2πi/n, i ∈ [0, n-1]
//!
//! ### **Ellipse Parametric Equations**
//! An ellipse with semi-axes a and b is parameterized as:
//! ```text
//! x(θ) = a·cos(θ)
//! y(θ) = b·sin(θ)
//! where θ ∈ [0, 2π]
//! ```
//! **Geometric Properties**: Area = πab, Circumference ≈ π(3(a+b) - √((3a+b)(a+3b)))
//!
//! ### **Regular Polygon Mathematics**
//! A regular n-gon inscribed in a circle of radius r:
//! ```text
//! Vertex_i = (r·cos(2πi/n), r·sin(2πi/n))
//! Interior angle = (n-2)π/n
//! Central angle = 2π/n
//! ```
//!
//! ### **Numerical Considerations**
//! - **Angular Precision**: Uses TAU (2π) for better numerical accuracy
//! - **Closure**: Ensures polygons are properly closed for geometric operations
//! - **Degenerate Cases**: Handles n < 3 gracefully
//!
//! All shapes are generated in the XY plane (z=0) for subsequent 3D operations.

use crate::csg::CSG;
use crate::core::float_types::{PI, Real, TAU};
use geo::{
    coord, Geometry, GeometryCollection, LineString,
    Polygon as GeoPolygon, line_string,
};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Creates a 2D rectangle in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the rectangle
    /// - `length`: the height of the rectangle
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// let rect = CSG::rectangle(2.0, 3.0, None);
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

        CSG::from_geo(
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
    /// let sq2 = CSG::square(2.0, None);
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
    /// - Uses TAU (2π) constant for better floating-point precision
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
            return CSG::new();
        }
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = 2.0 * PI * (i as Real) / (segments as Real);
                (radius * theta.cos(), radius * theta.sin())
            })
            .collect();
        // close it
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real, metadata: Option<S>) -> Self {
        let line_string = LineString::new(vec![coord!{x: 0.0, y: 0.0}, coord!{x: width, y: 0.0}, coord!{x: 0.0, y: height}]);
        let polygon = GeoPolygon::new(line_string, vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon)]), metadata)
    }

    /// Creates a 2D polygon in the XY plane from a list of `[x, y]` points.
    ///
    /// # Parameters
    ///
    /// - `points`: a sequence of 2D points (e.g. `[[0.0,0.0], [1.0,0.0], [0.5,1.0]]`)
    ///   describing the polygon boundary in order.
    ///
    /// # Example
    /// let pts = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]];
    /// let poly2d = CSG::polygon(&pts, None);
    pub fn polygon(points: &[[Real; 2]], metadata: Option<S>) -> Self {
        if points.len() < 3 {
            return CSG::new();
        }
        let mut coords: Vec<(Real, Real)> = points.iter().map(|p| (p[0], p[1])).collect();
        // close
        if coords[0] != *coords.last().unwrap() {
            coords.push(coords[0]);
        }
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// **Mathematical Foundation: Parametric Ellipse Generation**
    ///
    /// Creates an ellipse in XY plane, centered at (0,0), using parametric equations.
    /// This implements the standard ellipse parameterization with uniform parameter sampling.
    ///
    /// ## **Ellipse Mathematics**
    ///
    /// ### **Parametric Representation**
    /// For an ellipse with semi-major axis a and semi-minor axis b:
    /// ```text
    /// x(θ) = a·cos(θ)
    /// y(θ) = b·sin(θ)
    /// where θ ∈ [0, 2π]
    /// ```
    /// In our implementation: a = width/2, b = height/2
    ///
    /// ### **Geometric Properties**
    /// - **Area**: A = πab = π(width·height)/4
    /// - **Circumference** (Ramanujan's approximation):
    ///   ```text
    ///   C ≈ π[3(a+b) - √((3a+b)(a+3b))]
    ///   ```
    /// - **Eccentricity**: e = √(1 - b²/a²) for a ≥ b
    /// - **Focal distance**: c = a·e where foci are at (±c, 0)
    ///
    /// ### **Parametric vs Arc-Length Parameterization**
    /// **Note**: This uses parameter-uniform sampling (constant Δθ), not
    /// arc-length uniform sampling. For arc-length uniformity, use:
    /// ```text
    /// ds/dθ = √(a²sin²θ + b²cos²θ)
    /// ```
    /// Parameter-uniform is computationally simpler and sufficient for most applications.
    ///
    /// ### **Approximation Quality**
    /// For n segments, the polygonal approximation error behaves as O(1/n²):
    /// - **Maximum radial error**: Approximately (a-b)π²/(8n²) for a ≈ b
    /// - **Area convergence**: Exact area approached as n → ∞
    ///
    /// ## **Special Cases**
    /// - **Circle**: When width = height, reduces to parametric circle
    /// - **Degenerate**: When width = 0 or height = 0, becomes a line segment
    ///
    /// # Parameters
    /// - `width`: Full width (diameter) along x-axis
    /// - `height`: Full height (diameter) along y-axis  
    /// - `segments`: Number of polygon edges (minimum 3)
    /// - `metadata`: Optional metadata
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = TAU * (i as Real) / (segments as Real);
                (rx * theta.cos(), ry * theta.sin())
            })
            .collect();
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
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
    /// - Uses TAU for precise angular calculations
    /// - Explicit closure for geometric validity
    /// - Minimum n = 3 to avoid degenerate cases
    ///
    /// # Parameters
    /// - `sides`: Number of polygon edges (≥ 3)
    /// - `radius`: Circumscribed circle radius
    /// - `metadata`: Optional metadata
    pub fn regular_ngon(sides: usize, radius: Real, metadata: Option<S>) -> Self {
        if sides < 3 {
            return CSG::new();
        }
        let mut coords: Vec<(Real, Real)> = (0..sides)
            .map(|i| {
                let theta = TAU * (i as Real) / (sides as Real);
                (radius * theta.cos(), radius * theta.sin())
            })
            .collect();
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
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
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }
} 
