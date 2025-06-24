use crate::core::float_types::Real;
use nalgebra::{Point3, Vector3};

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub pos: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl Vertex {
    /// Create a new [`Vertex`].
    ///
    /// * `pos`    – the position in model space  
    /// * `normal` – (optionally non‑unit) normal; it will be **copied
    ///              verbatim**, so make sure it is oriented the way
    ///              you need it for lighting / BSP tests.
    pub const fn new(pos: Point3<Real>, normal: Vector3<Real>) -> Self {
        Vertex { pos, normal }
    }

    /// Flip vertex normal
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// **Mathematical Foundation: Barycentric Linear Interpolation**
    ///
    /// Compute the barycentric linear interpolation between `self` (`t = 0`) and `other` (`t = 1`).
    /// This implements the fundamental linear interpolation formula:
    ///
    /// ## **Interpolation Formula**
    /// For parameter t ∈ [0,1]:
    /// - **Position**: p(t) = (1-t)·p₀ + t·p₁ = p₀ + t·(p₁ - p₀)
    /// - **Normal**: n(t) = (1-t)·n₀ + t·n₁ = n₀ + t·(n₁ - n₀)
    ///
    /// ## **Mathematical Properties**
    /// - **Affine Combination**: Coefficients sum to 1: (1-t) + t = 1
    /// - **Endpoint Preservation**: p(0) = p₀, p(1) = p₁
    /// - **Linearity**: Second derivatives are zero (straight line in parameter space)
    /// - **Convexity**: Result lies on line segment between endpoints
    ///
    /// ## **Geometric Interpretation**
    /// The interpolated vertex represents a point on the edge connecting the two vertices,
    /// with both position and normal vectors smoothly blended. This is fundamental for:
    /// - **Polygon Splitting**: Creating intersection vertices during BSP operations
    /// - **Triangle Subdivision**: Generating midpoints for mesh refinement
    /// - **Smooth Shading**: Interpolating normals across polygon edges
    ///
    /// **Note**: Normals are linearly interpolated (not spherically), which is appropriate
    /// for most geometric operations but may require renormalization for lighting calculations.
    pub fn interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        // For positions (Point3): p(t) = p0 + t * (p1 - p0)
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // For normals (Vector3): n(t) = n0 + t * (n1 - n0)
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_pos, new_normal)
    }
}
