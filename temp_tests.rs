//! Tests for csgrs

// Common test utilities
use crate::float_types::Real;

// Test modules
pub mod vertex_tests;
pub mod polygon_tests;
pub mod plane_tests;
pub mod bsp_tests;
pub mod csg_tests;
pub mod stl_tests;
pub mod flatten_tests;
pub mod edge_case_tests;

// --------------------------------------------------------
//   Common Helper Functions
// --------------------------------------------------------

/// Returns the approximate bounding box `[min_x, min_y, min_z, max_x, max_y, max_z]`
/// for a set of polygons.
fn bounding_box(polygons: &[Polygon<()>]) -> [Real; 6] {
    let mut min_x = Real::MAX;
    let mut min_y = Real::MAX;
    let mut min_z = Real::MAX;
    let mut max_x = Real::MIN;
    let mut max_y = Real::MIN;
    let mut max_z = Real::MIN;

    for poly in polygons {
        for v in &poly.vertices {
            let p = v.pos;
            if p.x < min_x {
                min_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.z < min_z {
                min_z = p.z;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y > max_y {
                max_y = p.y;
            }
            if p.z > max_z {
                max_z = p.z;
            }
        }
    }

    [min_x, min_y, min_z, max_x, max_y, max_z]
}

/// Quick helper to compare floating-point results with an acceptable tolerance.
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}

/// Helper to check if a value is finite (not NaN or infinite)
fn is_finite(val: Real) -> bool {
    val.is_finite()
}

/// Helper to check if a value is NaN
fn is_nan(val: Real) -> bool {
    val.is_nan()
}

/// Helper to check if a value is infinite
fn is_infinite(val: Real) -> bool {
    val.is_infinite()
}
