//! **LITERATURE-BASED OPTIMIZATION**: Fast Triangle-Box Overlap Testing
//!
//! Implementation of Akenine-Möller's optimized triangle-box overlap algorithm
//! from "Fast 3D Triangle-Box Overlap Testing" (2001).
//!
//! **Performance Characteristics:**
//! - O(1) complexity per triangle-box test
//! - Branch-optimized for modern CPUs
//! - SIMD-friendly when vectorized
//! - Zero heap allocations
//!
//! **Design Principles Applied:**
//! - **KISS**: Simple, focused algorithm implementation
//! - **Zero-Cost Abstractions**: Inlined functions compile to optimal assembly
//! - **DRY**: Reusable separating axis tests
//! - **Literature-Based**: Proven algorithm from computer graphics research

use crate::float_types::Real;
use nalgebra::{Point3, Vector3};

/// **LITERATURE IMPLEMENTATION**: Akenine-Möller Triangle-Box Overlap Test
/// 
/// **Algorithm**: Uses separating axis theorem with 13 potential separating axes:
/// - 3 box face normals (x, y, z axes)
/// - 1 triangle normal
/// - 9 cross products of triangle edges with box edges
/// 
/// **Reference**: "Fast 3D Triangle-Box Overlap Testing" (Akenine-Möller, 2001)
/// Journal of Graphics Tools, Vol. 6, No. 1, pp. 29-33
#[inline(always)]
pub fn triangle_box_overlap_fast(
    triangle: &[Point3<Real>; 3],
    box_center: &Point3<Real>,
    box_half: Real,
) -> bool {
    // **OPTIMIZATION**: Translate triangle to box-centered coordinate system
    // This eliminates the need to translate the box, reducing computation
    let v0 = triangle[0] - box_center;
    let v1 = triangle[1] - box_center;
    let v2 = triangle[2] - box_center;
    
    // **OPTIMIZATION**: Precompute triangle edges
    let e0 = v1 - v0; // Edge 0: v0 -> v1
    let e1 = v2 - v1; // Edge 1: v1 -> v2
    let e2 = v0 - v2; // Edge 2: v2 -> v0
    
    // **TEST 1**: Box face normals (3 tests)
    // Test if triangle is completely outside any box face
    if !test_box_face_normals(&v0, &v1, &v2, box_half) {
        return false;
    }
    
    // **TEST 2**: Triangle normal
    // Test if box is completely on one side of triangle plane
    let triangle_normal = e0.cross(&e1);
    if !test_triangle_normal(&v0, &triangle_normal, box_half) {
        return false;
    }
    
    // **TEST 3**: Cross products of triangle edges with box edges (9 tests)
    // These are the most expensive tests, so we do them last
    if !test_edge_cross_products(&v0, &v1, &v2, &e0, &e1, &e2, box_half) {
        return false;
    }
    
    // All separating axis tests passed - triangle and box overlap
    true
}

/// **OPTIMIZATION**: Test box face normals (X, Y, Z axes)
/// 
/// **Performance**: Branchless implementation using min/max operations
#[inline(always)]
fn test_box_face_normals(
    v0: &Vector3<Real>,
    v1: &Vector3<Real>,
    v2: &Vector3<Real>,
    box_half: Real,
) -> bool {
    // X-axis test
    let min_x = v0.x.min(v1.x).min(v2.x);
    let max_x = v0.x.max(v1.x).max(v2.x);
    if min_x > box_half || max_x < -box_half {
        return false;
    }
    
    // Y-axis test
    let min_y = v0.y.min(v1.y).min(v2.y);
    let max_y = v0.y.max(v1.y).max(v2.y);
    if min_y > box_half || max_y < -box_half {
        return false;
    }
    
    // Z-axis test
    let min_z = v0.z.min(v1.z).min(v2.z);
    let max_z = v0.z.max(v1.z).max(v2.z);
    if min_z > box_half || max_z < -box_half {
        return false;
    }
    
    true
}

/// **OPTIMIZATION**: Test triangle normal as separating axis
/// 
/// **Algorithm**: Project box onto triangle normal and check for separation
#[inline(always)]
fn test_triangle_normal(
    v0: &Vector3<Real>,
    triangle_normal: &Vector3<Real>,
    box_half: Real,
) -> bool {
    // Distance from box center to triangle plane
    let plane_distance = triangle_normal.dot(v0);
    
    // Project box onto triangle normal (half-extent of projection)
    let box_projection = box_half * (
        triangle_normal.x.abs() + 
        triangle_normal.y.abs() + 
        triangle_normal.z.abs()
    );
    
    // Check for separation
    plane_distance.abs() <= box_projection
}

/// **OPTIMIZATION**: Test cross products of triangle edges with box edges
/// 
/// **Performance**: Most expensive part of algorithm, optimized for branch prediction
#[inline(always)]
fn test_edge_cross_products(
    v0: &Vector3<Real>,
    v1: &Vector3<Real>,
    v2: &Vector3<Real>,
    e0: &Vector3<Real>,
    e1: &Vector3<Real>,
    e2: &Vector3<Real>,
    box_half: Real,
) -> bool {
    // **OPTIMIZATION**: Test axes in order of likely early termination
    
    // Edge 0 (v0->v1) cross box edges
    if !test_axis_e0_x(v0, v2, e0, box_half) { return false; }
    if !test_axis_e0_y(v0, v2, e0, box_half) { return false; }
    if !test_axis_e0_z(v0, v2, e0, box_half) { return false; }
    
    // Edge 1 (v1->v2) cross box edges  
    if !test_axis_e1_x(v0, v1, e1, box_half) { return false; }
    if !test_axis_e1_y(v0, v1, e1, box_half) { return false; }
    if !test_axis_e1_z(v0, v1, e1, box_half) { return false; }
    
    // Edge 2 (v2->v0) cross box edges
    if !test_axis_e2_x(v1, v2, e2, box_half) { return false; }
    if !test_axis_e2_y(v1, v2, e2, box_half) { return false; }
    if !test_axis_e2_z(v1, v2, e2, box_half) { return false; }
    
    true
}

/// **MICRO-OPTIMIZATION**: Specialized axis tests for each edge-box combination
/// 
/// **Design Rationale**: Separate functions allow compiler to optimize each case
/// independently and improve branch prediction.

#[inline(always)]
fn test_axis_e0_x(v0: &Vector3<Real>, v2: &Vector3<Real>, e0: &Vector3<Real>, box_half: Real) -> bool {
    let p0 = e0.z * v0.y - e0.y * v0.z;
    let p2 = e0.z * v2.y - e0.y * v2.z;
    let min_p = p0.min(p2);
    let max_p = p0.max(p2);
    let radius = box_half * (e0.y.abs() + e0.z.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e0_y(v0: &Vector3<Real>, v2: &Vector3<Real>, e0: &Vector3<Real>, box_half: Real) -> bool {
    let p0 = -e0.z * v0.x + e0.x * v0.z;
    let p2 = -e0.z * v2.x + e0.x * v2.z;
    let min_p = p0.min(p2);
    let max_p = p0.max(p2);
    let radius = box_half * (e0.x.abs() + e0.z.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e0_z(v0: &Vector3<Real>, v2: &Vector3<Real>, e0: &Vector3<Real>, box_half: Real) -> bool {
    let p0 = e0.y * v0.x - e0.x * v0.y;
    let p2 = e0.y * v2.x - e0.x * v2.y;
    let min_p = p0.min(p2);
    let max_p = p0.max(p2);
    let radius = box_half * (e0.x.abs() + e0.y.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e1_x(v0: &Vector3<Real>, v1: &Vector3<Real>, e1: &Vector3<Real>, box_half: Real) -> bool {
    let p0 = e1.z * v0.y - e1.y * v0.z;
    let p1 = e1.z * v1.y - e1.y * v1.z;
    let min_p = p0.min(p1);
    let max_p = p0.max(p1);
    let radius = box_half * (e1.y.abs() + e1.z.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e1_y(v0: &Vector3<Real>, v1: &Vector3<Real>, e1: &Vector3<Real>, box_half: Real) -> bool {
    let p0 = -e1.z * v0.x + e1.x * v0.z;
    let p1 = -e1.z * v1.x + e1.x * v1.z;
    let min_p = p0.min(p1);
    let max_p = p0.max(p1);
    let radius = box_half * (e1.x.abs() + e1.z.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e1_z(v0: &Vector3<Real>, v1: &Vector3<Real>, e1: &Vector3<Real>, box_half: Real) -> bool {
    let p0 = e1.y * v0.x - e1.x * v0.y;
    let p1 = e1.y * v1.x - e1.x * v1.y;
    let min_p = p0.min(p1);
    let max_p = p0.max(p1);
    let radius = box_half * (e1.x.abs() + e1.y.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e2_x(v1: &Vector3<Real>, v2: &Vector3<Real>, e2: &Vector3<Real>, box_half: Real) -> bool {
    let p1 = e2.z * v1.y - e2.y * v1.z;
    let p2 = e2.z * v2.y - e2.y * v2.z;
    let min_p = p1.min(p2);
    let max_p = p1.max(p2);
    let radius = box_half * (e2.y.abs() + e2.z.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e2_y(v1: &Vector3<Real>, v2: &Vector3<Real>, e2: &Vector3<Real>, box_half: Real) -> bool {
    let p1 = -e2.z * v1.x + e2.x * v1.z;
    let p2 = -e2.z * v2.x + e2.x * v2.z;
    let min_p = p1.min(p2);
    let max_p = p1.max(p2);
    let radius = box_half * (e2.x.abs() + e2.z.abs());
    min_p <= radius && max_p >= -radius
}

#[inline(always)]
fn test_axis_e2_z(v1: &Vector3<Real>, v2: &Vector3<Real>, e2: &Vector3<Real>, box_half: Real) -> bool {
    let p1 = e2.y * v1.x - e2.x * v1.y;
    let p2 = e2.y * v2.x - e2.x * v2.y;
    let min_p = p1.min(p2);
    let max_p = p1.max(p2);
    let radius = box_half * (e2.x.abs() + e2.y.abs());
    min_p <= radius && max_p >= -radius
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_triangle_box_overlap_basic() {
        // Triangle inside box
        let triangle = [
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(0.2, 0.1, 0.1),
            Point3::new(0.1, 0.2, 0.1),
        ];
        let box_center = Point3::origin();
        let box_half = 1.0;
        
        assert!(triangle_box_overlap_fast(&triangle, &box_center, box_half));
    }
    
    #[test]
    fn test_triangle_box_no_overlap() {
        // Triangle completely outside box
        let triangle = [
            Point3::new(2.0, 2.0, 2.0),
            Point3::new(3.0, 2.0, 2.0),
            Point3::new(2.0, 3.0, 2.0),
        ];
        let box_center = Point3::origin();
        let box_half = 1.0;
        
        assert!(!triangle_box_overlap_fast(&triangle, &box_center, box_half));
    }
    
    #[test]
    fn test_triangle_box_edge_case() {
        // Triangle intersecting box boundary
        let triangle = [
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(1.5, -0.5, -0.5),
            Point3::new(0.5, 1.5, -0.5),
        ];
        let box_center = Point3::origin();
        let box_half = 1.0;
        
        assert!(triangle_box_overlap_fast(&triangle, &box_center, box_half));
    }
}
