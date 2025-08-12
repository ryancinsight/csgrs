//! **LITERATURE-BASED VALIDATION**: CSG Operation Property Testing
//!
//! This module implements comprehensive validation tests based on established
//! literature in computational geometry and solid modeling.
//!
//! **Primary References:**
//! - Requicha & Voelcker (1982): "Solid Modeling: A Historical Summary and Contemporary Assessment"
//! - Hoffmann (1989): "Geometric and Solid Modeling: An Introduction"
//! - Mantyla (1988): "An Introduction to Solid Modeling"
//!
//! **Design Principles Applied:**
//! - **Literature-Based**: All tests derived from established mathematical properties
//! - **Comprehensive**: Covers algebraic properties, geometric invariants, and edge cases
//! - **Automated**: Can be run as part of CI/CD pipeline for regression testing
//! - **Quantitative**: Uses numerical metrics for validation

use crate::float_types::Real;
use crate::voxels::csg::Voxels;
use crate::voxels::polygon::Polygon;
use crate::traits::CSG;
use nalgebra::Point3;
use std::fmt::Debug;

/// **LITERATURE VALIDATION**: CSG algebraic properties
/// 
/// **Reference**: Requicha & Voelcker (1982) - "Boolean Operations in Solid Modeling"
/// These properties must hold for any correct CSG implementation.
pub struct CsgAlgebraicValidator;

impl CsgAlgebraicValidator {
    /// **PROPERTY**: Commutativity of Union and Intersection
    /// 
    /// **Mathematical Basis**: A ∪ B = B ∪ A, A ∩ B = B ∩ A
    /// **Tolerance**: Accounts for floating-point precision and discretization errors
    pub fn test_commutativity<S: Clone + Debug + Send + Sync>(
        a: &Voxels<S>,
        b: &Voxels<S>,
        tolerance: Real,
    ) -> ValidationResult {
        let ab_union = a.union(b);
        let ba_union = b.union(a);
        let union_error = Self::compute_symmetric_difference(&ab_union, &ba_union);
        
        let ab_intersection = a.intersection(b);
        let ba_intersection = b.intersection(a);
        let intersection_error = Self::compute_symmetric_difference(&ab_intersection, &ba_intersection);
        
        ValidationResult {
            property: "Commutativity".to_string(),
            passed: union_error < tolerance && intersection_error < tolerance,
            error_magnitude: union_error.max(intersection_error),
            details: format!(
                "Union error: {:.6}, Intersection error: {:.6}",
                union_error, intersection_error
            ),
        }
    }
    
    /// **PROPERTY**: Associativity of Union and Intersection
    /// 
    /// **Mathematical Basis**: (A ∪ B) ∪ C = A ∪ (B ∪ C), (A ∩ B) ∩ C = A ∩ (B ∩ C)
    pub fn test_associativity<S: Clone + Debug + Send + Sync>(
        a: &Voxels<S>,
        b: &Voxels<S>,
        c: &Voxels<S>,
        tolerance: Real,
    ) -> ValidationResult {
        // Test union associativity: (A ∪ B) ∪ C = A ∪ (B ∪ C)
        let left_union = a.union(b).union(c);
        let right_union = a.union(&b.union(c));
        let union_error = Self::compute_symmetric_difference(&left_union, &right_union);
        
        // Test intersection associativity: (A ∩ B) ∩ C = A ∩ (B ∩ C)
        let left_intersection = a.intersection(b).intersection(c);
        let right_intersection = a.intersection(&b.intersection(c));
        let intersection_error = Self::compute_symmetric_difference(&left_intersection, &right_intersection);
        
        ValidationResult {
            property: "Associativity".to_string(),
            passed: union_error < tolerance && intersection_error < tolerance,
            error_magnitude: union_error.max(intersection_error),
            details: format!(
                "Union error: {:.6}, Intersection error: {:.6}",
                union_error, intersection_error
            ),
        }
    }
    
    /// **PROPERTY**: Distributivity Laws
    /// 
    /// **Mathematical Basis**: 
    /// - A ∪ (B ∩ C) = (A ∪ B) ∩ (A ∪ C)
    /// - A ∩ (B ∪ C) = (A ∩ B) ∪ (A ∩ C)
    pub fn test_distributivity<S: Clone + Debug + Send + Sync>(
        a: &Voxels<S>,
        b: &Voxels<S>,
        c: &Voxels<S>,
        tolerance: Real,
    ) -> ValidationResult {
        // Test: A ∪ (B ∩ C) = (A ∪ B) ∩ (A ∪ C)
        let left_dist1 = a.union(&b.intersection(c));
        let right_dist1 = a.union(b).intersection(&a.union(c));
        let dist1_error = Self::compute_symmetric_difference(&left_dist1, &right_dist1);
        
        // Test: A ∩ (B ∪ C) = (A ∩ B) ∪ (A ∩ C)
        let left_dist2 = a.intersection(&b.union(c));
        let right_dist2 = a.intersection(b).union(&a.intersection(c));
        let dist2_error = Self::compute_symmetric_difference(&left_dist2, &right_dist2);
        
        ValidationResult {
            property: "Distributivity".to_string(),
            passed: dist1_error < tolerance && dist2_error < tolerance,
            error_magnitude: dist1_error.max(dist2_error),
            details: format!(
                "Distribution 1 error: {:.6}, Distribution 2 error: {:.6}",
                dist1_error, dist2_error
            ),
        }
    }
    
    /// **PROPERTY**: De Morgan's Laws
    /// 
    /// **Mathematical Basis**: 
    /// - ¬(A ∪ B) = ¬A ∩ ¬B
    /// - ¬(A ∩ B) = ¬A ∪ ¬B
    pub fn test_de_morgan_laws<S: Clone + Debug + Send + Sync>(
        a: &Voxels<S>,
        b: &Voxels<S>,
        tolerance: Real,
    ) -> ValidationResult {
        // Test: ¬(A ∪ B) = ¬A ∩ ¬B
        let left_morgan1 = a.union(b).inverse();
        let right_morgan1 = a.inverse().intersection(&b.inverse());
        let morgan1_error = Self::compute_symmetric_difference(&left_morgan1, &right_morgan1);
        
        // Test: ¬(A ∩ B) = ¬A ∪ ¬B
        let left_morgan2 = a.intersection(b).inverse();
        let right_morgan2 = a.inverse().union(&b.inverse());
        let morgan2_error = Self::compute_symmetric_difference(&left_morgan2, &right_morgan2);
        
        ValidationResult {
            property: "De Morgan's Laws".to_string(),
            passed: morgan1_error < tolerance && morgan2_error < tolerance,
            error_magnitude: morgan1_error.max(morgan2_error),
            details: format!(
                "De Morgan 1 error: {:.6}, De Morgan 2 error: {:.6}",
                morgan1_error, morgan2_error
            ),
        }
    }
    
    /// **METRIC**: Compute symmetric difference between two voxel structures
    /// 
    /// **Algorithm**: |A ⊕ B| / |A ∪ B| where ⊕ is symmetric difference
    /// **Range**: [0, 1] where 0 = identical, 1 = completely different
    fn compute_symmetric_difference<S: Clone + Debug + Send + Sync>(
        a: &Voxels<S>,
        b: &Voxels<S>,
    ) -> Real {
        // Compute A ⊕ B = (A - B) ∪ (B - A)
        let a_minus_b = a.difference(b);
        let b_minus_a = b.difference(a);
        let symmetric_diff = a_minus_b.union(&b_minus_a);
        
        // Compute A ∪ B
        let union = a.union(b);
        
        // Estimate volumes using polygon count as proxy
        let sym_diff_volume = symmetric_diff.polygons().len() as Real;
        let union_volume = union.polygons().len() as Real;
        
        if union_volume == 0.0 {
            if sym_diff_volume == 0.0 { 0.0 } else { 1.0 }
        } else {
            sym_diff_volume / union_volume
        }
    }
}

/// **LITERATURE VALIDATION**: Surface quality metrics
/// 
/// **Reference**: Hoppe et al. (1992) - "Surface Reconstruction from Unorganized Points"
/// Validates that CSG operations maintain surface quality and don't introduce artifacts.
pub struct SurfaceQualityValidator;

impl SurfaceQualityValidator {
    /// **METRIC**: Hausdorff distance between surfaces
    /// 
    /// **Definition**: Maximum distance from any point on surface A to closest point on surface B
    /// **Use Case**: Validates that CSG operations don't introduce significant surface deviation
    pub fn compute_hausdorff_distance<S: Clone + Debug + Send + Sync>(
        surface_a: &[Polygon<S>],
        surface_b: &[Polygon<S>],
    ) -> Real {
        if surface_a.is_empty() || surface_b.is_empty() {
            return Real::INFINITY;
        }
        
        let mut max_distance: Real = 0.0;
        
        // Sample points from surface A and find closest points on surface B
        for poly_a in surface_a {
            for vertex in &poly_a.vertices {
                let point_a = &vertex.pos;
                let mut min_distance = Real::INFINITY;

                // Find closest point on surface B
                for poly_b in surface_b {
                    let distance = Self::point_to_polygon_distance(point_a, poly_b);
                    min_distance = min_distance.min(distance);
                }

                max_distance = max_distance.max(min_distance);
            }
        }
        
        max_distance
    }
    
    /// **METRIC**: Triangle quality distribution
    /// 
    /// **Definition**: Distribution of triangle aspect ratios and angles
    /// **Use Case**: Ensures CSG operations don't create degenerate triangles
    pub fn compute_triangle_quality_distribution<S: Clone + Debug + Send + Sync>(
        surface: &[Polygon<S>],
    ) -> TriangleQualityMetrics {
        let mut aspect_ratios = Vec::new();
        let mut min_angles = Vec::new();
        let mut degenerate_count = 0;
        
        for polygon in surface {
            // Triangulate polygon and analyze each triangle
            for triangle in polygon.triangulate() {
                if triangle.len() != 3 {
                    continue;
                }
                
                let v0 = &triangle[0].pos;
                let v1 = &triangle[1].pos;
                let v2 = &triangle[2].pos;
                
                // Compute edge lengths
                let e0 = (v1 - v0).norm();
                let e1 = (v2 - v1).norm();
                let e2 = (v0 - v2).norm();
                
                // Check for degeneracy
                if e0 < 1e-10 || e1 < 1e-10 || e2 < 1e-10 {
                    degenerate_count += 1;
                    continue;
                }
                
                // Compute aspect ratio (longest edge / shortest edge)
                let max_edge = e0.max(e1).max(e2);
                let min_edge = e0.min(e1).min(e2);
                aspect_ratios.push(max_edge / min_edge);
                
                // Compute minimum angle using law of cosines
                let cos_angle_0 = (e1 * e1 + e2 * e2 - e0 * e0) / (2.0 * e1 * e2);
                let cos_angle_1 = (e0 * e0 + e2 * e2 - e1 * e1) / (2.0 * e0 * e2);
                let cos_angle_2 = (e0 * e0 + e1 * e1 - e2 * e2) / (2.0 * e0 * e1);
                
                let angle_0 = cos_angle_0.clamp(-1.0, 1.0).acos();
                let angle_1 = cos_angle_1.clamp(-1.0, 1.0).acos();
                let angle_2 = cos_angle_2.clamp(-1.0, 1.0).acos();
                
                min_angles.push(angle_0.min(angle_1).min(angle_2));
            }
        }
        
        TriangleQualityMetrics {
            mean_aspect_ratio: aspect_ratios.iter().sum::<Real>() / aspect_ratios.len() as Real,
            max_aspect_ratio: aspect_ratios.iter().fold(0.0, |a, &b| a.max(b)),
            mean_min_angle: min_angles.iter().sum::<Real>() / min_angles.len() as Real,
            min_angle: min_angles.iter().fold(std::f64::consts::PI, |a, &b| a.min(b)),
            degenerate_triangle_count: degenerate_count,
            total_triangle_count: aspect_ratios.len() + degenerate_count,
        }
    }
    
    /// Helper: Compute distance from point to polygon
    fn point_to_polygon_distance<S: Clone + Debug + Send + Sync>(
        point: &Point3<Real>,
        polygon: &Polygon<S>,
    ) -> Real {
        // Simplified implementation - distance to closest vertex
        // A full implementation would compute distance to polygon surface
        polygon.vertices
            .iter()
            .map(|v| (point - &v.pos).norm())
            .fold(Real::INFINITY, |a, b| a.min(b))
    }
}

/// Validation result for a single property test
#[derive(Debug, Clone)]
pub struct ValidationResult {
    pub property: String,
    pub passed: bool,
    pub error_magnitude: Real,
    pub details: String,
}

/// Triangle quality metrics for surface validation
#[derive(Debug, Clone)]
pub struct TriangleQualityMetrics {
    pub mean_aspect_ratio: Real,
    pub max_aspect_ratio: Real,
    pub mean_min_angle: Real,
    pub min_angle: Real,
    pub degenerate_triangle_count: usize,
    pub total_triangle_count: usize,
}

impl TriangleQualityMetrics {
    /// Check if triangle quality meets acceptable standards
    pub fn is_acceptable(&self) -> bool {
        self.max_aspect_ratio < 10.0 &&           // No extremely elongated triangles
        self.mean_aspect_ratio < 3.0 &&           // Good average triangle shape
        self.min_angle > 0.1 &&                   // No extremely acute angles (< ~5.7°)
        self.degenerate_triangle_count == 0       // No degenerate triangles
    }
}
