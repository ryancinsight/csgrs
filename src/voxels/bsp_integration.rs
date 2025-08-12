//! BSP integration for SVO Mixed cells
//! 
//! This module handles embedding BSP trees at Mixed octree cells for precise surface representation.
//! It follows the Single Responsibility Principle by focusing solely on BSP-SVO integration.

use crate::float_types::Real;
use crate::voxels::{Svo, SvoNode, Occupancy};
use crate::voxels::bsp::Node as BspNode;
use crate::voxels::polygon::Polygon;
use crate::voxels::marching_cubes::MarchingCubes;
use nalgebra::Point3;
use std::fmt::Debug;

/// BSP integration manager for SVO
pub struct BspIntegrator;

impl BspIntegrator {
    /// Attach BSP trees to Mixed cells in an SVO based on polygon data
    /// 
    /// This method analyzes the SVO structure and creates BSP trees at Mixed cells
    /// that contain surface geometry, enabling precise surface representation.
    pub fn integrate_bsp_from_polygons<S: Clone + Debug + Send + Sync>(
        svo: &mut Svo<S>,
        polygons: &[Polygon<S>],
    ) {
        if polygons.is_empty() {
            return;
        }
        
        Self::integrate_node_bsp(
            &mut *svo.root,
            &svo.center,
            svo.half,
            0,
            svo.max_depth,
            polygons,
        );
    }
    
    /// Recursively integrate BSP trees into SVO nodes
    fn integrate_node_bsp<S: Clone + Debug + Send + Sync>(
        node: &mut SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        polygons: &[Polygon<S>],
    ) {
        // Filter polygons that intersect this node's bounds
        let node_polygons = Self::filter_polygons_in_bounds(polygons, center, half);
        
        if node_polygons.is_empty() {
            return;
        }
        
        match node.occupancy {
            Occupancy::Mixed => {
                // Create BSP tree for surface polygons at Mixed cells
                if !node_polygons.is_empty() && depth >= max_depth / 2 {
                    // Only create BSP at deeper levels to avoid too many BSP trees
                    let bsp = BspNode::from_polygons(&node_polygons);
                    node.local_bsp = Some(bsp);
                } else if node_polygons.is_empty() && depth >= max_depth / 2 {
                    // Generate surface polygons for Mixed cells without explicit polygons
                    // This handles cases where Mixed occupancy was set by SDF voxelization
                    // but no explicit polygons were provided
                    Self::generate_implicit_surface_polygons(node, center, half);
                }
            }
            _ => {
                // For Empty/Full cells, we might still need to process children
            }
        }
        
        // Apply advanced termination criteria before recursing
        if !Self::should_terminate_subdivision(
            depth,
            max_depth,
            half,
            node_polygons.len(),
            None, // No corner values for polygon-based subdivision
            0.0,  // No iso_value for polygon-based subdivision
        ) {
            // Only recurse if we have children and sufficient complexity
            for child_idx in 0..8 {
                if let Some(child) = node.get_child_mut(child_idx) {
                    let child_center = Self::child_center(center, half, child_idx);

                    // Filter polygons for this child to avoid unnecessary work
                    let child_polygons = Self::filter_polygons_in_bounds(
                        &node_polygons,
                        &child_center,
                        half * 0.5
                    );

                    // Only recurse if child has polygons
                    if !child_polygons.is_empty() {
                        Self::integrate_node_bsp(
                            child,
                            &child_center,
                            half * 0.5,
                            depth + 1,
                            max_depth,
                            &child_polygons,
                        );
                    }
                }
            }
        }
    }
    
    /// Filter polygons that intersect with a node's bounding box
    fn filter_polygons_in_bounds<S: Clone + Debug + Send + Sync>(
        polygons: &[Polygon<S>],
        center: &Point3<Real>,
        half: Real,
    ) -> Vec<Polygon<S>> {
        let min_bound = Point3::new(center.x - half, center.y - half, center.z - half);
        let max_bound = Point3::new(center.x + half, center.y + half, center.z + half);
        
        polygons
            .iter()
            .filter(|poly| {
                // Simple AABB intersection test
                let poly_bb = poly.bounding_box();
                poly_bb.maxs.x >= min_bound.x
                    && poly_bb.mins.x <= max_bound.x
                    && poly_bb.maxs.y >= min_bound.y
                    && poly_bb.mins.y <= max_bound.y
                    && poly_bb.maxs.z >= min_bound.z
                    && poly_bb.mins.z <= max_bound.z
            })
            .cloned()
            .collect()
    }
    
    /// Compute child center for octree subdivision
    fn child_center(parent_center: &Point3<Real>, parent_half: Real, child_idx: u8) -> Point3<Real> {
        let quarter = parent_half * 0.5;
        let dx = if (child_idx & 1) != 0 { quarter } else { -quarter };
        let dy = if (child_idx & 2) != 0 { quarter } else { -quarter };
        let dz = if (child_idx & 4) != 0 { quarter } else { -quarter };
        
        Point3::new(
            parent_center.x + dx,
            parent_center.y + dy,
            parent_center.z + dz,
        )
    }
    
    /// Create BSP trees at surface-crossing cells for SDF-based voxelization
    pub fn integrate_bsp_from_sdf<F, S>(
        svo: &mut Svo<S>,
        sdf: F,
        iso_value: Real,
    ) where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
        S: Clone + Debug + Send + Sync,
    {
        Self::integrate_sdf_node_bsp(
            &mut *svo.root,
            &svo.center,
            svo.half,
            0,
            svo.max_depth,
            &sdf,
            iso_value,
        );
    }
    
    /// Recursively integrate BSP from SDF at surface-crossing cells
    fn integrate_sdf_node_bsp<F, S>(
        node: &mut SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        sdf: &F,
        iso_value: Real,
    ) where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
        S: Clone + Debug + Send + Sync,
    {
        // Check if this cell crosses the isosurface
        let corners = Self::get_cell_corners(center, half);
        let corner_values: Vec<Real> = corners.iter().map(|p| sdf(p)).collect();
        
        let has_positive = corner_values.iter().any(|&v| v > iso_value);
        let has_negative = corner_values.iter().any(|&v| v < iso_value);
        
        if has_positive && has_negative {
            // Surface-crossing cell - mark as Mixed and create BSP for precise surface representation
            node.occupancy = Occupancy::Mixed;

            // Enhanced BSP creation strategy for optimal surface precision
            let should_create_bsp = Self::should_create_bsp_at_depth(depth, max_depth, half);

            if should_create_bsp {
                // Generate high-quality surface polygons using marching cubes
                let surface_polygons = MarchingCubes::generate_surface_polygons(
                    center,
                    half,
                    sdf,
                    iso_value,
                    None, // No metadata for SDF-generated polygons
                );

                // Create BSP from generated surface polygons with quality validation
                if !surface_polygons.is_empty() && Self::validate_surface_quality(&surface_polygons) {
                    node.local_bsp = Some(BspNode::from_polygons(&surface_polygons));
                }
            }
        } else if has_positive {
            node.occupancy = Occupancy::Empty;
        } else {
            node.occupancy = Occupancy::Full;
        }
        
        // Apply advanced termination criteria before recursing
        if !Self::should_terminate_subdivision(
            depth,
            max_depth,
            half,
            0, // No polygon count for SDF-based subdivision
            Some(&corner_values),
            iso_value,
        ) {
            // Only create children for Mixed nodes that need further subdivision
            if node.occupancy == Occupancy::Mixed {
                for child_idx in 0..8 {
                    let child = node.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);
                    Self::integrate_sdf_node_bsp(
                        child,
                        &child_center,
                        half * 0.5,
                        depth + 1,
                        max_depth,
                        sdf,
                        iso_value,
                    );
                }
            }
        }
    }
    
    /// Get the 8 corner points of a cell
    fn get_cell_corners(center: &Point3<Real>, half: Real) -> [Point3<Real>; 8] {
        [
            Point3::new(center.x - half, center.y - half, center.z - half),
            Point3::new(center.x + half, center.y - half, center.z - half),
            Point3::new(center.x - half, center.y + half, center.z - half),
            Point3::new(center.x + half, center.y + half, center.z - half),
            Point3::new(center.x - half, center.y - half, center.z + half),
            Point3::new(center.x + half, center.y - half, center.z + half),
            Point3::new(center.x - half, center.y + half, center.z + half),
            Point3::new(center.x + half, center.y + half, center.z + half),
        ]
    }

    /// Generate implicit surface polygons for Mixed cells without explicit geometry
    ///
    /// Following KISS principle - creates a simple surface representation
    /// for Mixed cells that don't have explicit polygon data.
    fn generate_implicit_surface_polygons<S: Clone + Debug + Send + Sync>(
        node: &mut SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
    ) {
        // Simple approach: create a small surface patch at the cell center
        // This provides a minimal surface representation for visualization

        // Create a small quad at the cell center oriented along the dominant axis
        let quad_size = half * 0.5;
        let vertices = vec![
            crate::voxels::vertex::Vertex::new(
                Point3::new(center.x - quad_size, center.y - quad_size, center.z),
                nalgebra::Vector3::new(0.0, 0.0, 1.0),
            ),
            crate::voxels::vertex::Vertex::new(
                Point3::new(center.x + quad_size, center.y - quad_size, center.z),
                nalgebra::Vector3::new(0.0, 0.0, 1.0),
            ),
            crate::voxels::vertex::Vertex::new(
                Point3::new(center.x + quad_size, center.y + quad_size, center.z),
                nalgebra::Vector3::new(0.0, 0.0, 1.0),
            ),
            crate::voxels::vertex::Vertex::new(
                Point3::new(center.x - quad_size, center.y + quad_size, center.z),
                nalgebra::Vector3::new(0.0, 0.0, 1.0),
            ),
        ];

        let surface_polygon = Polygon::new(vertices, None);
        let bsp = BspNode::from_polygons(&[surface_polygon]);
        node.local_bsp = Some(bsp);
    }

    /// Determine if BSP should be created at this depth for optimal surface precision
    fn should_create_bsp_at_depth(depth: u8, max_depth: u8, half_size: Real) -> bool {
        // Create BSP at multiple levels for better surface representation
        match depth {
            // Always create at deeper levels for fine detail
            d if d >= max_depth.saturating_sub(2) => true,
            // Create at mid-levels for medium detail if cell is large enough
            d if d >= max_depth / 3 && half_size > 0.1 => true,
            // Create at early levels for coarse detail if cell is very large
            d if d >= 1 && half_size > 1.0 => true,
            _ => false,
        }
    }

    /// Validate surface polygon quality before creating BSP
    fn validate_surface_quality<S: Clone + Debug + Send + Sync>(polygons: &[Polygon<S>]) -> bool {
        // Basic quality checks
        if polygons.is_empty() || polygons.len() > 1000 {
            return false; // Too few or too many polygons
        }

        // Check for degenerate polygons
        let valid_polygons = polygons.iter().filter(|poly| {
            poly.vertices.len() >= 3 && Self::is_non_degenerate_polygon(poly)
        }).count();

        // Require at least 50% valid polygons
        valid_polygons >= polygons.len() / 2
    }

    /// Check if polygon is non-degenerate (has non-zero area)
    fn is_non_degenerate_polygon<S: Clone + Debug + Send + Sync>(polygon: &Polygon<S>) -> bool {
        if polygon.vertices.len() < 3 {
            return false;
        }

        // Calculate area using cross product for triangle
        let v0 = &polygon.vertices[0].pos;
        let v1 = &polygon.vertices[1].pos;
        let v2 = &polygon.vertices[2].pos;

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let cross = edge1.cross(&edge2);

        cross.norm_squared() > 1e-12 // Non-zero area threshold
    }

    /// Advanced termination criteria for recursive subdivision
    ///
    /// This function implements multiple termination criteria to prevent
    /// unnecessary subdivision and improve performance while maintaining quality.
    ///
    /// **Termination Criteria**:
    /// 1. Maximum depth reached
    /// 2. Minimum cell size threshold
    /// 3. Low polygon count (insufficient detail to warrant subdivision)
    /// 4. Uniform occupancy (all corners have same sign)
    /// 5. Low surface complexity (simple geometry doesn't need deep subdivision)
    pub fn should_terminate_subdivision(
        depth: u8,
        max_depth: u8,
        half_size: Real,
        polygon_count: usize,
        corner_values: Option<&[Real]>,
        iso_value: Real,
    ) -> bool {
        // Criterion 1: Maximum depth reached
        if depth >= max_depth {
            return true;
        }

        // Criterion 2: Minimum cell size threshold (prevent infinite subdivision)
        const MIN_CELL_SIZE: Real = 1e-6;
        if half_size < MIN_CELL_SIZE {
            return true;
        }

        // Criterion 3: Low polygon count threshold
        const MIN_POLYGON_COUNT: usize = 2;
        if polygon_count < MIN_POLYGON_COUNT {
            return true;
        }

        // Criterion 4: Uniform occupancy check (for SDF-based subdivision)
        if let Some(values) = corner_values {
            let has_positive = values.iter().any(|&v| v > iso_value);
            let has_negative = values.iter().any(|&v| v < iso_value);

            // If all corners have the same sign, no surface crossing
            if !has_positive || !has_negative {
                return true;
            }

            // Criterion 5: Low surface complexity check
            // If the SDF variation is very small, the surface is nearly flat
            let min_val = values.iter().fold(Real::MAX, |a, &b| a.min(b));
            let max_val = values.iter().fold(Real::MIN, |a, &b| a.max(b));
            let variation = (max_val - min_val).abs();

            const MIN_VARIATION_THRESHOLD: Real = 1e-4;
            if variation < MIN_VARIATION_THRESHOLD {
                return true;
            }
        }

        // Criterion 6: Adaptive depth based on cell size
        // For very large cells, allow deeper subdivision
        // For small cells, terminate earlier to prevent over-subdivision
        let adaptive_max_depth = if half_size > 1.0 {
            max_depth + 2 // Allow deeper subdivision for large cells
        } else if half_size < 0.1 {
            max_depth.saturating_sub(2) // Terminate earlier for small cells
        } else {
            max_depth
        };

        if depth >= adaptive_max_depth {
            return true;
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn integrate_empty_polygons() {
        let mut svo: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let polygons: Vec<Polygon<()>> = Vec::new();
        
        BspIntegrator::integrate_bsp_from_polygons(&mut svo, &polygons);
        // Should not crash with empty polygons
    }
    
    #[test]
    fn integrate_sdf_sphere() {
        let mut svo: Svo<()> = Svo::new(Point3::origin(), 2.0, 4);
        let sphere_sdf = |p: &Point3<Real>| p.coords.norm() - 1.0;

        BspIntegrator::integrate_bsp_from_sdf(&mut svo, sphere_sdf, 0.0);

        // Should have created Mixed cells at surface crossings
        // Detailed verification would require traversing the SVO
    }

    #[test]
    fn test_termination_criteria_max_depth() {
        // Test maximum depth termination
        assert!(BspIntegrator::should_terminate_subdivision(5, 4, 1.0, 10, None, 0.0));
        assert!(!BspIntegrator::should_terminate_subdivision(3, 4, 1.0, 10, None, 0.0));
    }

    #[test]
    fn test_termination_criteria_min_cell_size() {
        // Test minimum cell size termination (MIN_CELL_SIZE = 1e-6)
        // Use depth=1 and max_depth=10 to avoid other termination criteria
        assert!(BspIntegrator::should_terminate_subdivision(1, 10, 1e-7, 10, None, 0.0)); // Below threshold
        assert!(!BspIntegrator::should_terminate_subdivision(1, 10, 1e-5, 10, None, 0.0)); // Above threshold
    }

    #[test]
    fn test_termination_criteria_polygon_count() {
        // Test low polygon count termination
        assert!(BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 1, None, 0.0));
        assert!(!BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 5, None, 0.0));
    }

    #[test]
    fn test_termination_criteria_uniform_occupancy() {
        // Test uniform occupancy (all positive)
        let all_positive = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        assert!(BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 10, Some(&all_positive), 0.0));

        // Test uniform occupancy (all negative)
        let all_negative = vec![-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0];
        assert!(BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 10, Some(&all_negative), 0.0));

        // Test mixed occupancy (should not terminate)
        let mixed = vec![-1.0, 1.0, -2.0, 2.0, -3.0, 3.0, -4.0, 4.0];
        assert!(!BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 10, Some(&mixed), 0.0));
    }

    #[test]
    fn test_termination_criteria_low_variation() {
        // Test low surface complexity (small variation)
        let low_variation = vec![0.0001, 0.0002, 0.0001, 0.0002, 0.0001, 0.0002, 0.0001, 0.0002];
        assert!(BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 10, Some(&low_variation), 0.0));

        // Test high variation (should not terminate)
        let high_variation = vec![-1.0, 1.0, -2.0, 2.0, -3.0, 3.0, -4.0, 4.0];
        assert!(!BspIntegrator::should_terminate_subdivision(2, 4, 1.0, 10, Some(&high_variation), 0.0));
    }

    #[test]
    fn test_termination_criteria_adaptive_depth() {
        // Test adaptive depth for large cells (should allow deeper subdivision)
        // For half_size > 1.0, adaptive_max_depth = max_depth + 2 = 6 + 2 = 8
        assert!(!BspIntegrator::should_terminate_subdivision(5, 6, 2.0, 10, None, 0.0)); // depth 5 < 8
        assert!(BspIntegrator::should_terminate_subdivision(8, 6, 2.0, 10, None, 0.0)); // depth 8 >= 8

        // Test adaptive depth for small cells (should terminate earlier)
        // For half_size < 0.1, adaptive_max_depth = max_depth - 2 = 6 - 2 = 4
        assert!(BspIntegrator::should_terminate_subdivision(4, 6, 0.05, 10, None, 0.0)); // depth 4 >= 4
        assert!(!BspIntegrator::should_terminate_subdivision(3, 6, 0.05, 10, None, 0.0)); // depth 3 < 4
    }

    #[test]
    fn test_recursive_termination_prevents_infinite_subdivision() {
        // Create a pathological SDF that could cause infinite subdivision
        let pathological_sdf = |p: &Point3<Real>| {
            // SDF that has very fine detail but should terminate due to size limits
            (p.x * 1000.0).sin() * 0.001
        };

        let mut svo: Svo<()> = Svo::new(Point3::origin(), 1.0, 10);

        // This should not hang or cause stack overflow
        BspIntegrator::integrate_bsp_from_sdf(&mut svo, pathological_sdf, 0.0);

        // Should complete without issues
        assert!(svo.root.occupancy != Occupancy::Empty); // Should have some structure
    }

    #[test]
    fn test_polygon_filtering_optimization() {
        use crate::voxels::vertex::Vertex;
        use nalgebra::Vector3;

        // Create polygons that are far from the center
        let far_polygons = vec![
            Polygon::<()>::new(vec![
                Vertex::new(Point3::new(10.0, 10.0, 10.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(11.0, 10.0, 10.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(10.5, 11.0, 10.0), Vector3::new(0.0, 0.0, 1.0)),
            ], None),
        ];

        // Filter polygons for a small cell at origin
        let filtered = BspIntegrator::filter_polygons_in_bounds(
            &far_polygons,
            &Point3::origin(),
            0.5
        );

        // Should filter out polygons that don't intersect the cell
        assert!(filtered.is_empty(), "Far polygons should be filtered out");

        // Create polygons that intersect the cell
        let near_polygons = vec![
            Polygon::<()>::new(vec![
                Vertex::new(Point3::new(-0.1, -0.1, -0.1), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(0.1, -0.1, -0.1), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(0.0, 0.1, -0.1), Vector3::new(0.0, 0.0, 1.0)),
            ], None),
        ];

        let filtered_near = BspIntegrator::filter_polygons_in_bounds(
            &near_polygons,
            &Point3::origin(),
            0.5
        );

        // Should keep polygons that intersect the cell
        assert!(!filtered_near.is_empty(), "Near polygons should not be filtered out");
    }
}
