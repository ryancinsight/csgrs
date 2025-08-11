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
        
        // Recursively process children if not at max depth
        if depth < max_depth {
            for child_idx in 0..8 {
                if let Some(child) = node.get_child_mut(child_idx) {
                    let child_center = Self::child_center(center, half, child_idx);
                    Self::integrate_node_bsp(
                        child,
                        &child_center,
                        half * 0.5,
                        depth + 1,
                        max_depth,
                        &node_polygons,
                    );
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
        
        // Recursively process children
        if depth < max_depth {
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
}
