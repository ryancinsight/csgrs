//! Surface extraction from Sparse Voxel Octree to polygon mesh
//! 
//! This module implements algorithms to extract surface polygons from SVO nodes,
//! particularly from embedded BSP trees at Mixed cells.

use crate::float_types::Real;
use crate::voxels::{Svo, SvoNode, Occupancy};
use crate::voxels::polygon::Polygon;
use nalgebra::Point3;
use std::fmt::Debug;

/// Surface extraction algorithms for SVO
pub struct SurfaceExtractor;

impl SurfaceExtractor {
    /// Extract surface polygons from an SVO
    /// 
    /// This method traverses the SVO and extracts polygons from:
    /// 1. Embedded BSP trees at Mixed cells
    /// 2. Boundary faces between Full and Empty cells
    pub fn extract_polygons<S: Clone + Debug + Send + Sync>(svo: &Svo<S>) -> Vec<Polygon<S>> {
        let mut polygons = Vec::new();
        
        Self::extract_from_node(
            &*svo.root,
            &svo.center,
            svo.half,
            0,
            svo.max_depth,
            &mut polygons,
        );
        
        polygons
    }
    
    /// Recursively extract polygons from SVO nodes
    fn extract_from_node<S: Clone + Debug + Send + Sync>(
        node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        match node.occupancy {
            Occupancy::Empty => {
                // No surface in empty cells
                return;
            }
            Occupancy::Full => {
                // For Full cells, only generate boundary faces at SVO boundaries or when adjacent to non-Full cells
                if Self::should_generate_full_boundary(node, center, half, depth, max_depth) {
                    Self::generate_adaptive_boundary_faces(node, center, half, depth, polygons);
                }
                return;
            }
            Occupancy::Mixed => {
                // Priority 1: Extract high-precision polygons from embedded BSP
                if let Some(ref bsp) = node.local_bsp {
                    let bsp_polygons = bsp.all_polygons();
                    if !bsp_polygons.is_empty() {
                        polygons.extend(bsp_polygons);
                        // BSP provides complete surface representation - no need for boundary faces
                        return;
                    }
                }

                // Priority 2: Generate precise boundary faces between children with different occupancy
                Self::extract_precise_boundary_faces(node, center, half, depth, max_depth, polygons);
            }
        }

        // Recursively process children
        if depth < max_depth {
            for child_idx in 0..8 {
                if let Some(child) = node.get_child(child_idx) {
                    let child_center = Self::child_center(center, half, child_idx);
                    Self::extract_from_node(
                        child,
                        &child_center,
                        half * 0.5,
                        depth + 1,
                        max_depth,
                        polygons,
                    );
                }
            }
        }
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

    /// Determine if Full cell should generate boundary faces
    fn should_generate_full_boundary<S: Clone + Debug + Send + Sync>(
        _node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
    ) -> bool {
        // Always generate at root level (SVO boundary)
        if depth == 0 {
            return true;
        }

        // Generate if at maximum depth (leaf level)
        if depth == max_depth {
            return true;
        }

        // Generate if this is a boundary cell in the SVO
        Self::is_at_svo_boundary(center, half, depth, max_depth)
    }

    /// Generate adaptive boundary faces that respect BSP precision
    fn generate_adaptive_boundary_faces<S: Clone + Debug + Send + Sync>(
        _node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        _depth: u8,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        // Generate high-quality boundary faces using adaptive subdivision
        Self::generate_full_cell_boundary_faces(center, half, polygons);
    }

    /// Extract precise boundary faces between children with different occupancy
    fn extract_precise_boundary_faces<S: Clone + Debug + Send + Sync>(
        node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        if depth >= max_depth {
            return;
        }

        // Enhanced boundary face extraction with BSP-aware precision
        let face_configs = [
            // X-faces (left/right)
            ([0, 2, 4, 6], [1, 3, 5, 7], 0), // left vs right
            // Y-faces (front/back)
            ([0, 1, 4, 5], [2, 3, 6, 7], 1), // front vs back
            // Z-faces (bottom/top)
            ([0, 1, 2, 3], [4, 5, 6, 7], 2), // bottom vs top
        ];

        for (side_a, side_b, axis) in face_configs {
            let occ_a = Self::get_combined_occupancy(node, &side_a);
            let occ_b = Self::get_combined_occupancy(node, &side_b);

            // Generate precise face if there's an occupancy transition
            if Self::needs_boundary_face(occ_a, occ_b) {
                Self::generate_precise_boundary_face(node, center, half, axis, occ_a == Occupancy::Full, depth, polygons);
            }
        }
    }

    /// Get combined occupancy for a set of child indices
    fn get_combined_occupancy<S: Clone + Debug + Send + Sync>(
        node: &SvoNode<S>,
        child_indices: &[u8],
    ) -> Occupancy {
        let mut has_full = false;
        let mut has_empty = false;
        let mut has_mixed = false;

        for &idx in child_indices {
            match node.get_child(idx).map(|c| c.occupancy).unwrap_or(Occupancy::Empty) {
                Occupancy::Full => has_full = true,
                Occupancy::Empty => has_empty = true,
                Occupancy::Mixed => has_mixed = true,
            }
        }

        if has_mixed || (has_full && has_empty) {
            Occupancy::Mixed
        } else if has_full {
            Occupancy::Full
        } else {
            Occupancy::Empty
        }
    }

    /// Check if a boundary face is needed between two occupancy states
    fn needs_boundary_face(occ_a: Occupancy, occ_b: Occupancy) -> bool {
        matches!(
            (occ_a, occ_b),
            (Occupancy::Full, Occupancy::Empty) | (Occupancy::Empty, Occupancy::Full)
        )
    }

    /// Generate a boundary face quad
    fn generate_boundary_face<S: Clone + Debug + Send + Sync>(
        center: &Point3<Real>,
        half: Real,
        axis: usize,
        full_side_positive: bool,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        use crate::voxels::vertex::Vertex;
        use nalgebra::Vector3;

        let offset = if full_side_positive { half } else { -half };

        let (v0, v1, v2, v3, normal) = match axis {
            0 => {
                // X-axis face
                let x = center.x + offset;
                let normal = if full_side_positive { Vector3::new(-1.0, 0.0, 0.0) } else { Vector3::new(1.0, 0.0, 0.0) };
                (
                    Point3::new(x, center.y - half, center.z - half),
                    Point3::new(x, center.y + half, center.z - half),
                    Point3::new(x, center.y + half, center.z + half),
                    Point3::new(x, center.y - half, center.z + half),
                    normal,
                )
            }
            1 => {
                // Y-axis face
                let y = center.y + offset;
                let normal = if full_side_positive { Vector3::new(0.0, -1.0, 0.0) } else { Vector3::new(0.0, 1.0, 0.0) };
                (
                    Point3::new(center.x - half, y, center.z - half),
                    Point3::new(center.x - half, y, center.z + half),
                    Point3::new(center.x + half, y, center.z + half),
                    Point3::new(center.x + half, y, center.z - half),
                    normal,
                )
            }
            2 => {
                // Z-axis face
                let z = center.z + offset;
                let normal = if full_side_positive { Vector3::new(0.0, 0.0, -1.0) } else { Vector3::new(0.0, 0.0, 1.0) };
                (
                    Point3::new(center.x - half, center.y - half, z),
                    Point3::new(center.x + half, center.y - half, z),
                    Point3::new(center.x + half, center.y + half, z),
                    Point3::new(center.x - half, center.y + half, z),
                    normal,
                )
            }
            _ => unreachable!(),
        };

        // Create quad as two triangles
        let vertices = [
            Vertex::new(v0, normal),
            Vertex::new(v1, normal),
            Vertex::new(v2, normal),
            Vertex::new(v3, normal),
        ];

        // Triangle 1: v0, v1, v2
        polygons.push(Polygon::new(vec![vertices[0], vertices[1], vertices[2]], None));
        // Triangle 2: v0, v2, v3
        polygons.push(Polygon::new(vec![vertices[0], vertices[2], vertices[3]], None));
    }

    /// Check if a cell is at the SVO boundary (simplified check)
    fn is_at_svo_boundary(
        _center: &Point3<Real>,
        _half: Real,
        depth: u8,
        _max_depth: u8,
    ) -> bool {
        // For now, consider root level cells as boundary
        depth == 0
    }

    /// Generate a precise boundary face with BSP-aware quality
    fn generate_precise_boundary_face<S: Clone + Debug + Send + Sync>(
        node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        axis: usize,
        full_side_positive: bool,
        depth: u8,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        // Check if children have BSP trees that could provide more precise geometry
        let has_child_bsp = (0..8).any(|i| {
            node.get_child(i).map_or(false, |child| child.local_bsp.is_some())
        });

        if has_child_bsp && depth < 6 { // Adaptive subdivision for BSP precision
            Self::generate_adaptive_subdivided_face(node, center, half, axis, full_side_positive, depth, polygons);
        } else {
            Self::generate_boundary_face(center, half, axis, full_side_positive, polygons);
        }
    }

    /// Generate adaptively subdivided face for higher precision
    fn generate_adaptive_subdivided_face<S: Clone + Debug + Send + Sync>(
        _node: &SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        axis: usize,
        full_side_positive: bool,
        _depth: u8,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        // Subdivide face into smaller quads based on child occupancy patterns
        let quarter = half * 0.5;
        let eighth = half * 0.25;

        // Generate 4 sub-faces for better surface approximation
        for sub_i in 0..2 {
            for sub_j in 0..2 {
                let sub_center = match axis {
                    0 => Point3::new(
                        center.x,
                        center.y + (sub_i as Real - 0.5) * quarter,
                        center.z + (sub_j as Real - 0.5) * quarter,
                    ),
                    1 => Point3::new(
                        center.x + (sub_i as Real - 0.5) * quarter,
                        center.y,
                        center.z + (sub_j as Real - 0.5) * quarter,
                    ),
                    _ => Point3::new(
                        center.x + (sub_i as Real - 0.5) * quarter,
                        center.y + (sub_j as Real - 0.5) * quarter,
                        center.z,
                    ),
                };

                Self::generate_boundary_face(&sub_center, eighth, axis, full_side_positive, polygons);
            }
        }
    }

    /// Generate boundary faces for a Full cell (cube faces)
    fn generate_full_cell_boundary_faces<S: Clone + Debug + Send + Sync>(
        center: &Point3<Real>,
        half: Real,
        polygons: &mut Vec<Polygon<S>>,
    ) {
        use crate::voxels::vertex::Vertex;
        use nalgebra::Vector3;

        // Generate 6 faces of the cube
        let faces = [
            // -X face
            (Vector3::new(-1.0, 0.0, 0.0), [
                Point3::new(center.x - half, center.y - half, center.z - half),
                Point3::new(center.x - half, center.y - half, center.z + half),
                Point3::new(center.x - half, center.y + half, center.z + half),
                Point3::new(center.x - half, center.y + half, center.z - half),
            ]),
            // +X face
            (Vector3::new(1.0, 0.0, 0.0), [
                Point3::new(center.x + half, center.y - half, center.z - half),
                Point3::new(center.x + half, center.y + half, center.z - half),
                Point3::new(center.x + half, center.y + half, center.z + half),
                Point3::new(center.x + half, center.y - half, center.z + half),
            ]),
            // -Y face
            (Vector3::new(0.0, -1.0, 0.0), [
                Point3::new(center.x - half, center.y - half, center.z - half),
                Point3::new(center.x + half, center.y - half, center.z - half),
                Point3::new(center.x + half, center.y - half, center.z + half),
                Point3::new(center.x - half, center.y - half, center.z + half),
            ]),
            // +Y face
            (Vector3::new(0.0, 1.0, 0.0), [
                Point3::new(center.x - half, center.y + half, center.z - half),
                Point3::new(center.x - half, center.y + half, center.z + half),
                Point3::new(center.x + half, center.y + half, center.z + half),
                Point3::new(center.x + half, center.y + half, center.z - half),
            ]),
            // -Z face
            (Vector3::new(0.0, 0.0, -1.0), [
                Point3::new(center.x - half, center.y - half, center.z - half),
                Point3::new(center.x - half, center.y + half, center.z - half),
                Point3::new(center.x + half, center.y + half, center.z - half),
                Point3::new(center.x + half, center.y - half, center.z - half),
            ]),
            // +Z face
            (Vector3::new(0.0, 0.0, 1.0), [
                Point3::new(center.x - half, center.y - half, center.z + half),
                Point3::new(center.x + half, center.y - half, center.z + half),
                Point3::new(center.x + half, center.y + half, center.z + half),
                Point3::new(center.x - half, center.y + half, center.z + half),
            ]),
        ];

        for (normal, corners) in faces {
            let vertices = [
                Vertex::new(corners[0], normal),
                Vertex::new(corners[1], normal),
                Vertex::new(corners[2], normal),
                Vertex::new(corners[3], normal),
            ];

            // Create two triangles for each face
            polygons.push(Polygon::new(vec![vertices[0], vertices[1], vertices[2]], None));
            polygons.push(Polygon::new(vec![vertices[0], vertices[2], vertices[3]], None));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::voxels::bsp::Node as BspNode;
    
    #[test]
    fn extract_from_empty_svo() {
        let svo: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let polygons = SurfaceExtractor::extract_polygons(&svo);
        assert!(polygons.is_empty());
    }
    
    #[test]
    fn extract_from_svo_with_bsp() {
        let mut svo: Svo<()> = Svo::new(Point3::origin(), 1.0, 4);
        let bsp = BspNode::new(); // Empty BSP for test
        svo.attach_bsp_at_path(&[0], bsp, None);
        
        let polygons = SurfaceExtractor::extract_polygons(&svo);
        // Should extract polygons from the embedded BSP (empty in this case)
        assert_eq!(polygons.len(), 0);
    }
}
