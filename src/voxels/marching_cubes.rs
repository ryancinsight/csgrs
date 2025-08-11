//! Simplified marching cubes for SVO-embedded BSP surface generation
//! 
//! Following KISS and YAGNI principles, this implements only the essential
//! marching cubes functionality needed for SVO surface-crossing cells.

use crate::float_types::Real;
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Simplified marching cubes implementation for octree cells
/// 
/// Following Single Responsibility Principle - only generates surface polygons
/// from SDF values at cell corners.
pub struct MarchingCubes;

impl MarchingCubes {
    /// Generate surface polygons for a cell using marching cubes
    /// 
    /// This is a minimal implementation following YAGNI - only handles
    /// the most common surface-crossing cases.
    pub fn generate_surface_polygons<S, F>(
        center: &Point3<Real>,
        half: Real,
        sdf: &F,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Vec<Polygon<S>>
    where
        F: Fn(&Point3<Real>) -> Real,
        S: Clone + Debug + Send + Sync,
    {
        // Get the 8 corner points of the cell
        let corners = Self::get_cell_corners(center, half);
        let corner_values: Vec<Real> = corners.iter().map(|p| sdf(p)).collect();
        
        // Determine which corners are inside/outside
        let mut cube_index = 0u8;
        for i in 0..8 {
            if corner_values[i] <= iso_value {
                cube_index |= 1 << i;
            }
        }
        
        // Early exit for trivial cases (YAGNI - avoid unnecessary computation)
        if cube_index == 0 || cube_index == 255 {
            return Vec::new();
        }
        
        // Generate surface polygons using simplified approach
        Self::generate_polygons_for_cube_index(
            cube_index,
            &corners,
            &corner_values,
            iso_value,
            metadata,
        )
    }
    
    /// Get the 8 corner points of a cell (DRY - reused from bsp_integration)
    fn get_cell_corners(center: &Point3<Real>, half: Real) -> [Point3<Real>; 8] {
        [
            Point3::new(center.x - half, center.y - half, center.z - half), // 0
            Point3::new(center.x + half, center.y - half, center.z - half), // 1
            Point3::new(center.x - half, center.y + half, center.z - half), // 2
            Point3::new(center.x + half, center.y + half, center.z - half), // 3
            Point3::new(center.x - half, center.y - half, center.z + half), // 4
            Point3::new(center.x + half, center.y - half, center.z + half), // 5
            Point3::new(center.x - half, center.y + half, center.z + half), // 6
            Point3::new(center.x + half, center.y + half, center.z + half), // 7
        ]
    }
    
    /// Generate polygons for a specific cube configuration
    /// 
    /// Following KISS principle - simplified implementation covering common cases
    fn generate_polygons_for_cube_index<S>(
        cube_index: u8,
        corners: &[Point3<Real>; 8],
        corner_values: &[Real],
        iso_value: Real,
        metadata: Option<S>,
    ) -> Vec<Polygon<S>>
    where
        S: Clone + Debug + Send + Sync,
    {
        let mut polygons = Vec::new();
        
        // Simplified marching cubes - handle most common cases
        // Following YAGNI - implement only what's needed for basic surface extraction
        match cube_index {
            // Single corner inside cases
            1 | 254 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 0, metadata.clone());
            }
            2 | 253 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 1, metadata.clone());
            }
            4 | 251 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 2, metadata.clone());
            }
            8 | 247 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 3, metadata.clone());
            }
            16 | 239 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 4, metadata.clone());
            }
            32 | 223 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 5, metadata.clone());
            }
            64 | 191 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 6, metadata.clone());
            }
            128 | 127 => {
                Self::add_corner_triangle(&mut polygons, corners, corner_values, iso_value, 7, metadata.clone());
            }
            // Edge cases - create simple quads
            _ => {
                Self::add_generic_surface(&mut polygons, corners, corner_values, iso_value, metadata.clone());
            }
        }
        
        polygons
    }
    
    /// Add a triangle for a single corner case
    /// 
    /// Following Single Responsibility - handles one specific marching cubes case
    fn add_corner_triangle<S>(
        polygons: &mut Vec<Polygon<S>>,
        corners: &[Point3<Real>; 8],
        corner_values: &[Real],
        iso_value: Real,
        corner_idx: usize,
        metadata: Option<S>,
    ) where
        S: Clone + Debug + Send + Sync,
    {
        // Get the three edges connected to this corner
        let edges = Self::get_corner_edges(corner_idx);
        
        // Interpolate points on the three edges
        let mut edge_points = Vec::new();
        for &(v1, v2) in &edges {
            if let Some(point) = Self::interpolate_edge_point(
                corners[v1],
                corners[v2],
                corner_values[v1],
                corner_values[v2],
                iso_value,
            ) {
                edge_points.push(point);
            }
        }
        
        // Create triangle if we have 3 edge points
        if edge_points.len() == 3 {
            let normal = Self::compute_triangle_normal(&edge_points[0], &edge_points[1], &edge_points[2]);
            let vertices = vec![
                Vertex::new(edge_points[0], normal),
                Vertex::new(edge_points[1], normal),
                Vertex::new(edge_points[2], normal),
            ];
            polygons.push(Polygon::new(vertices, metadata));
        }
    }
    
    /// Add a generic surface for complex cases
    /// 
    /// Following KISS - simple fallback for complex marching cubes cases
    fn add_generic_surface<S>(
        polygons: &mut Vec<Polygon<S>>,
        corners: &[Point3<Real>; 8],
        corner_values: &[Real],
        iso_value: Real,
        metadata: Option<S>,
    ) where
        S: Clone + Debug + Send + Sync,
    {
        // Simple approach: find all edge crossings and create a basic surface
        let mut edge_points = Vec::new();
        
        // Check all 12 edges of the cube
        let edges = [
            (0, 1), (1, 3), (3, 2), (2, 0), // bottom face
            (4, 5), (5, 7), (7, 6), (6, 4), // top face
            (0, 4), (1, 5), (2, 6), (3, 7), // vertical edges
        ];
        
        for &(v1, v2) in &edges {
            if let Some(point) = Self::interpolate_edge_point(
                corners[v1],
                corners[v2],
                corner_values[v1],
                corner_values[v2],
                iso_value,
            ) {
                edge_points.push(point);
            }
        }
        
        // Create triangles from edge points (simplified fan triangulation)
        if edge_points.len() >= 3 {
            let center = Self::compute_centroid(&edge_points);
            let normal = Self::estimate_surface_normal(&edge_points);
            
            for i in 0..edge_points.len() {
                let next = (i + 1) % edge_points.len();
                let vertices = vec![
                    Vertex::new(center, normal),
                    Vertex::new(edge_points[i], normal),
                    Vertex::new(edge_points[next], normal),
                ];
                polygons.push(Polygon::new(vertices, metadata.clone()));
            }
        }
    }
    
    /// Get the three edges connected to a corner
    fn get_corner_edges(corner_idx: usize) -> [(usize, usize); 3] {
        match corner_idx {
            0 => [(0, 1), (0, 2), (0, 4)],
            1 => [(1, 0), (1, 3), (1, 5)],
            2 => [(2, 0), (2, 3), (2, 6)],
            3 => [(3, 1), (3, 2), (3, 7)],
            4 => [(4, 0), (4, 5), (4, 6)],
            5 => [(5, 1), (5, 4), (5, 7)],
            6 => [(6, 2), (6, 4), (6, 7)],
            7 => [(7, 3), (7, 5), (7, 6)],
            _ => [(0, 1), (0, 2), (0, 4)], // fallback
        }
    }
    
    /// Interpolate point on edge where surface crosses
    fn interpolate_edge_point(
        p1: Point3<Real>,
        p2: Point3<Real>,
        v1: Real,
        v2: Real,
        iso_value: Real,
    ) -> Option<Point3<Real>> {
        // Check if edge actually crosses the surface
        if (v1 <= iso_value) == (v2 <= iso_value) {
            return None;
        }
        
        // Linear interpolation
        let t = (iso_value - v1) / (v2 - v1);
        let t = t.clamp(0.0, 1.0);
        
        Some(Point3::new(
            p1.x + t * (p2.x - p1.x),
            p1.y + t * (p2.y - p1.y),
            p1.z + t * (p2.z - p1.z),
        ))
    }
    
    /// Compute triangle normal
    fn compute_triangle_normal(p1: &Point3<Real>, p2: &Point3<Real>, p3: &Point3<Real>) -> Vector3<Real> {
        let v1 = p2 - p1;
        let v2 = p3 - p1;
        let normal = v1.cross(&v2);
        if normal.norm() > 1e-8 {
            normal.normalize()
        } else {
            Vector3::new(0.0, 0.0, 1.0) // fallback
        }
    }
    
    /// Compute centroid of points
    fn compute_centroid(points: &[Point3<Real>]) -> Point3<Real> {
        let sum = points.iter().fold(Point3::origin(), |acc, p| acc + p.coords);
        Point3::from(sum / points.len() as Real)
    }
    
    /// Estimate surface normal from edge points
    fn estimate_surface_normal(points: &[Point3<Real>]) -> Vector3<Real> {
        if points.len() < 3 {
            return Vector3::new(0.0, 0.0, 1.0);
        }
        
        Self::compute_triangle_normal(&points[0], &points[1], &points[2])
    }
}
