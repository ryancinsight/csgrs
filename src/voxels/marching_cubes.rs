//! Marching Cubes for SVO-embedded BSP surface generation
//!
//! Robust, table–driven implementation (classic Lorensen & Cline algorithm)
//! adapted for sparse voxel octree (SVO) cells. Balances: KISS (clear
//! structure), YAGNI (only required public API), DRY (shared helpers), SOLID
//! (single responsibility: surface extraction), GRASP (information / creator),
//! CUPID (composable, Unix-y, predictable, idiomatic, domain‑focused), DIP
//! (depends only on sdf function trait bound), and zero‑cost abstractions via
//! slices, iterators, `#[inline]` helpers, and precomputed const tables.
//!
//! Data tables (EDGE_TABLE & TRI_TABLE) are standard Marching Cubes lookup
//! data; they are factual numeric mappings (public domain / de-facto common
//! knowledge). Any triangle orientation issues can be resolved by flipping
//! normals if needed upstream.

use crate::float_types::Real;
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

// -------------------------------------------------------------------------------------------------
// Lookup Tables (const so they are embedded in the binary & allow maximum inlining / zero runtime)
// -------------------------------------------------------------------------------------------------

// Complete standard Marching Cubes edge table (256 entries)
// Each entry is a 12-bit mask indicating which edges are intersected by the isosurface
const EDGE_TABLE: [u16; 256] = [
    0x0000,0x0109,0x0203,0x030a,0x0406,0x050f,0x0605,0x070c,0x080c,0x0905,0x0a0f,0x0b06,0x0c0a,0x0d03,0x0e09,0x0f00,
    0x0190,0x0099,0x0393,0x029a,0x0596,0x049f,0x0795,0x069c,0x099c,0x0895,0x0b9f,0x0a96,0x0d9a,0x0c93,0x0f99,0x0e90,
    0x0230,0x0339,0x0033,0x013a,0x0636,0x073f,0x0435,0x053c,0x0a3c,0x0b35,0x083f,0x0936,0x0e3a,0x0f33,0x0c39,0x0d30,
    0x03a0,0x02a9,0x01a3,0x00aa,0x07a6,0x06af,0x05a5,0x04ac,0x0bac,0x0aa5,0x09af,0x08a6,0x0faa,0x0ea3,0x0da9,0x0ca0,
    0x0460,0x0569,0x0663,0x076a,0x0066,0x016f,0x0265,0x036c,0x0c6c,0x0d65,0x0e6f,0x0f66,0x086a,0x0963,0x0a69,0x0b60,
    0x05f0,0x04f9,0x07f3,0x06fa,0x01f6,0x00ff,0x03f5,0x02fc,0x0dfc,0x0cf5,0x0fff,0x0ef6,0x09fa,0x08f3,0x0bf9,0x0af0,
    0x0670,0x0779,0x0473,0x057a,0x0276,0x037f,0x0075,0x017c,0x0e7c,0x0f75,0x0c7f,0x0d76,0x0a7a,0x0b73,0x0879,0x0970,
    0x07e0,0x06e9,0x05e3,0x04ea,0x03e6,0x02ef,0x01e5,0x00ec,0x0fec,0x0ee5,0x0def,0x0ce6,0x0bea,0x0ae3,0x09e9,0x08e0,
    0x0c60,0x0d69,0x0e63,0x0f6a,0x0866,0x096f,0x0a65,0x0b6c,0x046c,0x0565,0x066f,0x0766,0x006a,0x0163,0x0269,0x0360,
    0x0df0,0x0cf9,0x0ff3,0x0efa,0x09f6,0x08ff,0x0bf5,0x0afc,0x05fc,0x04f5,0x07ff,0x06f6,0x01fa,0x00f3,0x03f9,0x02f0,
    0x0e70,0x0f79,0x0c73,0x0d7a,0x0a76,0x0b7f,0x0875,0x097c,0x067c,0x0775,0x047f,0x0576,0x027a,0x0373,0x0079,0x0170,
    0x0fe0,0x0ee9,0x0de3,0x0cea,0x0be6,0x0aef,0x09e5,0x08ec,0x07ec,0x06e5,0x05ef,0x04e6,0x03ea,0x02e3,0x01e9,0x00e0,
    0x0ca0,0x0da9,0x0ea3,0x0faa,0x08a6,0x09af,0x0aa5,0x0bac,0x04ac,0x05a5,0x06af,0x07a6,0x00aa,0x01a3,0x02a9,0x03a0,
    0x0d30,0x0c39,0x0f33,0x0e3a,0x0936,0x083f,0x0b35,0x0a3c,0x053c,0x0435,0x073f,0x0636,0x013a,0x0033,0x0339,0x0230,
    0x0e90,0x0f99,0x0c93,0x0d9a,0x0a96,0x0b9f,0x0895,0x099c,0x069c,0x0795,0x049f,0x0596,0x029a,0x0393,0x0099,0x0190,
    0x0f00,0x0e09,0x0d03,0x0c0a,0x0b06,0x0a0f,0x0905,0x080c,0x070c,0x0605,0x050f,0x0406,0x030a,0x0203,0x0109,0x0000,
];

// Triangle table: lists up to 5 triangles (15 indices) + terminator (-1) per cube config.
// Using i8 saves space; -1 indicates end.
#[allow(clippy::unreadable_literal)]
const TRI_TABLE: [[i8; 16]; 256] = include!("../../generated/mc_tri_table_complete.incl");

// NOTE: The large TRI_TABLE is moved to an include file for readability & to minimize this file's
// diff noise. If the include file does not yet exist it will be created by the patch system.

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
        // --- Gather corners (SRP & DRY) ---------------------------------------------------------
        let corners = Self::get_cell_corners(center, half);
        let corner_values = corners.map(|p| sdf(&p)); // fixed-size array, zero heap alloc

        // --- Build cube index bitmask (branchless-ish) -----------------------------------------
        let cube_index: u8 = corner_values.iter().enumerate().fold(0u8, |acc, (i, &v)| {
            acc | (((v <= iso_value) as u8) << i)
        });

        if cube_index == 0 || cube_index == 0xFF { return Vec::new(); }

        // --- Edge mask -> early exit when no edges intersect ------------------------------------
        let edge_mask = EDGE_TABLE[cube_index as usize];
        if edge_mask == 0 { return Vec::new(); }

        // --- Interpolate edges (cache) ---------------------------------------------------------
        let mut edge_points: [Option<Point3<Real>>; 12] = [None; 12];
        for e in 0..12 { if (edge_mask & (1 << e)) != 0 { edge_points[e as usize] = Some(Self::interpolate_on_edge(e as u8, &corners, &corner_values, iso_value)); } }

        // --- Emit triangles --------------------------------------------------------------------
        // Upper bound: 5 triangles per cube config.
        let mut polys: Vec<Polygon<S>> = Vec::with_capacity(5);
        let tri_row = &TRI_TABLE[cube_index as usize];
        let mut i = 0usize;
        while i < tri_row.len() && tri_row[i] != -1 {
            // Bounds check: ensure we have at least 3 more indices available
            if i + 2 >= tri_row.len() {
                break;
            }

            // Collect 3 intersection points (if any are missing, abort triangle)
            let (e0, e1, e2) = (tri_row[i] as usize, tri_row[i+1] as usize, tri_row[i+2] as usize);

            // Validate edge indices are within bounds (0-11 for cube edges)
            if e0 >= 12 || e1 >= 12 || e2 >= 12 {
                break;
            }

            if let (Some(p0), Some(p1), Some(p2)) = (edge_points[e0], edge_points[e1], edge_points[e2]) {
                // Compute flat normal (optionally could approximate gradient by finite diff)
                let normal = Self::triangle_normal(&p0, &p1, &p2);
                let vertices = vec![
                    Vertex::new(p0, normal),
                    Vertex::new(p1, normal),
                    Vertex::new(p2, normal),
                ];
                polys.push(Polygon::new(vertices, metadata.clone()));
            }
            i += 3;
        }

        polys
    }
    
    /// Get the 8 corner points of a cell (DRY - reused from bsp_integration)
    #[inline]
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

    #[inline]
    fn interpolate_on_edge(edge: u8, corners: &[Point3<Real>;8], values: &[Real;8], iso: Real) -> Point3<Real> {
        let (v1, v2) = EDGE_TO_VERTICES[edge as usize];
        Self::interp(corners[v1], corners[v2], values[v1], values[v2], iso)
    }

    #[inline]
    fn interp(p1: Point3<Real>, p2: Point3<Real>, valp1: Real, valp2: Real, iso: Real) -> Point3<Real> {
        // Avoid division by zero; bias towards midpoint if values almost equal
        let denom = valp2 - valp1;
        if denom.abs() < Real::EPSILON { return Point3::new((p1.x+p2.x)*0.5,(p1.y+p2.y)*0.5,(p1.z+p2.z)*0.5); }
        let t = ((iso - valp1) / denom).clamp(0.0, 1.0);
        Point3::new(
            p1.x + t * (p2.x - p1.x),
            p1.y + t * (p2.y - p1.y),
            p1.z + t * (p2.z - p1.z),
        )
    }

    #[inline]
    fn triangle_normal(p0: &Point3<Real>, p1: &Point3<Real>, p2: &Point3<Real>) -> Vector3<Real> {
        let a = p1 - p0; let b = p2 - p0; let n = a.cross(&b);
        if n.norm_squared() > 1e-14 { n.normalize() } else { Vector3::z() }
    }
}

// Edge index -> (corner a, corner b)
const EDGE_TO_VERTICES: [(usize, usize); 12] = [
    (0,1),(1,3),(3,2),(2,0), // bottom square (y- plane ordering matches original simplified version)
    (4,5),(5,7),(7,6),(6,4), // top square
    (0,4),(1,5),(2,6),(3,7), // verticals
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_cell_no_surface() {
        let center = Point3::origin();
        let half = 1.0;
        let sdf = |_: &Point3<Real>| 1.0; // all outside
        let polys: Vec<Polygon<()>> = MarchingCubes::generate_surface_polygons(&center, half, &sdf, 0.0, None);
        assert!(polys.is_empty());
    }

    #[test]
    fn full_cell_no_surface() {
        let center = Point3::origin();
        let half = 1.0;
        let sdf = |_: &Point3<Real>| -1.0; // all inside
        let polys: Vec<Polygon<()>> = MarchingCubes::generate_surface_polygons(&center, half, &sdf, 0.0, None);
        assert!(polys.is_empty());
    }

    #[test]
    fn sphere_crossing_generates_polys() {
        let center = Point3::origin();
        let half = 1.0; // Cube from (-1,-1,-1) to (1,1,1)

        // Create a sphere that definitely crosses the cube by having some corners inside and some outside
        // Use a sphere with radius 1.5 - this should put some corners inside and some outside
        // Corner distance is sqrt(3) ≈ 1.73, so with radius 1.5:
        // SDF = 1.73 - 1.5 = 0.23 > 0 (still all outside)

        // Create a plane that cuts through the cube - this guarantees mixed signs
        // Plane equation: x = 0 (divides cube into left and right halves)
        // Left corners (x = -1) will have SDF = -1 < 0 (inside)
        // Right corners (x = 1) will have SDF = 1 > 0 (outside)
        let sdf = |p: &Point3<Real>| p.x;
        let polys: Vec<Polygon<()>> = MarchingCubes::generate_surface_polygons(&center, half, &sdf, 0.0, None);

        // Debug: print corner values to understand the configuration
        let corners = [
            Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, 1.0), Point3::new(-1.0, -1.0, 1.0),
            Point3::new(-1.0, 1.0, -1.0), Point3::new(1.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0), Point3::new(-1.0, 1.0, 1.0),
        ];
        let corner_values: Vec<Real> = corners.iter().map(|p| sdf(p)).collect();
        println!("Corner SDF values (plane x=0): {:?}", corner_values);

        // Check if we have mixed signs (some positive, some negative)
        let has_positive = corner_values.iter().any(|&v| v > 0.0);
        let has_negative = corner_values.iter().any(|&v| v < 0.0);

        if has_positive && has_negative {
            assert!(!polys.is_empty(), "Mixed-sign corners should generate surface polygons");
            assert!(polys.len() <= 5, "Should not exceed maximum triangles per cube");
        } else {
            println!("All corners have same sign - no surface expected (this is correct MC behavior)");
            assert!(polys.is_empty(), "Same-sign corners should not generate surface");
        }
    }
}
