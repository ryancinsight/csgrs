//! Shape generation for Voxels using SVO-based voxelization
//!
//! This module provides unified shape generation that builds SVO directly,
//! using voxelization and embedded BSP for surface representation.

use crate::float_types::Real;
use crate::voxels::{Svo, SvoNode, Occupancy};
use crate::voxels::csg::Voxels;
use nalgebra::Point3;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    /// Create cube using SVO voxelization
    pub fn cube(size: Real, metadata: Option<S>) -> Voxels<S> {
        Self::cube_with_resolution(size, 6, metadata)
    }

    /// Create cube with specified resolution
    pub fn cube_with_resolution(size: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        Self::cuboid_with_resolution(size, size, size, resolution, metadata)
    }

    /// Create cuboid using SVO voxelization
    pub fn cuboid(width: Real, length: Real, height: Real, metadata: Option<S>) -> Voxels<S> {
        Self::cuboid_with_resolution(width, length, height, 6, metadata)
    }

    /// Create cuboid with specified resolution
    pub fn cuboid_with_resolution(width: Real, length: Real, height: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        let max_dim = width.max(length).max(height);
        let half_size = max_dim * 0.45; // Smaller than half so some corners are inside
        let center = Point3::origin(); // Center the SDF at origin

        let mut svo = Svo::new(center, half_size, resolution);

        // Voxelize the cuboid using SDF approach for proper occupancy
        let cuboid_sdf = |p: &Point3<Real>| {
            // Box SDF centered at origin with given dimensions
            let dx = p.x.abs() - width * 0.5;
            let dy = p.y.abs() - length * 0.5;
            let dz = p.z.abs() - height * 0.5;

            // Proper box SDF: negative inside, positive outside
            let outside_dist = (dx.max(0.0).powi(2) + dy.max(0.0).powi(2) + dz.max(0.0).powi(2)).sqrt();
            let inside_dist = dx.max(dy).max(dz).min(0.0);
            outside_dist + inside_dist
        };

        Self::voxelize_sdf(&mut svo, cuboid_sdf, 0.0);

        Self::from_svo(svo, metadata)
    }

    /// Create sphere using SVO voxelization
    pub fn sphere(radius: Real, _lat_segments: usize, _lon_segments: usize, metadata: Option<S>) -> Voxels<S> {
        Self::sphere_with_resolution(radius, 6, metadata)
    }

    /// Create sphere with specified resolution
    pub fn sphere_with_resolution(radius: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        // Use the proven Voxels::sdf approach for proper surface generation
        let sphere_sdf = |p: &Point3<Real>| p.coords.norm() - radius;

        // Use explicit bounds and high resolution like the working SDF sphere
        let bounds_margin = radius * 0.2;
        let bounds_min = Point3::new(-radius - bounds_margin, -radius - bounds_margin, -radius - bounds_margin);
        let bounds_max = Point3::new(radius + bounds_margin, radius + bounds_margin, radius + bounds_margin);
        let grid_resolution = (resolution as usize * 4, resolution as usize * 4, resolution as usize * 4);

        Self::sdf(sphere_sdf, grid_resolution, bounds_min, bounds_max, 0.0, metadata)
    }

    /// Create cylinder using SVO voxelization
    pub fn cylinder(radius: Real, height: Real, _segments: usize, metadata: Option<S>) -> Voxels<S> {
        Self::cylinder_with_resolution(radius, height, 6, metadata)
    }

    /// Create cylinder with specified resolution
    pub fn cylinder_with_resolution(radius: Real, height: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        // Use the proven Voxels::sdf approach for proper surface generation
        let cylinder_sdf = |p: &Point3<Real>| {
            let radial_dist = (p.x * p.x + p.y * p.y).sqrt() - radius;
            let height_dist = p.z.abs() - height * 0.5;
            radial_dist.max(height_dist)
        };

        // Use explicit bounds and high resolution like the working SDF sphere
        let bounds_margin = (radius.max(height * 0.5)) * 0.2;
        let bounds_min = Point3::new(-radius - bounds_margin, -radius - bounds_margin, -height * 0.5 - bounds_margin);
        let bounds_max = Point3::new(radius + bounds_margin, radius + bounds_margin, height * 0.5 + bounds_margin);
        let grid_resolution = (resolution as usize * 4, resolution as usize * 4, resolution as usize * 4);

        Self::sdf(cylinder_sdf, grid_resolution, bounds_min, bounds_max, 0.0, metadata)
    }

    // Legacy method names for backward compatibility
    pub fn cube_voxelized(size: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        Self::cube_with_resolution(size, resolution, metadata)
    }

    pub fn sphere_voxelized(radius: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        Self::sphere_with_resolution(radius, resolution, metadata)
    }

    pub fn cylinder_voxelized(radius: Real, height: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        Self::cylinder_with_resolution(radius, height, resolution, metadata)
    }
    /// Create Voxels from an existing SVO
    pub fn from_svo(svo: Svo<S>, metadata: Option<S>) -> Self {
        let center = svo.center;
        let half_size = svo.half;
        let max_depth = svo.max_depth;
        let mut voxels = Self::with_bounds(center, half_size, max_depth);
        *voxels.svo_mut() = svo;
        voxels.metadata = metadata;
        voxels
    }

    /// Voxelize an SDF into the SVO with proper occupancy states
    fn voxelize_sdf<F>(
        svo: &mut Svo<S>,
        sdf: F,
        iso_value: Real,
    ) where
        F: Fn(&Point3<Real>) -> Real,
    {
        Self::voxelize_node(
            &mut *svo.root,
            &svo.center,
            svo.half,
            0,
            svo.max_depth,
            &sdf,
            iso_value,
        );
    }



    /// Recursively voxelize SVO nodes based on SDF
    fn voxelize_node<F>(
        node: &mut SvoNode<S>,
        center: &Point3<Real>,
        half: Real,
        depth: u8,
        max_depth: u8,
        sdf: &F,
        iso_value: Real,
    ) where
        F: Fn(&Point3<Real>) -> Real,
    {
        // Sample the 8 corners of this cell
        let corners = [
            Point3::new(center.x - half, center.y - half, center.z - half),
            Point3::new(center.x + half, center.y - half, center.z - half),
            Point3::new(center.x - half, center.y + half, center.z - half),
            Point3::new(center.x + half, center.y + half, center.z - half),
            Point3::new(center.x - half, center.y - half, center.z + half),
            Point3::new(center.x + half, center.y - half, center.z + half),
            Point3::new(center.x - half, center.y + half, center.z + half),
            Point3::new(center.x + half, center.y + half, center.z + half),
        ];

        let corner_values: Vec<Real> = corners.iter().map(|p| sdf(p)).collect();

        let all_inside = corner_values.iter().all(|&v| v <= iso_value);
        let all_outside = corner_values.iter().all(|&v| v > iso_value);

        if all_inside {
            node.occupancy = Occupancy::Full;
            node.children_mask = 0;
            node.children.clear();
        } else if all_outside {
            node.occupancy = Occupancy::Empty;
            node.children_mask = 0;
            node.children.clear();
        } else {
            // Surface crosses this cell
            node.occupancy = Occupancy::Mixed;

            if depth < max_depth {
                // Recursively subdivide
                for child_idx in 0..8 {
                    let child = node.ensure_child(child_idx);
                    let child_center = Self::child_center(center, half, child_idx);
                    Self::voxelize_node(
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

}