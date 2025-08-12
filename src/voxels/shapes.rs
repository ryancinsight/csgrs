//! Shape generation for Voxels using SVO-based voxelization
//!
//! This module provides unified shape generation that builds SVO directly,
//! using voxelization and embedded BSP for surface representation.

use crate::float_types::Real;
use crate::voxels::Svo;
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
        let cuboid_sdf = |p: &Point3<Real>| {
            let dx = p.x.abs() - width * 0.5;
            let dy = p.y.abs() - length * 0.5;
            let dz = p.z.abs() - height * 0.5;
            let outside_dist = (dx.max(0.0).powi(2) + dy.max(0.0).powi(2) + dz.max(0.0).powi(2)).sqrt();
            let inside_dist = dx.max(dy).max(dz).min(0.0);
            outside_dist + inside_dist
        };

        let bounds_margin = (width.max(length).max(height)) * 0.1;
        let bounds_min = Point3::new(-width * 0.5 - bounds_margin, -length * 0.5 - bounds_margin, -height * 0.5 - bounds_margin);
        let bounds_max = Point3::new(width * 0.5 + bounds_margin, length * 0.5 + bounds_margin, height * 0.5 + bounds_margin);

        Self::from_sdf(cuboid_sdf, bounds_min, bounds_max, resolution, metadata)
    }

    /// Create sphere using SVO voxelization
    pub fn sphere(radius: Real, _lat_segments: usize, _lon_segments: usize, metadata: Option<S>) -> Voxels<S> {
        Self::sphere_with_resolution(radius, 6, metadata)
    }

    /// Create sphere with specified resolution
    pub fn sphere_with_resolution(radius: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        let sphere_sdf = |p: &Point3<Real>| p.coords.norm() - radius;
        let bounds_margin = radius * 0.2;
        let bounds_min = Point3::new(-radius - bounds_margin, -radius - bounds_margin, -radius - bounds_margin);
        let bounds_max = Point3::new(radius + bounds_margin, radius + bounds_margin, radius + bounds_margin);

        Self::from_sdf(sphere_sdf, bounds_min, bounds_max, resolution, metadata)
    }

    /// Create cylinder using SVO voxelization (matches mesh API)
    pub fn cylinder(radius: Real, height: Real, _segments: usize, metadata: Option<S>) -> Voxels<S> {
        let cylinder_sdf = |p: &Point3<Real>| {
            let radial_dist = (p.x * p.x + p.y * p.y).sqrt() - radius;
            let height_dist = p.z.abs() - height * 0.5;
            radial_dist.max(height_dist)
        };

        let bounds_margin = (radius.max(height * 0.5)) * 0.2;
        let bounds_min = Point3::new(-radius - bounds_margin, -radius - bounds_margin, -height * 0.5 - bounds_margin);
        let bounds_max = Point3::new(radius + bounds_margin, radius + bounds_margin, height * 0.5 + bounds_margin);

        Self::from_sdf(cylinder_sdf, bounds_min, bounds_max, 6, metadata)
    }

    /// Create cylinder with specified resolution
    pub fn cylinder_with_resolution(radius: Real, height: Real, resolution: u8, metadata: Option<S>) -> Voxels<S> {
        let cylinder_sdf = |p: &Point3<Real>| {
            let radial_dist = (p.x * p.x + p.y * p.y).sqrt() - radius;
            let height_dist = p.z.abs() - height * 0.5;
            radial_dist.max(height_dist)
        };

        let bounds_margin = (radius.max(height * 0.5)) * 0.2;
        let bounds_min = Point3::new(-radius - bounds_margin, -radius - bounds_margin, -height * 0.5 - bounds_margin);
        let bounds_max = Point3::new(radius + bounds_margin, radius + bounds_margin, height * 0.5 + bounds_margin);

        Self::from_sdf(cylinder_sdf, bounds_min, bounds_max, resolution, metadata)
    }


    /// Create Voxels from SDF function using optimal bounds and resolution
    ///
    /// This is the core shape generation method that all other shapes use.
    /// Follows the Single Responsibility Principle by focusing solely on SDF-to-Voxels conversion.
    pub fn from_sdf<F>(
        sdf: F,
        bounds_min: Point3<Real>,
        bounds_max: Point3<Real>,
        resolution: u8,
        metadata: Option<S>,
    ) -> Voxels<S>
    where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
    {
        let grid_resolution = (resolution as usize * 4, resolution as usize * 4, resolution as usize * 4);
        Self::sdf(sdf, grid_resolution, bounds_min, bounds_max, 0.0, metadata)
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

}