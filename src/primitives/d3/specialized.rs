//! Specialized 3D primitive shapes
//!
//! This module contains specialized shapes like arrows, eggs, and teardrops.

use crate::core::float_types::{EPSILON, Real};
use crate::csg::CSG;
use nalgebra::{Matrix4, Point3, Rotation3, Translation3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Creates an arrow CSG. The arrow is composed of:
    ///   - a cylindrical shaft, and
    ///   - a cone–like head (a frustum from a larger base to a small tip)
    /// built along the canonical +Z axis. The arrow is then rotated so that +Z aligns with the given
    /// direction, and finally translated so that either its base (if `orientation` is false)
    /// or its tip (if `orientation` is true) is located at `start`.
    ///
    /// The arrow's dimensions (shaft radius, head dimensions, etc.) are scaled proportionally to the
    /// total arrow length (the norm of the provided direction).
    ///
    /// # Parameters
    /// - `start`: the reference point (base or tip, depending on orientation)
    /// - `direction`: the vector defining arrow length and intended pointing direction
    /// - `segments`: number of segments for approximating the cylinder and frustum
    /// - `orientation`: when false (default) the arrow points away from start (its base is at start);
    ///                        when true the arrow points toward start (its tip is at start).
    /// - `metadata`: optional metadata for the generated polygons.
    pub fn arrow(
        start: Point3<Real>,
        direction: Vector3<Real>,
        segments: usize,
        orientation: bool,
        metadata: Option<S>,
    ) -> CSG<S> {
        // Compute the arrow's total length.
        let arrow_length = direction.norm();
        if arrow_length < EPSILON {
            return CSG::new();
        }
        // Compute the unit direction.
        let unit_dir = direction / arrow_length;

        // Define proportions:
        // - Arrow head occupies 20% of total length.
        // - Shaft occupies the remainder.
        let head_length = arrow_length * 0.2;
        let shaft_length = arrow_length - head_length;

        // Define thickness parameters proportional to the arrow length.
        let shaft_radius = arrow_length * 0.03; // shaft radius
        let head_base_radius = arrow_length * 0.06; // head base radius (wider than shaft)
        let tip_radius = arrow_length * 0.0; // tip radius (nearly a point)

        // Build the shaft as a vertical cylinder along Z from 0 to shaft_length.
        let shaft = CSG::cylinder(shaft_radius, shaft_length, segments, metadata.clone());

        // Build the arrow head as a frustum from z = shaft_length to z = shaft_length + head_length.
        let head = CSG::frustum_ptp(
            Point3::new(0.0, 0.0, shaft_length),
            Point3::new(0.0, 0.0, shaft_length + head_length),
            head_base_radius,
            tip_radius,
            segments,
            metadata.clone(),
        );

        // Combine the shaft and head.
        let mut canonical_arrow = shaft.union(&head);

        // If the arrow should point toward start, mirror the geometry in canonical space.
        // The mirror transform about the plane z = arrow_length/2 maps any point (0,0,z) to (0,0, arrow_length - z).
        if orientation {
            let l = arrow_length;
            let mirror_mat: Matrix4<Real> = Translation3::new(0.0, 0.0, l / 2.0)
                .to_homogeneous()
                * Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, 1.0, -1.0))
                * Translation3::new(0.0, 0.0, -l / 2.0).to_homogeneous();
            canonical_arrow = canonical_arrow.transform(&mirror_mat).inverse();
        }
        // In both cases, we now have a canonical arrow that extends from z=0 to z=arrow_length.
        // For orientation == false, z=0 is the base.
        // For orientation == true, after mirroring z=0 is now the tip.

        // Compute the rotation that maps the canonical +Z axis to the provided direction.
        let z_axis = Vector3::z();
        let rotation = Rotation3::rotation_between(&z_axis, &unit_dir)
            .unwrap_or_else(Rotation3::identity);
        let rot_mat: Matrix4<Real> = rotation.to_homogeneous();

        // Rotate the arrow.
        let rotated_arrow = canonical_arrow.transform(&rot_mat);

        // Finally, translate the arrow so that the anchored vertex (canonical (0,0,0)) moves to 'start'.
        // In the false case, (0,0,0) is the base (arrow extends from start to start+direction).
        // In the true case, after mirroring, (0,0,0) is the tip (arrow extends from start to start+direction).
        let final_arrow = rotated_arrow.translate(start.x, start.y, start.z);

        final_arrow
    }

    /// Creates a 3D "egg" shape by revolving the existing 2D `egg_outline` profile.
    ///
    /// # Parameters
    /// - `width`: The "width" of the 2D egg outline.
    /// - `length`: The "length" (height) of the 2D egg outline.
    /// - `revolve_segments`: Number of segments for the revolution.
    /// - `outline_segments`: Number of segments for the 2D egg outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "chull-io")]
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        let egg_2d = Self::egg_outline(width, length, outline_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(cutter_height, metadata.clone()).translate(
            -cutter_height,
            -cutter_height / 2.0,
            0.0,
        );

        let half_egg = egg_2d.difference(&rect_cutter);

        half_egg
            .rotate_extrude(360.0, revolve_segments)
            .unwrap()
            .convex_hull()
    }

    /// Creates a 3D "teardrop" solid by revolving the existing 2D `teardrop` profile 360° around the Y-axis (via rotate_extrude).
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "chull-io")]
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Self::teardrop_outline(width, length, shape_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(cutter_height, metadata.clone()).translate(
            -cutter_height,
            -cutter_height / 2.0,
            0.0,
        );

        let half_teardrop = td_2d.difference(&rect_cutter);

        // revolve 360 degrees
        half_teardrop
            .rotate_extrude(360.0, revolve_segments)
            .unwrap()
            .convex_hull()
    }

    /// Creates a 3D "teardrop cylinder" by extruding the existing 2D `teardrop` in the Z+ axis.
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `height`: Height of the extrusion.
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "chull-io")]
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Self::teardrop_outline(width, length, shape_segments, metadata.clone());
        td_2d.extrude(height).convex_hull()
    }
}
