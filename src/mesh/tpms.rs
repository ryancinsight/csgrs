//! Triply‑Periodic Minimal Surfaces rewritten to leverage the generic
//! signed‑distance mesher in `sdf.rs`.

use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::traits::CSG;
use nalgebra::Point3;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// **Generic helper** – build a TPMS inside `self` from the provided SDF.
    ///
    /// * `sdf_fn`     – smooth signed‑distance field _f(p)_; 0‑level set is the surface
    /// * `resolution` – voxel grid sampling resolution `(nx, ny, nz)`
    /// * `iso_value`  – iso‑contour value (normally 0.0)
    ///
    /// The result is intersected against `self`, so the surface only appears inside
    /// the original solid's bounding box / volume.
    #[inline]
    fn tpms_from_sdf<F>(
        &self,
        sdf_fn: F,
        resolution: (usize, usize, usize),
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S>
    where
        F: Fn(&Point3<Real>) -> Real + Send + Sync,
    {
        use nalgebra::Vector3;

        let aabb = self.bounding_box();
        let mut min_pt = aabb.mins;
        let mut max_pt = aabb.maxs;

        // ------------------------------------------------------------------
        //  Pad the AABB by **one voxel** on each side to guarantee that the
        //  implicit surface is fully sampled at the solid boundary.  Without
        //  this, aliasing can leave pin-holes where the TPMS meets the
        //  bounding sphere (or any other host geometry).
        // ------------------------------------------------------------------
        let res_vec = Vector3::new(
            resolution.0.max(2) as Real,
            resolution.1.max(2) as Real,
            resolution.2.max(2) as Real,
        );

        let diag = max_pt.coords - min_pt.coords;
        let res_minus_one = res_vec - Vector3::repeat(1.0);
        let cell = diag.component_div(&res_minus_one);

        // One-cell padding each side
        min_pt = min_pt - cell;
        max_pt = max_pt + cell;
        // Mesh the implicit surface with the generic surface‑nets backend
        let surf = Mesh::sdf(sdf_fn, resolution, min_pt, max_pt, iso_value, metadata);
        // Clip the infinite TPMS down to the original shape's volume
        let mut clipped = surf.intersection(self);

        // ------------------------------------------------------------------
        //  Final integrity pass – vertex welding
        // ------------------------------------------------------------------
        //  Use a tolerance proportional to the sampling cell size.  This
        //  scales naturally with user-supplied resolution and prevents both
        //  under- and over-welding on very small or very large models.
        let weld_tol = cell.x.min(cell.y.min(cell.z)); // smallest cell edge
        clipped.weld_vertices_mut(weld_tol.max(crate::float_types::EPSILON * 4.0));

        clipped
    }

    // ------------  Specific minimal‑surface flavours  --------------------

    /// Gyroid surface:  `sin x cos y + sin y cos z + sin z cos x = iso`  
    /// (`period` rescales the spatial frequency; larger => slower repeat)
    /// **Mathematical Foundation**: Gyroid is a triply periodic minimal surface with zero mean curvature.
    /// **Optimization**: Pre-compute trigonometric values for better performance.
    pub fn gyroid(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                // Pre-compute scaled coordinates for efficiency
                let x_scaled = p.x * period_inv;
                let y_scaled = p.y * period_inv;
                let z_scaled = p.z * period_inv;

                // Pre-compute trigonometric values to avoid redundant calculations
                let (sin_x, cos_x) = x_scaled.sin_cos();
                let (sin_y, cos_y) = y_scaled.sin_cos();
                let (sin_z, cos_z) = z_scaled.sin_cos();

                // **Mathematical Formula**: Gyroid surface equation
                // G(x,y,z) = sin(x)cos(y) + sin(y)cos(z) + sin(z)cos(x)
                (sin_x * cos_y) + (sin_y * cos_z) + (sin_z * cos_x)
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Schwarz‑P surface:  `cos x + cos y + cos z = iso`  (default iso = 0)
    /// **Mathematical Foundation**: Schwarz P-surface has constant mean curvature and cubic symmetry.
    /// **Optimization**: Use direct cosine computation for this simpler surface equation.
    pub fn schwarz_p(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                // Pre-compute scaled coordinates
                let x_scaled = p.x * period_inv;
                let y_scaled = p.y * period_inv;
                let z_scaled = p.z * period_inv;

                // **Mathematical Formula**: Schwarz P-surface equation
                // P(x,y,z) = cos(x) + cos(y) + cos(z)
                x_scaled.cos() + y_scaled.cos() + z_scaled.cos()
            },
            res,
            iso_value,
            metadata,
        )
    }

    /// Schwarz-D (Diamond) surface:  `sin x sin y sin z + sin x cos y cos z + ... = iso`
    /// **Mathematical Foundation**: Diamond surface exhibits tetrahedral symmetry and is self-intersecting.
    /// **Optimization**: Pre-compute all trigonometric values for maximum efficiency.
    pub fn schwarz_d(
        &self,
        resolution: usize,
        period: Real,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(
            move |p: &Point3<Real>| {
                // Pre-compute scaled coordinates
                let x_scaled = p.x * period_inv;
                let y_scaled = p.y * period_inv;
                let z_scaled = p.z * period_inv;

                // Pre-compute all trigonometric values once
                let (sin_x, cos_x) = x_scaled.sin_cos();
                let (sin_y, cos_y) = y_scaled.sin_cos();
                let (sin_z, cos_z) = z_scaled.sin_cos();

                // **Mathematical Formula**: Schwarz Diamond surface equation
                // D(x,y,z) = sin(x)sin(y)sin(z) + sin(x)cos(y)cos(z) + cos(x)sin(y)cos(z) + cos(x)cos(y)sin(z)
                (sin_x * sin_y * sin_z)
                    + (sin_x * cos_y * cos_z)
                    + (cos_x * sin_y * cos_z)
                    + (cos_x * cos_y * sin_z)
            },
            res,
            iso_value,
            metadata,
        )
    }
}