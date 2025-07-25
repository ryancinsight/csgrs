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
        //  **Enhancement**: Improved padding strategy for hole prevention
        //  Pad the AABB by **multiple voxels** and use adaptive padding
        //  based on the expected TPMS feature size to guarantee proper
        //  sampling at boundaries and prevent aliasing artifacts.
        // ------------------------------------------------------------------
        let res_vec = Vector3::new(
            resolution.0.max(8) as Real, // Increased minimum resolution
            resolution.1.max(8) as Real,
            resolution.2.max(8) as Real,
        );

        let diag = max_pt.coords - min_pt.coords;
        let res_minus_one = res_vec - Vector3::repeat(1.0);
        let cell = diag.component_div(&res_minus_one);

        // **Enhancement**: Multi-level padding strategy
        // 1. Base padding: 2 cells per side (was 1)
        // 2. Adaptive padding: Additional padding based on bounding box size
        let base_padding = cell * 2.0;
        let adaptive_padding = diag * 0.05; // 5% of bounding box
        let total_padding = base_padding + adaptive_padding;

        min_pt = min_pt - total_padding;
        max_pt = max_pt + total_padding;

        // **Enhancement**: Ensure the expanded bounding box has adequate resolution
        let expanded_diag = max_pt.coords - min_pt.coords;
        let min_feature_size = expanded_diag.min(); // Smallest dimension
        let recommended_resolution = (min_feature_size / cell.min() * 2.0) as usize;
        
        let enhanced_resolution = (
            resolution.0.max(recommended_resolution),
            resolution.1.max(recommended_resolution), 
            resolution.2.max(recommended_resolution)
        );

        // Mesh the implicit surface with the generic surface‑nets backend
        let surf = Mesh::sdf(sdf_fn, enhanced_resolution, min_pt, max_pt, iso_value, metadata);
        
        // **Enhancement**: Multi-stage clipping for better boundary handling
        // Stage 1: Clip the infinite TPMS down to the original shape's volume
        let mut clipped = surf.intersection(self);

        // **Enhancement**: Additional boundary smoothing stage
        // Stage 2: Apply boundary-aware smoothing to reduce artifacts
        let bbox_center = (aabb.mins + aabb.maxs.coords) * 0.5;
        let bbox_radius = (aabb.maxs.coords - aabb.mins.coords).norm() * 0.5;
        
        // Apply distance-based smoothing near boundaries
        for polygon in &mut clipped.polygons {
            for vertex in &mut polygon.vertices {
                let dist_from_center = (vertex.pos - bbox_center).norm();
                let proximity_to_boundary = (dist_from_center / bbox_radius).min(1.0);
                
                // **Enhancement**: Smooth normals near boundaries to reduce artifacts
                if proximity_to_boundary > 0.8 {
                    // Blend with radial normal for smoother boundary transitions
                    let radial_normal = (vertex.pos - bbox_center).normalize();
                    let blend_factor = (proximity_to_boundary - 0.8) / 0.2; // Linear ramp from 0.8 to 1.0
                    vertex.normal = vertex.normal.lerp(&radial_normal, blend_factor * 0.3);
                    vertex.normal = vertex.normal.normalize();
                }
            }
        }

        // ------------------------------------------------------------------
        //  **Enhancement**: Advanced vertex welding with multi-scale tolerance
        // ------------------------------------------------------------------
        //  Use a progressive welding strategy: start with fine tolerance
        //  for interior details, then use coarser tolerance for boundary
        //  regions to ensure connectivity without losing detail.
        
        // Stage 1: Fine welding for interior features
        let fine_weld_tol = cell.x.min(cell.y.min(cell.z)) * 0.25;
        clipped.weld_vertices_mut(fine_weld_tol.max(crate::float_types::EPSILON * 2.0));
        
        // Stage 2: Boundary-aware coarse welding
        let coarse_weld_tol = cell.x.min(cell.y.min(cell.z)) * 0.5;
        
        // **Enhancement**: Apply selective welding based on distance from boundary
        // Create a custom welding function that uses different tolerances
        // based on vertex proximity to the original bounding volume
        {
            use hashbrown::HashMap;
            use crate::mesh::plane::Plane;
            
            if !clipped.polygons.is_empty() {
                let boundary_weld_tol = coarse_weld_tol.max(crate::float_types::EPSILON * 4.0);
                
                // Adaptive tolerance based on distance from boundary
                fn adaptive_tolerance(pos: &Point3<Real>, center: &Point3<Real>, radius: Real, 
                                    fine_tol: Real, coarse_tol: Real) -> Real {
                    let dist_from_center = (pos - center).norm();
                    let proximity = (dist_from_center / radius).min(1.0);
                    
                    if proximity > 0.9 {
                        coarse_tol // Near boundary, use coarse tolerance
                    } else {
                        fine_tol // Interior, use fine tolerance
                    }
                }
                
                let mut vmap: HashMap<(i64, i64, i64), (crate::mesh::vertex::Vertex, Real)> = HashMap::new();
                
                for poly in &mut clipped.polygons {
                    for v in &mut poly.vertices {
                        let adaptive_tol = adaptive_tolerance(&v.pos, &bbox_center, bbox_radius, 
                                                            fine_weld_tol, boundary_weld_tol);
                        let cell_size = adaptive_tol;
                        let inv = 1.0 / cell_size;
                        let k = (
                            (v.pos.x * inv).round() as i64,
                            (v.pos.y * inv).round() as i64,
                            (v.pos.z * inv).round() as i64,
                        );
                        
                        if let Some((canon, _)) = vmap.get(&k) {
                            *v = canon.clone();
                        } else {
                            vmap.insert(k, (v.clone(), adaptive_tol));
                        }
                    }
                    
                    // Recompute plane from (potentially modified) vertices
                    poly.plane = Plane::from_vertices(poly.vertices.clone());
                    poly.bounding_box = std::sync::OnceLock::new();
                }
                
                clipped.invalidate_bounding_box();
            }
        }

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