//! 2D gear profiles and related mechanical components
//!
//! This module provides functions for generating various types of gear teeth profiles
//! including involute gears, cycloidal gears, and their corresponding rack profiles.

use super::utils::{epicycloid_xy, hypocycloid_xy, involute_angle_at_radius, involute_xy};
use crate::core::float_types::{EPSILON, PI, Real, TAU};
use crate::csg::CSG;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Generate an involute gear outline (2‑D).
    ///
    /// # Parameters
    /// - `module_`: gear module (pitch diameter / number of teeth)
    /// - `teeth`: number of teeth (>= 4)
    /// - `pressure_angle_deg`: pressure angle in degrees (typically 20°)
    /// - `clearance`: additional clearance for dedendum
    /// - `backlash`: backlash allowance
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    /// - `metadata`: optional metadata
    pub fn involute_gear_2d(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        assert!(teeth >= 4, "Need at least 4 teeth for a valid gear");
        assert!(segments_per_flank >= 3);

        // Standard proportions (ISO 21771)
        let m = module_;
        let z = teeth as Real;
        let pitch_radius = 0.5 * m * z;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;

        let rb = pitch_radius * (1.0_f64.to_radians() as Real * pressure_angle_deg).cos();
        let ra = pitch_radius + addendum;
        let rf = (pitch_radius - dedendum).max(0.0);

        // Angular pitch and base offsets
        let ang_pitch = TAU / z;
        let tooth_thick_ang = ang_pitch / 2.0 - backlash / pitch_radius;

        // φ at pitch and addendum circles
        let phi_p = involute_angle_at_radius(pitch_radius, rb);
        let phi_a = involute_angle_at_radius(ra, rb);

        // Helper to build a single half‑flank (left‑hand)
        let mut half_flank = Vec::<(Real, Real)>::with_capacity(segments_per_flank + 1);
        for i in 0..=segments_per_flank {
            let phi = phi_p + (phi_a - phi_p) * (i as Real) / (segments_per_flank as Real);
            let (ix, iy) = involute_xy(rb, phi);
            let theta = (iy).atan2(ix); // polar angle of involute point
            let global_theta = -tooth_thick_ang + theta; // left side offset
            let r = (ix * ix + iy * iy).sqrt();
            half_flank.push((r * global_theta.cos(), r * global_theta.sin()));
        }

        // Mirror to get right‑hand flank (reverse order so outline is CCW)
        let mut full_tooth = half_flank;
        for i in (0..full_tooth.len()).rev() {
            let (x, y) = full_tooth[i];
            // mirror across X axis and shift right
            let theta = (-y).atan2(x);
            let r = (x * x + y * y).sqrt();
            let global_theta = tooth_thick_ang - theta;
            full_tooth.push((r * global_theta.cos(), r * global_theta.sin()));
        }

        // Root circle arc between successive teeth
        let root_arc_steps = 4;
        let arc_step = (ang_pitch - 2.0 * tooth_thick_ang) / (root_arc_steps as Real);
        for i in 1..=root_arc_steps {
            let ang = tooth_thick_ang + (i as Real) * arc_step;
            full_tooth.push((rf * (ang).cos(), rf * (ang).sin()));
        }

        // Replicate the tooth profile around the gear
        let mut outline = Vec::<[Real; 2]>::with_capacity(full_tooth.len() * teeth + 1);
        for tooth_idx in 0..teeth {
            let rot = (tooth_idx as Real) * ang_pitch;
            let (c, s) = (rot.cos(), rot.sin());
            for &(x, y) in &full_tooth {
                outline.push([x * c - y * s, x * s + y * c]);
            }
        }
        // Close path
        outline.push(outline[0]);

        CSG::polygon(&outline, metadata)
    }

    /// Generate an (epicyclic) cycloidal gear outline (2‑D).
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `teeth`: number of teeth (>= 3)
    /// - `pin_teeth`: number of teeth in the pin wheel for pairing
    /// - `clearance`: additional clearance for dedendum
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    /// - `metadata`: optional metadata
    pub fn cycloidal_gear_2d(
        module_: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        assert!(teeth >= 3 && pin_teeth >= 3);
        let m = module_;
        let z = teeth as Real;
        let z_p = pin_teeth as Real; // for pin‑wheel pairing

        // Pitch and derived radii
        let r_p = 0.5 * m * z; // gear pitch radius
        let r_g = 0.5 * m * z_p; // (made‑up) mating wheel for hypocycloid – gives correct root
        let r_pin = r_p / z; // generating circle radius (standard assumes z_p = z ± 1)

        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let _ra = r_p + addendum;
        let rf = (r_p - dedendum).max(0.0);

        let ang_pitch = TAU / z;
        let flank_steps = segments_per_flank.max(4);

        let mut tooth_points = Vec::<(Real, Real)>::new();

        // 1. addendum epicycloid (tip)
        for i in 0..=flank_steps {
            let t = (i as Real) / (flank_steps as Real);
            let theta = t * ang_pitch / 2.0;
            let (x, y) = epicycloid_xy(r_p, r_pin, theta);
            tooth_points.push((x, y));
        }
        // 2. hypocycloid root (reverse order to keep CCW)
        for i in (0..=flank_steps).rev() {
            let t = (i as Real) / (flank_steps as Real);
            let theta = t * ang_pitch / 2.0;
            let (x, y) = hypocycloid_xy(r_g, r_pin, theta);
            let r = (x * x + y * y).sqrt();
            if r < rf - EPSILON {
                tooth_points.push((rf * theta.cos(), rf * theta.sin()));
            } else {
                tooth_points.push((x, y));
            }
        }

        // Replicate
        let mut outline = Vec::<[Real; 2]>::with_capacity(tooth_points.len() * teeth + 1);
        for k in 0..teeth {
            let rot = (k as Real) * ang_pitch;
            let (c, s) = (rot.cos(), rot.sin());
            for &(x, y) in &tooth_points {
                outline.push([x * c - y * s, x * s + y * c]);
            }
        }
        outline.push(outline[0]);

        CSG::polygon(&outline, metadata)
    }

    /// Generate a linear involute rack profile (lying in the XY plane, pitch‑line on Y = 0).
    /// The returned polygon is CCW and spans `num_teeth` pitches along +X.
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `num_teeth`: number of teeth along the rack
    /// - `pressure_angle_deg`: pressure angle in degrees
    /// - `clearance`: additional clearance for dedendum
    /// - `backlash`: backlash allowance
    /// - `metadata`: optional metadata
    pub fn involute_rack_2d(
        module_: Real,
        num_teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        assert!(num_teeth >= 1);
        let m = module_;
        let p = PI * m; // linear pitch
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let tip_y = addendum;
        let root_y = -dedendum;

        // Tooth thickness at pitch‑line (centre) minus backlash.
        let t = p / 2.0 - backlash;
        let half_t = t / 2.0;

        // Flank rises with slope = tan(pressure_angle)
        let alpha = pressure_angle_deg.to_radians();
        let rise = tip_y; // from pitch‑line (0) up to tip
        let run = rise / alpha.tan();

        // Build one tooth (start at pitch centre) – CCW
        // Points: Root‑left → Flank‑left → Tip‑left → Tip‑right → Flank‑right → Root‑right
        let tooth: Vec<[Real; 2]> = vec![
            [-half_t - run, root_y], // root left beneath flank
            [-half_t, 0.0],          // pitch left
            [-half_t + run, tip_y],  // tip left
            [half_t - run, tip_y],   // tip right
            [half_t, 0.0],           // pitch right
            [half_t + run, root_y],  // root right
        ];

        // Repeat teeth
        let mut outline = Vec::<[Real; 2]>::with_capacity(tooth.len() * num_teeth + 4);
        for i in 0..num_teeth {
            let dx = (i as Real) * p;
            for &[x, y] in &tooth {
                outline.push([x + dx, y]);
            }
        }

        // Close rectangle ends (simple straight ends) and add extensions
        outline.push([outline.last().unwrap()[0], 0.0]);
        outline.push([outline[0][0], 0.0]);
        outline.push(outline[0]);

        CSG::polygon(&outline, metadata)
    }

    /// Generate a linear cycloidal rack profile.
    /// The cycloidal rack is generated by rolling a circle of radius `r_p` along the
    /// rack's pitch‑line. The flanks become a trochoid; for practical purposes we
    /// approximate with the classic curtate cycloid equations.
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `num_teeth`: number of teeth along the rack
    /// - `generating_radius`: radius of the generating circle (usually = module_/2)
    /// - `clearance`: additional clearance for dedendum
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    /// - `metadata`: optional metadata
    pub fn cycloidal_rack_2d(
        module_: Real,
        num_teeth: usize,
        generating_radius: Real, // usually = module_/2
        clearance: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        assert!(num_teeth >= 1 && segments_per_flank >= 4);
        let m = module_;
        let p = PI * m;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let _tip_y = addendum;
        let root_y = -dedendum;

        let r = generating_radius;

        // Curtate cycloid y(t) spans 0..2πr giving height 2r.
        // We scale t so that y range equals addendum (= m)
        let scale = addendum / (2.0 * r);

        let mut flank: Vec<[Real; 2]> = Vec::with_capacity(segments_per_flank);
        for i in 0..=segments_per_flank {
            let t = PI * (i as Real) / (segments_per_flank as Real); // 0..π gives half‑trochoid
            let x = r * (t - t.sin());
            let y = r * (1.0 - t.cos());
            flank.push([x * scale, y * scale]);
        }

        // Build one tooth (CCW): left flank, mirrored right flank, root bridge
        let mut tooth: Vec<[Real; 2]> = Vec::with_capacity(flank.len() * 2 + 2);
        // Left side (reverse so CCW)
        for &[x, y] in flank.iter().rev() {
            tooth.push([-x, y]);
        }
        // Right side
        for &[x, y] in &flank {
            tooth.push([x, y]);
        }
        // Root bridge
        let bridge = tooth.last().unwrap()[0] + 2.0 * (r * scale - flank.last().unwrap()[0]);
        tooth.push([bridge, root_y]);
        tooth.push([-bridge, root_y]);

        // Repeat
        let mut outline = Vec::<[Real; 2]>::with_capacity(tooth.len() * num_teeth + 1);
        for k in 0..num_teeth {
            let dx = (k as Real) * p;
            for &[x, y] in &tooth {
                outline.push([x + dx, y]);
            }
        }
        outline.push(outline[0]);

        CSG::polygon(&outline, metadata)
    }
}
