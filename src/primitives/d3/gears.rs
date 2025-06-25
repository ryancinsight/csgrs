//! Gear 3D primitive shapes
//!
//! This module contains various types of gears including involute, cycloid, and helical gears.

use crate::core::float_types::Real;
use crate::csg::CSG;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    pub fn spur_gear_involute(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        CSG::involute_gear_2d(
            module_,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        )
        .extrude(thickness)
    }

    pub fn spur_gear_cycloid(
        module_: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        CSG::cycloidal_gear_2d(
            module_,
            teeth,
            pin_teeth,
            clearance,
            segments_per_flank,
            metadata.clone(),
        )
        .extrude(thickness)
    }

    // -------------------------------------------------------------------------------------------------
    // Helical involute gear (3‑D)                                                                    //
    // -------------------------------------------------------------------------------------------------

    pub fn helical_involute_gear(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        helix_angle_deg: Real, // β
        slices: usize,         // ≥ 2 – axial divisions
        metadata: Option<S>,
    ) -> CSG<S> {
        assert!(slices >= 2);
        let base_slice = CSG::involute_gear_2d(
            module_,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        );

        let dz = thickness / (slices as Real);
        let d_ψ = helix_angle_deg.to_radians() / (slices as Real);

        let mut acc = CSG::<S>::new();
        let mut z_curr = 0.0;
        for i in 0..slices {
            let slice = base_slice
                .rotate(0.0, 0.0, (i as Real) * d_ψ.to_degrees())
                .extrude(dz)
                .translate(0.0, 0.0, z_curr);
            acc = if i == 0 { slice } else { acc.union(&slice) };
            z_curr += dz;
        }
        acc
    }
}
