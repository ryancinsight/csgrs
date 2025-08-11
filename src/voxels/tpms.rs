//! TPMS for voxel pipelines using voxels::sdf mesher and voxels CSG

use crate::float_types::Real;
use crate::voxels::csg::Voxels;
use crate::traits::CSG;
use nalgebra::Point3;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    #[inline]
    fn tpms_from_sdf<F>(&self, sdf_fn: F, resolution: (usize, usize, usize), iso_value: Real, metadata: Option<S>) -> Voxels<S>
    where F: Fn(&Point3<Real>) -> Real + Send + Sync {
        let aabb = self.bounding_box();
        let min_pt = aabb.mins; let max_pt = aabb.maxs;
        let surf = Voxels::sdf(sdf_fn, resolution, min_pt, max_pt, iso_value, metadata);
        surf.intersection(self)
    }

    pub fn gyroid(&self, resolution: usize, period: Real, iso_value: Real, metadata: Option<S>) -> Voxels<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(move |p: &Point3<Real>| {
            let x = p.x * period_inv; let y = p.y * period_inv; let z = p.z * period_inv;
            let (sx, cx) = x.sin_cos(); let (sy, cy) = y.sin_cos(); let (sz, cz) = z.sin_cos();
            (sx * cy) + (sy * cz) + (sz * cx)
        }, res, iso_value, metadata)
    }

    pub fn schwarz_p(&self, resolution: usize, period: Real, iso_value: Real, metadata: Option<S>) -> Voxels<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(move |p: &Point3<Real>| {
            let x = p.x * period_inv; let y = p.y * period_inv; let z = p.z * period_inv;
            x.cos() + y.cos() + z.cos()
        }, res, iso_value, metadata)
    }

    pub fn schwarz_d(&self, resolution: usize, period: Real, iso_value: Real, metadata: Option<S>) -> Voxels<S> {
        let res = (resolution.max(2), resolution.max(2), resolution.max(2));
        let period_inv = 1.0 / period;
        self.tpms_from_sdf(move |p: &Point3<Real>| {
            let x = p.x * period_inv; let y = p.y * period_inv; let z = p.z * period_inv;
            let (sx, cx) = x.sin_cos(); let (sy, cy) = y.sin_cos(); let (sz, cz) = z.sin_cos();
            (sx * sy * sz) + (sx * cy * cz) + (cx * sy * cz) + (cx * cy * sz)
        }, res, iso_value, metadata)
    }
}

