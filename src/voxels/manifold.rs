use crate::float_types::Real;
use crate::voxels::csg::Voxels;
use nalgebra::Point3;
use std::collections::HashMap;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    pub fn is_manifold(&self) -> bool {
        const QUANTIZATION_FACTOR: Real = 1e6;
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)] struct QuantizedPoint(i64,i64,i64);
        fn quantize_point(p: &Point3<Real>) -> QuantizedPoint { QuantizedPoint((p.x*QUANTIZATION_FACTOR).round() as i64, (p.y*QUANTIZATION_FACTOR).round() as i64, (p.z*QUANTIZATION_FACTOR).round() as i64) }
        let mut edge_counts: HashMap<(QuantizedPoint,QuantizedPoint), u32> = HashMap::new();
        for poly in self.polygons() { for tri in poly.triangulate() { for &(i0,i1) in &[(0,1),(1,2),(2,0)] { let p0 = quantize_point(&tri[i0].pos); let p1 = quantize_point(&tri[i1].pos); let (a,b) = if (p0.0,p0.1,p0.2) < (p1.0,p1.1,p1.2) { (p0,p1) } else { (p1,p0) }; *edge_counts.entry((a,b)).or_insert(0) += 1; } } }
        edge_counts.values().all(|&count| count == 2)
    }
}

