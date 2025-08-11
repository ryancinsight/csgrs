use crate::float_types::{PI, Real};
use crate::voxels::csg::Voxels;
use crate::voxels::vertex::Vertex;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[derive(Debug, Clone)]
pub struct TriangleQuality { pub aspect_ratio: Real, pub min_angle: Real, pub max_angle: Real, pub edge_ratio: Real, pub area: Real, pub quality_score: Real }

#[derive(Debug, Clone)]
pub struct MeshQualityMetrics { pub avg_quality: Real, pub min_quality: Real, pub high_quality_ratio: Real, pub sliver_count: usize, pub avg_edge_length: Real, pub edge_length_std: Real }

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    pub fn analyze_triangle_quality(&self) -> Vec<TriangleQuality> {
        // Ensure triangles by triangulating polygons directly
        #[cfg(feature = "parallel")] let qualities: Vec<TriangleQuality> = self.polygons.par_iter().flat_map(|p| p.triangulate()).map(|tri| Self::compute_triangle_quality(&tri)).collect();
        #[cfg(not(feature = "parallel"))] let qualities: Vec<TriangleQuality> = self.polygons().iter().flat_map(|p| p.triangulate()).map(|tri| Self::compute_triangle_quality(tri.as_slice())).collect();
        qualities
    }

    fn compute_triangle_quality(vertices: &[Vertex]) -> TriangleQuality {
        if vertices.len() != 3 { return TriangleQuality { aspect_ratio: Real::INFINITY, min_angle: 0.0, max_angle: 0.0, edge_ratio: Real::INFINITY, area: 0.0, quality_score: 0.0 }; }
        let a = vertices[0].pos; let b = vertices[1].pos; let c = vertices[2].pos;
        let ab = b - a; let bc = c - b; let ca = a - c;
        let len_ab = ab.norm(); let len_bc = bc.norm(); let len_ca = ca.norm();
        if len_ab < Real::EPSILON || len_bc < Real::EPSILON || len_ca < Real::EPSILON { return TriangleQuality { aspect_ratio: Real::INFINITY, min_angle: 0.0, max_angle: 0.0, edge_ratio: Real::INFINITY, area: 0.0, quality_score: 0.0 }; }
        let area = 0.5 * ab.cross(&(-ca)).norm();
        if area < Real::EPSILON { return TriangleQuality { aspect_ratio: Real::INFINITY, min_angle: 0.0, max_angle: 0.0, edge_ratio: len_ab.max(len_bc).max(len_ca) / len_ab.min(len_bc).min(len_ca), area: 0.0, quality_score: 0.0 }; }
        let angle_a = ((len_bc.powi(2) + len_ca.powi(2) - len_ab.powi(2)) / (2.0 * len_bc * len_ca)).acos();
        let angle_b = ((len_ca.powi(2) + len_ab.powi(2) - len_bc.powi(2)) / (2.0 * len_ca * len_ab)).acos();
        let angle_c = ((len_ab.powi(2) + len_bc.powi(2) - len_ca.powi(2)) / (2.0 * len_ab * len_bc)).acos();
        let min_angle = angle_a.min(angle_b).min(angle_c); let max_angle = angle_a.max(angle_b).max(angle_c);
        let min_edge = len_ab.min(len_bc).min(len_ca); let max_edge = len_ab.max(len_bc).max(len_ca); let edge_ratio = max_edge / min_edge;
        let semiperimeter = (len_ab + len_bc + len_ca) / 2.0; let circumradius = (len_ab * len_bc * len_ca) / (4.0 * area); let inradius = area / semiperimeter; let aspect_ratio = circumradius / inradius;
        let angle_quality = (min_angle / (PI / 6.0)).min(1.0); let shape_quality = (1.0 / aspect_ratio).min(1.0); let edge_quality = (3.0 / edge_ratio).min(1.0);
        let quality_score = (0.4 * angle_quality + 0.4 * shape_quality + 0.2 * edge_quality).clamp(0.0, 1.0);
        TriangleQuality { aspect_ratio, min_angle, max_angle, edge_ratio, area, quality_score }
    }

    pub fn compute_mesh_quality(&self) -> MeshQualityMetrics {
        let qualities = self.analyze_triangle_quality();
        if qualities.is_empty() { return MeshQualityMetrics { avg_quality: 0.0, min_quality: 0.0, high_quality_ratio: 0.0, sliver_count: 0, avg_edge_length: 0.0, edge_length_std: 0.0 }; }
        let total_quality: Real = qualities.iter().map(|q| q.quality_score).sum(); let avg_quality = total_quality / qualities.len() as Real;
        let min_quality = qualities.iter().map(|q| q.quality_score).fold(Real::INFINITY, |a,b| a.min(b));
        let high_quality_count = qualities.iter().filter(|q| q.quality_score > 0.7).count(); let high_quality_ratio = high_quality_count as Real / qualities.len() as Real;
        let sliver_count = qualities.iter().filter(|q| q.min_angle < (10.0 as Real).to_radians()).count();
        let edge_lengths: Vec<Real> = self.polygons().iter().flat_map(|poly| poly.vertices.windows(2).map(|w| (w[1].pos - w[0].pos).norm()).chain(std::iter::once((poly.vertices[0].pos - poly.vertices.last().unwrap().pos).norm()))).collect();
        let avg_edge_length = if !edge_lengths.is_empty() { edge_lengths.iter().sum::<Real>() / edge_lengths.len() as Real } else { 0.0 };
        let edge_length_variance = if edge_lengths.len() > 1 { let variance: Real = edge_lengths.iter().map(|&len| (len - avg_edge_length).powi(2)).sum::<Real>() / (edge_lengths.len() - 1) as Real; variance.sqrt() } else { 0.0 };
        MeshQualityMetrics { avg_quality, min_quality, high_quality_ratio, sliver_count, avg_edge_length, edge_length_std: edge_length_variance }
    }
}

