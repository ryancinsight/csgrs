use crate::float_types::Real;
use crate::voxels::csg::Voxels;
use nalgebra::Point3;
use std::collections::HashMap;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    pub fn laplacian_smooth(&self, lambda: Real, iterations: usize, preserve_boundaries: bool) -> Voxels<S> {
        let (vertex_map, adjacency) = crate::voxels::connectivity::PolyMesh { polygons: self.polygons().to_vec() }.build_connectivity();
        let mut smoothed_polygons = self.polygons().to_vec();
        for _ in 0..iterations {
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            for polygon in &smoothed_polygons { for v in &polygon.vertices {
                for (pos, idx) in &vertex_map.position_to_index { if (v.pos - pos).norm() < vertex_map.epsilon { current_positions.insert(*idx, v.pos); break; } }
            }}
            let mut updates: HashMap<usize, Point3<Real>> = HashMap::new();
            for (&vid, neighbors) in &adjacency {
                if let Some(&p) = current_positions.get(&vid) {
                    if preserve_boundaries && neighbors.len() < 4 { updates.insert(vid, p); continue; }
                    let mut sum = Point3::origin(); let mut count = 0;
                    for &nid in neighbors { if let Some(&np) = current_positions.get(&nid) { sum += np.coords; count += 1; } }
                    if count > 0 { let avg = sum / count as Real; let newp = p + (avg - p) * lambda; updates.insert(vid, newp); } else { updates.insert(vid, p); }
                }
            }
            for polygon in &mut smoothed_polygons { for v in &mut polygon.vertices {
                for (pos, idx) in &vertex_map.position_to_index { if (v.pos - pos).norm() < vertex_map.epsilon { if let Some(&np) = updates.get(idx) { v.pos = np; } break; } }
            } polygon.set_new_normal(); }
        }
        Voxels::from_polygons(&smoothed_polygons, None)
    }

    pub fn taubin_smooth(&self, lambda: Real, mu: Real, iterations: usize, preserve_boundaries: bool) -> Voxels<S> {
        let (vertex_map, adjacency) = crate::voxels::connectivity::PolyMesh { polygons: self.polygons().to_vec() }.build_connectivity();
        let mut smoothed_polygons = self.polygons().to_vec();
        for _ in 0..iterations {
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            for polygon in &smoothed_polygons { for v in &polygon.vertices {
                for (pos, idx) in &vertex_map.position_to_index { if (v.pos - pos).norm() < vertex_map.epsilon { current_positions.insert(*idx, v.pos); break; } }
            }}
            let mut updates: HashMap<usize, Point3<Real>> = HashMap::new();
            for (&vid, neighbors) in &adjacency { if let Some(&p) = current_positions.get(&vid) {
                if preserve_boundaries && neighbors.len() < 4 { updates.insert(vid, p); continue; }
                let mut sum = Point3::origin(); let mut count = 0;
                for &nid in neighbors { if let Some(&np) = current_positions.get(&nid) { sum += np.coords; count += 1; } }
                if count > 0 { let avg = sum / count as Real; let newp = p + (avg - p) * lambda; updates.insert(vid, newp); }
            }}
            for polygon in &mut smoothed_polygons { for v in &mut polygon.vertices {
                for (pos, idx) in &vertex_map.position_to_index { if (v.pos - pos).norm() < vertex_map.epsilon { if let Some(&np) = updates.get(idx) { v.pos = np; } break; } }
            }}
            current_positions.clear(); updates.clear();
            for polygon in &smoothed_polygons { for v in &polygon.vertices {
                for (pos, idx) in &vertex_map.position_to_index { if (v.pos - pos).norm() < vertex_map.epsilon { current_positions.insert(*idx, v.pos); break; } }
            }}
            for (&vid, neighbors) in &adjacency { if let Some(&p) = current_positions.get(&vid) {
                if preserve_boundaries && neighbors.len() < 4 { updates.insert(vid, p); continue; }
                let mut sum = Point3::origin(); let mut count = 0;
                for &nid in neighbors { if let Some(&np) = current_positions.get(&nid) { sum += np.coords; count += 1; } }
                if count > 0 { let avg = sum / count as Real; let newp = p + (avg - p) * mu; updates.insert(vid, newp); }
            }}
            for polygon in &mut smoothed_polygons { for v in &mut polygon.vertices {
                for (pos, idx) in &vertex_map.position_to_index { if (v.pos - pos).norm() < vertex_map.epsilon { if let Some(&np) = updates.get(idx) { v.pos = np; } break; } }
            } polygon.set_new_normal(); }
        }
        Voxels::from_polygons(&smoothed_polygons, None)
    }
}

