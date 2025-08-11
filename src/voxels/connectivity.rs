//! Connectivity utilities for voxel polygons mirroring mesh::connectivity.

use crate::float_types::Real;
use crate::voxels::polygon::Polygon;
use hashbrown::HashMap;
use nalgebra::Point3;
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct VertexIndexMap {
    pub position_to_index: Vec<(Point3<Real>, usize)>,
    pub index_to_position: HashMap<usize, Point3<Real>>,
    pub epsilon: Real,
}

impl VertexIndexMap {
    pub fn new(epsilon: Real) -> Self {
        Self { position_to_index: Vec::new(), index_to_position: HashMap::new(), epsilon }
    }

    pub fn get_or_create_index(&mut self, pos: Point3<Real>) -> usize {
        for (existing_pos, existing_index) in &self.position_to_index {
            if (pos - existing_pos).norm() < self.epsilon { return *existing_index; }
        }
        let new_index = self.position_to_index.len();
        self.position_to_index.push((pos, new_index));
        self.index_to_position.insert(new_index, pos);
        new_index
    }

    pub fn get_position(&self, index: usize) -> Option<Point3<Real>> { self.index_to_position.get(&index).copied() }
    pub fn vertex_count(&self) -> usize { self.position_to_index.len() }
    pub const fn get_vertex_positions(&self) -> &Vec<(Point3<Real>, usize)> { &self.position_to_index }
}

#[derive(Debug, Clone)]
pub struct PolyMesh<S: Clone> {
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Debug + Send + Sync> PolyMesh<S> {
    pub fn build_connectivity(&self) -> (VertexIndexMap, HashMap<usize, Vec<usize>>) {
        let mut vertex_map = VertexIndexMap::new(Real::EPSILON * 100.0);
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();
        for polygon in &self.polygons { for vertex in &polygon.vertices { vertex_map.get_or_create_index(vertex.pos); } }
        for polygon in &self.polygons {
            let mut indices = Vec::new();
            for vertex in &polygon.vertices { let idx = vertex_map.get_or_create_index(vertex.pos); indices.push(idx); }
            for i in 0..indices.len() {
                let current = indices[i];
                let next = indices[(i + 1) % indices.len()];
                let prev = indices[(i + indices.len() - 1) % indices.len()];
                adjacency.entry(current).or_default().push(next);
                adjacency.entry(current).or_default().push(prev);
                adjacency.entry(next).or_default().push(current);
                adjacency.entry(prev).or_default().push(current);
            }
        }
        for (vertex_idx, neighbors) in adjacency.iter_mut() {
            neighbors.sort_unstable(); neighbors.dedup(); neighbors.retain(|&n| n != *vertex_idx);
        }
        (vertex_map, adjacency)
    }
}

