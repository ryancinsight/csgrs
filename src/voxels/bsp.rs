//! BSP node for voxel/BSP hybrid, ported from mesh::bsp to use voxels::{plane,polygon,vertex}

use crate::float_types::{EPSILON, Real};
use crate::voxels::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    pub plane: Option<Plane>,
    pub front: Option<Box<Node<S>>>,
    pub back: Option<Box<Node<S>>>,
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Default for Node<S> {
    fn default() -> Self { Self::new() }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    pub const fn new() -> Self {
        Self { plane: None, front: None, back: None, polygons: Vec::new() }
    }

    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() { node.build(polygons); }
        node
    }

    #[cfg(not(feature = "parallel"))]
    pub fn invert(&mut self) {
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane { plane.flip(); }
        if let Some(ref mut front) = self.front { front.invert(); }
        if let Some(ref mut back) = self.back { back.invert(); }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    pub fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        const K_SPANS: Real = 8.0; const K_BALANCE: Real = 1.0;
        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;
        let sample_size = polygons.len().min(20);
        for p in polygons.iter().take(sample_size) {
            let plane = &p.plane;
            let mut num_front = 0; let mut num_back = 0; let mut num_spanning = 0;
            for poly in polygons {
                match plane.classify_polygon(poly) {
                    COPLANAR => {}
                    FRONT => num_front += 1,
                    BACK => num_back += 1,
                    SPANNING => num_spanning += 1,
                    _ => num_spanning += 1,
                }
            }
            let score = K_SPANS * num_spanning as Real + K_BALANCE * ((num_front - num_back) as Real).abs();
            if score < best_score { best_score = score; best_plane = plane.clone(); }
        }
        best_plane
    }

    #[cfg(not(feature = "parallel"))]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // Iterative traversal for front/back children to avoid deep recursion
        if self.plane.is_none() { return polygons.to_vec(); }
        let plane = self.plane.as_ref().unwrap();
        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) = plane.split_polygon(polygon);
            for coplanar_poly in coplanar_front.into_iter().chain(coplanar_back.into_iter()) {
                if plane.orient_plane(&coplanar_poly.plane) == FRONT { front_parts.push(coplanar_poly); } else { back_parts.push(coplanar_poly); }
            }
            front_polys.append(&mut front_parts);
            back_polys.append(&mut back_parts);
        }
        let mut result = if let Some(front_node) = &self.front {
            front_node.clip_polygons(&front_polys)
        } else {
            front_polys
        };

        if let Some(back_node) = &self.back {
            result.extend(back_node.clip_polygons(&back_polys));
        }

        result
    }

    #[cfg(not(feature = "parallel"))]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front { front.clip_to(bsp); }
        if let Some(ref mut back) = self.back { back.clip_to(bsp); }
    }

    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];
        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);
            stack.extend([
                node.front.as_ref().map(|b| b.as_ref()),
                node.back.as_ref().map(|b| b.as_ref()),
            ].into_iter().flatten());
        }
        result
    }

    #[cfg(not(feature = "parallel"))]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() { return; }
        if self.plane.is_none() { self.plane = Some(self.pick_best_splitting_plane(polygons)); }
        let plane = self.plane.as_ref().unwrap();
        let mut front = Vec::with_capacity(polygons.len()/2);
        let mut back = Vec::with_capacity(polygons.len()/2);
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) = plane.split_polygon(polygon);
            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);
            front.append(&mut front_parts);
            back.append(&mut back_parts);
        }
        if !front.is_empty() {
            let child = self.front.get_or_insert_with(|| Box::new(Node::new()));
            child.build(&front);
        }
        if !back.is_empty() {
            let child = self.back.get_or_insert_with(|| Box::new(Node::new()));
            child.build(&back);
        }
    }

    #[cfg(not(feature = "parallel"))]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let all_polys = self.all_polygons();
        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();
        for poly in &all_polys {
            let vcount = poly.vertices.len(); if vcount < 2 { continue; }
            let types: Vec<_> = poly.vertices.iter().map(|v| slicing_plane.orient_point(&v.pos)).collect();
            let polygon_type = types.iter().fold(0, |acc, &t| acc | t);
            match polygon_type {
                COPLANAR => { coplanar_polygons.push(poly.clone()); }
                FRONT | BACK => {}
                SPANNING => {
                    let crossing_points: Vec<_> = (0..vcount).filter_map(|i| {
                        let j = (i + 1) % vcount;
                        let ti = types[i]; let tj = types[j];
                        let vi = &poly.vertices[i]; let vj = &poly.vertices[j];
                        if (ti | tj) == SPANNING {
                            let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                            if denom.abs() > EPSILON {
                                let t = (slicing_plane.offset() - slicing_plane.normal().dot(&vi.pos.coords)) / denom;
                                Some(vi.interpolate(vj, t))
                            } else { None }
                        } else { None }
                    }).collect();
                    intersection_edges.extend(crossing_points.chunks_exact(2).map(|c| [c[0].clone(), c[1].clone()]));
                }
                _ => {}
            }
        }
        (coplanar_polygons, intersection_edges)
    }

    /// Optimize memory usage by compacting vectors and removing empty nodes
    pub fn optimize_memory(&mut self) {
        // Shrink polygon vector to fit
        self.polygons.shrink_to_fit();

        // Recursively optimize children
        if let Some(ref mut front) = self.front {
            front.optimize_memory();

            // Remove empty front node
            if front.is_empty() {
                self.front = None;
            }
        }

        if let Some(ref mut back) = self.back {
            back.optimize_memory();

            // Remove empty back node
            if back.is_empty() {
                self.back = None;
            }
        }
    }

    /// Check if this BSP node is empty (no polygons and no children)
    pub fn is_empty(&self) -> bool {
        self.polygons.is_empty() && self.front.is_none() && self.back.is_none()
    }

    /// Get memory usage of this BSP tree
    pub fn memory_usage(&self) -> usize {
        let mut size = std::mem::size_of::<Self>();

        // Add polygon memory
        size += self.polygons.capacity() * std::mem::size_of::<Polygon<S>>();

        // Add children memory
        if let Some(ref front) = self.front {
            size += front.memory_usage();
        }
        if let Some(ref back) = self.back {
            size += back.memory_usage();
        }

        size
    }
}

