//! Parallel BSP operations using voxels::{plane,polygon,vertex}

use crate::voxels::bsp::Node;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use crate::voxels::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
#[cfg(feature = "parallel")]
use crate::voxels::polygon::Polygon;
#[cfg(feature = "parallel")]
use crate::voxels::vertex::Vertex;
#[cfg(feature = "parallel")]
use crate::float_types::EPSILON;
#[cfg(feature = "parallel")]
use rayon::prelude::*;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    #[cfg(feature = "parallel")]
    pub fn invert(&mut self) {
        let mut stack = vec![self];
        while let Some(node) = stack.pop() {
            node.polygons.par_iter_mut().for_each(|p| p.flip());
            if let Some(ref mut plane) = node.plane { plane.flip(); }
            std::mem::swap(&mut node.front, &mut node.back);
            if let Some(ref mut f) = node.front { stack.push(f.as_mut()); }
            if let Some(ref mut b) = node.back { stack.push(b.as_mut()); }
        }
    }

    #[cfg(feature = "parallel")]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if self.plane.is_none() { return polygons.to_vec(); }
        let plane = self.plane.as_ref().unwrap();
        let (coplanar_front, coplanar_back, mut front, mut back) = polygons.par_iter().map(|poly| plane.split_polygon(poly)).reduce(
            || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
            |mut acc, x| { acc.0.extend(x.0); acc.1.extend(x.1); acc.2.extend(x.2); acc.3.extend(x.3); acc }
        );
        for cp in coplanar_front { if plane.orient_plane(&cp.plane) == FRONT { front.push(cp); } else { back.push(cp); } }
        for cp in coplanar_back { if plane.orient_plane(&cp.plane) == FRONT { front.push(cp); } else { back.push(cp); } }
        let mut result = if let Some(ref f) = self.front { f.clip_polygons(&front) } else { front };
        if let Some(ref b) = self.back { result.extend(b.clip_polygons(&back)); }
        result
    }

    #[cfg(feature = "parallel")]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        let mut stack = vec![self];
        while let Some(node) = stack.pop() {
            node.polygons = bsp.clip_polygons(&node.polygons);
            if let Some(ref mut f) = node.front { stack.push(f.as_mut()); }
            if let Some(ref mut b) = node.back { stack.push(b.as_mut()); }
        }
    }

    #[cfg(feature = "parallel")]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() { return; }
        if self.plane.is_none() { self.plane = Some(self.pick_best_splitting_plane(polygons)); }
        let plane = self.plane.as_ref().unwrap();
        let (mut coplanar_front, mut coplanar_back, front, back) = polygons.par_iter().map(|p| plane.split_polygon(p)).reduce(
            || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
            |mut acc, x| { acc.0.extend(x.0); acc.1.extend(x.1); acc.2.extend(x.2); acc.3.extend(x.3); acc }
        );
        self.polygons.append(&mut coplanar_front);
        self.polygons.append(&mut coplanar_back);
        if !front.is_empty() { let mut f = self.front.take().unwrap_or_else(|| Box::new(Node::new())); f.build(&front); self.front = Some(f); }
        if !back.is_empty() { let mut b = self.back.take().unwrap_or_else(|| Box::new(Node::new())); b.build(&back); self.back = Some(b); }
    }

    #[cfg(feature = "parallel")]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let all_polys = self.all_polygons();
        let (coplanar_polygons, intersection_edges) = all_polys.par_iter().map(|poly| {
            let vcount = poly.vertices.len();
            if vcount < 2 { return (Vec::new(), Vec::new()); }
            let mut polygon_type = 0; let mut types = Vec::with_capacity(vcount);
            for vertex in &poly.vertices { let t = slicing_plane.orient_point(&vertex.pos); polygon_type |= t; types.push(t); }
            match polygon_type {
                COPLANAR => (vec![poly.clone()], Vec::new()),
                FRONT | BACK => (Vec::new(), Vec::new()),
                SPANNING => {
                    let mut crossing_points = Vec::new();
                    for i in 0..vcount {
                        let j = (i + 1) % vcount; let ti = types[i]; let tj = types[j];
                        let vi = &poly.vertices[i]; let vj = &poly.vertices[j];
                        if (ti | tj) == SPANNING {
                            let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                            if denom.abs() > EPSILON {
                                let t = (slicing_plane.offset() - slicing_plane.normal().dot(&vi.pos.coords)) / denom;
                                crossing_points.push(vi.interpolate(vj, t));
                            }
                        }
                    }
                    let mut edges = Vec::new();
                    for chunk in crossing_points.chunks_exact(2) { edges.push([chunk[0].clone(), chunk[1].clone()]); }
                    (Vec::new(), edges)
                }
                _ => (Vec::new(), Vec::new()),
            }
        }).reduce(|| (Vec::new(), Vec::new()), |mut acc, x| { acc.0.extend(x.0); acc.1.extend(x.1); acc });
        (coplanar_polygons, intersection_edges)
    }
}

