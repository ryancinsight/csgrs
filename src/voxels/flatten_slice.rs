//! Flatten and slice operations for Voxels, mirroring mesh::flatten_slice

use crate::float_types::{EPSILON, Real};
use crate::voxels::bsp::Node;
use crate::voxels::csg::Voxels;
use crate::voxels::plane::Plane;
use crate::voxels::vertex::Vertex;
use crate::sketch::Sketch;
use geo::{BooleanOps, Geometry, GeometryCollection, LineString, MultiPolygon, Orient, Polygon as GeoPolygon, coord, orient::Direction};
use hashbrown::HashMap;
use nalgebra::Point3;
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    pub fn flatten(&self) -> Sketch<S> {
        let mut flattened_3d = Vec::new();
        for poly in self.polygons() { for tri in poly.triangulate() {
            let ring = vec![(tri[0].pos.x, tri[0].pos.y), (tri[1].pos.x, tri[1].pos.y), (tri[2].pos.x, tri[2].pos.y), (tri[0].pos.x, tri[0].pos.y)];
            let polygon_2d = geo::Polygon::new(LineString::from(ring), vec![]);
            flattened_3d.push(polygon_2d);
        }}
        let unioned = if flattened_3d.is_empty() { MultiPolygon::new(Vec::new()) } else { let mut mp = MultiPolygon(vec![flattened_3d[0].clone()]); for p in flattened_3d.iter().skip(1) { mp = mp.union(&MultiPolygon(vec![p.clone()])); } mp };
        let oriented = unioned.orient(Direction::Default);
        let mut new_gc = GeometryCollection::default(); new_gc.0.push(Geometry::MultiPolygon(oriented));
        Sketch { geometry: new_gc, bounding_box: OnceLock::new(), metadata: None }
    }

    pub fn slice(&self, plane: Plane) -> Sketch<S> {
        let node = Node::from_polygons(&self.polygons().to_vec());
        let (coplanar_polys, intersection_edges) = node.slice(&plane);
        let polylines_3d = unify_intersection_edges(&intersection_edges);
        let mut new_gc = GeometryCollection::default();
        for mut chain in polylines_3d { let n = chain.len(); if n < 2 { continue; }
            let dist_sq = (chain[0].pos - chain[n-1].pos).norm_squared(); if dist_sq < EPSILON*EPSILON { chain[n-1] = chain[0].clone(); }
            let polyline = LineString::new(chain.iter().map(|v| coord!{x: v.pos.x, y: v.pos.y}).collect());
            if polyline.is_closed() { let polygon = GeoPolygon::new(polyline, vec![]); let oriented = polygon.orient(Direction::Default); new_gc.0.push(Geometry::Polygon(oriented)); } else { new_gc.0.push(Geometry::LineString(polyline)); }
        }
        // Append coplanar polygons’ XY projections too (optional)
        for p in coplanar_polys { let ring: Vec<_> = p.vertices.iter().map(|v| (v.pos.x, v.pos.y)).chain(std::iter::once((p.vertices[0].pos.x, p.vertices[0].pos.y))).collect(); let poly2d = geo::Polygon::new(LineString::from(ring), vec![]); new_gc.0.push(Geometry::Polygon(poly2d)); }
        Sketch { geometry: new_gc, bounding_box: OnceLock::new(), metadata: None }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EndKey(i64, i64, i64);
fn quantize(x: Real) -> i64 { (x * 1e8).round() as i64 }
fn make_key(p: &Point3<Real>) -> EndKey { EndKey(quantize(p.x), quantize(p.y), quantize(p.z)) }

fn unify_intersection_edges(edges: &[[Vertex; 2]]) -> Vec<Vec<Vertex>> {
    let mut adjacency: HashMap<EndKey, Vec<(usize, usize)>> = HashMap::new();
    for (i, edge) in edges.iter().enumerate() { for (end_idx, v) in edge.iter().enumerate() { let k = make_key(&v.pos); adjacency.entry(k).or_default().push((i, end_idx)); }}
    let mut visited = vec![false; edges.len()]; let mut chains: Vec<Vec<Vertex>> = Vec::new();
    for start_idx in 0..edges.len() { if visited[start_idx] { continue; } visited[start_idx] = true; let e = &edges[start_idx]; let mut chain = vec![e[0].clone(), e[1].clone()]; extend_chain_forward(&mut chain, &adjacency, &mut visited, edges); chain.reverse(); extend_chain_forward(&mut chain, &adjacency, &mut visited, edges); chain.reverse(); chains.push(chain); }
    chains
}

fn extend_chain_forward(chain: &mut Vec<Vertex>, adjacency: &HashMap<EndKey, Vec<(usize, usize)>>, visited: &mut [bool], edges: &[[Vertex;2]]) {
    loop {
        let last_v = chain.last().unwrap();
        let key = make_key(&last_v.pos);
        let candidates_opt = adjacency.get(&key);
        let candidates = match candidates_opt { Some(c) => c, None => break };
        let mut found = None;
        for &(edge_idx, end_idx) in candidates {
            if visited[edge_idx] { continue; }
            let other_end = 1 - end_idx;
            let next_v = &edges[edge_idx][other_end];
            visited[edge_idx] = true;
            found = Some(next_v.clone());
            break;
        }
        match found { Some(v) => chain.push(v), None => break }
    }
}

