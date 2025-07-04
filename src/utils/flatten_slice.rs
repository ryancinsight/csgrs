use crate::core::float_types::{EPSILON, Real};
use crate::csg::CSG;
use crate::spatial::bsp::Node;
use crate::geometry::Plane;
use crate::geometry::Vertex;
use geo::{
    BooleanOps, Geometry, GeometryCollection, LineString, MultiPolygon, Orient,
    Polygon as GeoPolygon, coord, orient::Direction,
};
use hashbrown::HashMap;
use nalgebra::Point3;
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Flattens any 3D polygons by projecting them onto the XY plane (z=0),
    /// unifies them into one or more 2D polygons, and returns a purely 2D CSG.
    ///
    /// Returns CSG with empty polygons and populated 2D geometry.
    pub fn flatten(&self) -> CSG<S> {
        if self.polygons.is_empty() {
            return self.clone();
        }


        #[cfg(feature = "parallel")]
        let flattened_3d: Vec<geo::Polygon<Real>> = {
            if self.polygons.len() > 1000 {
                use rayon::prelude::*;

                // Use parallel processing for large polygon collections
                self.polygons
                    .par_iter()
                    .flat_map(|poly| {
                        // Tessellate this polygon into triangles
                        let triangles = poly.tessellate();
                        // Each triangle has 3 vertices [v0, v1, v2].
                        // Project them onto XY => build a 2D polygon (triangle).
                        triangles
                            .into_par_iter()
                            .map(|tri| {
                                let ring = vec![
                                    (tri[0].pos.x, tri[0].pos.y),
                                    (tri[1].pos.x, tri[1].pos.y),
                                    (tri[2].pos.x, tri[2].pos.y),
                                    (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
                                ];
                                geo::Polygon::new(LineString::from(ring), vec![])
                            })
                    })
                    .collect()
            } else {
                // Sequential processing for smaller collections
                self.polygons
                    .iter()
                    .flat_map(|poly| {
                        let triangles = poly.tessellate();
                        triangles
                            .into_iter()
                            .map(|tri| {
                                let ring = vec![
                                    (tri[0].pos.x, tri[0].pos.y),
                                    (tri[1].pos.x, tri[1].pos.y),
                                    (tri[2].pos.x, tri[2].pos.y),
                                    (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
                                ];
                                geo::Polygon::new(LineString::from(ring), vec![])
                            })
                    })
                    .collect()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let flattened_3d: Vec<geo::Polygon<Real>> = self.polygons
            .iter()
            .flat_map(|poly| {
                let triangles = poly.tessellate();
                triangles
                    .into_iter()
                    .map(|tri| {
                        let ring = vec![
                            (tri[0].pos.x, tri[0].pos.y),
                            (tri[1].pos.x, tri[1].pos.y),
                            (tri[2].pos.x, tri[2].pos.y),
                            (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
                        ];
                        geo::Polygon::new(LineString::from(ring), vec![])
                    })
            })
            .collect();


        let unioned_from_3d = if flattened_3d.is_empty() {
            MultiPolygon::new(Vec::new())
        } else {
            // Use fold() to accumulate union operations efficiently
            flattened_3d
                .iter()
                .skip(1)
                .fold(
                    MultiPolygon(vec![flattened_3d[0].clone()]),
                    |mp_acc, p| mp_acc.union(&MultiPolygon(vec![p.clone()]))
                )
        };

        // 4) Union this with any existing 2D geometry (polygons) from self.geometry
        let existing_2d = &self.to_multipolygon(); // turns geometry -> MultiPolygon
        let final_union = unioned_from_3d.union(existing_2d);
        // Optionally ensure consistent orientation (CCW for exteriors):
        let oriented = final_union.orient(Direction::Default);

        // 5) Store final polygons as a MultiPolygon in a new GeometryCollection
        let mut new_gc = GeometryCollection::default();
        new_gc.0.push(Geometry::MultiPolygon(oriented));

        // 6) Return a purely 2D CSG: polygons empty, geometry has the final shape
        CSG {
            polygons: Vec::new(),
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Slice this solid by a given `plane`, returning a new `CSG` whose polygons
    /// are either:
    /// - The polygons that lie exactly in the slicing plane (coplanar), or
    /// - Polygons formed by the intersection edges (each a line, possibly open or closed).
    ///
    /// The returned `CSG` can contain:
    /// - **Closed polygons** that are coplanar,
    /// - **Open polygons** (poly-lines) if the plane cuts through edges,
    /// - Potentially **closed loops** if the intersection lines form a cycle.
    ///
    /// # Example
    /// ```
    /// use csgrs::CSG;
    /// use csgrs::geometry::Plane;
    /// use nalgebra::Vector3;
    /// let cylinder: CSG<()> = CSG::cylinder(1.0, 2.0, 32, None);
    /// let plane_z0 = Plane::from_normal(Vector3::z(), 0.0);
    /// let cross_section = cylinder.slice(plane_z0);
    /// // `cross_section` will contain:
    /// //   - Possibly an open or closed polygon(s) at z=0
    /// //   - Or empty if no intersection
    /// ```
    pub fn slice(&self, plane: Plane) -> CSG<S> {
        // Build a BSP from all of our polygons:
        let node = Node::from_polygons(&self.polygons.clone());

        // Ask the BSP for coplanar polygons + intersection edges:
        let (coplanar_polys, intersection_edges) = node.slice(&plane);

        // "Knit" those intersection edges into polylines. Each edge is [vA, vB].
        let polylines_3d = unify_intersection_edges(&intersection_edges);

        // Convert each polyline of vertices into a Polygon<S>
        let mut result_polygons = Vec::new();

        // Add the coplanar polygons using extend() for efficient collection
        // We can re‐assign their plane to `plane` to ensure they share the exact plane definition
        result_polygons.extend(coplanar_polys);

        let mut new_gc = GeometryCollection::default();

        // Convert the "chains" or loops into open/closed polygons using iterator patterns
        let geometries: Vec<Geometry<Real>> = polylines_3d
            .into_iter()
            .filter_map(|mut chain| {
                let n = chain.len();
                if n < 2 {
                    return None; // degenerate
                }

                // check if first and last point are within EPSILON of each other
                let dist_sq = (chain[0].pos - chain[n - 1].pos).norm_squared();
                if dist_sq < EPSILON * EPSILON {
                    // Force them to be exactly the same, closing the line
                    chain[n - 1] = chain[0].clone();
                }

                let polyline = LineString::new(
                    chain
                        .iter()
                        .map(|vertex| {
                            coord! {x: vertex.pos.x, y: vertex.pos.y}
                        })
                        .collect(),
                );

                if polyline.is_closed() {
                    let polygon = GeoPolygon::new(polyline, vec![]);
                    let oriented = polygon.orient(Direction::Default);
                    Some(Geometry::Polygon(oriented))
                } else {
                    Some(Geometry::LineString(polyline))
                }
            })
            .collect();

        new_gc.0.extend(geometries);

        // Return a purely 2D CSG: polygons empty, geometry has the final shape
        CSG {
            polygons: Vec::new(),
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

// Build a small helper for hashing endpoints:
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EndKey(i64, i64, i64);

/// Round a floating to a grid for hashing
fn quantize(x: Real) -> i64 {
    // For example, scale by 1e8
    (x * 1e8).round() as i64
}

/// Convert a Vertex's position to an EndKey
fn make_key(pos: &Point3<Real>) -> EndKey {
    EndKey(quantize(pos.x), quantize(pos.y), quantize(pos.z))
}

/// Take a list of intersection edges `[Vertex;2]` and merge them into polylines.
/// Each edge is a line segment between two 3D points.  We want to "knit" them together by
/// matching endpoints that lie within EPSILON of each other, forming either open or closed chains.
///
/// This returns a `Vec` of polylines, where each polyline is a `Vec<Vertex>`.
fn unify_intersection_edges(edges: &[[Vertex; 2]]) -> Vec<Vec<Vertex>> {
    // We will store adjacency by a "key" that identifies an endpoint up to EPSILON,
    // then link edges that share the same key.

    // Adjacency map: key -> list of (edge_index, is_start_or_end)
    // We'll store "(edge_idx, which_end)" as which_end = 0 or 1 for edges[edge_idx][0/1].
    let mut adjacency: HashMap<EndKey, Vec<(usize, usize)>> = HashMap::new();

    // Collect all endpoints using iterator patterns with enumerate and flat_map
    edges
        .iter()
        .enumerate()
        .flat_map(|(i, edge)| {
            (0..2).map(move |end_idx| {
                let v = &edge[end_idx];
                let k = make_key(&v.pos);
                (k, (i, end_idx))
            })
        })
        .for_each(|(k, edge_info)| {
            adjacency.entry(k).or_default().push(edge_info);
        });

    // We'll keep track of which edges have been "visited" in the final polylines.
    let mut visited = vec![false; edges.len()];

    let mut chains: Vec<Vec<Vertex>> = Vec::new();

    // Build chains using iterator patterns with filter_map for unvisited edges
    let edge_chains: Vec<Vec<Vertex>> = (0..edges.len())
        .filter_map(|start_edge_idx| {
            if visited[start_edge_idx] {
                return None;
            }
            // Mark it visited
            visited[start_edge_idx] = true;

            // Our chain starts with `edges[start_edge_idx]`. We can build a small function to "walk":
            // We'll store it in the direction edge[0] -> edge[1]
            let e = &edges[start_edge_idx];
            let mut chain = vec![e[0].clone(), e[1].clone()];

            // We walk "forward" from edge[1] if possible
            extend_chain_forward(&mut chain, &adjacency, &mut visited, edges);

            // We also might walk "backward" from edge[0], but
            // we can do that by reversing the chain at the end if needed. Alternatively,
            // we can do a separate pass.  Let's do it in place for clarity:
            chain.reverse();
            extend_chain_forward(&mut chain, &adjacency, &mut visited, edges);
            // Then reverse back so it goes in the original direction
            chain.reverse();

            Some(chain)
        })
        .collect();

    chains.extend(edge_chains);

    chains
}

/// Extends a chain "forward" by repeatedly finding any unvisited edge that starts
/// at the chain's current end vertex.
fn extend_chain_forward(
    chain: &mut Vec<Vertex>,
    adjacency: &HashMap<EndKey, Vec<(usize, usize)>>,
    visited: &mut [bool],
    edges: &[[Vertex; 2]],
) {
    loop {
        // The chain's current end point:
        let last_v = chain.last().unwrap();
        let key = make_key(&last_v.pos);

        // Find candidate edges that share this endpoint
        let Some(candidates) = adjacency.get(&key) else {
            break;
        };

        // Among these candidates, find the first unvisited edge using iterator patterns
        let found_next = candidates
            .iter()
            .find_map(|&(edge_idx, end_idx)| {
                if visited[edge_idx] {
                    return None;
                }
                // If this is edges[edge_idx][end_idx], the "other" end is edges[edge_idx][1-end_idx].
                // We want that other end to continue the chain.
                let other_end_idx = 1 - end_idx;
                let next_vertex = &edges[edge_idx][other_end_idx];

                // But we must also confirm that the last_v is indeed edges[edge_idx][end_idx]
                // (within EPSILON) which we have checked via the key, so likely yes.

                // Mark visited
                visited[edge_idx] = true;
                Some(next_vertex.clone())
            });

        match found_next {
            Some(v) => {
                chain.push(v);
            },
            None => {
                break;
            },
        }
    }
}
