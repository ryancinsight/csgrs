use crate::core::float_types::parry3d::{
    bounding_volume::Aabb,
};
use crate::core::float_types::rapier3d::prelude::*;
use crate::core::float_types::{Real};
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use geo::{
    Coord, CoordsIter, Geometry,
    GeometryCollection, LineString, MultiPolygon, Polygon as GeoPolygon,
};
use nalgebra::{
    Point3, Vector3,
};
use std::fmt::Debug;
use std::sync::OnceLock;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

pub mod ops;
pub mod transform;
pub mod distribute;
pub mod mesh;
pub mod query;
pub mod interop;
pub mod bsp;

/// The main CSG solid structure. Contains a list of 3D polygons, 2D polylines, and some metadata.
#[derive(Debug, Clone)]
pub struct CSG<S: Clone> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,

    /// 2D geometry
    pub geometry: GeometryCollection<Real>,

    /// Lazily calculated AABB that spans `polygons` **and** any 2‑D geometry.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Create an empty CSG
    pub fn new() -> Self {
        CSG {
            polygons: Vec::new(),
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Helper to collect all vertices from the CSG.
    pub fn vertices(&self) -> Vec<Vertex> {
        #[cfg(feature = "parallel")]
        {
            self.polygons
                .par_iter()
                .flat_map(|p| p.vertices.clone())
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.polygons
                .iter()
                .flat_map(|p| p.vertices.clone())
                .collect()
        }
    }

    /// Build a CSG from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut csg = CSG::new();
        csg.polygons = polygons.to_vec();
        csg
    }

    /// Convert internal polylines into polygons and return along with any existing internal polygons.
    pub fn to_polygons(&self) -> Vec<Polygon<S>> {
        // Helper function to convert a geo::Polygon to a Vec<crate::geometry::Polygon>
        fn geo_poly_to_csg_polys<S: Clone + Debug + Send + Sync>(
            poly2d: &GeoPolygon<Real>,
            metadata: &Option<S>,
        ) -> Vec<Polygon<S>> {
            let mut all_polygons = Vec::new();

            // Handle the exterior ring
            let outer_vertices_3d: Vec<_> = poly2d
                .exterior()
                .coords_iter()
                .map(|c| Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()))
                .collect();

            if outer_vertices_3d.len() >= 3 {
                all_polygons.push(Polygon::new(outer_vertices_3d, metadata.clone()));
            }

            // Handle interior rings (holes)
            for ring in poly2d.interiors() {
                let hole_vertices_3d: Vec<_> = ring
                    .coords_iter()
                    .map(|c| Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()))
                    .collect();
                if hole_vertices_3d.len() >= 3 {
                    all_polygons.push(Polygon::new(hole_vertices_3d, metadata.clone()));
                }
            }
            all_polygons
        }

        self.geometry
            .iter()
            .flat_map(|geom| -> Vec<Polygon<S>> {
                match geom {
                    Geometry::Polygon(poly2d) => geo_poly_to_csg_polys(poly2d, &self.metadata),
                    Geometry::MultiPolygon(multipoly) => multipoly
                        .iter()
                        .flat_map(|poly2d| geo_poly_to_csg_polys(poly2d, &self.metadata))
                        .collect(),
                    _ => vec![],
                }
            })
            .collect()
    }

    /// Create a CSG that holds *only* 2D geometry in a `geo::GeometryCollection`.
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: Option<S>) -> Self {
        let mut csg = CSG::new();
        csg.geometry = geometry;
        csg.metadata = metadata;
        csg
    }

    /// Take the [`geo::Polygon`]'s from the `CSG`'s geometry collection
    pub fn to_multipolygon(&self) -> MultiPolygon<Real> {
        let polygons = self
            .geometry
            .iter()
            .flat_map(|geom| match geom {
                Geometry::Polygon(poly) => vec![poly.clone()],
                Geometry::MultiPolygon(mp) => mp.0.clone(),
                _ => vec![],
            })
            .collect();

        MultiPolygon(polygons)
    }

    pub fn tessellate_2d(
        outer: &[[Real; 2]],
        holes: &[&[[Real; 2]]],
    ) -> Vec<[Point3<Real>; 3]> {
        #[cfg(feature = "earcut")]
        {
            use geo::TriangulateEarcut;
            // Convert the outer ring into a `LineString`
			let outer_coords: Vec<Coord<Real>> =
				outer.iter().map(|&[x, y]| Coord { x, y }).collect();

			// Convert each hole into its own `LineString`
			let holes_coords: Vec<LineString<Real>> = holes
				.iter()
				.map(|hole| {
					let coords: Vec<Coord<Real>> =
						hole.iter().map(|&[x, y]| Coord { x, y }).collect();
					LineString::new(coords)
				})
				.collect();

			// Ear-cut triangulation on the polygon (outer + holes)
			let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);
            
            let triangulation = polygon.earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert the 2D result (x,y) into 3D triangles with z=0
            let mut result = Vec::with_capacity(triangle_indices.len() / 3);
            for tri in triangle_indices.chunks_exact(3) {
                let pts = [
                    Point3::new(vertices[2 * tri[0]], vertices[2 * tri[0] + 1], 0.0),
                    Point3::new(vertices[2 * tri[1]], vertices[2 * tri[1] + 1], 0.0),
                    Point3::new(vertices[2 * tri[2]], vertices[2 * tri[2] + 1], 0.0),
                ];
                result.push(pts);
            }
            result
        }

		// Helper that forces |v| > SPADE_MIN or 0.0 to avoid a panic.
        #[cfg(feature = "delaunay")]
        #[inline]
        fn clamp_spade(v: Real) -> Real {
            // This should be shared with Polygon::tessellate()
            const SPADE_MIN: Real = 1.793662034335766e-43;
            if v.abs() < SPADE_MIN {
                0.0
            } else {
                v
            }
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;
            // Apply clamping **before** building the geo‑polygon so that
            // spade never sees an out‑of‑range coordinate.
            let outer_coords: Vec<Coord<Real>> = outer
                .iter()
                .map(|&[x, y]| Coord {
                    x: clamp_spade(x),
                    y: clamp_spade(y),
                })
                .collect();
            let holes_coords: Vec<LineString<Real>> = holes
                .iter()
                .map(|hole| {
                    let coords: Vec<Coord<Real>> = hole
                        .iter()
                        .map(|&[x, y]| Coord {
                            x: clamp_spade(x),
                            y: clamp_spade(y),
                        })
                        .collect();
                    LineString::new(coords)
                })
                .collect();
            let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);

            let Ok(tris) = polygon.constrained_triangulation(Default::default()) else {
                return Vec::new();
            };

            let mut result = Vec::with_capacity(tris.len());
            for triangle in tris {
                let [a, b, c] = [triangle.0, triangle.1, triangle.2];
                result.push([
                    Point3::new(a.x, a.y, 0.0),
                    Point3::new(b.x, b.y, 0.0),
                    Point3::new(c.x, c.y, 0.0),
                ]);
            }
            result
        }
    }
}
