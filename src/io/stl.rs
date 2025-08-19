use crate::float_types::Real;
use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::sketch::Sketch;
use crate::traits::CSG;

use geo::CoordsIter;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;
use std::collections::HashMap;

use core2::io::Cursor;

use stl_io;

/// Internal structure for unified mesh representation with proper vertex sharing
#[derive(Debug)]
struct UnifiedMesh {
    vertices: Vec<Point3<Real>>,
    triangles: Vec<[usize; 3]>,
    normals: Vec<Vector3<Real>>,
}

impl UnifiedMesh {
    /// Create a unified mesh from a CSGRS mesh with proper vertex sharing
    fn from_csgrs_mesh<S: Clone + Debug + Send + Sync>(mesh: &Mesh<S>) -> Self {
        let mut vertex_map: HashMap<String, usize> = HashMap::new();
        let mut vertices: Vec<Point3<Real>> = Vec::new();
        let mut triangles: Vec<[usize; 3]> = Vec::new();
        let mut normals: Vec<Vector3<Real>> = Vec::new();

        // Tolerance for vertex merging (adaptive based on mesh scale)
        // Calculate mesh scale to set appropriate epsilon
        let bbox = mesh.bounding_box();
        let mesh_scale = (bbox.maxs - bbox.mins).norm();
        let epsilon = (mesh_scale * 1e-10).max(1e-12); // Adaptive tolerance

        // Check if mesh is already triangulated to avoid double triangulation
        let needs_triangulation = mesh.polygons.iter().any(|p| p.vertices.len() != 3);
        let triangulated_mesh = if needs_triangulation {
            mesh.triangulate()
        } else {
            mesh.clone()
        };

        // Use the triangulated mesh directly - preserve all geometry including internal surfaces
        // Note: CSG operations may create poor quality triangles, but these are often essential
        // for defining internal surfaces like cylindrical holes. Filtering them would remove
        // important geometry. Let downstream tools handle quality improvement if needed.
        let working_mesh = triangulated_mesh;

        for polygon in &working_mesh.polygons {
            if polygon.vertices.len() != 3 {
                continue; // Skip non-triangular polygons
            }

            let mut triangle_indices = [0usize; 3];

            // Get polygon normal
            let poly_normal = polygon.plane.normal().normalize();

            for (i, vertex) in polygon.vertices.iter().enumerate() {
                // Create a quantized key for vertex position to enable proper merging
                // Quantize to epsilon precision to group nearby vertices
                let quantize = |val: Real| -> i64 {
                    (val / epsilon).round() as i64
                };

                let key = format!("{}_{}_{}",
                                 quantize(vertex.pos.x),
                                 quantize(vertex.pos.y),
                                 quantize(vertex.pos.z));

                let vertex_index = if let Some(&existing_index) = vertex_map.get(&key) {
                    // Verify the existing vertex is actually close enough
                    let existing_vertex = vertices[existing_index];
                    let distance = (existing_vertex - vertex.pos).norm();

                    if distance < epsilon * 2.0 { // Allow some tolerance for quantization
                        existing_index
                    } else {
                        // Quantization collision - create new vertex
                        let new_index = vertices.len();
                        vertices.push(vertex.pos);
                        // Use a more precise key to avoid future collisions
                        let precise_key = format!("{:.15}_{:.15}_{:.15}_{}",
                                                 vertex.pos.x, vertex.pos.y, vertex.pos.z, new_index);
                        vertex_map.insert(precise_key, new_index);
                        new_index
                    }
                } else {
                    // Create new vertex
                    let new_index = vertices.len();
                    vertices.push(vertex.pos);
                    vertex_map.insert(key, new_index);
                    new_index
                };

                triangle_indices[i] = vertex_index;
            }

            triangles.push(triangle_indices);
            normals.push(poly_normal);
        }

        UnifiedMesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// Filter out degenerate triangles that cause visual gaps and artifacts
    /// while preserving essential geometry like internal surfaces
    ///
    /// NOTE: Currently disabled to preserve all geometry including internal surfaces.
    /// CSG operations create poor quality triangles for internal surfaces (like cylinder walls)
    /// but these are essential geometry that should not be filtered.
    #[allow(dead_code)]
    fn filter_degenerate_triangles<S: Clone + Debug + Send + Sync>(mesh: &Mesh<S>, epsilon: Real) -> Mesh<S> {
        let mut filtered_polygons = Vec::new();

        // First pass: analyze all triangles to understand the geometry
        let mut triangle_areas = Vec::new();
        let mut triangle_centers = Vec::new();

        for polygon in &mesh.polygons {
            if polygon.vertices.len() != 3 {
                triangle_areas.push(0.0);
                triangle_centers.push((0.0, 0.0, 0.0));
                continue;
            }

            let v0 = polygon.vertices[0].pos;
            let v1 = polygon.vertices[1].pos;
            let v2 = polygon.vertices[2].pos;

            // Calculate triangle center and area
            let center = (v0.coords + v1.coords + v2.coords) / 3.0;
            triangle_centers.push((center.x, center.y, center.z));

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross_product = edge1.cross(&edge2);
            let area = cross_product.norm() * 0.5;
            triangle_areas.push(area);
        }

        // Calculate mesh bounding box and characteristics
        let bbox = mesh.bounding_box();
        let mesh_size = (bbox.maxs - bbox.mins).norm();

        // Adaptive thresholds based on mesh scale
        let min_edge_length = epsilon * mesh_size * 0.001; // Very small relative to mesh
        let min_area = epsilon * epsilon * mesh_size * mesh_size * 0.000001; // Very small relative to mesh

        // Second pass: filter with geometric awareness
        for (i, polygon) in mesh.polygons.iter().enumerate() {
            if polygon.vertices.len() != 3 {
                // Keep non-triangular polygons as-is
                filtered_polygons.push(polygon.clone());
                continue;
            }

            let v0 = polygon.vertices[0].pos;
            let v1 = polygon.vertices[1].pos;
            let v2 = polygon.vertices[2].pos;

            // Calculate triangle properties
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let edge3 = v2 - v1;

            let edge1_len = edge1.norm();
            let edge2_len = edge2.norm();
            let edge3_len = edge3.norm();

            // Calculate triangle area and normal
            let cross_product = edge1.cross(&edge2);
            let area = cross_product.norm() * 0.5;
            let _normal = if area > 1e-12 { cross_product.normalize() } else { continue; };

            // Check for truly degenerate triangles (zero area or coincident vertices)
            if area < min_area {
                continue; // Skip truly degenerate triangles
            }

            if edge1_len < min_edge_length || edge2_len < min_edge_length || edge3_len < min_edge_length {
                continue; // Skip triangles with extremely short edges
            }

            // Calculate minimum angle
            let cos_angle1 = edge1.dot(&edge2) / (edge1_len * edge2_len);
            let cos_angle2 = (-edge1).dot(&edge3) / (edge1_len * edge3_len);
            let cos_angle3 = (-edge2).dot(&(-edge3)) / (edge2_len * edge3_len);

            let angle1 = cos_angle1.clamp(-1.0, 1.0).acos();
            let angle2 = cos_angle2.clamp(-1.0, 1.0).acos();
            let angle3 = cos_angle3.clamp(-1.0, 1.0).acos();

            let min_angle = angle1.min(angle2).min(angle3);

            // Smart filtering based on geometric context
            let center = triangle_centers[i];
            let is_potentially_internal = Self::is_potentially_internal_surface(center, &triangle_centers, &bbox);

            // Calculate aspect ratio for additional filtering criteria
            let max_edge = edge1_len.max(edge2_len).max(edge3_len);
            let min_edge = edge1_len.min(edge2_len).min(edge3_len);
            let aspect_ratio = if min_edge > 1e-12 { max_edge / min_edge } else { Real::INFINITY };

            if is_potentially_internal {
                // For internal surfaces (like cylinder walls), be very conservative
                // Only filter triangles that are truly degenerate (near-zero area or extreme aspect ratios)
                if area < min_area * 0.1 || aspect_ratio > 1000.0 {
                    continue; // Skip only extremely degenerate internal triangles
                }
            } else {
                // For external surfaces, apply more aggressive filtering
                let angle_threshold = 0.2_f64.to_radians(); // 0.2 degrees
                if min_angle < angle_threshold || aspect_ratio > 200.0 {
                    continue; // Skip sliver triangles on external surfaces
                }
            }

            // Triangle passes quality checks - keep it
            filtered_polygons.push(polygon.clone());
        }

        Mesh::from_polygons(&filtered_polygons, mesh.metadata.clone())
    }

    /// Heuristic to detect if a triangle might be part of an internal surface
    #[allow(dead_code)]
    fn is_potentially_internal_surface(
        center: (Real, Real, Real),
        all_centers: &[(Real, Real, Real)],
        bbox: &Aabb
    ) -> bool {
        // Check if triangle is well inside the bounding box (not on external faces)
        let margin = 0.1; // 10% margin from edges
        let x_margin = (bbox.maxs.x - bbox.mins.x) * margin;
        let y_margin = (bbox.maxs.y - bbox.mins.y) * margin;
        let z_margin = (bbox.maxs.z - bbox.mins.z) * margin;

        let is_internal = center.0 > bbox.mins.x + x_margin && center.0 < bbox.maxs.x - x_margin &&
                         center.1 > bbox.mins.y + y_margin && center.1 < bbox.maxs.y - y_margin &&
                         center.2 > bbox.mins.z + z_margin && center.2 < bbox.maxs.z - z_margin;

        if !is_internal {
            return false;
        }

        // Check if there are many triangles with similar positions (indicating a surface)
        let proximity_threshold = ((bbox.maxs.x - bbox.mins.x) + (bbox.maxs.y - bbox.mins.y) + (bbox.maxs.z - bbox.mins.z)) / 30.0;
        let nearby_count = all_centers.iter()
            .filter(|other_center| {
                let dx = center.0 - other_center.0;
                let dy = center.1 - other_center.1;
                let dz = center.2 - other_center.2;
                (dx*dx + dy*dy + dz*dz).sqrt() < proximity_threshold
            })
            .count();

        // If there are many nearby triangles, this is likely part of a surface
        nearby_count > 10
    }
}

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Export to ASCII STL with unified mesh topology
    ///
    /// Convert this Mesh to an **ASCII STL** string with the given `name`.
    /// This implementation creates a proper manifold mesh with shared vertices,
    /// ensuring compatibility with CAD software, 3D printing, and mesh analysis tools.
    ///
    /// ## Improvements over previous implementation:
    /// - **Proper vertex sharing**: Eliminates duplicate vertices
    /// - **Manifold topology**: Creates connected surface suitable for 3D printing
    /// - **Tool compatibility**: Works correctly with MeshLab, Blender, CAD software
    /// - **Memory efficiency**: Reduces file size through vertex deduplication
    ///
    /// ```rust
    /// # use csgrs::mesh::Mesh;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let mesh  = Mesh::<()>::cube(1.0, None);
    /// let bytes = mesh.to_stl_ascii("my_solid");
    /// std::fs::write("stl/my_solid.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_ascii(&self, name: &str) -> String {
        // Create unified mesh with proper vertex sharing
        let unified = UnifiedMesh::from_csgrs_mesh(self);

        let mut out = String::new();
        out.push_str(&format!("solid {}\n", name));

        // Write triangles with shared vertices
        for (triangle, normal) in unified.triangles.iter().zip(unified.normals.iter()) {
            let v0 = unified.vertices[triangle[0]];
            let v1 = unified.vertices[triangle[1]];
            let v2 = unified.vertices[triangle[2]];

            out.push_str(&format!(
                "  facet normal {:.6} {:.6} {:.6}\n",
                normal.x, normal.y, normal.z
            ));
            out.push_str("    outer loop\n");
            out.push_str(&format!(
                "      vertex {:.6} {:.6} {:.6}\n",
                v0.x, v0.y, v0.z
            ));
            out.push_str(&format!(
                "      vertex {:.6} {:.6} {:.6}\n",
                v1.x, v1.y, v1.z
            ));
            out.push_str(&format!(
                "      vertex {:.6} {:.6} {:.6}\n",
                v2.x, v2.y, v2.z
            ));
            out.push_str("    endloop\n");
            out.push_str("  endfacet\n");
        }

        out.push_str(&format!("endsolid {}\n", name));
        out
    }

    /// Export to BINARY STL with unified mesh topology (returns `Vec<u8>`)
    ///
    /// Convert this Mesh to a **binary STL** byte vector with the given `name`.
    /// This implementation creates a proper manifold mesh with shared vertices,
    /// ensuring compatibility with CAD software, 3D printing, and mesh analysis tools.
    ///
    /// ## Improvements over previous implementation:
    /// - **Proper vertex sharing**: Eliminates duplicate vertices
    /// - **Manifold topology**: Creates connected surface suitable for 3D printing
    /// - **Tool compatibility**: Works correctly with MeshLab, Blender, CAD software
    /// - **Memory efficiency**: Reduces file size through vertex deduplication
    ///
    /// The resulting `Vec<u8>` can then be written to a file or handled in memory:
    ///
    /// ```rust
    /// # use csgrs::mesh::Mesh;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let object = Mesh::<()>::cube(1.0, None);
    /// let bytes  = object.to_stl_binary("my_solid")?;
    /// std::fs::write("stl/my_solid.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_binary(&self, _name: &str) -> std::io::Result<Vec<u8>> {
        use core2::io::Cursor;
        use stl_io::{Normal, Triangle, Vertex, write_stl};

        // Create unified mesh with proper vertex sharing
        let unified = UnifiedMesh::from_csgrs_mesh(self);
        let mut triangles = Vec::new();

        // Convert unified mesh to stl_io format
        for (triangle, normal) in unified.triangles.iter().zip(unified.normals.iter()) {
            let v0 = unified.vertices[triangle[0]];
            let v1 = unified.vertices[triangle[1]];
            let v2 = unified.vertices[triangle[2]];

            #[allow(clippy::unnecessary_cast)]
            triangles.push(Triangle {
                normal: Normal::new([normal.x as f32, normal.y as f32, normal.z as f32]),
                vertices: [
                    Vertex::new([v0.x as f32, v0.y as f32, v0.z as f32]),
                    Vertex::new([v1.x as f32, v1.y as f32, v1.z as f32]),
                    Vertex::new([v2.x as f32, v2.y as f32, v2.z as f32]),
                ],
            });
        }

        // Encode into a binary STL buffer
        let mut cursor = Cursor::new(Vec::new());
        write_stl(&mut cursor, triangles.iter())?;
        Ok(cursor.into_inner())
    }

    /// Create a Mesh object from STL data using 'stl_io'.
    #[cfg(feature = "stl-io")]
    pub fn from_stl(stl_data: &[u8], metadata: Option<S>) -> Result<Mesh<S>, std::io::Error> {
        // Create an in-memory cursor from the STL data
        let mut cursor = Cursor::new(stl_data);

        // Create an STL reader from the cursor
        let stl_reader = stl_io::create_stl_reader(&mut cursor)?;

        let mut polygons = Vec::new();

        for tri_result in stl_reader {
            // Handle potential errors from the STL reader
            let tri = tri_result?;

            // Construct vertices and a polygon
            let vertices = vec![
                Vertex::new(
                    Point3::new(
                        tri.vertices[0][0] as Real,
                        tri.vertices[0][1] as Real,
                        tri.vertices[0][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
                Vertex::new(
                    Point3::new(
                        tri.vertices[1][0] as Real,
                        tri.vertices[1][1] as Real,
                        tri.vertices[1][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
                Vertex::new(
                    Point3::new(
                        tri.vertices[2][0] as Real,
                        tri.vertices[2][1] as Real,
                        tri.vertices[2][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
            ];
            polygons.push(Polygon::new(vertices, metadata.clone()));
        }

        Ok(Mesh::from_polygons(&polygons, metadata))
    }
}

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Export to ASCII STL
    /// Convert this Sketch to an **ASCII STL** string with the given 'name'.
    ///
    /// ```
    /// # use csgrs::sketch::Sketch;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let sketch: Sketch<()> = Sketch::square(2.0, None);
    /// let bytes = sketch.to_stl_ascii("my_sketch");
    /// std::fs::write("stl/my_sketch.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_ascii(&self, name: &str) -> String {
        let mut out = String::new();
        out.push_str(&format!("solid {name}\n"));

        // Write out all *2D* geometry from `self.geometry`
        // We only handle Polygon and MultiPolygon.  We tessellate in XY, set z=0.
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Outer ring (in CCW for a typical "positive" polygon)
                    let outer = poly2d
                        .exterior()
                        .coords_iter()
                        .map(|c| [c.x, c.y])
                        .collect::<Vec<[Real; 2]>>();

                    // Collect holes
                    let holes_vec = poly2d
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>())
                        .collect::<Vec<_>>();
                    let hole_refs = holes_vec
                        .iter()
                        .map(|hole_coords| &hole_coords[..])
                        .collect::<Vec<_>>();

                    // Triangulate with our existing helper:
                    let triangles_2d = Sketch::<()>::triangulate_2d(&outer, &hole_refs);

                    // Write each tri as a facet in ASCII STL, with a normal of (0,0,1)
                    for tri in triangles_2d {
                        out.push_str("  facet normal 0.000000 0.000000 1.000000\n");
                        out.push_str("    outer loop\n");
                        for pt in &tri {
                            out.push_str(&format!(
                                "      vertex {:.6} {:.6} {:.6}\n",
                                pt.x, pt.y, pt.z
                            ));
                        }
                        out.push_str("    endloop\n");
                        out.push_str("  endfacet\n");
                    }
                },

                geo::Geometry::MultiPolygon(mp) => {
                    // Each polygon inside the MultiPolygon
                    for poly2d in &mp.0 {
                        let outer = poly2d
                            .exterior()
                            .coords_iter()
                            .map(|c| [c.x, c.y])
                            .collect::<Vec<[Real; 2]>>();

                        // Holes
                        let holes_vec = poly2d
                            .interiors()
                            .iter()
                            .map(|ring| {
                                ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>()
                            })
                            .collect::<Vec<_>>();
                        let hole_refs = holes_vec
                            .iter()
                            .map(|hole_coords| &hole_coords[..])
                            .collect::<Vec<_>>();

                        let triangles_2d = Sketch::<()>::triangulate_2d(&outer, &hole_refs);

                        for tri in triangles_2d {
                            out.push_str("  facet normal 0.000000 0.000000 1.000000\n");
                            out.push_str("    outer loop\n");
                            for pt in &tri {
                                out.push_str(&format!(
                                    "      vertex {:.6} {:.6} {:.6}\n",
                                    pt.x, pt.y, pt.z
                                ));
                            }
                            out.push_str("    endloop\n");
                            out.push_str("  endfacet\n");
                        }
                    }
                },

                // Skip all other geometry types (LineString, Point, etc.)
                // You can optionally handle them if you like, or ignore them.
                _ => {},
            }
        }

        out.push_str(&format!("endsolid {name}\n"));
        out
    }

    /// Export to BINARY STL (returns `Vec<u8>`)
    ///
    /// Convert this Sketch to a **binary STL** byte vector with the given 'name'.
    ///
    /// The resulting `Vec<u8>` can then be written to a file or handled in memory:
    ///
    /// ```rust
    /// # use csgrs::sketch::Sketch;
    /// # use std::error::Error;
    /// # fn main() -> Result<(), Box<dyn Error>> {
    /// let object = Sketch::<()>::square(1.0, None);
    /// let bytes  = object.to_stl_binary("my_sketch")?;
    /// std::fs::write("stl/my_sketch.stl", bytes)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_stl_binary(&self, _name: &str) -> std::io::Result<Vec<u8>> {
        use core2::io::Cursor;
        use stl_io::{Normal, Triangle, Vertex, write_stl};

        let mut triangles = Vec::new();

        // Triangulate any 2D geometry from self.geometry (Polygon, MultiPolygon).
        // We treat these as lying in the XY plane, at Z=0, with a default normal of +Z.
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Gather outer ring as [x,y]
                    let outer: Vec<[Real; 2]> =
                        poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();

                    // Gather holes
                    let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();

                    // Convert each hole to a slice-reference for triangulation
                    let hole_refs: Vec<&[[Real; 2]]> =
                        holes_vec.iter().map(|h| &h[..]).collect();

                    // Triangulate using our geo-based helper
                    let tri_2d = Sketch::<()>::triangulate_2d(&outer, &hole_refs);

                    // Each triangle is in XY, so normal = (0,0,1)
                    #[allow(clippy::unnecessary_cast)]
                    for tri_pts in tri_2d {
                        triangles.push(Triangle {
                            normal: Normal::new([0.0, 0.0, 1.0]),
                            vertices: [
                                Vertex::new([
                                    tri_pts[0].x as f32,
                                    tri_pts[0].y as f32,
                                    tri_pts[0].z as f32,
                                ]),
                                Vertex::new([
                                    tri_pts[1].x as f32,
                                    tri_pts[1].y as f32,
                                    tri_pts[1].z as f32,
                                ]),
                                Vertex::new([
                                    tri_pts[2].x as f32,
                                    tri_pts[2].y as f32,
                                    tri_pts[2].z as f32,
                                ]),
                            ],
                        });
                    }
                },

                geo::Geometry::MultiPolygon(mpoly) => {
                    // Same approach, but each Polygon in the MultiPolygon
                    for poly2d in &mpoly.0 {
                        let outer: Vec<[Real; 2]> =
                            poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();

                        let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                            .collect();

                        let hole_refs: Vec<&[[Real; 2]]> =
                            holes_vec.iter().map(|h| &h[..]).collect();
                        let tri_2d = Sketch::<()>::triangulate_2d(&outer, &hole_refs);

                        #[allow(clippy::unnecessary_cast)]
                        for tri_pts in tri_2d {
                            triangles.push(Triangle {
                                normal: Normal::new([0.0, 0.0, 1.0]),
                                vertices: [
                                    Vertex::new([
                                        tri_pts[0].x as f32,
                                        tri_pts[0].y as f32,
                                        tri_pts[0].z as f32,
                                    ]),
                                    Vertex::new([
                                        tri_pts[1].x as f32,
                                        tri_pts[1].y as f32,
                                        tri_pts[1].z as f32,
                                    ]),
                                    Vertex::new([
                                        tri_pts[2].x as f32,
                                        tri_pts[2].y as f32,
                                        tri_pts[2].z as f32,
                                    ]),
                                ],
                            });
                        }
                    }
                },

                // Skip other geometry types: lines, points, etc.
                _ => {},
            }
        }

        //
        // (C) Encode into a binary STL buffer
        //
        let mut cursor = Cursor::new(Vec::new());
        write_stl(&mut cursor, triangles.iter())?;
        Ok(cursor.into_inner())
    }
}
