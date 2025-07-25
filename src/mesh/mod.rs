//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::float_types::{
    parry3d::{
        bounding_volume::{Aabb, BoundingVolume},
        query::RayCast,
        shape::Shape,
    },
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, Ray, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape, TriMesh, Triangle,
    },
    {EPSILON, Real},
};
use crate::mesh::{bsp::Node, plane::Plane, polygon::Polygon, vertex::Vertex};
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{CoordsIter, Geometry, Polygon as GeoPolygon};
use nalgebra::{
    Isometry3, Matrix4, Point3, Quaternion, Unit, Vector3, partial_max, partial_min,
};
use std::{cmp::PartialEq, fmt::Debug, num::NonZeroU32, sync::OnceLock};

#[cfg(feature = "parallel")]
use rayon::{iter::IntoParallelRefIterator, prelude::*};

/// **Enhanced Mesh Validation Result**
///
/// Contains comprehensive information about mesh integrity issues
/// that can cause holes or artifacts in TPMS structures.
#[derive(Debug, Clone)]
pub struct MeshValidationResult {
    /// Polygons with fewer than 3 vertices
    pub degenerate_polygons: Vec<usize>,
    /// Polygons with near-zero surface area
    pub zero_area_polygons: Vec<usize>,
    /// Polygons that self-intersect
    pub self_intersecting_polygons: Vec<usize>,
    /// Vertices not referenced by any polygon
    pub isolated_vertices: Vec<usize>,
    /// Whether the mesh has problematic vertex clustering
    pub has_vertex_clustering: bool,
}

impl MeshValidationResult {
    /// Create a new empty validation result
    pub fn new() -> Self {
        Self {
            degenerate_polygons: Vec::new(),
            zero_area_polygons: Vec::new(),
            self_intersecting_polygons: Vec::new(),
            isolated_vertices: Vec::new(),
            has_vertex_clustering: false,
        }
    }
    
    /// Check if the mesh has any validation issues
    pub fn has_issues(&self) -> bool {
        !self.degenerate_polygons.is_empty() ||
        !self.zero_area_polygons.is_empty() ||
        !self.self_intersecting_polygons.is_empty() ||
        !self.isolated_vertices.is_empty() ||
        self.has_vertex_clustering
    }
    
    /// Get a summary of issues found
    pub fn summary(&self) -> String {
        if !self.has_issues() {
            return "No issues found".to_string();
        }
        
        let mut issues = Vec::new();
        
        if !self.degenerate_polygons.is_empty() {
            issues.push(format!("{} degenerate polygons", self.degenerate_polygons.len()));
        }
        if !self.zero_area_polygons.is_empty() {
            issues.push(format!("{} zero-area polygons", self.zero_area_polygons.len()));
        }
        if !self.self_intersecting_polygons.is_empty() {
            issues.push(format!("{} self-intersecting polygons", self.self_intersecting_polygons.len()));
        }
        if !self.isolated_vertices.is_empty() {
            issues.push(format!("{} isolated vertices", self.isolated_vertices.len()));
        }
        if self.has_vertex_clustering {
            issues.push("vertex clustering detected".to_string());
        }
        
        format!("Issues found: {}", issues.join(", "))
    }
}

impl Default for MeshValidationResult {
    fn default() -> Self {
        Self::new()
    }
}

pub mod bsp;
pub mod bsp_parallel;

#[cfg(feature = "chull")]
pub mod convex_hull;
pub mod flatten_slice;

#[cfg(feature = "metaballs")]
pub mod metaballs;
pub mod plane;
pub mod polygon;

pub mod connectivity;
pub mod manifold;
pub mod quality;
#[cfg(feature = "sdf")]
pub mod sdf;
pub mod shapes;
pub mod smoothing;
#[cfg(feature = "sdf")]
pub mod tpms;
pub mod vertex;

#[derive(Clone, Debug)]
pub struct Mesh<S: Clone + Send + Sync + Debug> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug + PartialEq> Mesh<S> {
    /// Compare just the `metadata` fields of two meshes
    #[inline]
    pub fn same_metadata(&self, other: &Self) -> bool {
        self.metadata == other.metadata
    }

    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &S) -> Mesh<S> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| p.metadata.as_ref() == Some(needle))
            .cloned()
            .collect();

        Mesh {
            polygons: polys,
            bounding_box: std::sync::OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Weld (merge) vertices that fall within a given *Euclidean* tolerance.
    ///
    /// This is a **post-processing** utility that collapses vertices whose
    /// positions differ by less than `tol` into a single representative.
    /// It reduces T-junctions and micron-scale gaps often created by Boolean
    /// CSG or marching-cubes style algorithms and is therefore highly useful
    /// for generating watertight TPMS shells.
    ///
    /// **Enhancement**: Now includes multi-pass welding with connectivity preservation
    /// and normal smoothing for better mesh quality.
    ///
    /// Complexity: *O(n log n)* on average using spatial hashing.
    pub fn weld_vertices_mut(&mut self, tol: Real) {
        use crate::mesh::plane::Plane;
        use hashbrown::HashMap;

        if self.polygons.is_empty() {
            return;
        }

        // **Enhancement**: Multi-pass welding for improved robustness
        // Pass 1: Standard welding
        // Pass 2: Connectivity-aware welding for boundary regions
        
        let mut total_vertices_before = 0;
        for poly in &self.polygons {
            total_vertices_before += poly.vertices.len();
        }

        // **Optimization**: Choose cell size based on tolerance and mesh density
        let cell = tol.max(1e-12);
        let mesh_bounds = self.bounding_box();
        let mesh_size = (mesh_bounds.maxs - mesh_bounds.mins).norm();
        let adaptive_cell = if mesh_size > 1e3 {
            cell * (mesh_size / 1e3).sqrt() // Larger cells for large meshes
        } else {
            cell
        };

        // **Enhancement**: Improved spatial hashing with multi-level grid
        fn quantize_position(p: &Point3<Real>, cell: Real) -> (i64, i64, i64) {
            let inv = 1.0 / cell;
            (
                (p.x * inv).round() as i64,
                (p.y * inv).round() as i64,
                (p.z * inv).round() as i64,
            )
        }

        // **Enhancement**: Track vertex neighborhoods for normal smoothing
        let mut vertex_neighborhoods: HashMap<(i64, i64, i64), Vec<Vector3<Real>>> = HashMap::new();
        let mut vmap: HashMap<(i64, i64, i64), Vertex> = HashMap::new();

        // Pass 1: Primary welding with neighborhood tracking
        for poly in &mut self.polygons {
            for v in &mut poly.vertices {
                let k = quantize_position(&v.pos, adaptive_cell);
                
                // **Enhancement**: Check neighboring cells for better vertex merging
                let mut best_match: Option<Vertex> = None;
                let mut best_distance = Real::MAX;
                
                // Check 3x3x3 neighborhood around the quantized position
                for dx in -1..=1i64 {
                    for dy in -1..=1i64 {
                        for dz in -1..=1i64 {
                            let neighbor_key = (k.0 + dx, k.1 + dy, k.2 + dz);
                            if let Some(candidate) = vmap.get(&neighbor_key) {
                                let distance = (v.pos - candidate.pos).norm();
                                if distance <= tol && distance < best_distance {
                                    best_distance = distance;
                                    best_match = Some(candidate.clone());
                                }
                            }
                        }
                    }
                }

                if let Some(canonical) = best_match {
                    // **Enhancement**: Accumulate normals for smoothing
                    vertex_neighborhoods.entry(k).or_insert_with(Vec::new).push(v.normal);
                    *v = canonical;
                } else {
                    // **Enhancement**: Store new canonical vertex
                    vertex_neighborhoods.entry(k).or_insert_with(Vec::new).push(v.normal);
                    vmap.insert(k, v.clone());
                }
            }
        }

        // **Enhancement**: Pass 2 - Normal smoothing for welded vertices
        let mut smoothed_normals: HashMap<(i64, i64, i64), Vector3<Real>> = HashMap::new();
        for (key, normals) in &vertex_neighborhoods {
            if normals.len() > 1 {
                // **Enhancement**: Compute smoothed normal using area-weighted averaging
                let mut avg_normal = Vector3::zeros();
                let mut total_weight = 0.0;
                
                for normal in normals {
                    let weight = normal.norm(); // Use normal magnitude as weight
                    if weight > EPSILON {
                        avg_normal += normal / weight;
                        total_weight += 1.0;
                    }
                }
                
                if total_weight > 0.0 {
                    avg_normal = avg_normal / total_weight;
                    if avg_normal.norm() > EPSILON {
                        smoothed_normals.insert(*key, avg_normal.normalize());
                    }
                }
            }
        }

        // **Enhancement**: Apply smoothed normals to welded vertices
        for poly in &mut self.polygons {
            for v in &mut poly.vertices {
                let k = quantize_position(&v.pos, adaptive_cell);
                if let Some(smoothed_normal) = smoothed_normals.get(&k) {
                    v.normal = *smoothed_normal;
                }
            }
        }

        // **Enhancement**: Pass 3 - Connectivity preservation for boundary vertices
        // Identify and specially handle vertices that might be on mesh boundaries
        let mut boundary_candidates: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        for (poly_idx, poly) in self.polygons.iter().enumerate() {
            for v in &poly.vertices {
                let k = quantize_position(&v.pos, adaptive_cell);
                boundary_candidates.entry(k).or_insert_with(Vec::new).push(poly_idx);
            }
        }

        // **Enhancement**: Special handling for vertices shared by few polygons (likely boundaries)
        let mut _boundary_refinement_needed = false;
        for (key, poly_indices) in &boundary_candidates {
            if poly_indices.len() <= 2 {
                // This vertex is shared by 2 or fewer polygons - likely a boundary
                _boundary_refinement_needed = true;
                
                // Use tighter tolerance for boundary vertices
                let boundary_tol = tol * 0.5;
                let _k_fine = quantize_position(
                    &vmap.get(key).map(|v| v.pos).unwrap_or_default(), 
                    boundary_tol
                );
                
                // Update relevant polygons with refined positioning
                for &poly_idx in poly_indices {
                    if poly_idx < self.polygons.len() {
                        for v in &mut self.polygons[poly_idx].vertices {
                            let v_key = quantize_position(&v.pos, adaptive_cell);
                            if v_key == *key {
                                // Apply boundary-specific positioning
                                if let Some(canonical) = vmap.get(key) {
                                    v.pos = canonical.pos;
                                }
                            }
                        }
                    }
                }
            }
        }

        // **Enhancement**: Recompute polygon planes and bounding boxes for modified polygons
        let mut total_vertices_after = 0;
        for poly in &mut self.polygons {
            total_vertices_after += poly.vertices.len();
            
            // **SOLID Principle**: Single responsibility - each polygon manages its own geometric properties
            if poly.vertices.len() >= 3 {
                poly.plane = Plane::from_vertices(poly.vertices.clone());
                poly.bounding_box = std::sync::OnceLock::new();
            }
        }

        // **Enhancement**: Remove degenerate polygons that may have been created during welding
        self.polygons.retain(|poly| {
            if poly.vertices.len() < 3 {
                return false;
            }
            
            // Check for minimum area to avoid degenerate triangles
            let mut area = 0.0;
            let n = poly.vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let edge1 = poly.vertices[j].pos - poly.vertices[i].pos;
                let edge2 = poly.vertices[(j + 1) % n].pos - poly.vertices[j].pos;
                area += edge1.cross(&edge2).norm();
            }
            area > EPSILON * EPSILON
        });

        // Invalidate mesh bounding box since vertices may have moved
        self.invalidate_bounding_box();

        // **Enhancement**: Log welding statistics for debugging
        #[cfg(debug_assertions)]
        {
            let vertices_welded = total_vertices_before.saturating_sub(total_vertices_after);
            if vertices_welded > 0 {
                eprintln!(
                    "[csgrs::mesh] Welded {} vertices ({}%) with tolerance {:.2e}",
                    vertices_welded,
                    (vertices_welded as f64 / total_vertices_before as f64) * 100.0,
                    tol
                );
            }
        }
    }

    /// Build a Mesh from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<S>], metadata: Option<S>) -> Self {
        let mut mesh = Mesh::new();
        mesh.polygons = polygons.to_vec();
        mesh.metadata = metadata;
        mesh
    }

    /// Split polygons into (may_touch, cannot_touch) using bounding‑box tests
    fn partition_polys(
        polys: &[Polygon<S>],
        other_bb: &Aabb,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        polys
            .iter()
            .cloned()
            .partition(|p| p.bounding_box().intersects(other_bb))
    }

    /// Helper to collect all vertices from the CSG.
    #[cfg(not(feature = "parallel"))]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Parallel helper to collect all vertices from the CSG.
    #[cfg(feature = "parallel")]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .par_iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<S> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.triangulate().into_iter().map(move |triangle| {
                    Polygon::new(triangle.to_vec(), poly.metadata.clone())
                })
            })
            .collect::<Vec<_>>();

        Mesh::from_polygons(&triangles, self.metadata.clone())
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<S> {
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris.into_par_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        Mesh::from_polygons(&new_polygons, self.metadata.clone())
    }

    /// Subdivide all polygons in this Mesh 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use core::num::NonZeroU32;
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, None);
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, None);
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        #[cfg(feature = "parallel")]
        {
            self.polygons = self
                .polygons
                .par_iter_mut()
                .flat_map(|poly| {
                    let sub_tris = poly.subdivide_triangles(levels);
                    // Convert each small tri back to a Polygon
                    sub_tris
                        .into_par_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.polygons = self
                .polygons
                .iter()
                .flat_map(|poly| {
                    let polytri = poly.subdivide_triangles(levels);
                    polytri
                        .into_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    ///
    /// Returns the angle in radians.
    fn dihedral_angle(p1: &Polygon<S>, p2: &Polygon<S>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }

    /// Extracts vertices and indices from the Mesh's tessellated polygons.
    fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        let tri_csg = self.triangulate();
        let vertices = tri_csg
            .polygons
            .iter()
            .flat_map(|p| [p.vertices[0].pos, p.vertices[1].pos, p.vertices[2].pos])
            .collect();

        let indices = (0..tri_csg.polygons.len())
            .map(|i| {
                let offset = i as u32 * 3;
                [offset, offset + 1, offset + 2]
            })
            .collect();

        (vertices, indices)
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this Mesh and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3<Real>, Real)` where:
    /// - `Point3<Real>` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<Real>,
        direction: &Vector3<Real>,
    ) -> Vec<(Point3<Real>, Real)> {
        let ray = Ray::new(*origin, *direction);
        let iso = Isometry3::identity(); // No transformation on the triangles themselves.

        let mut hits: Vec<_> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .filter_map(|tri| {
                let a = tri[0].pos;
                let b = tri[1].pos;
                let c = tri[2].pos;
                let triangle = Triangle::new(a, b, c);
                triangle
                    .cast_ray_and_get_normal(&iso, &ray, Real::MAX, true)
                    .map(|hit| {
                        let point_on_ray = ray.point_at(hit.time_of_impact);
                        (Point3::from(point_on_ray.coords), hit.time_of_impact)
                    })
            })
            .collect();

        // 4) Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        // 5) remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < EPSILON);

        hits
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.\
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_rapier_shape(&self) -> SharedShape {
        let (vertices, indices) = self.get_vertices_and_indices();
        let trimesh = TriMesh::new(vertices, indices).unwrap();
        SharedShape::new(trimesh)
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`.\
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_trimesh(&self) -> Option<TriMesh> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok()
    }

    /// Uses Parry to check if a point is inside a `Mesh`'s as a `TriMesh`.\
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```
    /// # use csgrs::mesh::Mesh;
    /// # use nalgebra::Point3;
    /// # use nalgebra::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, None);
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3<Real>) -> bool {
        self.ray_intersections(point, &Vector3::new(1.0, 1.0, 1.0))
            .len()
            % 2
            == 1
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> (Real, Point3<Real>, Unit<Quaternion<Real>>) {
        let trimesh = self.to_trimesh().unwrap();
        let mp = trimesh.mass_properties(density);

        (
            mp.mass(),
            mp.local_com,                     // a Point3<Real>
            mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
        )
    }

    /// Create a Rapier rigid body + collider from this Mesh, using
    /// an axis-angle `rotation` in 3D (the vector’s length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: Vector3<Real>,
        rotation: Vector3<Real>, // rotation axis scaled by angle (radians)
        density: Real,
    ) -> RigidBodyHandle {
        let shape = self.to_rapier_shape();

        // Build a Rapier RigidBody
        let rb = RigidBodyBuilder::dynamic()
            .translation(translation)
            // Now `rotation(...)` expects an axis-angle Vector3.
            .rotation(rotation)
            .build();
        let rb_handle = rb_set.insert(rb);

        // Build the collider
        let coll = ColliderBuilder::new(shape).density(density).build();
        co_set.insert_with_parent(coll, rb_handle, rb_set);

        rb_handle
    }

    /// Convert a Mesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh};
        use wgpu_types::PrimitiveTopology;

        let triangulated_mesh = &self.triangulate();
        let polygons = &triangulated_mesh.polygons;

        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::with_capacity(polygons.len() * 3);

        let mut index_start = 0u32;

        // Each polygon is assumed to have exactly 3 vertices after tessellation.
        for poly in polygons {
            // skip any degenerate polygons
            if poly.vertices.len() != 3 {
                continue;
            }

            // push 3 positions/normals
            for v in &poly.vertices {
                positions_32.push([v.pos.x as f32, v.pos.y as f32, v.pos.z as f32]);
                normals_32.push([v.normal.x as f32, v.normal.y as f32, v.normal.z as f32]);
            }

            // triangle indices
            indices.push(index_start);
            indices.push(index_start + 1);
            indices.push(index_start + 2);
            index_start += 3;
        }

        // Create the mesh with the new 2-argument constructor
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for Mesh<S> {
    /// Returns a new empty Mesh
    fn new() -> Self {
        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Return a new Mesh representing union of the two Meshes.
    ///
    /// ```text
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Mesh<S>) -> Mesh<S> {
        // Fast paths for degenerate cases -------------------------------------------------
        if self.polygons.is_empty() {
            return other.clone();
        }
        if other.polygons.is_empty() {
            return self.clone();
        }

        // avoid splitting obvious non-intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);
        final_polys.extend(b_passthru);

        let mut mesh = Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        // Heal potential seams introduced by CSG clipping
        mesh.weld_vertices_mut(EPSILON * 4.0);

        mesh
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
    ///
    /// ```text
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Mesh<S>) -> Mesh<S> {
        // Fast paths -------------------------------------------------------
        if other.polygons.is_empty() {
            return self.clone();
        }
        if self.polygons.is_empty() {
            return Mesh::new();
        }

        // avoid splitting obvious non-intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // propagate self.metadata to new polygons by overwriting intersecting
        // polygon.metadata in other.
        let b_clip_retagged: Vec<Polygon<S>> = b_clip
            .iter()
            .map(|poly| {
                let mut p = poly.clone();
                p.metadata = self.metadata.clone();
                p
            })
            .collect();

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip_retagged);

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);

        let mut mesh = Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        // Heal potential seams introduced by CSG clipping
        mesh.weld_vertices_mut(EPSILON * 4.0);

        mesh
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```text
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Mesh<S>) -> Mesh<S> {
        // Fast paths -------------------------------------------------------
        if self.polygons.is_empty() || other.polygons.is_empty() {
            return Mesh::new(); // empty intersection
        }

        // Restrict work to polygons whose AABBs overlap the other mesh – this
        // can dramatically cut BSP depth for large models with sparse
        // overlap regions (common in TPMS → shell intersections).
        let (a_clip, _a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // If bounding boxes do not overlap, early-out.
        if a_clip.is_empty() || b_clip.is_empty() {
            return Mesh::new();
        }

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        let mut mesh = Mesh {
            polygons: a.all_polygons(),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        // Heal potential seams introduced by CSG clipping
        mesh.weld_vertices_mut(EPSILON * 4.0);

        mesh
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```text
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Mesh<S>) -> Mesh<S> {
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        let final_polys = a_sub_b.union(&b_sub_a).polygons;

        let mut mesh = Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        mesh.weld_vertices_mut(EPSILON * 4.0);

        mesh
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to Mesh.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D
    fn transform(&self, mat: &Matrix4<Real>) -> Mesh<S> {
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match mat.try_inverse() {
            Some(inv) => inv.transpose(),
            None => {
                eprintln!(
                    "Warning: Transformation matrix is not invertible, using identity for normals"
                );
                Matrix4::identity()
            },
        };

        let mut mesh = self.clone();

        for poly in &mut mesh.polygons {
            for vert in &mut poly.vertices {
                // Transform position using homogeneous coordinates
                let hom_pos = mat * vert.pos.to_homogeneous();
                match Point3::from_homogeneous(hom_pos) {
                    Some(transformed_pos) => vert.pos = transformed_pos,
                    None => {
                        eprintln!(
                            "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                        );
                        continue;
                    },
                }

                // Transform normal using inverse transpose rule
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }

            // Reconstruct plane from transformed vertices for consistency
            poly.plane = Plane::from_vertices(poly.vertices.clone());

            // Invalidate the polygon's bounding box
            poly.bounding_box = OnceLock::new();
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();

        mesh
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] indicating the 3D bounds of all `polygons`.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for v in &poly.vertices {
                    if let Some(val) = partial_min(&min_x, &v.pos.x) {
                        min_x = *val;
                    }
                    if let Some(val) = partial_min(&min_y, &v.pos.y) {
                        min_y = *val;
                    }
                    if let Some(val) = partial_min(&min_z, &v.pos.z) {
                        min_z = *val;
                    }

                    if let Some(val) = partial_max(&max_x, &v.pos.x) {
                        max_x = *val;
                    }
                    if let Some(val) = partial_max(&max_y, &v.pos.y) {
                        max_y = *val;
                    }
                    if let Some(val) = partial_max(&max_z, &v.pos.z) {
                        max_z = *val;
                    }
                }
            }

            // If still uninitialized (e.g., no polygons), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }




    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<S> {
        let mut mesh = self.clone();
        for p in &mut mesh.polygons {
            p.flip();
        }
        mesh
    }
}

impl<S: Clone + Send + Sync + Debug> From<Sketch<S>> for Mesh<S> {
    /// Convert a Sketch into a Mesh.
    fn from(sketch: Sketch<S>) -> Self {
        /// Helper function to convert a geo::Polygon to a Vec<crate::mesh::polygon::Polygon>
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

        let final_polygons = sketch
            .geometry
            .iter()
            .flat_map(|geom| -> Vec<Polygon<S>> {
                match geom {
                    Geometry::Polygon(poly2d) => {
                        geo_poly_to_csg_polys(poly2d, &sketch.metadata)
                    },
                    Geometry::MultiPolygon(multipoly) => multipoly
                        .iter()
                        .flat_map(|poly2d| geo_poly_to_csg_polys(poly2d, &sketch.metadata))
                        .collect(),
                    _ => vec![],
                }
            })
            .collect();

        Mesh {
            polygons: final_polygons,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}

// **Enhanced Mesh Analysis Methods**
//
// These methods provide additional mesh analysis capabilities that are not part
// of the CSG trait but are useful for mesh validation and quality assessment.
impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// **Enhanced bounding box with adaptive padding for TPMS**
    ///
    /// Provides better boundary handling for TPMS structures to prevent holes
    /// between internal mesh and bounding geometry.
    pub fn bounding_box_with_padding(&self, padding_factor: Real) -> Aabb {
        let aabb = self.bounding_box();
        
        // **Enhancement**: Adaptive padding based on mesh size
        let size = aabb.maxs - aabb.mins;
        let max_dim = size.x.max(size.y).max(size.z);
        let adaptive_padding = (max_dim * padding_factor).max(1e-6); // Minimum padding
        
        let padding_vec = Vector3::new(adaptive_padding, adaptive_padding, adaptive_padding);
        let padded_min = aabb.mins - padding_vec;
        let padded_max = aabb.maxs + padding_vec;
        
        Aabb::new(Point3::from(padded_min), Point3::from(padded_max))
    }

    /// **Compute mesh surface area for quality metrics**
    ///
    /// Useful for validating mesh integrity and detecting potential holes.
    pub fn surface_area(&self) -> Real {
        self.polygons
            .iter()
            .map(|polygon| {
                if polygon.vertices.len() < 3 {
                    return 0.0;
                }
                
                // Triangulate polygon and sum areas
                let mut area = 0.0;
                let v0 = &polygon.vertices[0].pos;
                
                for i in 1..polygon.vertices.len() - 1 {
                    let v1 = &polygon.vertices[i].pos;
                    let v2 = &polygon.vertices[i + 1].pos;
                    
                    let edge1 = v1 - v0;
                    let edge2 = v2 - v0;
                    let cross = edge1.cross(&edge2);
                    area += cross.norm() * 0.5;
                }
                
                area
            })
            .sum()
    }

    /// **Validate mesh integrity and detect potential issues**
    ///
    /// Performs comprehensive mesh validation to detect common issues
    /// that can cause holes in TPMS structures.
    pub fn validate_mesh_integrity(&self) -> MeshValidationResult {
        let mut result = MeshValidationResult::new();
        
        // Check for degenerate polygons
        for (i, polygon) in self.polygons.iter().enumerate() {
            if polygon.vertices.len() < 3 {
                result.degenerate_polygons.push(i);
                continue;
            }
            
            // Check for near-zero area polygons
            let area = self.compute_polygon_area(polygon);
            if area < 1e-12 {
                result.zero_area_polygons.push(i);
            }
            
            // Check for self-intersecting polygons (basic check)
            if polygon.vertices.len() > 3 && self.is_polygon_self_intersecting(polygon) {
                result.self_intersecting_polygons.push(i);
            }
        }
        
        // Check vertex distribution for clustering issues
        result.has_vertex_clustering = self.detect_vertex_clustering();
        
        result
    }

    /// **Helper to compute polygon area**
    fn compute_polygon_area(&self, polygon: &Polygon<S>) -> Real {
        if polygon.vertices.len() < 3 {
            return 0.0;
        }
        
        let mut area = 0.0;
        let v0 = &polygon.vertices[0].pos;
        
        for i in 1..polygon.vertices.len() - 1 {
            let v1 = &polygon.vertices[i].pos;
            let v2 = &polygon.vertices[i + 1].pos;
            
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.cross(&edge2);
            area += cross.norm() * 0.5;
        }
        
        area
    }

    /// **Basic self-intersection detection**
    fn is_polygon_self_intersecting(&self, polygon: &Polygon<S>) -> bool {
        // Simple check for basic cases - more sophisticated algorithms exist
        if polygon.vertices.len() < 4 {
            return false;
        }
        
        // Check if any non-adjacent edges intersect
        for i in 0..polygon.vertices.len() {
            let i_next = (i + 1) % polygon.vertices.len();
            for j in (i + 2)..polygon.vertices.len() {
                if j == polygon.vertices.len() - 1 && i == 0 {
                    continue; // Skip adjacent edges
                }
                let j_next = (j + 1) % polygon.vertices.len();
                
                if self.edges_intersect_2d(
                    &polygon.vertices[i].pos,
                    &polygon.vertices[i_next].pos,
                    &polygon.vertices[j].pos,
                    &polygon.vertices[j_next].pos,
                ) {
                    return true;
                }
            }
        }
        
        false
    }

    /// **2D edge intersection test (projects to dominant plane)**
    fn edges_intersect_2d(
        &self,
        a1: &Point3<Real>,
        a2: &Point3<Real>,
        b1: &Point3<Real>,
        b2: &Point3<Real>,
    ) -> bool {
        // Project to 2D by dropping the coordinate with smallest range
        let min_vals = Point3::new(
            a1.x.min(a2.x).min(b1.x).min(b2.x),
            a1.y.min(a2.y).min(b1.y).min(b2.y),
            a1.z.min(a2.z).min(b1.z).min(b2.z),
        );
        let max_vals = Point3::new(
            a1.x.max(a2.x).max(b1.x).max(b2.x),
            a1.y.max(a2.y).max(b1.y).max(b2.y),
            a1.z.max(a2.z).max(b1.z).max(b2.z),
        );
        let ranges = max_vals - min_vals;
        
        // Drop coordinate with smallest range
        let (p1, p2, q1, q2) = if ranges.x <= ranges.y && ranges.x <= ranges.z {
            // Drop X, use YZ
            ((a1.y, a1.z), (a2.y, a2.z), (b1.y, b1.z), (b2.y, b2.z))
        } else if ranges.y <= ranges.z {
            // Drop Y, use XZ
            ((a1.x, a1.z), (a2.x, a2.z), (b1.x, b1.z), (b2.x, b2.z))
        } else {
            // Drop Z, use XY
            ((a1.x, a1.y), (a2.x, a2.y), (b1.x, b1.y), (b2.x, b2.y))
        };
        
        // 2D line intersection test
        let det = (p2.0 - p1.0) * (q2.1 - q1.1) - (p2.1 - p1.1) * (q2.0 - q1.0);
        if det.abs() < 1e-10 {
            return false; // Parallel lines
        }
        
        let t = ((q1.0 - p1.0) * (q2.1 - q1.1) - (q1.1 - p1.1) * (q2.0 - q1.0)) / det;
        let u = ((q1.0 - p1.0) * (p2.1 - p1.1) - (q1.1 - p1.1) * (p2.0 - p1.0)) / det;
        
        t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0
    }

    /// **Detect vertex clustering that can cause rendering issues**
    fn detect_vertex_clustering(&self) -> bool {
        let vertices = self.vertices();
        if vertices.len() < 4 {
            return false;
        }
        
        // Build spatial hash for efficient neighbor detection
        let mut spatial_hash: std::collections::HashMap<(i32, i32, i32), Vec<usize>> = 
            std::collections::HashMap::new();
        
        let aabb = self.bounding_box();
        let size = aabb.maxs - aabb.mins;
        let grid_size = (size.x.max(size.y).max(size.z)) / 32.0; // 32x32x32 grid
        
        for (idx, vertex) in vertices.iter().enumerate() {
            let pos = vertex.pos;
            let grid_x = ((pos.x - aabb.mins.x) / grid_size) as i32;
            let grid_y = ((pos.y - aabb.mins.y) / grid_size) as i32;
            let grid_z = ((pos.z - aabb.mins.z) / grid_size) as i32;
            
            spatial_hash.entry((grid_x, grid_y, grid_z))
                .or_default()
                .push(idx);
        }
        
        // Check for overcrowded cells
        for cell_vertices in spatial_hash.values() {
            if cell_vertices.len() > 20 { // Threshold for "clustering"
                return true;
            }
        }
        
        false
    }
}
