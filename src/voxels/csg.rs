//! Voxels CSG trait implementation using SVO with embedded BSP

use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::Real;
use crate::traits::CSG;
use crate::voxels::Svo;

use crate::voxels::polygon::Polygon;
use crate::voxels::surface_extraction::SurfaceExtractor;
use crate::voxels::svo_csg::SvoCsg;
use nalgebra::{Matrix4, Point3, Vector3};
use std::fmt::Debug;
use std::sync::OnceLock;

/// Voxels container using Sparse Voxel Octree with embedded BSP for precise surface representation
#[derive(Debug)]
pub struct Voxels<S: Clone> {
    /// Sparse Voxel Octree storing occupancy and embedded BSP at Mixed cells
    svo: Svo<S>,
    /// Cached surface polygons extracted from SVO (lazy evaluation)
    surface_cache: OnceLock<Vec<Polygon<S>>>,
    /// Global metadata for the entire voxel structure
    pub metadata: Option<S>,
}

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    /// Create empty voxels with default spatial bounds
    pub fn new() -> Self {
        Self::with_bounds(Point3::origin(), 1.0, 8)
    }

    /// Create empty voxels with specified spatial bounds
    pub fn with_bounds(center: Point3<Real>, half_size: Real, max_depth: u8) -> Self {
        Self {
            svo: Svo::new(center, half_size, max_depth),
            surface_cache: OnceLock::new(),
            metadata: None,
        }
    }

    /// Create voxels from polygon list (legacy compatibility)
    pub fn from_polygons(polys: &[Polygon<S>], metadata: Option<S>) -> Self {
        if polys.is_empty() {
            return Self::new();
        }

        // Compute bounding box of all polygons
        let mut min_pt = polys[0].vertices[0].pos;
        let mut max_pt = min_pt;

        for poly in polys {
            for vertex in &poly.vertices {
                let p = vertex.pos;
                min_pt.x = min_pt.x.min(p.x);
                min_pt.y = min_pt.y.min(p.y);
                min_pt.z = min_pt.z.min(p.z);
                max_pt.x = max_pt.x.max(p.x);
                max_pt.y = max_pt.y.max(p.y);
                max_pt.z = max_pt.z.max(p.z);
            }
        }

        // Create SVO with appropriate bounds
        let center = Point3::new(
            (min_pt.x + max_pt.x) * 0.5,
            (min_pt.y + max_pt.y) * 0.5,
            (min_pt.z + max_pt.z) * 0.5,
        );
        let half = ((max_pt.x - min_pt.x) * 0.5)
            .max((max_pt.y - min_pt.y) * 0.5)
            .max((max_pt.z - min_pt.z) * 0.5) * 1.1; // Slightly larger

        let resolution = 6; // Reasonable default depth
        let mut voxels = Self::with_bounds(center, half, resolution);
        voxels.metadata = metadata;

        // Voxelize polygons into SVO using point sampling
        Self::voxelize_polygons_into_svo(&mut voxels.svo, polys);

        // Cache the original polygons for immediate access
        voxels.surface_cache.set(polys.to_vec()).ok();

        voxels
    }

    /// Access to underlying SVO for advanced operations
    pub fn svo(&self) -> &Svo<S> { &self.svo }

    /// Mutable access to SVO for construction
    pub fn svo_mut(&mut self) -> &mut Svo<S> {
        self.invalidate_surface_cache();
        &mut self.svo
    }

    /// Get surface polygons (extracted from SVO or cached)
    pub fn polygons(&self) -> &[Polygon<S>] {
        self.surface_cache.get_or_init(|| self.extract_surface_polygons())
    }

    /// Invalidate surface cache when SVO changes
    fn invalidate_surface_cache(&mut self) {
        self.surface_cache.take();
    }

    /// Set surface cache (for internal use during construction)
    pub(crate) fn set_surface_cache(&mut self, polygons: Vec<Polygon<S>>) {
        self.surface_cache.set(polygons).ok();
    }

    /// Extract surface polygons from SVO using dedicated surface extractor
    fn extract_surface_polygons(&self) -> Vec<Polygon<S>> {
        SurfaceExtractor::extract_polygons(&self.svo)
    }

    #[inline]
    pub fn metadata(&self) -> Option<S> { self.metadata.clone() }

    /// Optimize memory usage and performance
    pub fn optimize(&mut self) {
        // Optimize SVO structure
        self.svo.optimize_memory();
        self.svo.simplify();

        // Invalidate surface cache to force regeneration with optimized structure
        self.invalidate_surface_cache();
    }

    /// Get performance statistics
    pub fn statistics(&self) -> crate::voxels::SvoStatistics {
        self.svo.statistics()
    }

    /// Get memory usage in bytes
    pub fn memory_usage(&self) -> usize {
        let mut size = self.svo.memory_usage();

        // Add surface cache memory if present
        if let Some(polygons) = self.surface_cache.get() {
            size += polygons.len() * std::mem::size_of::<crate::voxels::polygon::Polygon<S>>();
        }

        size
    }

    /// Voxelize polygons into SVO using point sampling
    fn voxelize_polygons_into_svo(svo: &mut Svo<S>, polys: &[Polygon<S>]) {
        if polys.is_empty() {
            return;
        }

        // Sample points within the SVO bounds and test against polygons
        let samples_per_dim = 1 << svo.max_depth.min(6); // Limit sampling for performance
        let step = (svo.half * 2.0) / samples_per_dim as Real;

        for i in 0..samples_per_dim {
            for j in 0..samples_per_dim {
                for k in 0..samples_per_dim {
                    let x = svo.center.x - svo.half + (i as Real + 0.5) * step;
                    let y = svo.center.y - svo.half + (j as Real + 0.5) * step;
                    let z = svo.center.z - svo.half + (k as Real + 0.5) * step;
                    let point = Point3::new(x, y, z);

                    // Test if point is inside the mesh using ray casting
                    if Self::point_inside_mesh(&point, polys) {
                        svo.insert_point(&point);
                    }
                }
            }
        }
    }

    /// Test if a point is inside a mesh using ray casting
    fn point_inside_mesh(point: &Point3<Real>, polys: &[Polygon<S>]) -> bool {
        use nalgebra::Vector3;

        // Cast ray in +X direction and count intersections
        let ray_dir = Vector3::new(1.0, 0.0, 0.0);
        let mut intersection_count = 0;

        for poly in polys {
            if Self::ray_intersects_polygon(point, &ray_dir, poly) {
                intersection_count += 1;
            }
        }

        // Odd number of intersections means inside
        intersection_count % 2 == 1
    }

    /// Test ray-polygon intersection
    fn ray_intersects_polygon(origin: &Point3<Real>, direction: &Vector3<Real>, poly: &Polygon<S>) -> bool {
        if poly.vertices.len() < 3 {
            return false;
        }

        // Simple triangle intersection for first 3 vertices
        let v0 = poly.vertices[0].pos;
        let v1 = poly.vertices[1].pos;
        let v2 = poly.vertices[2].pos;

        // Möller-Trumbore intersection algorithm
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let h = direction.cross(&edge2);
        let a = edge1.dot(&h);

        if a.abs() < 1e-8 {
            return false; // Ray parallel to triangle
        }

        let f = 1.0 / a;
        let s = Vector3::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);
        let u = f * s.dot(&h);

        if u < 0.0 || u > 1.0 {
            return false;
        }

        let q = s.cross(&edge1);
        let v = f * direction.dot(&q);

        if v < 0.0 || u + v > 1.0 {
            return false;
        }

        let t = f * edge2.dot(&q);
        t > 1e-8 // Ray intersects triangle in forward direction
    }
}

impl<S: Clone + Debug + Send + Sync> Clone for Voxels<S> {
    fn clone(&self) -> Self {
        Self {
            svo: self.svo.clone(),
            surface_cache: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Debug + Send + Sync> CSG for Voxels<S> {
    fn new() -> Self { Self::new() }

    fn union(&self, other: &Self) -> Self {
        // Use SVO-based union for true voxel operations
        let result_svo = SvoCsg::union(&self.svo, &other.svo);
        Self::from_svo(result_svo, self.metadata.clone())
    }

    fn difference(&self, other: &Self) -> Self {
        // Use SVO-based difference for true voxel operations
        let result_svo = SvoCsg::difference(&self.svo, &other.svo);
        Self::from_svo(result_svo, self.metadata.clone())
    }

    fn intersection(&self, other: &Self) -> Self {
        // Use SVO-based intersection for true voxel operations
        let result_svo = SvoCsg::intersection(&self.svo, &other.svo);
        Self::from_svo(result_svo, self.metadata.clone())
    }

    fn xor(&self, other: &Self) -> Self {
        let u = self.union(other);
        let i = self.intersection(other);
        u.difference(&i)
    }

    fn transform(&self, matrix: &Matrix4<Real>) -> Self {
        // TODO: Implement proper SVO transformation
        let polygons: Vec<Polygon<S>> = self.polygons().iter().map(|p| p.transform(matrix)).collect();
        Self::from_polygons(&polygons, self.metadata.clone())
    }

    fn inverse(&self) -> Self {
        // TODO: Implement proper SVO inversion
        let polygons: Vec<Polygon<S>> = self.polygons().iter().cloned().map(|mut p| {
            p.flip();
            p
        }).collect();
        Self::from_polygons(&polygons, self.metadata.clone())
    }

    fn bounding_box(&self) -> Aabb {
        self.svo.aabb()
    }

    fn invalidate_bounding_box(&mut self) {
        // SVO bounds are immutable, no need to invalidate
    }
}



impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    /// Triangulate the current polygons and return a triangle-only Voxels
    pub fn triangulate(&self) -> Voxels<S> {
        let mut tris = Vec::new();
        for poly in self.polygons() {
            for tri in poly.triangulate() {
                tris.push(Polygon::new(tri.to_vec(), self.metadata.clone()));
            }
        }
        Voxels::from_polygons(&tris, self.metadata.clone())
    }
}
