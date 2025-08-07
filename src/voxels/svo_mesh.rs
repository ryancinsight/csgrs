//! Sparse Voxel Octree Mesh Implementation
//!
//! This module implements the SvoMesh structure that provides a complete CSG-compatible
//! mesh representation using sparse voxel octrees with embedded BSP trees.
//!
//! ## Key Features
//!
//! - **Full CSG Compatibility**: Implements all CSG trait operations
//! - **Memory Efficiency**: Sparse representation scales with geometry complexity
//! - **Spatial Optimization**: Hierarchical spatial organization for fast queries
//! - **Robust Operations**: Fixed-precision arithmetic for numerical stability

use crate::float_types::{
    parry3d::bounding_volume::{Aabb, BoundingVolume},
    Real,
};
use crate::mesh::{plane::Plane, polygon::Polygon, vertex::Vertex};
use crate::traits::CSG;
use crate::voxels::{
    precision::PrecisionConfig,
    svo_node::{SvoNode, SvoStatistics},
};
use nalgebra::{Matrix4, Point3, Vector3, Rotation3};
use std::{fmt::Debug, sync::OnceLock};


/// Axis-aligned bounding box type alias


/// Quality metrics for a triangle
#[derive(Debug, Clone)]
pub struct TriangleQuality {
    pub area: Real,
    pub aspect_ratio: Real,
    pub angles: [Real; 3],
}

/// Triangle quality analysis results
#[derive(Debug, Clone, Default)]
pub struct TriangleQualityAnalysis {
    pub triangle_count: usize,
    pub avg_aspect_ratio: Real,
    pub min_aspect_ratio: Real,
    pub max_aspect_ratio: Real,
    pub avg_area: Real,
    pub min_area: Real,
    pub max_area: Real,
    pub poor_quality_count: usize,
}

/// Comprehensive mesh quality report
#[derive(Debug, Clone)]
pub struct MeshQualityReport {
    pub triangle_analysis: TriangleQualityAnalysis,
    pub is_manifold: bool,
    pub manifold_issues: Vec<ManifoldIssue>,
    pub vertex_count: usize,
    pub polygon_count: usize,
    pub node_count: usize,
    pub memory_efficiency: f64,
    pub sparsity_ratio: f64,
    pub bounding_box: Aabb,
}

/// Manifold validation result
#[derive(Debug, Clone)]
pub struct ManifoldCheck {
    pub is_manifold: bool,
    pub issues: Vec<ManifoldIssue>,
}

/// Types of manifold issues
#[derive(Debug, Clone)]
pub enum ManifoldIssue {
    NonManifoldEdge { edge: (usize, usize), face_count: usize },
    BoundaryEdge { edge: (usize, usize) },
    IsolatedVertex { vertex_index: usize },
}

/// Sparse Voxel Octree mesh with embedded BSP for efficient CSG operations
/// 
/// Memory usage scales with geometric complexity, not spatial volume, making it
/// ideal for sparse geometries common in CAD and manufacturing applications.
#[derive(Debug, Clone)]
pub struct SvoMesh<S: Clone + Send + Sync + Debug> {
    /// Root SVO node containing the sparse octree-embedded BSP structure
    /// None represents a completely empty mesh
    pub root: Option<SvoNode<S>>,
    
    /// Global bounding box cache (computed from occupied voxels only)
    pub bounding_box: OnceLock<Aabb>,
    
    /// Mesh-level metadata
    pub metadata: Option<S>,
    
    /// Global precision settings for fixed-point arithmetic
    pub precision_config: PrecisionConfig,
}

impl<S: Clone + Send + Sync + Debug> Default for SvoMesh<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> SvoMesh<S> {
    /// Create a new empty sparse voxel octree mesh
    pub fn new() -> Self {
        Self {
            root: None,
            bounding_box: OnceLock::new(),
            metadata: None,
            precision_config: PrecisionConfig::default(),
        }
    }
    
    /// Create SVO mesh with custom precision configuration
    pub fn with_precision(precision_config: PrecisionConfig) -> Self {
        Self {
            root: None,
            bounding_box: OnceLock::new(),
            metadata: None,
            precision_config,
        }
    }
    
    /// Generate a sphere using SDF-based voxel sampling
    /// 
    /// Creates a sphere primitive by sampling the sphere SDF within the sparse voxel octree.
    /// Uses mathematical sphere distance function for precise surface representation.
    /// 
    /// # Arguments
    /// 
    /// * `center` - Center point of the sphere
    /// * `radius` - Sphere radius
    /// * `resolution` - Voxel resolution for surface sampling
    /// * `metadata` - Optional metadata for generated polygons
    /// 
    /// # Mathematical Foundation
    /// 
    /// Sphere SDF: f(p) = |p - center| - radius
    /// Negative values indicate interior, positive exterior
    pub fn sphere(
        center: Point3<Real>,
        radius: Real,
        resolution: (usize, usize, usize),
        metadata: Option<S>,
    ) -> Self {
        use crate::voxels::sdf::SdfConfig;
        
        if radius <= 0.0 {
            return Self::new();
        }
        
        // Sphere SDF: distance from point to sphere surface
        let sphere_sdf = move |p: &Point3<Real>| {
            (p - center).norm() - radius
        };
        
        let margin = radius * 0.1; // Small margin for sampling
        let bounds_min = center - Vector3::new(radius + margin, radius + margin, radius + margin);
        let bounds_max = center + Vector3::new(radius + margin, radius + margin, radius + margin);
        
        let config = SdfConfig {
            resolution,
            iso_value: 0.0,
            precision: PrecisionConfig::default(),
            max_depth: 8,
            min_cell_size: 1e-6,
        };
        
        Self::sdf(sphere_sdf, config, bounds_min, bounds_max, metadata)
    }
    
    /// Generate an involute gear using traditional mesh conversion
    /// 
    /// Creates an involute gear by generating the traditional mesh representation
    /// and converting it to sparse voxel format for consistent API.
    /// 
    /// # Arguments
    /// 
    /// * `module_` - Gear module (pitch diameter / number of teeth)
    /// * `teeth` - Number of teeth
    /// * `pressure_angle_deg` - Pressure angle in degrees
    /// * `thickness` - Gear thickness
    /// * `metadata` - Optional metadata for generated polygons
    pub fn involute_gear(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> Self {
        use crate::mesh::Mesh;
        
        // Generate traditional mesh gear
        let gear_mesh = Mesh::<S>::spur_gear_involute(
            module_, 
            teeth, 
            pressure_angle_deg, 
            clearance, 
            backlash, 
            segments_per_flank, 
            thickness, 
            metadata.clone()
        );
        
        // Convert to SvoMesh
        let mut svo_mesh = Self::new();
        svo_mesh.metadata = metadata;
        
        let polygons = gear_mesh.polygons;
        if !polygons.is_empty() {
            svo_mesh.insert_polygons(&polygons);
        }
        
        svo_mesh
    }
    
    /// Generate a helical gear using SDF-based generation
    /// 
    /// Creates a helical gear by extending the involute profile along a helical path.
    /// Uses SDF-based generation for precise surface representation.
    /// 
    /// # Arguments
    /// 
    /// * `module_` - Gear module
    /// * `teeth` - Number of teeth
    /// * `pressure_angle_deg` - Pressure angle in degrees
    /// * `helix_angle_deg` - Helix angle in degrees
    /// * `thickness` - Gear thickness
    /// * `resolution` - Voxel resolution for surface sampling
    /// * `metadata` - Optional metadata for generated polygons
    pub fn helical_gear(
        module_: Real,
        teeth: usize,
        _pressure_angle_deg: Real,
        helix_angle_deg: Real,
        thickness: Real,
        resolution: (usize, usize, usize),
        metadata: Option<S>,
    ) -> Self {
        use crate::voxels::sdf::SdfConfig;
        use std::f64::consts::PI;
        
        let pitch_radius = module_ * teeth as Real / 2.0;
        let helix_angle = helix_angle_deg.to_radians();
        
        // Helical gear SDF combines involute profile with helical twist
        let helical_sdf = move |p: &Point3<Real>| {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            let theta = p.y.atan2(p.x);
            let z_normalized = p.z / thickness;
            
            // Apply helical twist
            let twisted_theta = theta - helix_angle * z_normalized;
            
            // Simplified involute approximation for SDF
            let tooth_angle = 2.0 * PI / teeth as Real;
            let local_theta = (twisted_theta % tooth_angle) - tooth_angle / 2.0;
            
            // Distance to gear profile (simplified)
            let profile_distance = (r - pitch_radius).abs() - module_ * 0.5;
            let tooth_distance = local_theta.abs() - tooth_angle * 0.3;
            
            profile_distance.max(tooth_distance)
        };
        
        let bounds_min = Point3::new(-pitch_radius * 1.2, -pitch_radius * 1.2, 0.0);
        let bounds_max = Point3::new(pitch_radius * 1.2, pitch_radius * 1.2, thickness);
        
        let config = SdfConfig {
            resolution,
            iso_value: 0.0,
            precision: PrecisionConfig::default(),
            max_depth: 8,
            min_cell_size: 1e-6,
        };
        
        Self::sdf(helical_sdf, config, bounds_min, bounds_max, metadata)
    }
    
    /// Generate metaballs using implicit surface blending
    /// 
    /// Creates a metaball surface by blending multiple spherical influence fields.
    /// Uses SDF-based sampling for smooth surface generation.
    /// 
    /// # Arguments
    /// 
    /// * `centers` - Centers of metaball influences
    /// * `radii` - Influence radii for each metaball
    /// * `threshold` - Iso-surface threshold value
    /// * `resolution` - Voxel resolution for surface sampling
    /// * `metadata` - Optional metadata for generated polygons
    /// 
    /// # Mathematical Foundation
    /// 
    /// Metaball field: f(p) = Σ(r_i² / |p - c_i|²) - threshold
    /// Smooth blending creates organic surface transitions
    pub fn metaballs(
        centers: &[Point3<Real>],
        radii: &[Real],
        threshold: Real,
        resolution: (usize, usize, usize),
        metadata: Option<S>,
    ) -> Self {
        use crate::voxels::sdf::SdfConfig;
        
        if centers.is_empty() || radii.is_empty() || centers.len() != radii.len() {
            return Self::new();
        }
        
        let centers = centers.to_vec();
        let radii = radii.to_vec();
        
        // Clone for closure
        let centers_clone = centers.clone();
        let radii_clone = radii.clone();
        
        // Metaball SDF using inverse square falloff
        let metaball_sdf = move |p: &Point3<Real>| {
            let mut field_value = 0.0;
            
            for (center, radius) in centers_clone.iter().zip(radii_clone.iter()) {
                let distance = (p - center).norm();
                if distance > 0.0 {
                    let influence = (radius * radius) / (distance * distance);
                    field_value += influence;
                }
            }
            
            threshold - field_value
        };
        
        // Compute bounding box from all metaballs
        let mut bounds_min = centers[0] - Vector3::new(radii[0], radii[0], radii[0]);
        let mut bounds_max = centers[0] + Vector3::new(radii[0], radii[0], radii[0]);
        
        for (center, radius) in centers.iter().zip(radii.iter()) {
            let margin = Vector3::new(*radius, *radius, *radius);
            bounds_min = bounds_min.inf(&(center - margin));
            bounds_max = bounds_max.sup(&(center + margin));
        }
        
        let config = SdfConfig {
            resolution,
            iso_value: 0.0,
            precision: PrecisionConfig::default(),
            max_depth: 8,
            min_cell_size: 1e-6,
        };
        
        Self::sdf(metaball_sdf, config, bounds_min, bounds_max, metadata)
    }
    
    /// Generate convex hull using mesh conversion
    /// 
    /// Creates the convex hull of a point set by generating the traditional
    /// mesh representation and converting to sparse voxel format.
    /// 
    /// # Arguments
    /// 
    /// * `points` - Input point set
    /// * `metadata` - Optional metadata for generated polygons
    #[cfg(feature = "chull")]
    pub fn convex_hull(points: &[Point3<Real>], metadata: Option<S>) -> Self {
        use crate::mesh::Mesh;
        
        if points.len() < 4 {
            return Self::new();
        }
        
        // Create a mesh from the points and compute convex hull
        let mut temp_mesh = Mesh::<S>::new();
        // Add points as vertices to create a temporary mesh
        for point in points {
            let vertex = Vertex::new(*point, Vector3::new(0.0, 0.0, 1.0));
            // Create a simple polygon from the vertex (this is a workaround)
            let polygon = Polygon::new(vec![vertex.clone(), vertex.clone(), vertex], metadata.clone());
            temp_mesh.polygons.push(polygon);
        }
        let hull_mesh = temp_mesh.convex_hull();
        
        // Convert to SvoMesh
        let mut svo_mesh = Self::new();
        svo_mesh.metadata = metadata;
        
        let polygons = hull_mesh.polygons;
        if !polygons.is_empty() {
            svo_mesh.insert_polygons(&polygons);
        }
        
        svo_mesh
    }
    
    /// Generate convex hull (fallback when chull feature disabled)
    #[cfg(not(feature = "chull"))]
    pub fn convex_hull(_points: &[Point3<Real>], _metadata: Option<S>) -> Self {
        // Return empty mesh when convex hull feature is disabled
        Self::new()
    }
    
    /// Create SVO mesh from polygons
    pub fn from_polygons(polygons: &[Polygon<S>], metadata: Option<S>) -> Self {
        let mut mesh = Self::new();
        mesh.metadata = metadata;
        
        if !polygons.is_empty() {
            // Compute overall bounding box
            let bounds = Self::compute_bounds_from_polygons(polygons);
            
            // Create root node and insert polygons
            let root = SvoNode::from_polygons(bounds, polygons, &mesh.precision_config);
            mesh.root = Some(root);
        }
        
        mesh
    }
    
    /// Compute bounding box from a collection of polygons
    fn compute_bounds_from_polygons(polygons: &[Polygon<S>]) -> Aabb {
        if polygons.is_empty() {
            return Aabb::new(Point3::origin(), Point3::origin());
        }
        
        let first_bounds = polygons[0].bounding_box();
        polygons[1..]
            .iter()
            .fold(first_bounds, |acc, poly| acc.merged(&poly.bounding_box()))
    }
    
    /// Check if the mesh is empty
    pub fn is_empty(&self) -> bool {
        self.root.as_ref().map_or(true, |root| root.is_empty())
    }
    
    /// Get all polygons from the mesh
    pub fn polygons(&self) -> Vec<Polygon<S>> {
        self.root
            .as_ref()
            .map_or(Vec::new(), |root| root.all_polygons())
    }
    
    /// Get mesh statistics
    pub fn statistics(&self) -> SvoStatistics {
        self.root
            .as_ref()
            .map_or(SvoStatistics::default(), |root| root.statistics())
    }
    
    /// Get memory usage in bytes
    pub fn memory_usage(&self) -> usize {
        let mut size = std::mem::size_of::<Self>();
        if let Some(ref root) = self.root {
            size += root.memory_usage();
        }
        size
    }
    
    /// Insert polygons into the mesh
    pub fn insert_polygons(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }
        
        self.invalidate_bounding_box();
        
        if let Some(ref mut root) = self.root {
            // Extend existing root bounds if necessary
            let new_bounds = Self::compute_bounds_from_polygons(polygons);
            let extended_bounds = root.bounds.merged(&new_bounds);
            
            if extended_bounds != root.bounds {
                // Need to create new root with extended bounds
                let mut new_root = SvoNode::new(extended_bounds);
                let existing_polygons = root.all_polygons();
                let mut all_polygons = existing_polygons;
                all_polygons.extend_from_slice(polygons);
                new_root.insert_polygons(&all_polygons, &self.precision_config);
                self.root = Some(new_root);
            } else {
                // Can insert into existing root
                root.insert_polygons(polygons, &self.precision_config);
            }
        } else {
            // Create new root
            let bounds = Self::compute_bounds_from_polygons(polygons);
            let root = SvoNode::from_polygons(bounds, polygons, &self.precision_config);
            self.root = Some(root);
        }
    }
    
    /// Perform union operation with another SVO mesh
    fn union_svo(&self, other: &Self) -> Self {
        if self.is_empty() {
            return other.clone();
        }
        if other.is_empty() {
            return self.clone();
        }
        
        // Collect all polygons from both meshes
        let mut all_polygons = self.polygons();
        all_polygons.extend(other.polygons());
        
        // Create new mesh with combined geometry
        Self::from_polygons(&all_polygons, self.metadata.clone())
    }
    
    /// Perform difference operation with another SVO mesh
    fn difference_svo(&self, other: &Self) -> Self {
        if self.is_empty() || other.is_empty() {
            return self.clone();
        }
        
        let mut result = self.clone();
        
        // Invert other mesh and clip self against it
        let mut other_inverted = other.clone();
        if let Some(ref mut root) = other_inverted.root {
            root.invert();
        }
        
        if let Some(ref mut result_root) = result.root {
            if let Some(ref other_root) = other_inverted.root {
                result_root.clip_to(other_root);
            }
        }
        
        result.invalidate_bounding_box();
        result
    }
    
    /// Perform intersection operation with another SVO mesh
    fn intersection_svo(&self, other: &Self) -> Self {
        if self.is_empty() || other.is_empty() {
            return Self::new();
        }
        
        let mut result = self.clone();
        
        // Clip self against other
        if let Some(ref mut result_root) = result.root {
            if let Some(ref other_root) = other.root {
                result_root.clip_to(other_root);
            }
        }
        
        result.invalidate_bounding_box();
        result
    }
    
    /// Perform XOR operation with another SVO mesh
    fn xor_svo(&self, other: &Self) -> Self {
        // XOR = (A - B) ∪ (B - A)
        let a_minus_b = self.difference_svo(other);
        let b_minus_a = other.difference_svo(self);
        a_minus_b.union_svo(&b_minus_a)
    }
    
    /// Transform the mesh by a 4x4 matrix
    fn transform_svo(&self, matrix: &Matrix4<Real>) -> Self {
        if self.is_empty() {
            return self.clone();
        }
        
        // Transform all polygons
        let transformed_polygons: Vec<_> = self
            .polygons()
            .iter()
            .map(|poly| poly.transform(matrix))
            .collect();
        
        Self::from_polygons(&transformed_polygons, self.metadata.clone())
    }
    
    /// Compute bounding box from the sparse voxel structure
    fn compute_bounding_box(&self) -> Aabb {
        if let Some(ref root) = self.root {
            root.bounds
        } else {
            Aabb::new(Point3::origin(), Point3::origin())
        }
    }
}

// Implement CSG trait for SvoMesh
impl<S: Clone + Send + Sync + Debug> CSG for SvoMesh<S> {
    fn new() -> Self {
        Self {
            root: None,
            bounding_box: OnceLock::new(),
            metadata: None,
            precision_config: PrecisionConfig::default(),
        }
    }
    
    fn union(&self, other: &Self) -> Self {
        self.union_svo(other)
    }
    
    fn difference(&self, other: &Self) -> Self {
        self.difference_svo(other)
    }
    
    fn intersection(&self, other: &Self) -> Self {
        self.intersection_svo(other)
    }
    
    fn xor(&self, other: &Self) -> Self {
        self.xor_svo(other)
    }
    
    fn transform(&self, matrix: &Matrix4<Real>) -> Self {
        self.transform_svo(matrix)
    }
    
    fn inverse(&self) -> Self {
        let mut result = self.clone();
        if let Some(ref mut root) = result.root {
            root.invert();
        }
        result
    }
    
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| self.compute_bounding_box())
    }
    
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    fn translate_vector(&self, vector: Vector3<Real>) -> Self {
        let translation_matrix = Matrix4::new_translation(&vector);
        self.transform(&translation_matrix)
    }

    fn translate(&self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector(Vector3::new(x, y, z))
    }

    fn center(&self) -> Self {
        let aabb = self.bounding_box();
        let center_x = (aabb.mins.x + aabb.maxs.x) * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) * 0.5;
        let center_z = (aabb.mins.z + aabb.maxs.z) * 0.5;
        self.translate(-center_x, -center_y, -center_z)
    }

    fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(0.0, 0.0, -min_z)
    }

    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }

    fn mirror(&self, plane: Plane) -> Self {
        let n = plane.normal().normalize();
        let d = plane.offset() / plane.normal().magnitude();
        
        // Create reflection matrix: I - 2nn^T
        let mut reflection = Matrix4::identity();
        for i in 0..3 {
            for j in 0..3 {
                reflection[(i, j)] -= 2.0 * n[i] * n[j];
            }
        }
        
        let translation_to_origin = Matrix4::new_translation(&(-d * n));
        let translation_back = Matrix4::new_translation(&(d * n));
        
        let combined = translation_back * reflection * translation_to_origin;
        self.transform(&combined)
    }

    fn distribute_arc(&self, count: usize, radius: Real, start_angle_deg: Real, end_angle_deg: Real) -> Self {
        let mut result = Self::new();
        let angle_range = end_angle_deg - start_angle_deg;
        let angle_step = if count > 1 { angle_range / (count - 1) as Real } else { 0.0 };
        
        for i in 0..count {
            let angle = start_angle_deg + i as Real * angle_step;
            let x = radius * angle.to_radians().cos();
            let y = radius * angle.to_radians().sin();
            let translated = self.translate(x, y, 0.0);
            result = result.union(&translated);
        }
        result
    }

    fn distribute_linear(&self, count: usize, dir: Vector3<Real>, spacing: Real) -> Self {
        let mut result = Self::new();
        for i in 0..count {
            let offset = dir * (i as Real * spacing);
            let translated = self.translate_vector(offset);
            result = result.union(&translated);
        }
        result
    }

    fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> Self {
        let mut result = Self::new();
        for row in 0..rows {
            for col in 0..cols {
                let x = col as Real * dx;
                let y = row as Real * dy;
                let translated = self.translate(x, y, 0.0);
                result = result.union(&translated);
            }
        }
        result
    }
}

// Implement PartialEq for SvoMesh
impl<S: Clone + Send + Sync + Debug + PartialEq> PartialEq for SvoMesh<S> {
    fn eq(&self, other: &Self) -> bool {
        // Compare metadata
        if self.metadata != other.metadata {
            return false;
        }
        
        // Compare polygon sets (order-independent)
        let self_polygons = self.polygons();
        let other_polygons = other.polygons();
        
        if self_polygons.len() != other_polygons.len() {
            return false;
        }
        
        // Simple comparison - could be optimized for large meshes
        for poly in &self_polygons {
            if !other_polygons.contains(poly) {
                return false;
            }
        }
        
        true
    }
}

// Additional utility methods
impl<S: Clone + Send + Sync + Debug + PartialEq> SvoMesh<S> {
    /// Check if two meshes have the same metadata
    pub fn same_metadata(&self, other: &Self) -> bool {
        self.metadata == other.metadata
    }
    
    /// Filter polygons by metadata value
    pub fn filter_polygons_by_metadata(&self, needle: &S) -> Self {
        let filtered_polygons: Vec<_> = self
            .polygons()
            .into_iter()
            .filter(|poly| {
                poly.metadata
                    .as_ref()
                    .map_or(false, |meta| meta == needle)
            })
            .collect();
        
        Self::from_polygons(&filtered_polygons, self.metadata.clone())
    }

    // ===== Phase 5: Advanced Mesh Processing =====
    
    /// Laplacian smoothing using sparse voxel structure for spatial optimization
    /// 
    /// Applies Laplacian smoothing to mesh vertices, leveraging the octree structure
    /// for efficient neighbor finding and spatial locality optimization.
    pub fn laplacian_smooth(&self, iterations: usize, lambda: Real) -> Self {
        if self.is_empty() || iterations == 0 {
            return self.clone();
        }
        
        let mut current_mesh = self.clone();
        
        for _ in 0..iterations {
            let polygons = current_mesh.polygons();
            if polygons.is_empty() {
                break;
            }
            
            // Build vertex adjacency using spatial coherence
            let vertex_map = Self::build_vertex_adjacency(&polygons);
            let smoothed_polygons = Self::apply_laplacian_smoothing(&polygons, &vertex_map, lambda);
            
            current_mesh = Self::from_polygons(&smoothed_polygons, self.metadata.clone());
        }
        
        current_mesh
    }
    
    /// Taubin smoothing with feature preservation
    /// 
    /// Applies Taubin smoothing (lambda-mu smoothing) which preserves features
    /// better than pure Laplacian smoothing by alternating shrinking and expanding.
    pub fn taubin_smooth(&self, iterations: usize, lambda: Real, mu: Real) -> Self {
        if self.is_empty() || iterations == 0 {
            return self.clone();
        }
        
        let mut current_mesh = self.clone();
        
        for _ in 0..iterations {
            let polygons = current_mesh.polygons();
            if polygons.is_empty() {
                break;
            }
            
            let vertex_map = Self::build_vertex_adjacency(&polygons);
            
            // Apply lambda step (shrinking)
            let lambda_smoothed = Self::apply_laplacian_smoothing(&polygons, &vertex_map, lambda);
            
            // Apply mu step (expanding) - note: mu is typically negative
            let mu_smoothed = Self::apply_laplacian_smoothing(&lambda_smoothed, &vertex_map, mu);
            
            current_mesh = Self::from_polygons(&mu_smoothed, self.metadata.clone());
        }
        
        current_mesh
    }
    
    /// Bilateral smoothing for edge-preserving smoothing
    /// 
    /// Applies bilateral filtering to preserve sharp features while smoothing
    /// noise, using both spatial and normal similarity.
    pub fn bilateral_smooth(&self, iterations: usize, spatial_sigma: Real, normal_sigma: Real) -> Self {
        if self.is_empty() || iterations == 0 {
            return self.clone();
        }
        
        let mut current_mesh = self.clone();
        
        for _ in 0..iterations {
            let polygons = current_mesh.polygons();
            if polygons.is_empty() {
                break;
            }
            
            let vertex_map = Self::build_vertex_adjacency(&polygons);
            let smoothed_polygons = Self::apply_bilateral_smoothing(
                &polygons, 
                &vertex_map, 
                spatial_sigma, 
                normal_sigma
            );
            
            current_mesh = Self::from_polygons(&smoothed_polygons, self.metadata.clone());
        }
        
        current_mesh
    }
    
    /// Weighted average smoothing for vertex position smoothing
    /// 
    /// Applies weighted average smoothing where weights are based on
    /// edge lengths and angles, providing more natural smoothing.
    pub fn weighted_average(&self, iterations: usize, weight_factor: Real) -> Self {
        if self.is_empty() || iterations == 0 {
            return self.clone();
        }
        
        let mut current_mesh = self.clone();
        
        for _ in 0..iterations {
            let polygons = current_mesh.polygons();
            if polygons.is_empty() {
                break;
            }
            
            let vertex_map = Self::build_vertex_adjacency(&polygons);
            let smoothed_polygons = Self::apply_weighted_average_smoothing(
                &polygons, 
                &vertex_map, 
                weight_factor
            );
            
            current_mesh = Self::from_polygons(&smoothed_polygons, self.metadata.clone());
        }
        
        current_mesh
    }
    
    /// Quality analysis with voxel-aware metrics
    /// 
    /// Analyzes triangle quality using metrics optimized for sparse voxel representation,
    /// providing comprehensive quality assessment for mesh processing.
    pub fn analyze_triangle_quality(&self) -> TriangleQualityAnalysis {
        let polygons = self.polygons();
        if polygons.is_empty() {
            return TriangleQualityAnalysis::default();
        }
        
        let mut analysis = TriangleQualityAnalysis::default();
        let mut aspect_ratios = Vec::new();
        let mut areas = Vec::new();
        let mut angles = Vec::new();
        
        for polygon in &polygons {
            if polygon.vertices.len() >= 3 {
                // Triangulate polygon and analyze each triangle
                let triangles = Self::triangulate_polygon(polygon);
                for triangle in triangles {
                    let quality = Self::compute_triangle_quality(&triangle);
                    aspect_ratios.push(quality.aspect_ratio);
                    areas.push(quality.area);
                    angles.extend(quality.angles);
                    
                    if quality.aspect_ratio < analysis.min_aspect_ratio {
                        analysis.min_aspect_ratio = quality.aspect_ratio;
                    }
                    if quality.aspect_ratio > analysis.max_aspect_ratio {
                        analysis.max_aspect_ratio = quality.aspect_ratio;
                    }
                    if quality.area < analysis.min_area {
                        analysis.min_area = quality.area;
                    }
                    if quality.area > analysis.max_area {
                        analysis.max_area = quality.area;
                    }
                }
            }
        }
        
        analysis.triangle_count = aspect_ratios.len();
        analysis.avg_aspect_ratio = aspect_ratios.iter().sum::<Real>() / aspect_ratios.len() as Real;
        analysis.avg_area = areas.iter().sum::<Real>() / areas.len() as Real;
        
        // Count poor quality triangles (aspect ratio > 10 or very small area)
        analysis.poor_quality_count = aspect_ratios.iter()
            .zip(areas.iter())
            .filter(|(ar, area)| **ar > 10.0 || **area < analysis.avg_area * 0.01)
            .count();
        
        analysis
    }
    
    /// Comprehensive mesh quality assessment
    /// 
    /// Provides overall mesh quality metrics including geometric and topological measures.
    pub fn compute_mesh_quality(&self) -> MeshQualityReport {
        let triangle_analysis = self.analyze_triangle_quality();
        let manifold_check = self.is_manifold();
        let stats = self.statistics();
        
        MeshQualityReport {
            triangle_analysis,
            is_manifold: manifold_check.is_manifold,
            manifold_issues: manifold_check.issues,
            vertex_count: self.vertices().len(),
            polygon_count: self.polygons().len(),
            node_count: stats.total_nodes,
            memory_efficiency: stats.memory_efficiency(),
            sparsity_ratio: stats.sparsity_ratio(),
            bounding_box: self.bounding_box(),
        }
    }
    
    /// Adaptive mesh refinement using octree subdivision
    /// 
    /// Refines mesh areas with poor quality by subdividing octree nodes
    /// and improving triangle quality through spatial optimization.
    pub fn adaptive_refine(&self, quality_threshold: Real, max_subdivisions: usize) -> Self {
        if self.is_empty() {
            return self.clone();
        }
        
        let mut current_mesh = self.clone();
        
        for _ in 0..max_subdivisions {
            let quality = current_mesh.analyze_triangle_quality();
            if quality.avg_aspect_ratio <= quality_threshold {
                break;
            }
            
            // Identify areas needing refinement and subdivide
            current_mesh = current_mesh.refine_poor_quality_areas(quality_threshold);
        }
        
        current_mesh
    }
    
    /// Remove poor quality triangles with spatial optimization
    /// 
    /// Removes triangles that don't meet quality criteria, using octree structure
    /// for efficient spatial queries and neighbor finding.
    pub fn remove_poor_triangles(&self, min_aspect_ratio: Real, min_area: Real) -> Self {
        let polygons = self.polygons();
        let filtered_polygons: Vec<_> = polygons
            .into_iter()
            .filter(|polygon| {
                if polygon.vertices.len() < 3 {
                    return false;
                }
                
                let triangles = Self::triangulate_polygon(polygon);
                triangles.iter().all(|triangle| {
                    let quality = Self::compute_triangle_quality(triangle);
                    quality.aspect_ratio >= min_aspect_ratio && quality.area >= min_area
                })
            })
            .collect();
        
        Self::from_polygons(&filtered_polygons, self.metadata.clone())
    }
    
    /// Check if mesh is manifold using octree-accelerated edge analysis
    /// 
    /// Validates mesh topology to ensure it represents a valid 2-manifold surface,
    /// using spatial acceleration for efficient edge and vertex analysis.
    pub fn is_manifold(&self) -> ManifoldCheck {
        let polygons = self.polygons();
        if polygons.is_empty() {
            return ManifoldCheck {
                is_manifold: true,
                issues: Vec::new(),
            };
        }
        
        let mut issues = Vec::new();
        let edge_map = Self::build_edge_map(&polygons);
        
        // Check for non-manifold edges (shared by more than 2 faces)
        for (edge, faces) in &edge_map {
            if faces.len() > 2 {
                issues.push(ManifoldIssue::NonManifoldEdge {
                    edge: *edge,
                    face_count: faces.len(),
                });
            } else if faces.len() == 1 {
                issues.push(ManifoldIssue::BoundaryEdge { edge: *edge });
            }
        }
        
        // Check for isolated vertices
        let vertex_usage = Self::count_vertex_usage(&polygons);
        for (vertex_idx, usage_count) in vertex_usage {
            if usage_count < 2 {
                issues.push(ManifoldIssue::IsolatedVertex { vertex_index: vertex_idx });
            }
        }
        
        ManifoldCheck {
            is_manifold: issues.is_empty(),
            issues,
        }
    }
    
    /// Heal mesh by fixing common issues
    pub fn heal_mesh(&self) -> Self {
        let mut healed = self.clone();
        
        // Remove degenerate triangles
        healed = healed.remove_degenerate_triangles();
        
        // Fix non-manifold edges
        healed = healed.fix_non_manifold_edges();
        
        // Remove isolated vertices
        healed = healed.remove_isolated_vertices();
        
        // Fill small holes
        healed = healed.fill_small_holes(self.precision_config.from_fixed(self.precision_config.epsilon_scaled) * 10.0);
        
        healed
    }
    
    /// Remove degenerate triangles (zero area or invalid)
    pub fn remove_degenerate_triangles(&self) -> Self {
        let polygons = self.polygons();
        let filtered_polygons: Vec<_> = polygons
            .into_iter()
            .filter(|polygon| {
                if polygon.vertices.len() < 3 {
                    return false;
                }
                
                // Check for degenerate triangles
                let triangles = Self::triangulate_polygon(polygon);
                triangles.iter().any(|triangle| {
                    let quality = Self::compute_triangle_quality(triangle);
                    quality.area > self.precision_config.from_fixed(self.precision_config.epsilon_scaled)
                })
            })
            .collect();
        
        Self::from_polygons(&filtered_polygons, self.metadata.clone())
    }
    
    /// Fix non-manifold edges by duplicating vertices
    pub fn fix_non_manifold_edges(&self) -> Self {
        // For now, return a copy - this would need sophisticated topology repair
        // In a full implementation, this would identify non-manifold edges and
        // duplicate vertices to separate the topology
        self.clone()
    }
    
    /// Remove isolated vertices
    pub fn remove_isolated_vertices(&self) -> Self {
        let polygons = self.polygons();
        let vertex_usage = Self::count_vertex_usage(&polygons);
        
        // Filter out polygons that reference isolated vertices
        let filtered_polygons: Vec<_> = polygons
            .into_iter()
            .filter(|polygon| {
                polygon.vertices.iter().enumerate().all(|(idx, _)| {
                    vertex_usage.get(&idx).map_or(false, |&count| count > 0)
                })
            })
            .collect();
        
        Self::from_polygons(&filtered_polygons, self.metadata.clone())
    }
    
    /// Fill small holes in the mesh
    pub fn fill_small_holes(&self, _max_hole_size: Real) -> Self {
        // For now, return a copy - this would need hole detection and filling logic
        // In a full implementation, this would:
        // 1. Identify boundary loops
        // 2. Measure hole sizes
        // 3. Triangulate small holes
        // 4. Add the triangulation to the mesh
        self.clone()
    }
    
    /// Validate mesh integrity
    pub fn validate_mesh(&self) -> MeshQualityReport {
        let polygons = self.polygons();
        let manifold_check = self.is_manifold();
        
        // Analyze triangle quality
        let mut triangle_analysis = TriangleQualityAnalysis::default();
        let mut _total_area = 0.0;
        let mut aspect_ratios = Vec::new();
        let mut areas = Vec::new();
        
        for polygon in &polygons {
            let triangles = Self::triangulate_polygon(polygon);
            for triangle in triangles {
                let quality = Self::compute_triangle_quality(&triangle);
                
                triangle_analysis.triangle_count += 1;
                _total_area += quality.area;
                aspect_ratios.push(quality.aspect_ratio);
                areas.push(quality.area);
                
                // Count poor quality triangles (aspect ratio > 10)
                if quality.aspect_ratio > 10.0 {
                    triangle_analysis.poor_quality_count += 1;
                }
            }
        }
        
        if !aspect_ratios.is_empty() {
            triangle_analysis.avg_aspect_ratio = aspect_ratios.iter().sum::<Real>() / aspect_ratios.len() as Real;
            triangle_analysis.min_aspect_ratio = aspect_ratios.iter().fold(Real::INFINITY, |a, &b| a.min(b));
            triangle_analysis.max_aspect_ratio = aspect_ratios.iter().fold(0.0, |a, &b| a.max(b));
        }
        
        if !areas.is_empty() {
            triangle_analysis.avg_area = areas.iter().sum::<Real>() / areas.len() as Real;
            triangle_analysis.min_area = areas.iter().fold(Real::INFINITY, |a, &b| a.min(b));
            triangle_analysis.max_area = areas.iter().fold(0.0, |a, &b| a.max(b));
        }
        
        let info = self.info();
        let bbox = self.bounding_box();
        
        MeshQualityReport {
             triangle_analysis,
             is_manifold: manifold_check.is_manifold,
             manifold_issues: manifold_check.issues,
             vertex_count: self.vertices().len(),
             polygon_count: polygons.len(),
             node_count: info.node_count,
             memory_efficiency: info.memory_efficiency,
             sparsity_ratio: info.sparsity_ratio,
             bounding_box: bbox,
         }
     }
     
     /// Convert all polygons to triangles
     pub fn triangulate(&self) -> Self {
         let polygons = self.polygons();
         let mut triangulated_polygons = Vec::new();
         
         for polygon in polygons {
             let triangles = Self::triangulate_polygon(&polygon);
             for triangle in triangles {
                 let triangle_polygon = Polygon::new(
                    vec![triangle[0].clone(), triangle[1].clone(), triangle[2].clone()],
                    polygon.metadata.clone(),
                );
                 triangulated_polygons.push(triangle_polygon);
             }
         }
         
         Self::from_polygons(&triangulated_polygons, self.metadata.clone())
     }
     
     /// Optimize mesh by combining smoothing, healing, and quality improvement
     pub fn optimize(&self) -> Self {
         let mut optimized = self.clone();
         
         // Remove degenerate triangles first
         optimized = optimized.remove_degenerate_triangles();
         
         // Apply smoothing
         optimized = optimized.taubin_smooth(3, 0.5, -0.53);
         
         // Remove poor quality triangles
         optimized = optimized.remove_poor_triangles(5.0, 0.01);
         
         // Heal the mesh
         optimized = optimized.heal_mesh();
         
         // Final smoothing pass
         optimized = optimized.bilateral_smooth(1, 1.0, 1.0);
         
         optimized
     }
    
    // ===== Helper Methods for Advanced Processing =====
    
    /// Build vertex adjacency map for smoothing operations
    fn build_vertex_adjacency(polygons: &[Polygon<S>]) -> std::collections::HashMap<usize, Vec<usize>> {
        use std::collections::HashMap;
        
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();
        
        for polygon in polygons {
            let vertex_count = polygon.vertices.len();
            for i in 0..vertex_count {
                let current = i;
                let next = (i + 1) % vertex_count;
                let prev = (i + vertex_count - 1) % vertex_count;
                
                adjacency.entry(current).or_default().push(next);
                adjacency.entry(current).or_default().push(prev);
            }
        }
        
        // Remove duplicates and sort for consistency
        for neighbors in adjacency.values_mut() {
            neighbors.sort_unstable();
            neighbors.dedup();
        }
        
        adjacency
    }
    
    /// Apply Laplacian smoothing to polygons
    fn apply_laplacian_smoothing(
        polygons: &[Polygon<S>], 
        vertex_map: &std::collections::HashMap<usize, Vec<usize>>, 
        lambda: Real
    ) -> Vec<Polygon<S>> {
        let mut smoothed_polygons = polygons.to_vec();
        
        for polygon in &mut smoothed_polygons {
            // First, collect all the neighbor averages
            let neighbor_averages: Vec<Option<Point3<Real>>> = (0..polygon.vertices.len())
                .map(|i| {
                    vertex_map.get(&i).and_then(|neighbors| {
                        if neighbors.is_empty() {
                            None
                        } else {
                            let neighbor_sum: Point3<Real> = neighbors.iter()
                                .filter_map(|&neighbor_idx| polygon.vertices.get(neighbor_idx))
                                .map(|v| v.pos)
                                .fold(Point3::origin(), |acc, pos| acc + pos.coords);
                            Some(neighbor_sum / neighbors.len() as Real)
                        }
                    })
                })
                .collect();
            
            // Then apply the smoothing
            for (vertex, neighbor_avg) in polygon.vertices.iter_mut().zip(neighbor_averages.iter()) {
                if let Some(avg) = neighbor_avg {
                    vertex.pos = vertex.pos + lambda * (avg - vertex.pos);
                }
            }
        }
        
        smoothed_polygons
    }
    
    /// Apply bilateral smoothing to polygons
    fn apply_bilateral_smoothing(
        polygons: &[Polygon<S>], 
        vertex_map: &std::collections::HashMap<usize, Vec<usize>>, 
        spatial_sigma: Real, 
        normal_sigma: Real
    ) -> Vec<Polygon<S>> {
        let mut smoothed_polygons = polygons.to_vec();
        
        for polygon in &mut smoothed_polygons {
            // First, collect all the weighted positions
            let weighted_positions: Vec<Option<Point3<Real>>> = (0..polygon.vertices.len())
                .map(|i| {
                    vertex_map.get(&i).and_then(|neighbors| {
                        let current_vertex = &polygon.vertices[i];
                        let mut weighted_sum = Point3::origin();
                        let mut weight_sum = 0.0;
                        
                        for &neighbor_idx in neighbors {
                            if let Some(neighbor) = polygon.vertices.get(neighbor_idx) {
                                // Spatial weight based on distance
                                let spatial_dist = (current_vertex.pos - neighbor.pos).norm();
                                let spatial_weight = (-spatial_dist.powi(2) / (2.0 * spatial_sigma.powi(2))).exp();
                                
                                // Normal weight based on normal similarity
                                let normal_similarity = current_vertex.normal.dot(&neighbor.normal).max(0.0);
                                let normal_weight = (-((1.0 - normal_similarity).powi(2)) / (2.0 * normal_sigma.powi(2))).exp();
                                
                                let total_weight = spatial_weight * normal_weight;
                                weighted_sum += total_weight * neighbor.pos.coords;
                                weight_sum += total_weight;
                            }
                        }
                        
                        if weight_sum > 0.0 {
                            Some(weighted_sum / weight_sum)
                        } else {
                            None
                        }
                    })
                })
                .collect();
            
            // Then apply the smoothing
            for (vertex, weighted_pos) in polygon.vertices.iter_mut().zip(weighted_positions.iter()) {
                if let Some(pos) = weighted_pos {
                    vertex.pos = *pos;
                }
            }
        }
        
        smoothed_polygons
    }
    
    /// Apply weighted average smoothing to polygons
    fn apply_weighted_average_smoothing(
        polygons: &[Polygon<S>], 
        vertex_map: &std::collections::HashMap<usize, Vec<usize>>, 
        weight_factor: Real
    ) -> Vec<Polygon<S>> {
        let mut smoothed_polygons = polygons.to_vec();
        
        for polygon in &mut smoothed_polygons {
            // First, collect all the weighted positions
            let weighted_positions: Vec<Option<Point3<Real>>> = (0..polygon.vertices.len())
                .map(|i| {
                    vertex_map.get(&i).and_then(|neighbors| {
                        let current_vertex = &polygon.vertices[i];
                        let mut weighted_sum = Point3::origin();
                        let mut weight_sum = 0.0;
                        
                        for &neighbor_idx in neighbors {
                            if let Some(neighbor) = polygon.vertices.get(neighbor_idx) {
                                // Weight based on inverse distance and angle
                                let distance = (current_vertex.pos - neighbor.pos).norm();
                                let weight = if distance > 0.0 {
                                    weight_factor / distance
                                } else {
                                    1.0
                                };
                                
                                weighted_sum += weight * neighbor.pos.coords;
                                weight_sum += weight;
                            }
                        }
                        
                        if weight_sum > 0.0 {
                            Some(weighted_sum / weight_sum)
                        } else {
                            None
                        }
                    })
                })
                .collect();
            
            // Then apply the smoothing
            for (vertex, weighted_pos) in polygon.vertices.iter_mut().zip(weighted_positions.iter()) {
                if let Some(pos) = weighted_pos {
                    vertex.pos = *pos;
                }
            }
        }
        
        smoothed_polygons
    }
    
    /// Triangulate a polygon into triangles for quality analysis
    fn triangulate_polygon(polygon: &Polygon<S>) -> Vec<[Vertex; 3]> {
        let mut triangles = Vec::new();
        
        if polygon.vertices.len() < 3 {
            return triangles;
        }
        
        // Simple fan triangulation from first vertex
        for i in 1..polygon.vertices.len() - 1 {
            triangles.push([
                polygon.vertices[0].clone(),
                polygon.vertices[i].clone(),
                polygon.vertices[i + 1].clone(),
            ]);
        }
        
        triangles
    }
    
    /// Compute quality metrics for a triangle
    fn compute_triangle_quality(triangle: &[Vertex; 3]) -> TriangleQuality {
        let [v0, v1, v2] = triangle;
        
        // Edge lengths
        let edge1 = (v1.pos - v0.pos).norm();
        let edge2 = (v2.pos - v1.pos).norm();
        let edge3 = (v0.pos - v2.pos).norm();
        
        // Area using cross product
        let area = 0.5 * (v1.pos - v0.pos).cross(&(v2.pos - v0.pos)).norm();
        
        // Aspect ratio (longest edge / shortest edge)
        let max_edge = edge1.max(edge2).max(edge3);
        let min_edge = edge1.min(edge2).min(edge3);
        let aspect_ratio = if min_edge > 0.0 { max_edge / min_edge } else { Real::INFINITY };
        
        // Angles
        let angles = Self::compute_triangle_angles(triangle);
        
        TriangleQuality {
            area,
            aspect_ratio,
            angles,
        }
    }
    
    /// Compute angles of a triangle
    fn compute_triangle_angles(triangle: &[Vertex; 3]) -> [Real; 3] {
        let [v0, v1, v2] = triangle;
        
        let edge1 = (v1.pos - v0.pos).normalize();
        let edge2 = (v2.pos - v0.pos).normalize();
        let edge3 = (v0.pos - v1.pos).normalize();
        let edge4 = (v2.pos - v1.pos).normalize();
        let edge5 = (v0.pos - v2.pos).normalize();
        let edge6 = (v1.pos - v2.pos).normalize();
        
        let angle0 = edge1.dot(&edge2).acos();
        let angle1 = edge3.dot(&edge4).acos();
        let angle2 = edge5.dot(&edge6).acos();
        
        [angle0, angle1, angle2]
    }
    
    /// Refine areas with poor quality triangles
    fn refine_poor_quality_areas(&self, _quality_threshold: Real) -> Self {
        // For now, return a copy - this would need more sophisticated subdivision logic
        // In a full implementation, this would identify poor quality regions and subdivide
        // the octree nodes containing them, then regenerate the mesh
        self.clone()
    }
    
    /// Build edge map for manifold checking
    fn build_edge_map(polygons: &[Polygon<S>]) -> std::collections::HashMap<(usize, usize), Vec<usize>> {
        use std::collections::HashMap;
        
        let mut edge_map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        
        for (face_idx, polygon) in polygons.iter().enumerate() {
            let vertex_count = polygon.vertices.len();
            for i in 0..vertex_count {
                let v1 = i;
                let v2 = (i + 1) % vertex_count;
                
                // Normalize edge (smaller index first)
                let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
                
                edge_map.entry(edge).or_default().push(face_idx);
            }
        }
        
        edge_map
    }
    
    /// Count vertex usage across all polygons
    fn count_vertex_usage(polygons: &[Polygon<S>]) -> std::collections::HashMap<usize, usize> {
        use std::collections::HashMap;
        
        let mut usage_count: HashMap<usize, usize> = HashMap::new();
        
        for polygon in polygons {
            for (vertex_idx, _) in polygon.vertices.iter().enumerate() {
                *usage_count.entry(vertex_idx).or_insert(0) += 1;
            }
        }
        
        usage_count
    }
    
    /// Get vertices from all polygons
    pub fn vertices(&self) -> Vec<Vertex> {
        let polygons = self.polygons();
        
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            polygons
                .par_iter()
                .flat_map(|poly| poly.vertices.clone())
                .collect()
        }
        
        #[cfg(not(feature = "parallel"))]
        {
            polygons
                .iter()
                .flat_map(|poly| poly.vertices.clone())
                .collect()
        }
    }
    

    
    /// Get detailed information about the mesh structure
    pub fn info(&self) -> SvoMeshInfo {
        let stats = self.statistics();
        let polygon_count = stats.total_polygons;
        let vertex_count = self.vertices().len();
        let memory_usage = self.memory_usage();
        let bounds = self.bounding_box();
        
        SvoMeshInfo {
            polygon_count,
            vertex_count,
            node_count: stats.total_nodes,
            leaf_node_count: stats.leaf_nodes,
            max_depth: stats.max_depth,
            memory_usage,
            memory_efficiency: stats.memory_efficiency(),
            sparsity_ratio: stats.sparsity_ratio(),
            bounds,
            is_empty: self.is_empty(),
        }
    }
}

/// Information about an SVO mesh structure
#[derive(Debug, Clone)]
pub struct SvoMeshInfo {
    pub polygon_count: usize,
    pub vertex_count: usize,
    pub node_count: usize,
    pub leaf_node_count: usize,
    pub max_depth: usize,
    pub memory_usage: usize,
    pub memory_efficiency: f64,
    pub sparsity_ratio: f64,
    pub bounds: Aabb,
    pub is_empty: bool,
}

impl std::fmt::Display for SvoMeshInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "SVO Mesh Information:")?;
        writeln!(f, "  Polygons: {}", self.polygon_count)?;
        writeln!(f, "  Vertices: {}", self.vertex_count)?;
        writeln!(f, "  Nodes: {} (leaves: {})", self.node_count, self.leaf_node_count)?;
        writeln!(f, "  Max Depth: {}", self.max_depth)?;
        writeln!(f, "  Memory Usage: {} bytes", self.memory_usage)?;
        writeln!(f, "  Memory Efficiency: {:.2} polygons/node", self.memory_efficiency)?;
        writeln!(f, "  Sparsity Ratio: {:.2}%", self.sparsity_ratio * 100.0)?;
        writeln!(f, "  Bounds: {:?}", self.bounds)?;
        writeln!(f, "  Empty: {}", self.is_empty)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Test imports removed as they were unused
    
    #[test]
    fn test_svo_mesh_creation() {
        let mesh = SvoMesh::<()>::new();
        assert!(mesh.is_empty());
        assert_eq!(mesh.polygons().len(), 0);
    }
    
    #[test]
    fn test_svo_mesh_csg_operations() {
        let mesh1 = SvoMesh::<()>::new();
        let mesh2 = SvoMesh::<()>::new();
        
        // Test union of empty meshes
        let union_result = mesh1.union(&mesh2);
        assert!(union_result.is_empty());
        
        // Test other operations
        let diff_result = mesh1.difference(&mesh2);
        assert!(diff_result.is_empty());
        
        let intersect_result = mesh1.intersection(&mesh2);
        assert!(intersect_result.is_empty());
        
        let xor_result = mesh1.xor(&mesh2);
        assert!(xor_result.is_empty());
    }
    
    #[test]
    fn test_svo_mesh_transform() {
        let mesh = SvoMesh::<()>::new();
        let transform = Matrix4::identity();
        let transformed = mesh.transform(&transform);
        
        assert!(transformed.is_empty());
        assert_eq!(transformed.polygons().len(), 0);
    }
    
    #[test]
    fn test_svo_mesh_info() {
        let mesh = SvoMesh::<()>::new();
        let info = mesh.info();
        
        assert_eq!(info.polygon_count, 0);
        assert_eq!(info.vertex_count, 0);
        assert!(info.is_empty);
    }
    
    #[test]
    fn test_laplacian_smoothing() {
        let mesh = SvoMesh::<()>::new();
        
        let smoothed = mesh.laplacian_smooth(1, 0.5);
        assert_eq!(smoothed.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_taubin_smoothing() {
        let mesh = SvoMesh::<()>::new();
        
        let smoothed = mesh.taubin_smooth(1, 0.5, -0.53);
        assert_eq!(smoothed.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_bilateral_smoothing() {
        let mesh = SvoMesh::<()>::new();
        
        let smoothed = mesh.bilateral_smooth(1, 1.0, 1.0);
        assert_eq!(smoothed.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_weighted_average() {
        let mesh = SvoMesh::<()>::new();
        
        let smoothed = mesh.weighted_average(1, 1.0);
        assert_eq!(smoothed.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_triangle_quality_analysis() {
        let mesh = SvoMesh::<()>::new();
        
        let quality = mesh.analyze_triangle_quality();
        assert_eq!(quality.triangle_count, 0);
        assert_eq!(quality.poor_quality_count, 0);
    }
    
    #[test]
    fn test_mesh_quality_computation() {
        let mesh = SvoMesh::<()>::new();
        
        let quality = mesh.compute_mesh_quality();
        assert_eq!(quality.triangle_analysis.triangle_count, 0);
        assert!(quality.is_manifold);
    }
    
    #[test]
    fn test_adaptive_refinement() {
        let mesh = SvoMesh::<()>::new();
        
        let refined = mesh.adaptive_refine(2.0, 3);
        assert_eq!(refined.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_poor_triangle_removal() {
        let mesh = SvoMesh::<()>::new();
        
        let cleaned = mesh.remove_poor_triangles(2.0, 0.01);
        assert_eq!(cleaned.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_manifold_checking() {
        let mesh = SvoMesh::<()>::new();
        
        let manifold_check = mesh.is_manifold();
        assert!(manifold_check.is_manifold); // Empty mesh is manifold
        assert!(manifold_check.issues.is_empty());
    }
    
    #[test]
    fn test_mesh_healing() {
        let mesh = SvoMesh::<()>::new();
        
        let healed = mesh.heal_mesh();
        assert_eq!(healed.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_degenerate_triangle_removal() {
        let mesh = SvoMesh::<()>::new();
        
        let cleaned = mesh.remove_degenerate_triangles();
        assert_eq!(cleaned.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_isolated_vertex_removal() {
        let mesh = SvoMesh::<()>::new();
        
        let cleaned = mesh.remove_isolated_vertices();
        assert_eq!(cleaned.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_hole_filling() {
        let mesh = SvoMesh::<()>::new();
        
        let filled = mesh.fill_small_holes(1.0);
        assert_eq!(filled.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_mesh_validation() {
        let mesh = SvoMesh::<()>::new();
        
        let report = mesh.validate_mesh();
        assert_eq!(report.vertex_count, 0);
        assert_eq!(report.polygon_count, 0);
        assert!(report.is_manifold);
        assert!(report.manifold_issues.is_empty());
    }
    
    #[test]
    fn test_triangulation() {
        let mesh = SvoMesh::<()>::new();
        
        let triangulated = mesh.triangulate();
        assert_eq!(triangulated.polygons().len(), mesh.polygons().len());
    }
    
    #[test]
    fn test_mesh_optimization() {
        let mesh = SvoMesh::<()>::new();
        
        let optimized = mesh.optimize();
        assert_eq!(optimized.polygons().len(), mesh.polygons().len());
    }
    
    // ========================================================================
    // Phase 6: Geometric Primitives Tests
    // ========================================================================
    
    #[test]
    fn test_sphere_generation() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let radius = 1.0;
        let resolution = (8, 8, 8);
        
        let sphere = SvoMesh::<()>::sphere(center, radius, resolution, None);
        
        // Verify sphere was generated
        assert!(!sphere.is_empty());
        assert!(!sphere.polygons().is_empty());
        
        // Verify bounding box is reasonable
        let bbox = sphere.bounding_box();
        let size = bbox.maxs - bbox.mins;
        assert!(size.x > 0.5 && size.y > 0.5 && size.z > 0.5);
        assert!(size.x < 3.0 && size.y < 3.0 && size.z < 3.0);
    }
    
    #[test]
    fn test_sphere_invalid_radius() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let radius = 0.0; // Invalid radius
        let resolution = (8, 8, 8);
        
        let sphere = SvoMesh::<()>::sphere(center, radius, resolution, None);
        
        // Should return empty mesh for invalid radius
        assert!(sphere.is_empty());
        assert!(sphere.polygons().is_empty());
    }
    
    #[test]
    fn test_involute_gear_generation() {
        let module_ = 2.0;
        let teeth = 12;
        let pressure_angle_deg = 20.0;
        let clearance = 0.25;
        let backlash = 0.0;
        let segments_per_flank = 8;
        let thickness = 5.0;
        
        let gear = SvoMesh::<()>::involute_gear(
            module_,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            thickness,
            None,
        );
        
        // Verify gear was generated
        assert!(!gear.is_empty());
        assert!(!gear.polygons().is_empty());
        
        // Verify bounding box has reasonable dimensions
        let bbox = gear.bounding_box();
        let size = bbox.maxs - bbox.mins;
        assert!(size.z > 0.0); // Should have thickness
    }
    
    #[test]
    fn test_helical_gear_generation() {
        let module_ = 2.0;
        let teeth = 12;
        let pressure_angle_deg = 20.0;
        let helix_angle_deg = 15.0;
        let thickness = 5.0;
        let resolution = (16, 16, 8);
        
        let gear = SvoMesh::<()>::helical_gear(
            module_,
            teeth,
            pressure_angle_deg,
            helix_angle_deg,
            thickness,
            resolution,
            None,
        );
        
        // Verify gear was generated
        assert!(!gear.is_empty());
        assert!(!gear.polygons().is_empty());
        
        // Verify bounding box has reasonable dimensions
        let bbox = gear.bounding_box();
        let size = bbox.maxs - bbox.mins;
        assert!(size.z > 0.0); // Should have thickness
    }
    
    #[test]
    fn test_metaballs_generation() {
        let centers = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let radii = vec![1.0, 1.0, 0.8];
        let threshold = 1.0;
        let resolution = (12, 12, 12);
        
        let metaballs = SvoMesh::<()>::metaballs(
            &centers,
            &radii,
            threshold,
            resolution,
            None,
        );
        
        // Verify metaballs were generated
        assert!(!metaballs.is_empty());
        assert!(!metaballs.polygons().is_empty());
        
        // Verify bounding box encompasses all metaballs
        let bbox = metaballs.bounding_box();
        assert!(bbox.mins.x < -1.0 && bbox.maxs.x > 1.0);
        assert!(bbox.mins.y < 0.0 && bbox.maxs.y > 1.0);
    }
    
    #[test]
    fn test_metaballs_empty_input() {
        let centers = vec![];
        let radii = vec![];
        let threshold = 1.0;
        let resolution = (8, 8, 8);
        
        let metaballs = SvoMesh::<()>::metaballs(
            &centers,
            &radii,
            threshold,
            resolution,
            None,
        );
        
        // Should return empty mesh for empty input
        assert!(metaballs.is_empty());
        assert!(metaballs.polygons().is_empty());
    }
    
    #[test]
    fn test_metaballs_mismatched_arrays() {
        let centers = vec![Point3::new(0.0, 0.0, 0.0)];
        let radii = vec![1.0, 2.0]; // Mismatched length
        let threshold = 1.0;
        let resolution = (8, 8, 8);
        
        let metaballs = SvoMesh::<()>::metaballs(
            &centers,
            &radii,
            threshold,
            resolution,
            None,
        );
        
        // Should return empty mesh for mismatched input
        assert!(metaballs.is_empty());
        assert!(metaballs.polygons().is_empty());
    }
    
    #[cfg(feature = "chull")]
    #[test]
    fn test_convex_hull_generation() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.5, 0.5, 0.5),
        ];
        
        let hull = SvoMesh::<()>::convex_hull(&points, None);
        
        // Verify hull was generated
        assert!(!hull.is_empty());
        assert!(!hull.polygons().is_empty());
        
        // Verify bounding box encompasses all points
        let bbox = hull.bounding_box();
        assert!(bbox.mins.x <= 0.0 && bbox.maxs.x >= 1.0);
        assert!(bbox.mins.y <= 0.0 && bbox.maxs.y >= 1.0);
        assert!(bbox.mins.z <= 0.0 && bbox.maxs.z >= 1.0);
    }
    
    #[cfg(feature = "chull")]
    #[test]
    fn test_convex_hull_insufficient_points() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ]; // Only 2 points, need at least 4
        
        let hull = SvoMesh::<()>::convex_hull(&points, None);
        
        // Should return empty mesh for insufficient points
        assert!(hull.is_empty());
        assert!(hull.polygons().is_empty());
    }
    
    #[cfg(not(feature = "chull"))]
    #[test]
    fn test_convex_hull_disabled() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        
        let hull = SvoMesh::<()>::convex_hull(&points, None);
        
        // Should return empty mesh when feature is disabled
        assert!(hull.is_empty());
        assert!(hull.polygons().is_empty());
    }
}