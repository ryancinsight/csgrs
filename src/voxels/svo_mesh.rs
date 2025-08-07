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
    
    /// Triangulate all polygons in the mesh
    pub fn triangulate(&self) -> Self {
        let triangulated_polygons: Vec<_> = self
            .polygons()
            .iter()
            .flat_map(|poly| {
                poly.triangulate()
                    .into_iter()
                    .map(|triangle| {
                        Polygon::new(triangle.to_vec(), poly.metadata.clone())
                    })
            })
            .collect();
        
        Self::from_polygons(&triangulated_polygons, self.metadata.clone())
    }
    
    /// Optimize the SVO structure by rebuilding with current polygons
    pub fn optimize(&self) -> Self {
        if self.is_empty() {
            return self.clone();
        }
        
        let polygons = self.polygons();
        Self::from_polygons(&polygons, self.metadata.clone())
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
}