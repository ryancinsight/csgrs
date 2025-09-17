//! # Dense Voxel Grid System
//!
//! This module provides dense voxel grid representations for volume-based processing.
//! Dense voxel grids use a regular 3D grid structure optimized for uniform access patterns.

use crate::float_types::parry3d::query::PointQuery;
use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use crate::mesh::{Mesh, polygon::Polygon};
use crate::traits::CSG;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;
use std::sync::OnceLock;

/// Represents a 3D grid of voxels for volume-based processing
#[derive(Debug, Clone)]
pub struct VoxelGrid<S: Clone + Debug + Send + Sync> {
    /// The origin point of the voxel grid (minimum corner)
    pub origin: Point3<Real>,
    /// Size of each voxel in world units
    pub voxel_size: Real,
    /// Number of voxels in each dimension (nx, ny, nz)
    pub dimensions: (usize, usize, usize),
    /// Voxel data stored as a flat vector (x + y*nx + z*nx*ny indexing)
    pub voxels: Vec<VoxelData<S>>,
    /// Metadata associated with the voxel grid
    pub metadata: Option<S>,
}

/// Individual voxel data containing occupancy and material information
#[derive(Debug, Clone)]
pub enum VoxelData<S: Clone + Debug + Send + Sync> {
    /// Empty voxel (no geometry)
    Empty,
    /// Occupied voxel with associated metadata
    Occupied { metadata: Option<S> },
}

/// Voxelization modes for mesh-to-voxel conversion
#[derive(Debug, Clone, Copy)]
pub enum VoxelizationMode {
    /// Surface voxelization - only surface voxels are marked
    Surface,
    /// Solid voxelization - all voxels inside the mesh volume are marked
    Solid,
}

/// Voxelization configuration parameters
#[derive(Debug, Clone)]
pub struct VoxelizationConfig<S: Clone + Debug + Send + Sync> {
    /// Voxelization mode (surface or solid)
    pub mode: VoxelizationMode,
    /// Default metadata for occupied voxels
    pub default_metadata: Option<S>,
    /// Whether to use parallel processing for voxelization
    pub parallel: bool,
}

impl<S: Clone + Debug + Send + Sync> Default for VoxelizationConfig<S> {
    fn default() -> Self {
        Self {
            mode: VoxelizationMode::Solid,
            default_metadata: None,
            parallel: true,
        }
    }
}

impl<S: Clone + Debug + Send + Sync> VoxelGrid<S> {
    /// Create a new voxel grid with specified dimensions
    pub fn new(
        origin: Point3<Real>,
        voxel_size: Real,
        dimensions: (usize, usize, usize),
        metadata: Option<S>,
    ) -> Self {
        let total_voxels = dimensions.0 * dimensions.1 * dimensions.2;
        let voxels = vec![VoxelData::Empty; total_voxels];

        Self {
            origin,
            voxel_size,
            dimensions,
            voxels,
            metadata,
        }
    }

    /// Create a voxel grid that bounds a given mesh
    pub fn from_mesh_bounds(
        mesh: &Mesh<S>,
        voxel_size: Real,
        padding: Real,
        metadata: Option<S>,
    ) -> Self {
        let bbox = mesh.bounding_box();
        let padded_mins = bbox.mins - Vector3::new(padding, padding, padding);
        let padded_maxs = bbox.maxs + Vector3::new(padding, padding, padding);
        let size = padded_maxs - padded_mins;

        let nx = ((size.x / voxel_size).ceil() as usize).max(1);
        let ny = ((size.y / voxel_size).ceil() as usize).max(1);
        let nz = ((size.z / voxel_size).ceil() as usize).max(1);

        Self::new(padded_mins, voxel_size, (nx, ny, nz), metadata)
    }

    /// Get voxel data at grid coordinates
    pub fn get_voxel(&self, x: usize, y: usize, z: usize) -> Option<&VoxelData<S>> {
        if x >= self.dimensions.0 || y >= self.dimensions.1 || z >= self.dimensions.2 {
            return None;
        }

        let index = x + y * self.dimensions.0 + z * self.dimensions.0 * self.dimensions.1;
        self.voxels.get(index)
    }

    /// Set voxel data at grid coordinates
    pub fn set_voxel(&mut self, x: usize, y: usize, z: usize, data: VoxelData<S>) -> bool {
        if x >= self.dimensions.0 || y >= self.dimensions.1 || z >= self.dimensions.2 {
            return false;
        }

        let index = x + y * self.dimensions.0 + z * self.dimensions.0 * self.dimensions.1;
        if let Some(voxel) = self.voxels.get_mut(index) {
            *voxel = data;
            true
        } else {
            false
        }
    }

    /// Convert grid coordinates to world coordinates
    pub fn voxel_to_world(&self, x: usize, y: usize, z: usize) -> Point3<Real> {
        Point3::new(
            self.origin.x + (x as Real + 0.5) * self.voxel_size,
            self.origin.y + (y as Real + 0.5) * self.voxel_size,
            self.origin.z + (z as Real + 0.5) * self.voxel_size,
        )
    }

    /// Convert world coordinates to grid coordinates
    pub fn world_to_voxel(&self, point: &Point3<Real>) -> (usize, usize, usize) {
        let x = ((point.x - self.origin.x) / self.voxel_size).floor() as usize;
        let y = ((point.y - self.origin.y) / self.voxel_size).floor() as usize;
        let z = ((point.z - self.origin.z) / self.voxel_size).floor() as usize;

        (
            x.min(self.dimensions.0.saturating_sub(1)),
            y.min(self.dimensions.1.saturating_sub(1)),
            z.min(self.dimensions.2.saturating_sub(1)),
        )
    }

    /// Voxelize a mesh into the grid
    pub fn voxelize_mesh(&mut self, mesh: &Mesh<S>, config: &VoxelizationConfig<S>) -> usize {
        match config.mode {
            VoxelizationMode::Surface => self.voxelize_surface(mesh, config),
            VoxelizationMode::Solid => self.voxelize_solid(mesh, config),
        }
    }

    /// Surface voxelization - mark only surface voxels
    fn voxelize_surface(&mut self, mesh: &Mesh<S>, config: &VoxelizationConfig<S>) -> usize {
        let mut occupied_count = 0;

        // For each triangle in the mesh
        for polygon in &mesh.polygons {
            if polygon.vertices.len() < 3 {
                continue;
            }

            // Process triangles in the polygon
            for i in 0..polygon.vertices.len() - 2 {
                let triangle = [
                    polygon.vertices[0].pos,
                    polygon.vertices[i + 1].pos,
                    polygon.vertices[i + 2].pos,
                ];

                // Find voxels that intersect this triangle
                occupied_count += self.voxelize_triangle(&triangle, config);
            }
        }

        occupied_count
    }

    /// Solid voxelization - mark all voxels inside the mesh volume
    fn voxelize_solid(&mut self, mesh: &Mesh<S>, config: &VoxelizationConfig<S>) -> usize {
        let mut occupied_count = 0;

        // Get mesh bounding box for initial filtering (optimization)
        let mesh_bbox = mesh.bounding_box();

        for x in 0..self.dimensions.0 {
            for y in 0..self.dimensions.1 {
                for z in 0..self.dimensions.2 {
                    let center = self.voxel_to_world(x, y, z);

                    // First check: voxel center must be inside mesh bounding box
                    if mesh_bbox.contains_point(&nalgebra::Isometry3::identity(), &center) {
                        // Second check: use ray casting to determine if point is actually inside mesh
                        if self.point_in_mesh(&center, mesh) {
                            let data = VoxelData::Occupied {
                                metadata: config.default_metadata.clone(),
                            };
                            self.set_voxel(x, y, z, data);
                            occupied_count += 1;
                        }
                    }
                }
            }
        }

        occupied_count
    }

    /// Voxelize a single triangle
    fn voxelize_triangle(
        &mut self,
        triangle: &[Point3<Real>; 3],
        config: &VoxelizationConfig<S>,
    ) -> usize {
        let mut occupied_count = 0;

        // Find bounding box of triangle in voxel coordinates
        let mut min_x = usize::MAX;
        let mut min_y = usize::MAX;
        let mut min_z = usize::MAX;
        let mut max_x = 0;
        let mut max_y = 0;
        let mut max_z = 0;

        for vertex in triangle {
            let (vx, vy, vz) = self.world_to_voxel(vertex);
            min_x = min_x.min(vx);
            min_y = min_y.min(vy);
            min_z = min_z.min(vz);
            max_x = max_x.max(vx);
            max_y = max_y.max(vy);
            max_z = max_z.max(vz);
        }

        // Clamp to grid bounds
        min_x = min_x.max(0);
        min_y = min_y.max(0);
        min_z = min_z.max(0);
        max_x = max_x.min(self.dimensions.0.saturating_sub(1));
        max_y = max_y.min(self.dimensions.1.saturating_sub(1));
        max_z = max_z.min(self.dimensions.2.saturating_sub(1));

        // Check voxels in the bounding box
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                for z in min_z..=max_z {
                    let voxel_center = self.voxel_to_world(x, y, z);
                    if self.point_in_triangle(&voxel_center, triangle) {
                        let data = VoxelData::Occupied {
                            metadata: config.default_metadata.clone(),
                        };
                        if self.set_voxel(x, y, z, data) {
                            occupied_count += 1;
                        }
                    }
                }
            }
        }

        occupied_count
    }

    /// Convert the voxel grid to a mesh
    pub fn to_mesh(&self, metadata: Option<S>) -> Mesh<S> {
        let mut polygons = Vec::new();

        for x in 0..self.dimensions.0 {
            for y in 0..self.dimensions.1 {
                for z in 0..self.dimensions.2 {
                    if let Some(VoxelData::Occupied { .. }) = self.get_voxel(x, y, z) {
                        let center = self.voxel_to_world(x, y, z);
                        let half_size = self.voxel_size * 0.5;
                        let cube_polygons =
                            self.create_voxel_cube(center, half_size, metadata.clone());
                        polygons.extend(cube_polygons);
                    }
                }
            }
        }

        Mesh {
            polygons,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Create a cube mesh for a single voxel
    fn create_voxel_cube(
        &self,
        center: Point3<Real>,
        half_size: Real,
        metadata: Option<S>,
    ) -> Vec<Polygon<S>> {
        // Cube vertices (8 corners)
        let vertices = [
            center + Vector3::new(-half_size, -half_size, -half_size), // 0: ---
            center + Vector3::new(half_size, -half_size, -half_size),  // 1: +--
            center + Vector3::new(half_size, half_size, -half_size),   // 2: ++-
            center + Vector3::new(-half_size, half_size, -half_size),  // 3: -+-
            center + Vector3::new(-half_size, -half_size, half_size),  // 4: --+
            center + Vector3::new(half_size, -half_size, half_size),   // 5: +-+
            center + Vector3::new(half_size, half_size, half_size),    // 6: +++
            center + Vector3::new(-half_size, half_size, half_size),   // 7: -++
        ];

        // Cube faces (6 faces, each with 4 vertices and a normal)
        // Vertex ordering gives correct outward normals using right-hand rule
        let faces = [
            // Bottom face (z-): normal pointing down
            ([0, 3, 2, 1], Vector3::new(0.0, 0.0, -1.0)),
            // Top face (z+): normal pointing up
            ([4, 5, 6, 7], Vector3::new(0.0, 0.0, 1.0)),
            // Front face (y-): normal pointing toward negative y
            ([0, 1, 5, 4], Vector3::new(0.0, -1.0, 0.0)),
            // Back face (y+): normal pointing toward positive y
            ([3, 7, 6, 2], Vector3::new(0.0, 1.0, 0.0)),
            // Left face (x-): normal pointing toward negative x
            ([0, 4, 7, 3], Vector3::new(-1.0, 0.0, 0.0)),
            // Right face (x+): normal pointing toward positive x
            ([1, 2, 6, 5], Vector3::new(1.0, 0.0, 0.0)),
        ];

        let mut polygons = Vec::new();

        for (indices, normal) in faces {
            let face_vertices =
                indices.map(|i| crate::mesh::vertex::Vertex::new(vertices[i], normal));

            if let Ok(polygon) = Polygon::try_new(face_vertices.to_vec(), metadata.clone()) {
                polygons.push(polygon);
            }
        }

        polygons
    }

    /// Get bounding box of the voxel grid
    pub fn bounding_box(&self) -> Aabb {
        let max_x = self.origin.x + (self.dimensions.0 as Real) * self.voxel_size;
        let max_y = self.origin.y + (self.dimensions.1 as Real) * self.voxel_size;
        let max_z = self.origin.z + (self.dimensions.2 as Real) * self.voxel_size;

        Aabb::new(self.origin, Point3::new(max_x, max_y, max_z))
    }

    /// Get number of occupied voxels
    pub fn occupied_voxels(&self) -> usize {
        self.voxels
            .iter()
            .filter(|v| matches!(v, VoxelData::Occupied { .. }))
            .count()
    }

    /// Get occupancy ratio (0.0 to 1.0)
    pub fn occupancy_ratio(&self) -> Real {
        let occupied = self.occupied_voxels() as Real;
        let total = self.voxels.len() as Real;
        if total > 0.0 { occupied / total } else { 0.0 }
    }

    /// Test if a point is inside a triangle (2D projection)
    fn point_in_triangle(&self, point: &Point3<Real>, triangle: &[Point3<Real>; 3]) -> bool {
        // Project to XY plane and use barycentric coordinates
        let p = point.coords.xy();
        let a = triangle[0].coords.xy();
        let b = triangle[1].coords.xy();
        let c = triangle[2].coords.xy();

        let area = 0.5 * ((b - a).perp(&(c - a)));
        if area.abs() < 1e-10 {
            return false; // Degenerate triangle
        }

        // Barycentric coordinates
        let bary_a = 0.5 * ((b - p).perp(&(c - p))) / area;
        let bary_b = 0.5 * ((c - p).perp(&(a - p))) / area;
        let bary_c = 1.0 - bary_a - bary_b;

        (0.0..=1.0).contains(&bary_a)
            && (0.0..=1.0).contains(&bary_b)
            && (0.0..=1.0).contains(&bary_c)
    }

    /// Test if a point is inside a mesh using ray casting algorithm
    fn point_in_mesh(&self, point: &Point3<Real>, mesh: &Mesh<S>) -> bool {
        // Use ray casting algorithm: cast ray along +X axis and count intersections
        let ray_origin = *point;
        let ray_direction = Vector3::new(1.0, 0.0, 0.0); // Cast ray along +X axis

        let mut intersection_count = 0;

        // Test intersection with each triangle in the mesh
        for polygon in &mesh.polygons {
            if polygon.vertices.len() < 3 {
                continue;
            }

            // Process triangles in the polygon (handle both triangles and quads)
            for i in 0..polygon.vertices.len() - 2 {
                let triangle = [
                    polygon.vertices[0].pos,
                    polygon.vertices[i + 1].pos,
                    polygon.vertices[i + 2].pos,
                ];

                if self.ray_triangle_intersect(&ray_origin, &ray_direction, &triangle) {
                    intersection_count += 1;
                }
            }
        }

        // Odd number of intersections means point is inside
        intersection_count % 2 == 1
    }

    /// Test if a ray intersects a triangle using Möller-Trumbore algorithm
    fn ray_triangle_intersect(
        &self,
        ray_origin: &Point3<Real>,
        ray_direction: &Vector3<Real>,
        triangle: &[Point3<Real>; 3],
    ) -> bool {
        let epsilon = 1e-8; // Numerical tolerance

        let v0 = triangle[0];
        let v1 = triangle[1];
        let v2 = triangle[2];

        // Compute triangle edges
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;

        // Compute determinant
        let h = ray_direction.cross(&edge2);
        let a = edge1.dot(&h);

        // Check if ray is parallel to triangle
        if a.abs() < epsilon {
            return false;
        }

        let f = 1.0 / a;
        let s = ray_origin - v0;
        let u = f * s.dot(&h);

        // Check if intersection point is outside triangle bounds
        if !(0.0..=1.0).contains(&u) {
            return false;
        }

        let q = s.cross(&edge1);
        let v = f * ray_direction.dot(&q);

        // Check if intersection point is outside triangle bounds
        if v < 0.0 || u + v > 1.0 {
            return false;
        }

        // Compute intersection distance
        let t = f * edge2.dot(&q);

        // Check if intersection is in front of ray origin
        t > epsilon
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_voxel_grid_creation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let grid = VoxelGrid::<()>::new(origin, 1.0, (10, 10, 10), None);

        assert_eq!(grid.dimensions, (10, 10, 10));
        assert_eq!(grid.voxels.len(), 1000);
        assert_eq!(grid.origin, origin);
        assert_eq!(grid.voxel_size, 1.0);
    }

    #[test]
    fn test_voxel_indexing() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut grid = VoxelGrid::<()>::new(origin, 1.0, (5, 5, 5), None);

        // Test setting and getting voxels
        assert!(grid.set_voxel(2, 3, 1, VoxelData::Occupied { metadata: Some(()) }));
        if let Some(voxel) = grid.get_voxel(2, 3, 1) {
            assert!(matches!(voxel, VoxelData::Occupied { .. }));
        } else {
            panic!("Voxel should exist");
        }
    }

    #[test]
    fn test_voxel_coordinate_conversion() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let grid = VoxelGrid::<()>::new(origin, 2.0, (10, 10, 10), None);

        // Test world to voxel conversion
        let world_point = Point3::new(3.0, 5.0, 7.0);
        let (vx, vy, vz) = grid.world_to_voxel(&world_point);
        assert_eq!((vx, vy, vz), (1, 2, 3)); // floor(3/2)=1, floor(5/2)=2, floor(7/2)=3

        // Test voxel to world conversion
        let voxel_center = grid.voxel_to_world(1, 2, 3);
        // voxel_to_world centers at origin + (x + 0.5) * voxel_size
        // For voxel (1,2,3) with voxel_size=2.0: centers at (1.5*2, 2.5*2, 3.5*2) = (3.0, 5.0, 7.0)
        assert!((voxel_center.x - 3.0).abs() < 1e-10);
        assert!((voxel_center.y - 5.0).abs() < 1e-10);
        assert!((voxel_center.z - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_voxel_grid_from_mesh_bounds() {
        let mesh = Mesh::<()>::cube(2.0, None).expect("Failed to create cube mesh");

        let grid = VoxelGrid::<()>::from_mesh_bounds(&mesh, 0.5, 0.1, None);

        // Cube of size 2 has bounding box from (0,0,0) to (2,2,2)
        // With padding 0.1, from (-0.1,-0.1,-0.1) to (2.1,2.1,2.1)
        // Size becomes 2.2 in each dimension
        // With voxel size 0.5, dimensions should be ceil(2.2/0.5) = ceil(4.4) = 5
        assert_eq!(grid.dimensions.0, 5);
        assert_eq!(grid.dimensions.1, 5);
        assert_eq!(grid.dimensions.2, 5);

        // Check that origin is correct (padded mins)
        assert!((grid.origin.x + 0.1).abs() < 1e-10);
        assert!((grid.origin.y + 0.1).abs() < 1e-10);
        assert!((grid.origin.z + 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_voxel_occupancy() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut grid = VoxelGrid::<()>::new(origin, 1.0, (3, 3, 3), None);

        // Set some voxels as occupied
        grid.set_voxel(0, 0, 0, VoxelData::Occupied { metadata: None });
        grid.set_voxel(1, 1, 1, VoxelData::Occupied { metadata: None });
        grid.set_voxel(2, 2, 2, VoxelData::Occupied { metadata: None });

        assert_eq!(grid.occupied_voxels(), 3);
        assert!((grid.occupancy_ratio() - 3.0 / 27.0).abs() < 1e-10);
    }

    #[test]
    fn test_voxel_bounding_box() {
        let origin = Point3::new(-1.0, -2.0, -3.0);
        let grid = VoxelGrid::<()>::new(origin, 2.0, (3, 4, 5), None);

        let bbox = grid.bounding_box();

        // Expected bounds: from (-1,-2,-3) to (-1+3*2, -2+4*2, -3+5*2) = (-1+6, -2+8, -3+10) = (5,6,7)
        assert!((bbox.mins.x - (-1.0)).abs() < 1e-10);
        assert!((bbox.mins.y - (-2.0)).abs() < 1e-10);
        assert!((bbox.mins.z - (-3.0)).abs() < 1e-10);
        assert!((bbox.maxs.x - 5.0).abs() < 1e-10);
        assert!((bbox.maxs.y - 6.0).abs() < 1e-10);
        assert!((bbox.maxs.z - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_voxel_grid_voxelize_cube_surface() {
        let mesh = Mesh::<()>::cube(2.0, None).expect("Failed to create cube mesh");
        let mut grid = VoxelGrid::<()>::from_mesh_bounds(&mesh, 0.5, 0.1, None);
        let config = VoxelizationConfig {
            mode: VoxelizationMode::Surface,
            default_metadata: None,
            parallel: false,
        };

        let occupied = grid.voxelize_mesh(&mesh, &config);

        // Should have some occupied voxels
        assert!(occupied > 0);
        // occupied is the count of triangle-voxel intersections (may include duplicates)
        // grid.occupied_voxels() is the count of unique occupied voxels
        // The occupied count should be >= the unique voxel count
        assert!(occupied >= grid.occupied_voxels());
    }

    #[test]
    fn test_voxel_grid_voxelize_cube_solid() {
        let mesh = Mesh::<()>::cube(2.0, None).expect("Failed to create cube mesh");
        let mut grid = VoxelGrid::<()>::from_mesh_bounds(&mesh, 0.5, 0.1, None);
        let config = VoxelizationConfig {
            mode: VoxelizationMode::Solid,
            default_metadata: None,
            parallel: false,
        };

        let occupied = grid.voxelize_mesh(&mesh, &config);

        // Mesh::cube(2.0) creates a cube from (0,0,0) to (2,2,2)
        // Grid bounds: [-0.1, -0.1, -0.1] to [2.1, 2.1, 2.1] with voxel size 0.5
        // Voxel centers: 0.15, 0.65, 1.15, 1.65, 2.15
        // With proper ray casting, not all voxels within bounding box are inside the cube
        // The exact count depends on which voxels have centers actually inside the cube mesh
        assert!(occupied > 0, "Should have some occupied voxels");
        assert_eq!(
            occupied,
            grid.occupied_voxels(),
            "Occupied count should match grid state"
        );

        // Verify that occupied voxels are reasonably distributed within the expected region
        // The core 3x3x3 region should be mostly occupied (voxels 0,1,2 in each dimension)
        let mut occupied_in_core = 0;
        for x in 0..3 {
            for y in 0..3 {
                for z in 0..3 {
                    if matches!(grid.get_voxel(x, y, z), Some(VoxelData::Occupied { .. })) {
                        occupied_in_core += 1;
                    }
                }
            }
        }
        assert!(
            occupied_in_core >= 15,
            "Core region should have at least 15 occupied voxels, got {}",
            occupied_in_core
        );

        // Verify reasonable occupancy ratio (should be less than bounding box approximation)
        let occupancy_ratio = grid.occupancy_ratio();
        assert!(
            occupancy_ratio > 0.3,
            "Occupancy ratio should be > 30%, got {:.2}%",
            occupancy_ratio * 100.0
        );
        assert!(
            occupancy_ratio < 0.8,
            "Occupancy ratio should be < 80%, got {:.2}%",
            occupancy_ratio * 100.0
        );
    }

    #[test]
    fn test_voxel_grid_to_mesh() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut grid = VoxelGrid::<()>::new(origin, 1.0, (2, 2, 2), None);

        // Set one voxel as occupied
        grid.set_voxel(0, 0, 0, VoxelData::Occupied { metadata: None });

        let mesh = grid.to_mesh(None);

        // Should have 6 faces for the cube
        assert_eq!(mesh.polygons.len(), 6);

        // Check bounding box
        let bbox = mesh.bounding_box();
        // Voxel at (0,0,0) with size 1 creates a cube from (0,0,0) to (1,1,1)
        // The voxel center is at (0.5,0.5,0.5) with half_size = 0.5
        assert!((bbox.mins.x - 0.0).abs() < 1e-10);
        assert!((bbox.mins.y - 0.0).abs() < 1e-10);
        assert!((bbox.mins.z - 0.0).abs() < 1e-10);
        assert!((bbox.maxs.x - 1.0).abs() < 1e-10);
        assert!((bbox.maxs.y - 1.0).abs() < 1e-10);
        assert!((bbox.maxs.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_voxel_config_default() {
        let config: VoxelizationConfig<()> = Default::default();

        assert!(matches!(config.mode, VoxelizationMode::Solid));
        assert!(config.default_metadata.is_none());
        assert!(config.parallel);
    }

    #[test]
    fn test_voxel_cube_normals_outward() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut grid = VoxelGrid::<()>::new(origin, 1.0, (1, 1, 1), None);
        grid.set_voxel(0, 0, 0, VoxelData::Occupied { metadata: None });

        let mesh = grid.to_mesh(None);

        // Should have 6 faces for a cube
        assert_eq!(mesh.polygons.len(), 6);

        // Check that all normals point outward (away from center)
        let center = Point3::new(0.5, 0.5, 0.5); // Center of the voxel

        for (i, polygon) in mesh.polygons.iter().enumerate() {
            let normal = polygon.plane.normal().normalize();

            // Debug: print normal info
            let plane_point = polygon.vertices[0].pos;
            println!("Face {}: Point: {:?}, Normal: {:?}", i, plane_point, normal);

            // The normal should point away from the center
            // For a point on the plane, dot(normal, point - center) should be positive
            let to_point = plane_point - center;
            let dot_product = normal.dot(&to_point);

            if dot_product <= 0.0 {
                println!(
                    "Face {} FAILED: Normal {:?} at point {:?}, dot product: {}",
                    i, normal, plane_point, dot_product
                );
            }
        }

        // Now assert they are all correct
        for (i, polygon) in mesh.polygons.iter().enumerate() {
            let normal = polygon.plane.normal().normalize();
            let plane_point = polygon.vertices[0].pos;
            let to_point = plane_point - center;
            let dot_product = normal.dot(&to_point);

            assert!(
                dot_product > 0.0,
                "Face {}: Normal {:?} at point {:?} should point outward, dot product: {}",
                i,
                normal,
                plane_point,
                dot_product
            );
        }
    }
}
