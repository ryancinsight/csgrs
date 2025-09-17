# CSGRS User Guide

This guide provides practical examples and usage patterns for the `csgrs` Constructive Solid Geometry library. It covers basic operations, advanced techniques, and best practices for geometric modeling.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Basic Shapes](#basic-shapes)
3. [Boolean Operations](#boolean-operations)
4. [Transformations](#transformations)
5. [Mesh Processing](#mesh-processing)
6. [File I/O](#file-io)
7. [Advanced Techniques](#advanced-techniques)
8. [Performance Tips](#performance-tips)
9. [Troubleshooting](#troubleshooting)

## Getting Started

### Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
csgrs = "0.20"
```

### Basic Usage Pattern

```rust
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

// Create primitive shapes
let cube = Mesh::cube(2.0, None)?;           // 2x2x2 cube
let sphere = Mesh::sphere(1.0, 16, 8, None)?; // Radius 1, 16 segments, 8 stacks

// Perform boolean operations
let union = cube.union(&sphere);
let difference = cube.difference(&sphere);
let intersection = cube.intersection(&sphere);

// Export result
union.to_stl_ascii(&mut std::fs::File::create("output.stl")?)?;
```

## Basic Shapes

### Primitive Shapes

```rust
use csgrs::mesh::Mesh;

// Cubes and boxes
let cube = Mesh::cube(2.0, None)?;              // Side length 2.0
let box_shape = Mesh::box(3.0, 2.0, 1.0, None)?; // Different dimensions

// Spheres and ellipsoids
let sphere = Mesh::sphere(1.0, 16, 8, None)?;    // Radius, segments, stacks
let ellipsoid = Mesh::ellipsoid(2.0, 1.0, 1.5, 16, 8, None)?;

// Cylinders and cones
let cylinder = Mesh::cylinder(1.0, 2.0, 16, None)?; // Radius, height, segments
let cone = Mesh::cone(1.0, 2.0, 16, None)?;        // Base radius, height, segments

// Platonic solids
let tetrahedron = Mesh::tetrahedron(2.0, None)?;
let octahedron = Mesh::octahedron(2.0, None)?;
let icosahedron = Mesh::icosahedron(2.0, None)?;
```

### 2D to 3D Extrusion

```rust
use csgrs::sketch::{Sketch, shapes};

// Create 2D profile
let mut sketch = Sketch::new();
let circle = shapes::circle(1.0, 32, None);
sketch.add_polygon(circle);

// Extrude to 3D
let cylinder = sketch.extrude(2.0)?;        // Height 2.0
let revolved = sketch.revolve(360.0, 32)?; // Full revolution

// Linear extrusion along path
let path = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]];
let swept = sketch.sweep(&path, 16)?;
```

## Boolean Operations

### Basic CSG Operations

```rust
let a = Mesh::cube(2.0, None)?;
let b = Mesh::sphere(1.5, 16, 8, None)?;

// Union: A ∪ B
let union = a.union(&b);

// Intersection: A ∩ B
let intersection = a.intersection(&b);

// Difference: A - B
let difference = a.difference(&b);

// XOR: (A - B) ∪ (B - A)
let xor = a.xor(&b);
```

### Complex Assemblies

```rust
// Create base plate
let base = Mesh::box(10.0, 10.0, 1.0, None)?;

// Add mounting holes
let hole = Mesh::cylinder(0.5, 2.0, 16, None)?;
let holes = hole.translate(2.0, 2.0, 0.0)?
             .union(&hole.translate(-2.0, 2.0, 0.0)?)
             .union(&hole.translate(2.0, -2.0, 0.0)?)
             .union(&hole.translate(-2.0, -2.0, 0.0)?);

// Combine base with holes
let plate = base.difference(&holes);

// Add standoffs
let standoff = Mesh::cylinder(0.3, 5.0, 12, None)?;
let standoffs = standoff.translate(3.0, 3.0, 0.0)?
                .union(&standoff.translate(-3.0, 3.0, 0.0)?)
                .union(&standoff.translate(3.0, -3.0, 0.0)?)
                .union(&standoff.translate(-3.0, -3.0, 0.0)?);

// Final assembly
let assembly = plate.union(&standoffs);
```

## Transformations

### Translation, Rotation, Scaling

```rust
let cube = Mesh::cube(1.0, None)?;

// Translation
let moved = cube.translate(1.0, 2.0, 3.0)?;

// Rotation (degrees around axes)
let rotated = cube.rotate(45.0, 30.0, 60.0)?;

// Scaling (uniform and non-uniform)
let scaled = cube.scale(2.0, 1.5, 0.8)?;

// Mirroring across plane
use csgrs::mesh::plane::Plane;
use nalgebra::Vector3;
let mirror_plane = Plane::from_normal(
    Vector3::new(0.0, 0.0, 1.0), // XY plane
    0.0
);
let mirrored = cube.mirror(mirror_plane)?;
```

### Transformation Composition

```rust
let original = Mesh::cylinder(1.0, 3.0, 16, None)?;

// Chain transformations (applied in sequence)
let transformed = original
    .translate(0.0, 0.0, 1.5)?    // Move to origin at base
    .rotate(0.0, 0.0, 45.0)?      // Rotate 45° around Z
    .scale(1.0, 2.0, 1.0)?        // Stretch in Y direction
    .translate(5.0, 0.0, 0.0)?;   // Move to final position

// Matrix-based transformations
use nalgebra::Isometry3;
let iso = Isometry3::new(
    Vector3::new(1.0, 2.0, 3.0),  // Translation
    Vector3::new(0.1, 0.2, 0.3)   // Rotation (Euler angles)
);
let matrix_transformed = original.transform(&iso.to_homogeneous())?;
```

## Mesh Processing

### IndexedMesh Operations

```rust
use csgrs::indexed_mesh::IndexedMesh;

// Convert to IndexedMesh for efficient processing
let mesh = Mesh::sphere(1.0, 32, 16, None)?;
let indexed: IndexedMesh<()> = mesh.to_indexed_mesh()?;

// Automatic vertex deduplication
let deduplicated = indexed.deduplicate_vertices(1e-6)?;

// Mesh repair operations
let repaired = indexed.repair_manifold()?;

// Subdivision for smoother surfaces
let subdivided = indexed.subdivide_loop(1)?; // 1 iteration

// Convert back to regular mesh
let final_mesh = indexed.to_mesh();
```

### Quality Analysis

```rust
let mesh = Mesh::cube(1.0, None)?;

// Analyze mesh quality
let stats = mesh.analyze_quality()?;

// Check manifold properties
let is_manifold = mesh.is_manifold();
let has_boundary = mesh.has_boundary();

// Calculate volume and surface area
let volume = mesh.volume()?;
let surface_area = mesh.surface_area()?;

// Mass properties (assuming density = 1.0)
let (mass, center_of_mass, inertia) = mesh.mass_properties(1.0)?;
```

## Voxelization

CSGRS provides two main approaches to voxelization with different characteristics:

### Dense Voxel Grids (Solid Voxelization)

For creating solid, filled voxel representations from meshes:

```rust
use csgrs::voxels::{VoxelGrid, VoxelizationConfig, VoxelizationMode};

// Create a mesh to voxelize
let sphere_mesh = Mesh::sphere(1.5, 16, 8, None)?;

// Create voxel grid sized to fit the mesh
let mut voxel_grid = VoxelGrid::from_mesh_bounds(&sphere_mesh, 0.2, 0.1, None);

// Configure for solid voxelization (fills entire volume)
let config = VoxelizationConfig {
    mode: VoxelizationMode::Solid,
    default_metadata: None,
    parallel: false,
};

// Perform voxelization
let occupied_count = voxel_grid.voxelize_mesh(&sphere_mesh, &config);

// Convert to mesh for visualization
let voxelized_mesh = voxel_grid.to_mesh(None);
```

**Result**: Produces solid, filled voxel objects (like `voxelized_cube.stl` and `voxelized_sphere.stl`)

### Sparse Voxel Octrees (Point-Based Voxelization)

For CSG operations and memory-efficient sparse representations:

```rust
use csgrs::voxels::SparseVoxelOctree;
use nalgebra::Point3;

// Create octree for point-based voxelization
let mut octree = SparseVoxelOctree::new(Point3::new(0.0, 0.0, 0.0), 8.0, 4, None);

// Manually set voxels (creates sparse grid)
let center = Point3::new(3.0, 3.0, 3.0);
let radius = 2.5;

for x in (0..160).step_by(4) {
    for y in (0..160).step_by(4) {
        for z in (0..160).step_by(4) {
            let point = Point3::new(x as f64 * 0.25, y as f64 * 0.25, z as f64 * 0.25);
            if (point - center).norm() <= radius {
                octree.set_voxel(&point, true, None);
            }
        }
    }
}
```

**Result**: Produces sparse voxel grids optimized for CSG operations (like `voxel_sphere1.stl`, `voxel_sphere2.stl`)

### Key Differences

| Aspect | Dense Voxel Grids | Sparse Voxel Octrees |
|--------|------------------|---------------------|
| **Appearance** | Solid, filled volumes | Sparse point grids |
| **Memory Usage** | Higher (dense arrays) | Lower (tree structure) |
| **CSG Operations** | Limited | Optimized for boolean ops |
| **Best For** | Visualization, export | Complex CSG pipelines |

## File I/O

### STL Export/Import

```rust
use std::fs::File;

// Export to STL (ASCII format - human readable)
let mesh = Mesh::cube(1.0, None)?;
let mut file = File::create("cube.stl")?;
mesh.to_stl_ascii(&mut file)?;

// Export to STL (binary format - compact)
let mut file = File::create("cube_binary.stl")?;
mesh.to_stl_binary(&mut file)?;

// Import from STL
let imported = Mesh::from_stl(&mut File::open("cube.stl")?)?;
```

### Other Formats

```rust
// OBJ export (Wavefront OBJ)
let mut file = File::create("model.obj")?;
mesh.to_obj(&mut file)?;

// PLY export (Polygon File Format)
let mut file = File::create("model.ply")?;
mesh.to_ply(&mut file)?;

// AMF export (Additive Manufacturing Format)
let mut file = File::create("model.amf")?;
mesh.to_amf(&mut file)?;
```

## Advanced Techniques

### Parametric Modeling

```rust
// Create a parametric bracket
fn create_bracket(width: f64, height: f64, thickness: f64) -> Result<Mesh<()>, Box<dyn std::error::Error>> {
    // Base plate
    let base = Mesh::box(width, height, thickness, None)?;

    // Lightening holes
    let hole_radius = width.min(height) * 0.15;
    let hole = Mesh::cylinder(hole_radius, thickness * 2.0, 16, None)?;

    // Position holes in a grid
    let mut holes = Vec::new();
    let spacing = width * 0.3;
    for x in [-spacing, spacing].iter() {
        for y in [-spacing, spacing].iter() {
            holes.push(hole.translate(*x, *y, 0.0)?);
        }
    }

    // Union all holes
    let all_holes = holes.into_iter()
        .reduce(|a, b| a.union(&b))
        .unwrap_or(Mesh::empty());

    // Subtract holes from base
    Ok(base.difference(&all_holes))
}

// Usage
let bracket = create_bracket(10.0, 8.0, 2.0)?;
```

### Assembly Patterns

```rust
// Create modular assembly
struct Component {
    base: Mesh<()>,
    connectors: Vec<Mesh<()>>,
}

impl Component {
    fn new(shape: Mesh<()>) -> Self {
        Self {
            base: shape,
            connectors: Vec::new(),
        }
    }

    fn add_connector(&mut self, position: [f64; 3], orientation: [f64; 3]) -> &mut Self {
        let connector = Mesh::cylinder(0.5, 1.0, 8, None)
            .unwrap()
            .translate(position[0], position[1], position[2])
            .unwrap()
            .rotate(orientation[0], orientation[1], orientation[2])
            .unwrap();
        self.connectors.push(connector);
        self
    }

    fn assemble(&self) -> Result<Mesh<()>, Box<dyn std::error::Error>> {
        let mut result = self.base.clone();
        for connector in &self.connectors {
            result = result.union(connector)?;
        }
        Ok(result)
    }
}

// Usage
let mut component = Component::new(Mesh::cube(5.0, None)?);
component
    .add_connector([2.0, 0.0, 2.5], [0.0, 90.0, 0.0])
    .add_connector([-2.0, 0.0, 2.5], [0.0, 90.0, 0.0]);

let assembly = component.assemble()?;
```

## Performance Tips

### Memory Management

```rust
// Reuse allocations where possible
let mut result = Mesh::empty();
for shape in shapes {
    result = result.union(shape)?;
}

// Use IndexedMesh for repeated operations
let indexed = mesh.to_indexed_mesh()?;
let processed = indexed.deduplicate_vertices(1e-6)?
                      .repair_manifold()?;
let final_mesh = processed.to_mesh();
```

### Quality vs Performance Trade-offs

```rust
// High quality (more triangles)
let high_quality = Mesh::sphere(1.0, 64, 32, None)?;

// Fast processing (fewer triangles)
let fast_processing = Mesh::sphere(1.0, 16, 8, None)?;

// Balance quality and performance
let balanced = Mesh::sphere(1.0, 32, 16, None)?;
```

### Parallel Processing

```rust
use rayon::prelude::*;

// Parallel boolean operations
let results: Vec<Mesh<()>> = shapes.par_iter()
    .map(|shape| base.difference(shape))
    .collect::<Result<Vec<_>, _>>()?;

// Parallel file export
shapes.par_iter().enumerate().try_for_each(|(i, shape)| {
    let filename = format!("part_{}.stl", i);
    let mut file = File::create(filename)?;
    shape.to_stl_binary(&mut file)
})?;
```

## Troubleshooting

### Common Issues

#### Non-Manifold Meshes
```rust
// Check if mesh is manifold
if !mesh.is_manifold() {
    // Attempt repair
    let indexed = mesh.to_indexed_mesh()?;
    let repaired = indexed.repair_manifold()?;
    mesh = repaired.to_mesh();
}
```

#### Self-Intersecting Geometry
```rust
// Validate mesh before operations
let validation = mesh.validate()?;
if !validation.is_valid() {
    eprintln!("Mesh validation failed: {:?}", validation.errors());
}
```

#### Precision Issues
```rust
// Use appropriate tolerances for your scale
let tolerance = mesh.bounding_box().diagonal() * 1e-6;
let processed = mesh.to_indexed_mesh()?
                   .deduplicate_vertices(tolerance)?;
```

#### Memory Issues
```rust
// Process large meshes in parts
let chunks: Vec<Mesh<()>> = large_mesh.split_into_chunks(1000)?;
let processed: Vec<Mesh<()>> = chunks.par_iter()
    .map(|chunk| expensive_operation(chunk))
    .collect::<Result<Vec<_>, _>>()?;
let result = processed.into_iter()
    .reduce(|a, b| a.union(&b))
    .unwrap_or(Mesh::empty());
```

### Debug Visualization

```rust
// Export intermediate results for inspection
let step1 = base.union(&part1)?;
step1.to_stl_ascii(&mut File::create("debug_step1.stl")?)?;

let step2 = step1.difference(&part2)?;
step2.to_stl_ascii(&mut File::create("debug_step2.stl")?)?;

// Analyze mesh statistics
println!("Vertices: {}", step2.vertices().len());
println!("Faces: {}", step2.polygons().len());
println!("Volume: {}", step2.volume()?);
println!("Is manifold: {}", step2.is_manifold());
```

### Error Handling

```rust
use csgrs::errors::CsgError;

fn robust_operation(mesh1: &Mesh<()>, mesh2: &Mesh<()>) -> Result<Mesh<()>, Box<dyn std::error::Error>> {
    // Validate inputs
    if mesh1.polygons().is_empty() || mesh2.polygons().is_empty() {
        return Err("Empty meshes not supported".into());
    }

    // Check for degenerate geometry
    if !mesh1.is_manifold() || !mesh2.is_manifold() {
        eprintln!("Warning: Non-manifold input meshes");
    }

    // Perform operation with error handling
    mesh1.union(mesh2).map_err(|e| {
        eprintln!("Union operation failed: {:?}", e);
        e
    })
}
```

This guide covers the essential patterns and techniques for effective use of `csgrs`. For API reference documentation, see the generated Rustdoc documentation. For mathematical foundations, refer to the [Mathematical Foundations](mathematical_foundations.md) document.
