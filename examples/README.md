# CSGRS Voxel Operation Examples

This directory contains comprehensive examples demonstrating the voxel-based geometric operations available in csgrs.

## Overview

The voxel system in csgrs provides three main types of voxel representations:

1. **Dense Voxel Grids**: Regular 3D grids with uniform voxel sizes
2. **Sparse Voxel Octrees**: Hierarchical octree structures for efficient sparse volume representation
3. **CSG Operations**: Boolean operations (union, intersection, difference, XOR) on voxel representations

## Running Examples

### Basic Usage

```bash
# Run all examples
cargo run --bin csgrs-examples

# Run specific example category
cargo run --bin csgrs-examples -- voxels
```

### Individual Voxel Demonstrations

```bash
# Dense voxel grid operations
cargo run --bin csgrs-examples -- voxels

# Or run individual functions programmatically
```

## Example Output

When you run the voxel examples, they will create:

### STL Files (for visualization)
- `stl/voxel_grid_manual.stl` - Manually created voxel pattern
- `stl/voxelized_cube.stl` - Cube mesh converted to voxels
- `stl/sparse_voxel_sphere.stl` - Sphere created with sparse octree
- `stl/compressed_voxel_sphere.stl` - Same sphere with DAG compression
- `stl/voxel_union.stl` - Boolean union of two voxel spheres
- `stl/voxel_intersection.stl` - Boolean intersection
- `stl/voxel_difference.stl` - Boolean difference
- `stl/voxel_xor.stl` - Boolean XOR (symmetric difference)
- `stl/voxel_original.stl` - Original voxel cube
- `stl/voxel_translated.stl` - Translated version
- `stl/voxel_rotated.stl` - Rotated version
- `stl/voxel_scaled.stl` - Scaled version

### Statistics Files
- `voxels/grid_stats.txt` - Dense voxel grid statistics
- `voxels/cube_voxelization_stats.txt` - Mesh voxelization statistics
- `voxels/sparse_octree_stats.txt` - Sparse octree memory usage
- `voxels/compression_comparison.txt` - Compression efficiency comparison
- `voxels/csg_operations_stats.txt` - CSG operation voxel counts
- `voxels/transformation_stats.txt` - Transformation statistics

## Voxel Operations Demonstrated

### 1. Dense Voxel Grid Operations

**Features:**
- Manual voxel placement
- Memory-efficient storage
- Direct coordinate access
- Mesh conversion and export

**Example Output:**
```
Voxel Grid Statistics:
Dimensions: 10x10x10
Voxel Size: 0.5
Occupied Voxels: 10
Occupancy Ratio: 1.00%
Memory Usage: 4000 bytes
```

### 2. Sparse Voxel Octree Operations

**Features:**
- Hierarchical volume representation
- Automatic subdivision
- Memory-efficient for sparse data
- Configurable depth limits

**Example Output:**
```
Sparse Voxel Octree Statistics:
Octree Size: 8.0
Max Depth: 4
Occupied Leaves: 45
Total Nodes: 156
Memory Usage: 6240 bytes
Compression Ratio: N/A
Compression Savings: N/A
```

### 3. DAG Compression

**Features:**
- Deduplication of identical subtrees
- Massive memory savings for repetitive structures
- Transparent operation (no API changes)

**Example Output:**
```
Compression Comparison:
Uncompressed - Memory: 6240 bytes, Nodes: 156
Compressed - Memory: 3120 bytes, Nodes: 78
Space Savings: 50.0%
```

### 4. CSG Operations

**Features:**
- Boolean union, intersection, difference, XOR
- Efficient octree-based algorithms
- Automatic result optimization

**Example Output:**
```
Voxel CSG Operations Statistics:
Sphere 1 - Occupied: 42
Sphere 2 - Occupied: 38
Union - Occupied: 58
Intersection - Occupied: 22
Difference - Occupied: 20
XOR - Occupied: 36
```

### 5. Voxel Transformations

**Features:**
- Translation, rotation, scaling via CSG trait
- Matrix-based transformations
- Automatic voxel resampling

**Example Output:**
```
Voxel Transformation Statistics:
Original - Occupied: 64
Translated - Occupied: 64
Rotated - Occupied: 62
Scaled - Occupied: 45
Transformations applied via CSG trait methods
```

## Visualization

The generated STL files can be visualized using:
- **Meshlab** - Free 3D mesh processing software
- **f3d** - Fast and minimalist 3D viewer
- **Online STL Viewers** - Various web-based tools
- **Blender** - Full 3D modeling suite

## Performance Characteristics

### Dense Voxel Grids
- **Memory**: O(width × height × depth)
- **Access**: O(1) - direct indexing
- **Best for**: Uniform grids, small volumes, direct manipulation

### Sparse Voxel Octrees
- **Memory**: O(occupied voxels × log(max_depth))
- **Access**: O(log(max_depth))
- **Best for**: Large sparse volumes, hierarchical operations

### CSG Operations
- **Time**: O(tree size × operation complexity)
- **Memory**: O(result size)
- **Optimization**: Automatic DAG compression reduces memory usage

## API Usage Examples

### Creating a Voxel Grid

```rust
use csgrs::voxels::{VoxelGrid, VoxelData};
use nalgebra::Point3;

let mut grid = VoxelGrid::new(
    Point3::new(0.0, 0.0, 0.0), // origin
    0.5,                         // voxel size
    (10, 10, 10),               // dimensions
    None                         // metadata
);

// Set some voxels
grid.set_voxel(5, 5, 5, VoxelData::Occupied { metadata: Some(()) });

// Convert to mesh
let mesh = grid.to_mesh(None);
```

### Creating a Sparse Voxel Octree

```rust
use csgrs::voxels::SparseVoxelOctree;
use nalgebra::Point3;

let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(
    Point3::new(0.0, 0.0, 0.0), // origin
    8.0,                         // size
    4,                          // max depth
    None                         // metadata
);

// Set voxels (automatic subdivision)
octree.set_voxel(&Point3::new(2.0, 2.0, 2.0), true, None);

// Convert to mesh
let mesh = octree.to_mesh();
```

### CSG Operations

```rust
use csgrs::voxels::SparseVoxelOctree;

// Create two shapes
let mut sphere1: SparseVoxelOctree<()> = SparseVoxelOctree::new(/* ... */);
let mut sphere2: SparseVoxelOctree<()> = SparseVoxelOctree::new(/* ... */);

// Perform boolean operations
let union = sphere1.csg_union(&sphere2);
let intersection = sphere1.csg_intersection(&sphere2);
let difference = sphere1.csg_difference(&sphere2);
let xor = sphere1.csg_symmetric_difference(&sphere2);
```

### Transformations

```rust
use csgrs::traits::CSG;
use nalgebra::{Translation3, Rotation3, Vector3};

// Apply transformations using CSG trait
let translate_matrix = Translation3::new(3.0, 0.0, 0.0).to_homogeneous();
let translated = octree.transform(&translate_matrix);

let rotate_matrix = Rotation3::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI / 4.0).to_homogeneous();
let rotated = octree.transform(&rotate_matrix);
```

## Advanced Features

### Custom Voxel Metadata

```rust
#[derive(Clone, Debug)]
struct Material {
    density: f64,
    color: [f64; 3],
}

// Use with voxel operations
let mut grid: VoxelGrid<Material> = VoxelGrid::new(/* ... */);
```

### Performance Benchmarking

```rust
use csgrs::voxels::SparseVoxelOctree;

// Get memory statistics
let stats = octree.memory_stats();
println!("Memory usage: {} bytes", stats.memory_usage_bytes);
println!("Compression ratio: {:?}", stats.compression_ratio);
```

## Troubleshooting

### Common Issues

1. **STL Export Fails**: Ensure the `stl-io` feature is enabled
   ```toml
   csgrs = { version = "0.20", features = ["stl-io"] }
   ```

2. **High Memory Usage**: Use sparse octrees for large volumes
   ```rust
   let octree = SparseVoxelOctree::new(origin, size, max_depth, metadata);
   ```

3. **Slow Operations**: Enable compression for repetitive structures
   ```rust
   let mut octree = SparseVoxelOctree::new_compressed(/* ... */);
   ```

### Performance Tips

- Use sparse octrees for volumes with < 10% occupancy
- Enable compression for structures with repetitive patterns
- Limit octree depth based on required precision
- Use CSG operations instead of manual voxel manipulation for complex shapes

## Integration with Existing Code

The voxel system integrates seamlessly with existing csgrs Mesh and IndexedMesh types:

```rust
// Convert mesh to voxels
let mesh = csgrs::mesh::Mesh::cube(2.0, None).expect("Failed to create mesh");
let octree = SparseVoxelOctree::from_mesh(&mesh, 0.1, None);

// Convert back to mesh
let voxel_mesh = octree.to_mesh();

// Export as STL
#[cfg(feature = "stl-io")]
{
    let stl_data = voxel_mesh.to_stl_binary("voxelized_mesh")?;
    std::fs::write("output.stl", stl_data)?;
}
```

This enables powerful workflows combining traditional mesh-based modeling with voxel-based boolean operations and volume processing.
