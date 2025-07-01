# Octree Implementation

## Overview

This module provides a high-performance Octree implementation optimized for hierarchical 3D space subdivision. Octrees excel at volume queries, level-of-detail operations, and handling sparse or non-uniformly distributed data. They are particularly effective for adaptive mesh refinement, frustum culling, and spatial clustering operations.

## Mathematical Foundation

### Core Concept

An Octree recursively subdivides 3D space into eight octants (cubic regions). Each node represents a cubic volume, with child nodes representing the eight sub-cubes formed by splitting along the X, Y, and Z axes simultaneously. This creates a hierarchical spatial index that naturally adapts to data distribution.

### Algorithm Complexity

- **Construction**: O(n log n) average case, O(n²) worst case
- **Volume Query**: O(log n + k) where k is the number of results
- **Level-of-Detail**: O(log n) for distance-based queries
- **Space**: O(n) for n polygons

### Octant Organization

The eight octants are organized as follows:
```
     6-------7
    /|      /|     Top level (max Z)
   / |     / |
  2-------3  |
  |  4----|--5     Bottom level (min Z)
  | /     | /
  |/      |/
  0-------1

Octant indices:
0: Bottom-Front-Left   (min X, min Y, min Z)
1: Bottom-Front-Right  (max X, min Y, min Z)
2: Bottom-Back-Left    (min X, max Y, min Z)
3: Bottom-Back-Right   (max X, max Y, min Z)
4: Top-Front-Left      (min X, min Y, max Z)
5: Top-Front-Right     (max X, min Y, max Z)
6: Top-Back-Left       (min X, max Y, max Z)
7: Top-Back-Right      (max X, max Y, max Z)
```

## Module Structure

```
src/spatial/octree/
├── mod.rs           # Module declarations and re-exports
├── README.md        # This documentation
├── core.rs          # Core Node structure and octant management
├── construction.rs  # Tree building and adaptive refinement
└── queries.rs       # Volume queries and level-of-detail operations
```

## Core API

### Node Structure

```rust
pub struct Node<S: Clone> {
    pub bounds: Aabb,                        // Bounding box of this node
    pub children: [Option<Box<Node<S>>>; 8], // Eight child octants
    pub polygons: Vec<Polygon<S>>,           // Polygons in this node
    pub level: usize,                        // Subdivision level (0 = root)
    pub is_subdivided: bool,                 // Whether node has children
}
```

### Basic Operations

#### Creating Octrees

```rust
use csgrs::spatial::octree::Node;
use csgrs::geometry::Polygon;

// Create empty tree
let mut octree = Node::new();

// Build from polygons (automatic method selection)
let octree = Node::from_polygons(&polygons);

// Build with custom configuration
let config = OctreeConfig {
    max_depth: 6,
    max_polygons_per_leaf: 5,
    adaptive_refinement: true,
    ..Default::default()
};
let octree = Node::from_polygons_with_config(&polygons, &config);
```

#### Tree Information

```rust
// Get tree statistics
let stats = octree.statistics();
println!("Nodes: {}, Depth: {}, Memory: {} bytes", 
         stats.node_count, stats.max_depth, stats.memory_usage_bytes);

// Get subdivision statistics
let sub_stats = octree.subdivision_stats();
println!("Leaf nodes: {}, Avg polygons per leaf: {:.2}", 
         sub_stats.leaf_nodes, sub_stats.average_polygons_per_leaf());

// Get all polygons
let all_polygons = octree.all_polygons();
```

## Spatial Query Operations

### Volume Queries

Find all polygons within a 3D bounding box:

```rust
use csgrs::spatial::traits::Aabb;
use nalgebra::Point3;

let query_bounds = Aabb::new(
    Point3::new(0.0, 0.0, 0.0),
    Point3::new(10.0, 10.0, 10.0)
);

// Automatic method selection
let results = octree.volume_query_auto(&query_bounds);

// Explicit sequential search
let results = octree.volume_query(&query_bounds);

// Explicit parallel search (requires "parallel" feature)
#[cfg(feature = "parallel")]
let results = octree.volume_query_parallel(&query_bounds);
```

**Performance Characteristics**:
- **Complexity**: O(log n + k) where k is the number of results
- **Optimal For**: Cubic and rectangular volume queries
- **Parallel Threshold**: > 500 polygons for automatic parallel selection

### Level-of-Detail Queries

Adaptive polygon selection based on distance from viewpoint:

```rust
let viewpoint = Point3::new(0.0, 0.0, 50.0);
let detail_threshold = 0.1; // Smaller = more detail

// Automatic method selection
let lod_polygons = octree.level_of_detail_auto(&viewpoint, detail_threshold);

// Explicit sequential search
let lod_polygons = octree.level_of_detail(&viewpoint, detail_threshold);

// Explicit parallel search (requires "parallel" feature)
#[cfg(feature = "parallel")]
let lod_polygons = octree.level_of_detail_parallel(&viewpoint, detail_threshold);
```

**Use Cases**:
- **Real-time Rendering**: Reduce polygon count for distant objects
- **Adaptive Mesh Refinement**: Focus detail where needed
- **Memory Management**: Load/unload geometry based on proximity

### Frustum Culling

Efficient visibility determination for rendering:

```rust
use csgrs::geometry::Plane;

// Define frustum planes (typically from camera projection matrix)
let frustum_planes = vec![
    Plane::new(Vector3::new(1.0, 0.0, 0.0), 10.0),  // Right
    Plane::new(Vector3::new(-1.0, 0.0, 0.0), 10.0), // Left
    Plane::new(Vector3::new(0.0, 1.0, 0.0), 10.0),  // Top
    Plane::new(Vector3::new(0.0, -1.0, 0.0), 10.0), // Bottom
    Plane::new(Vector3::new(0.0, 0.0, 1.0), 1.0),   // Near
    Plane::new(Vector3::new(0.0, 0.0, -1.0), 100.0), // Far
];

let visible_polygons = octree.frustum_query(&frustum_planes);
```

## Performance Optimization

### Automatic Method Selection

The Octree implementation automatically chooses between sequential and parallel algorithms:

```rust
// These methods automatically select the best implementation
let volume_results = octree.volume_query_auto(&bounds);
let lod_results = octree.level_of_detail_auto(&viewpoint, threshold);
```

**Selection Thresholds**:
- **Parallel Construction**: > 200 polygons
- **Parallel Queries**: > 500 polygons
- **Adaptive Refinement**: Enabled by default for better space utilization

### Configuration Tuning

```rust
let config = OctreeConfig {
    max_depth: 8,                     // Prevent excessive subdivision
    min_polygons_per_leaf: 1,         // Minimum before stopping
    max_polygons_per_leaf: 8,         // Maximum before subdividing
    min_subdivision_volume: 0.001,    // Prevent tiny subdivisions
    adaptive_refinement: true,        // Enable density-based refinement
};
```

### Memory Optimization

- **Sparse Representation**: Only creates child nodes when needed
- **Adaptive Subdivision**: Subdivides based on polygon density
- **Efficient Bounds Calculation**: Hierarchical bounding box management

## Usage Examples

### Adaptive Mesh Refinement

```rust
// Create octree with adaptive refinement
let mut octree = Node::from_polygons_with_config(&mesh_polygons, &config);

// Refine based on error metrics or user interaction
octree.adaptive_refine(&config);

// Extract refined mesh at different levels
for level in 0..octree.depth() {
    let level_polygons = octree.polygons_at_level(level);
    // Process polygons at this detail level
}
```

### Spatial Clustering

```rust
// Find clusters of polygons in 3D space
let cluster_size = 5.0;
let mut clusters = Vec::new();

// Traverse octree to find dense regions
fn find_clusters<S: Clone>(node: &Node<S>, min_density: f64, clusters: &mut Vec<Vec<Polygon<S>>>) {
    let density = node.polygons.len() as f64 / node.volume();
    
    if density > min_density && node.is_leaf() {
        clusters.push(node.polygons.clone());
    } else {
        for child in &node.children {
            if let Some(ref child_node) = child {
                find_clusters(child_node, min_density, clusters);
            }
        }
    }
}

find_clusters(&octree, 10.0, &mut clusters);
```

### Progressive Mesh Loading

```rust
// Load mesh data progressively based on viewing distance
struct ProgressiveMesh {
    octree: Node<MeshData>,
    loaded_levels: Vec<bool>,
}

impl ProgressiveMesh {
    fn update_for_viewpoint(&mut self, viewpoint: &Point3<Real>) {
        // Determine required detail levels
        let required_detail = self.calculate_required_detail(viewpoint);
        
        // Load/unload mesh data as needed
        for level in 0..self.octree.depth() {
            if required_detail[level] && !self.loaded_levels[level] {
                self.load_level(level);
            } else if !required_detail[level] && self.loaded_levels[level] {
                self.unload_level(level);
            }
        }
    }
}
```

## Polymorphic Usage with SpatialIndex Trait

Octrees implement the `SpatialIndex` trait, enabling polymorphic usage alongside BSP trees and KD-trees:

```rust
use csgrs::spatial::{SpatialIndex, SpatialStructureFactory, QueryType};

// Create Octree as trait object
let octree: Box<dyn SpatialIndex<i32>> = SpatialStructureFactory::create_octree(&polygons);

// Use polymorphically with other structures
fn analyze_structure<S: Clone + Send + Sync + std::fmt::Debug>(
    structure: &dyn SpatialIndex<S>
) -> usize {
    structure.statistics().node_count
}

let node_count = analyze_structure(octree.as_ref());

// Automatic optimal selection for volume queries
let optimal_structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::VolumeQuery);
// Will likely choose Octree for volume-based operations
```

This enables writing generic spatial algorithms that work with any spatial structure type.

### Configuration Examples

```rust
use csgrs::spatial::{SpatialConfig, SpatialStructureFactory};
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};

// Create test data
let vertices = vec![
    Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, None);
let polygons = vec![polygon];

// Use predefined configuration optimized for volume queries
let volume_config = SpatialConfig::for_volume_queries();
let octree = SpatialStructureFactory::create_octree_with_config(&polygons, &volume_config);

// Customize configuration for adaptive refinement
let mut adaptive_config = SpatialConfig::for_volume_queries();
adaptive_config.max_depth = 6;
adaptive_config.adaptive_refinement = true;
adaptive_config.min_subdivision_volume = 0.01;

let adaptive_octree = SpatialStructureFactory::create_octree_with_config(&polygons, &adaptive_config);

// Both trees work with the same interface
assert_eq!(octree.all_polygons().len(), 1);
assert_eq!(adaptive_octree.all_polygons().len(), 1);
```

## Best Practices

### When to Use Octrees

**Optimal For**:
- Volume-based queries and operations
- Level-of-detail rendering and mesh simplification
- Sparse or non-uniformly distributed data
- Adaptive mesh refinement
- Frustum culling and visibility determination
- Spatial clustering and density analysis

**Less Optimal For**:
- Point location queries (use KD-trees instead)
- Boolean operations (use BSP trees instead)
- Highly uniform, dense datasets
- 2D operations (consider Quadtrees)

### Performance Tips

1. **Configuration Optimization**:
   - Adjust `max_polygons_per_leaf` based on query patterns
   - Use `adaptive_refinement` for non-uniform data
   - Set appropriate `min_subdivision_volume` to prevent over-subdivision

2. **Query Optimization**:
   - Use level-of-detail queries for rendering applications
   - Batch similar volume queries when possible
   - Consider frustum culling for real-time applications

3. **Memory Management**:
   - Monitor subdivision statistics with `subdivision_stats()`
   - Use `optimize()` method after construction
   - Consider memory usage vs. query performance trade-offs

### Integration with Other Spatial Structures

```rust
// Use Octree for coarse filtering, KD-tree for precise queries
let octree = Node::from_polygons(&all_polygons);
let coarse_results = octree.volume_query(&large_query_bounds);

// Build KD-tree from filtered results for precise nearest neighbor
let kdtree = csgrs::spatial::kdtree::Node::from_polygons(&coarse_results);
let nearest = kdtree.nearest_neighbor(&query_point);
```

## Validation and Debugging

```rust
// Validate tree structure
if let Err(error) = octree.validate() {
    eprintln!("Octree validation failed: {}", error);
}

// Optimize tree after construction
octree.optimize();

// Analyze subdivision efficiency
let stats = octree.subdivision_stats();
println!("Subdivision efficiency: {:.2}% active children per node", 
         stats.average_children_per_internal_node() / 8.0 * 100.0);
```

## See Also

- [`../kdtree/README.md`](../kdtree/README.md) - KD-tree documentation
- [`../bsp/README.md`](../bsp/README.md) - BSP tree documentation
- [`../README.md`](../README.md) - Spatial module overview
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) - Overall architecture
- [Wikipedia: Octree](https://en.wikipedia.org/wiki/Octree)
