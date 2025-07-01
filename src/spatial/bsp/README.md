# Binary Space Partitioning (BSP) Trees

## Overview

This module provides a high-performance, modular implementation of [Binary Space Partitioning (BSP) trees](https://en.wikipedia.org/wiki/Binary_space_partitioning) for 3D geometric operations. BSP trees recursively partition 3D space using hyperplanes, enabling efficient Boolean operations, visibility determination, and spatial queries.

## Mathematical Foundation

### Core Concept

A BSP tree recursively divides 3D space using hyperplanes (splitting planes). Each node in the tree represents a region of space, with polygons classified relative to the splitting plane as:

- **FRONT**: Polygons entirely in the positive half-space
- **BACK**: Polygons entirely in the negative half-space  
- **COPLANAR**: Polygons lying exactly on the plane
- **SPANNING**: Polygons crossing the plane (split into FRONT and BACK parts)

### Algorithm Complexity

- **Construction**: O(n log n) expected time for balanced trees, O(n²) worst case
- **Queries**: O(log d) where d is tree depth
- **Space**: O(n) for n polygons

## Module Structure

```
src/spatial/bsp/
├── mod.rs           # Module declarations and re-exports
├── README.md        # This documentation
├── core.rs          # Node structure and basic operations
├── operations.rs    # Non-parallel complex operations
└── parallel.rs      # Parallel implementations (requires "parallel" feature)
```

## Core API

### Node Structure

```rust
pub struct Node<S: Clone> {
    pub plane: Option<Plane>,           // Splitting plane
    pub front: Option<Box<Node<S>>>,    // Front subtree
    pub back: Option<Box<Node<S>>>,     // Back subtree
    pub polygons: Vec<Polygon<S>>,      // Coplanar polygons
}
```

### Basic Operations

#### Creating BSP Trees

```rust
use csgrs::spatial::bsp::Node;
use csgrs::geometry::Polygon;

// Create empty tree
let mut bsp = Node::new();

// Build from polygons
bsp.build(&polygons);

// Or create directly
let bsp = Node::from_polygons(&polygons);
```

#### Tree Inversion

```rust
// Invert all polygons and flip tree structure
bsp.invert();
```

#### Polygon Extraction

```rust
// Get all polygons from the tree
let all_polygons = bsp.all_polygons();
```

## Advanced Operations

### Clipping Operations

Clipping removes polygons that are inside the BSP tree:

```rust
// Remove polygons inside the BSP tree
let clipped = bsp.clip_polygons(&input_polygons);

// Clip this tree against another BSP tree
bsp.clip_to(&other_bsp);
```

**Mathematical Foundation**: Uses plane classification to determine polygon visibility. Polygons entirely in BACK half-space are clipped (removed).

### Slicing Operations

Slicing extracts geometry at the intersection with a plane:

```rust
use csgrs::geometry::Plane;

let slicing_plane = Plane::new(normal, offset);
let (coplanar_polygons, intersection_edges) = bsp.slice(&slicing_plane);
```

Returns:
- **Coplanar polygons**: Polygons lying exactly on the slicing plane
- **Intersection edges**: Line segments where spanning polygons cross the plane

## Performance Optimization

### Automatic Method Selection

The BSP implementation automatically chooses between parallel and sequential algorithms:

```rust
// Automatically selects best implementation
let clipped = bsp.clip_polygons_auto(&polygons);
let (coplanar, edges) = bsp.slice_auto(&plane);
bsp.clip_to_auto(&other_bsp);
```

### Parallel Processing

Enable the `parallel` feature for multi-threaded operations:

```toml
[dependencies]
csgrs = { version = "0.19", features = ["parallel"] }
```

```rust
// Explicit parallel operations (requires "parallel" feature)
bsp.build_parallel(&polygons);
let clipped = bsp.clip_polygons_parallel(&polygons);
```

**Performance Thresholds**:
- Parallel build: > 100 polygons
- Parallel clipping: > 50 polygons
- Parallel slicing: > 50 polygons

### Memory Optimization

- **Lazy Initialization**: Child nodes created only when needed
- **Pre-allocation**: Vectors sized based on input estimates
- **Iterator Patterns**: Efficient polygon processing without unnecessary copies

## Usage Examples

### CSG Boolean Operations

```rust
use csgrs::{CSG, spatial::bsp::Node};

// Create CSG objects
let cube: CSG<()> = CSG::cube(10.0, None);
let sphere: CSG<()> = CSG::sphere(6.0, 16, 8, None);

// Convert to BSP trees for Boolean operations
let mut cube_bsp = Node::from_polygons(&cube.polygons);
let mut sphere_bsp = Node::from_polygons(&sphere.polygons);

// Perform union: A ∪ B = A + (B - A)
sphere_bsp.clip_to(&cube_bsp);
cube_bsp.clip_to(&sphere_bsp);
sphere_bsp.invert();
cube_bsp.clip_to(&sphere_bsp);
sphere_bsp.invert();

// Combine results
let mut result_polygons = cube_bsp.all_polygons();
result_polygons.extend(sphere_bsp.all_polygons());
```

### Visibility Determination

```rust
// Check if polygons are visible from a viewpoint
let view_plane = Plane::from_point_normal(viewpoint, view_direction);
let mut visibility_bsp = Node::new();
visibility_bsp.build(&scene_polygons);

let visible_polygons = visibility_bsp.clip_polygons(&query_polygons);
```

### Cross-Section Analysis

```rust
// Extract cross-section at a specific plane
let section_plane = Plane::new(Vector3::z(), 0.0); // XY plane at Z=0
let (section_polygons, intersection_edges) = bsp.slice(&section_plane);

// Convert to 2D for further processing
let section_2d = convert_to_2d(&section_polygons, &intersection_edges);
```

## Polymorphic Usage with SpatialIndex Trait

BSP trees implement the `SpatialIndex` trait, enabling polymorphic usage alongside KD-trees and Octrees:

```rust
use csgrs::spatial::{SpatialIndex, SpatialStructureFactory, QueryType};

// Create BSP tree as trait object
let bsp: Box<dyn SpatialIndex<i32>> = SpatialStructureFactory::create_bsp(&polygons);

// Use polymorphically with other structures
fn analyze_structure<S: Clone + Send + Sync + std::fmt::Debug>(
    structure: &dyn SpatialIndex<S>
) -> usize {
    structure.statistics().node_count
}

let node_count = analyze_structure(bsp.as_ref());

// Automatic optimal selection for Boolean operations
let optimal_structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::BooleanOperations);
// Will choose BSP tree for Boolean operations
```

**Note**: Some SpatialIndex methods like `nearest_neighbor` and `ray_intersections` are not efficiently implemented for BSP trees, as they are optimized for Boolean operations rather than point queries.

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

// Use predefined configuration optimized for Boolean operations
let boolean_config = SpatialConfig::for_boolean_operations();
let bsp = SpatialStructureFactory::create_bsp_with_config(&polygons, &boolean_config);

// Customize configuration for precision-critical applications
let mut precision_config = SpatialConfig::for_boolean_operations();
precision_config.max_depth = 30;
precision_config.balance_factor = 0.8;    // Favor balanced splits
precision_config.spanning_factor = 0.2;   // Minimize spanning polygons

let precision_bsp = SpatialStructureFactory::create_bsp_with_config(&polygons, &precision_config);

// Both trees work with the same interface
assert_eq!(bsp.all_polygons().len(), 1);
assert_eq!(precision_bsp.all_polygons().len(), 1);
```

## Best Practices

### Plane Selection

The quality of BSP trees depends heavily on splitting plane selection:

- **Balanced splits**: Prefer planes that divide polygons evenly
- **Minimize spanning**: Avoid planes that split many polygons
- **Heuristic scoring**: Uses weighted combination of balance and spanning factors

### Memory Management

- Use `clip_to_auto()` for automatic parallel/sequential selection
- Consider polygon count when choosing explicit parallel operations
- Pre-allocate vectors when polygon counts are known

### Error Handling

BSP operations are generally robust, but consider:

- **Degenerate polygons**: Polygons with < 3 vertices are skipped
- **Numerical precision**: Uses `EPSILON` for floating-point comparisons
- **Empty inputs**: Operations handle empty polygon lists gracefully

## Integration with CSG Module

While BSP trees are now in the spatial module, they integrate seamlessly with CSG operations:

```rust
use csgrs::{CSG, BspNode}; // BspNode is re-exported for convenience

// CSG operations use BSP trees internally
let result = csg1.union(&csg2);

// Direct BSP usage for custom operations
let bsp = BspNode::from_polygons(&csg1.polygons);
```

## See Also

- [Wikipedia: Binary Space Partitioning](https://en.wikipedia.org/wiki/Binary_space_partitioning)
- [`../README.md`](../README.md) - Spatial module overview
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) - Overall architecture
- [`../../docs/adr/001-bsp-spatial-module-migration.md`](../../docs/adr/001-bsp-spatial-module-migration.md) - Migration rationale
