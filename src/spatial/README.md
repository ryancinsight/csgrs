# Spatial Data Structures Module

## Overview

The `spatial` module provides a comprehensive collection of spatial data structures and algorithms for geometric operations, following the Cathedral Engineering principles of modular design and single responsibility. This module serves as a dedicated architectural space for spatial indexing structures, separate from CSG operations to maintain clear separation of concerns.

## Design Philosophy

### The Law of Sacred Spaces

The spatial module follows the Cathedral Engineering principle of **The Law of Sacred Spaces** by organizing spatial data structures as independent, reusable components. This architectural decision provides:

- **Modularity**: Spatial structures can be used independently of CSG operations
- **Reusability**: BSP trees and other structures serve multiple geometric purposes
- **Maintainability**: Clear separation between spatial indexing and geometric operations
- **Extensibility**: Easy addition of new spatial data structures (kdtree, octree, etc.)

### Architectural Benefits

1. **Single Responsibility**: Each spatial structure has a focused, well-defined purpose
2. **Composability**: Spatial structures can be combined and composed for complex operations
3. **Performance**: Optimized implementations with optional parallel processing
4. **Future-Proof**: Architecture accommodates emerging spatial algorithms

## Module Structure

```
src/spatial/
├── mod.rs              # Module declarations and re-exports
├── README.md           # This documentation
└── bsp/                # Binary Space Partitioning trees
    ├── mod.rs          # BSP module declarations
    ├── README.md       # BSP-specific documentation
    ├── core.rs         # Core Node structure and basic operations
    ├── operations.rs   # Non-parallel BSP operations
    └── parallel.rs     # Parallel BSP implementations
```

## Available Spatial Structures

### Binary Space Partitioning (BSP) Trees

BSP trees provide efficient spatial subdivision for 3D geometric operations:

- **Purpose**: Recursive space partitioning using hyperplanes
- **Use Cases**: CSG operations, visibility determination, collision detection
- **Performance**: O(n log n) construction, O(log n) queries
- **Features**: Parallel processing support, optimized clipping operations

**Quick Example:**
```rust
use csgrs::spatial::bsp::Node;
use csgrs::geometry::Polygon;

// Create BSP tree from polygons
let mut bsp_tree = Node::new();
bsp_tree.build(&polygons);

// Perform spatial operations
let clipped = bsp_tree.clip_polygons(&other_polygons);
```

### K-Dimensional (KD) Trees

KD-trees excel at point-based spatial queries and nearest neighbor searches:

- **Purpose**: Axis-aligned recursive space partitioning
- **Use Cases**: Point location, nearest neighbor searches, range queries
- **Performance**: O(n log n) construction, O(log n) nearest neighbor
- **Features**: Automatic parallel/sequential selection, optimized for point queries

**Quick Example:**
```rust
use csgrs::spatial::kdtree::Node;
use nalgebra::Point3;

// Create KD-tree from polygons
let kdtree = Node::from_polygons(&polygons);

// Find nearest polygon to a point
let query_point = Point3::new(1.0, 2.0, 3.0);
let nearest = kdtree.nearest_neighbor_auto(&query_point);
```

### Octrees

Octrees provide hierarchical 3D space subdivision optimized for volume operations:

- **Purpose**: Hierarchical cubic space subdivision
- **Use Cases**: Volume queries, level-of-detail rendering, sparse data handling
- **Performance**: O(n log n) construction, O(log n + k) volume queries
- **Features**: Adaptive refinement, level-of-detail support, frustum culling

**Quick Example:**
```rust
use csgrs::spatial::octree::Node;
use csgrs::spatial::traits::Aabb;

// Create Octree from polygons
let octree = Node::from_polygons(&polygons);

// Perform volume query
let query_bounds = Aabb::new(min_point, max_point);
let results = octree.volume_query_auto(&query_bounds);

// Level-of-detail based on viewpoint
let viewpoint = Point3::new(0.0, 0.0, 10.0);
let lod_polygons = octree.level_of_detail_auto(&viewpoint, 0.1);
```

### R-Trees

R-trees provide dynamic spatial indexing optimized for bounding box queries and range operations:

- **Purpose**: Dynamic spatial indexing with minimum bounding rectangles
- **Use Cases**: Range queries, spatial databases, GIS applications, collision detection
- **Performance**: O(log n) insertion/deletion, O(log n + k) range queries
- **Features**: Dynamic operations, bulk loading, multiple split algorithms

**Quick Example:**
```rust
use csgrs::spatial::{rtree::Node, traits::Aabb, SpatialIndex};

// Create R-tree from polygons
let rtree = Node::from_polygons(&polygons);

// Dynamic insertion
let mut dynamic_rtree: Node<i32> = Node::new();
dynamic_rtree.insert_rtree(new_polygon);

// Range query
let query_bounds = Aabb::new(min_point, max_point);
let results = rtree.query_range(&query_bounds);

// Remove by metadata
let removed = dynamic_rtree.remove_by_metadata(&Some(42));
```

### Future Spatial Structures

The spatial module is designed to accommodate additional spatial data structures:

- **Spatial Hashing**: For fast collision detection
- **BVH (Bounding Volume Hierarchies)**: For ray tracing and collision detection

## Usage Patterns and Structure Selection

### When to Use Each Structure

**BSP Trees** - Best for:
- Boolean operations (union, intersection, difference)
- Visibility determination and hidden surface removal
- Precise geometric operations requiring plane-based subdivision
- CSG operations on complex meshes

**KD-Trees** - Best for:
- Point location queries
- Nearest neighbor searches
- Range queries with axis-aligned bounding boxes
- Uniform to moderately clustered data distributions

**Octrees** - Best for:
- Volume-based queries and operations
- Level-of-detail rendering and mesh simplification
- Sparse or non-uniformly distributed data
- Adaptive mesh refinement and spatial clustering

### API Access Patterns

All spatial structures support multiple access patterns:

```rust
// Direct module access (most explicit)
use csgrs::spatial::bsp::Node as BspNode;
use csgrs::spatial::kdtree::Node as KdTreeNode;
use csgrs::spatial::octree::Node as OctreeNode;

// Spatial module re-exports (recommended)
use csgrs::spatial::{BspNode, KdTreeNode, OctreeNode};

// Crate-level re-exports (most convenient)
use csgrs::{BspNode, KdTreeNode, OctreeNode};
```

### Unified Configuration System

All spatial structures now support a unified configuration system for consistent behavior:

```rust
use csgrs::spatial::{SpatialConfig, SpatialStructureFactory, QueryType};

// Use predefined optimized configurations
let boolean_config = SpatialConfig::for_boolean_operations();
let point_config = SpatialConfig::for_point_queries();
let volume_config = SpatialConfig::for_volume_queries();
let range_config = SpatialConfig::for_range_queries();
let large_config = SpatialConfig::for_large_datasets();
let memory_config = SpatialConfig::for_memory_efficiency();

// Create structures with specific configurations
let bsp = SpatialStructureFactory::create_bsp_with_config(&polygons, &boolean_config);
let kdtree = SpatialStructureFactory::create_kdtree_with_config(&polygons, &point_config);
let octree = SpatialStructureFactory::create_octree_with_config(&polygons, &volume_config);
let rtree = SpatialStructureFactory::create_rtree_with_config(&polygons, &range_config);

// Customize configuration for specific needs
let mut custom_config = SpatialConfig::default();
custom_config.max_depth = 15;
custom_config.max_polygons_per_leaf = 5;
custom_config.parallel_threshold = 200;

let custom_structure = SpatialStructureFactory::create_structure_with_config(
    &polygons,
    SpatialStructureType::Kdtree,
    &custom_config
);
```

### Automatic Structure Selection

```rust
use csgrs::spatial::{SpatialStructureSelector, QueryType, SpatialStructureFactory};

// Analyze your data
let characteristics = SpatialStructureSelector::analyze_dataset(&polygons);

// Get recommendation based on intended use
let recommended = SpatialStructureSelector::recommend_structure(
    &characteristics,
    QueryType::NearestNeighbor
);

// Option 1: Create specific structures
match recommended {
    SpatialStructureType::Bsp => {
        let structure = BspNode::from_polygons(&polygons);
        // Use for Boolean operations
    },
    SpatialStructureType::Kdtree => {
        let structure = KdTreeNode::from_polygons(&polygons);
        // Use for point queries
    },
    SpatialStructureType::Octree => {
        let structure = OctreeNode::from_polygons(&polygons);
        // Use for volume queries
    },
    SpatialStructureType::Rtree => {
        let structure = RTreeNode::from_polygons(&polygons);
        // Use for range queries and dynamic indexing
    },
    _ => {
        // Handle hybrid or other recommendations
    }
}

// Option 2: Use factory for polymorphic structures
let structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::NearestNeighbor);
// structure is now Box<dyn SpatialIndex<S>>
```

### Polymorphic Usage Patterns

The spatial module supports polymorphic usage through the `SpatialIndex` trait, enabling you to write generic code that works with any spatial structure:

```rust
use csgrs::spatial::{SpatialIndex, SpatialStructureFactory, QueryType};

// Generic function that works with any spatial structure
fn analyze_spatial_data<S: Clone + Send + Sync + std::fmt::Debug>(
    structure: &dyn SpatialIndex<S>
) -> (usize, usize) {
    let stats = structure.statistics();
    let polygon_count = structure.all_polygons().len();
    (polygon_count, stats.node_count)
}

// Create different structures as trait objects
let bsp = SpatialStructureFactory::create_bsp(&polygons);
let kdtree = SpatialStructureFactory::create_kdtree(&polygons);
let octree = SpatialStructureFactory::create_octree(&polygons);
let rtree = SpatialStructureFactory::create_rtree(&polygons);

// Use them polymorphically
let (bsp_polys, bsp_nodes) = analyze_spatial_data(bsp.as_ref());
let (kd_polys, kd_nodes) = analyze_spatial_data(kdtree.as_ref());
let (oct_polys, oct_nodes) = analyze_spatial_data(octree.as_ref());
let (rt_polys, rt_nodes) = analyze_spatial_data(rtree.as_ref());

// Store different structures in collections
let mut structures: Vec<Box<dyn SpatialIndex<i32>>> = Vec::new();
structures.push(SpatialStructureFactory::create_optimal(&polygons, QueryType::BooleanOperations));
structures.push(SpatialStructureFactory::create_optimal(&polygons, QueryType::PointLocation));
structures.push(SpatialStructureFactory::create_optimal(&polygons, QueryType::VolumeQuery));
structures.push(SpatialStructureFactory::create_optimal(&polygons, QueryType::RangeQuery));

// Process them uniformly
for structure in &structures {
    let bounds = structure.bounding_box();
    let stats = structure.statistics();
    println!("Structure has {} polygons in {} nodes", stats.polygon_count, stats.node_count);
}
```

### Runtime Structure Selection

```rust
use csgrs::spatial::{SpatialStructureFactory, QueryType, SpatialIndex, Aabb};
use csgrs::geometry::Polygon;
use nalgebra::Point3;

// Choose structure based on runtime conditions
fn choose_structure_for_task(
    polygons: &[Polygon<i32>],
    task_type: &str
) -> Box<dyn SpatialIndex<i32>> {
    match task_type {
        "boolean" => SpatialStructureFactory::create_optimal(polygons, QueryType::BooleanOperations),
        "search" => SpatialStructureFactory::create_optimal(polygons, QueryType::PointLocation),
        "volume" => SpatialStructureFactory::create_optimal(polygons, QueryType::VolumeQuery),
        _ => SpatialStructureFactory::create_optimal(polygons, QueryType::RangeQuery),
    }
}

// Use at runtime
let task = determine_task_type(); // Your application logic
let structure = choose_structure_for_task(&polygons, &task);

// All structures support the same interface
let query_bounds = Aabb::new(min_point, max_point);
let results = structure.query_range(&query_bounds);
```

### Comprehensive Runtime Examples

```rust
use csgrs::spatial::{SpatialStructureFactory, SpatialConfig, QueryType, SpatialIndex};
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

// Example 1: Adaptive structure selection based on data characteristics
fn select_optimal_structure(
    polygons: &[Polygon<i32>],
    operation_type: &str,
    performance_priority: &str
) -> Box<dyn SpatialIndex<i32>> {
    match (operation_type, performance_priority) {
        ("boolean", _) => {
            // Always use BSP for Boolean operations
            SpatialStructureFactory::create_optimal(polygons, QueryType::BooleanOperations)
        },
        ("search", "speed") => {
            // Use KD-tree with speed-optimized configuration
            let config = SpatialConfig::for_point_queries();
            SpatialStructureFactory::create_kdtree_with_config(polygons, &config)
        },
        ("search", "memory") => {
            // Use memory-optimized configuration
            SpatialStructureFactory::create_memory_optimized(polygons, QueryType::PointLocation)
        },
        ("volume", "large_data") => {
            // Use configuration optimized for large datasets
            SpatialStructureFactory::create_for_large_dataset(polygons, QueryType::VolumeQuery)
        },
        _ => {
            // Default to automatic optimal selection
            SpatialStructureFactory::create_optimal(polygons, QueryType::RangeQuery)
        }
    }
}

// Example 2: Multi-structure application
struct SpatialProcessor {
    structures: Vec<Box<dyn SpatialIndex<i32>>>,
    current_index: usize,
}

impl SpatialProcessor {
    fn new(polygons: &[Polygon<i32>]) -> Self {
        let mut structures = Vec::new();

        // Create multiple structures for different use cases
        structures.push(SpatialStructureFactory::create_optimal(polygons, QueryType::BooleanOperations));
        structures.push(SpatialStructureFactory::create_optimal(polygons, QueryType::PointLocation));
        structures.push(SpatialStructureFactory::create_optimal(polygons, QueryType::VolumeQuery));

        Self {
            structures,
            current_index: 0,
        }
    }

    fn switch_to_optimal_for_query(&mut self, query_type: QueryType) {
        self.current_index = match query_type {
            QueryType::BooleanOperations => 0,
            QueryType::PointLocation | QueryType::NearestNeighbor => 1,
            QueryType::VolumeQuery | QueryType::RangeQuery => 2,
            _ => 1, // Default to point queries
        };
    }

    fn current_structure(&self) -> &dyn SpatialIndex<i32> {
        self.structures[self.current_index].as_ref()
    }
}

// Usage example
let mut processor = SpatialProcessor::new(&polygons);

// Switch to optimal structure for different operations
processor.switch_to_optimal_for_query(QueryType::PointLocation);
let stats = processor.current_structure().statistics();
assert!(stats.polygon_count > 0);

processor.switch_to_optimal_for_query(QueryType::VolumeQuery);
let all_polys = processor.current_structure().all_polygons();
assert_eq!(all_polys.len(), 1);
```

### Configuration Parameters

The `SpatialConfig` struct provides unified control over all spatial structure parameters:

```rust
use csgrs::spatial::SpatialConfig;

let config = SpatialConfig {
    max_depth: 20,                    // Maximum tree depth
    min_polygons_per_leaf: 1,         // Minimum polygons before stopping subdivision
    max_polygons_per_leaf: 10,        // Maximum polygons before forcing subdivision
    parallel_threshold: 100,          // Polygon count threshold for parallel processing
    use_sah: true,                    // Use Surface Area Heuristic (KD-trees)
    adaptive_refinement: true,        // Enable adaptive refinement (Octrees)
    min_subdivision_volume: 0.001,    // Minimum volume for subdivision (Octrees)
    balance_factor: 0.5,              // Balance factor for plane selection (BSP)
    spanning_factor: 0.5,             // Spanning factor for plane selection (BSP)
};

// Apply to any structure type
let structure = SpatialStructureFactory::create_structure_with_config(
    &polygons,
    SpatialStructureType::Kdtree,
    &config
);
```

**Predefined Configurations**:
- `SpatialConfig::for_boolean_operations()` - Optimized for CSG operations
- `SpatialConfig::for_point_queries()` - Optimized for nearest neighbor and point location
- `SpatialConfig::for_volume_queries()` - Optimized for volume-based operations
- `SpatialConfig::for_large_datasets()` - Balanced for large polygon counts
- `SpatialConfig::for_memory_efficiency()` - Minimizes memory usage

### Advanced Configuration Examples

```rust
use csgrs::spatial::{SpatialConfig, SpatialStructureFactory, SpatialStructureType};
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};

// Create test polygons
let vertices = vec![
    Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, None);
let polygons = vec![polygon];

// Example 1: Fine-tuned configuration for real-time applications
let mut realtime_config = SpatialConfig::for_point_queries();
realtime_config.max_depth = 12;           // Limit depth for consistent performance
realtime_config.parallel_threshold = 50;  // Lower threshold for responsiveness
realtime_config.max_polygons_per_leaf = 3; // Smaller leaves for precision

let realtime_structure = SpatialStructureFactory::create_kdtree_with_config(
    &polygons,
    &realtime_config
);

// Example 2: Memory-constrained configuration
let mut memory_config = SpatialConfig::for_memory_efficiency();
memory_config.max_polygons_per_leaf = 20; // Larger leaves to reduce nodes
memory_config.adaptive_refinement = false; // Disable adaptive features
memory_config.parallel_threshold = 2000;   // Prefer sequential processing

let memory_structure = SpatialStructureFactory::create_octree_with_config(
    &polygons,
    &memory_config
);

// Example 3: High-precision configuration for CAD applications
let mut precision_config = SpatialConfig::for_boolean_operations();
precision_config.max_depth = 30;           // Deep subdivision for precision
precision_config.balance_factor = 0.8;     // Favor balanced splits
precision_config.spanning_factor = 0.2;    // Minimize spanning polygons

let precision_structure = SpatialStructureFactory::create_bsp_with_config(
    &polygons,
    &precision_config
);

// All structures work with the same interface
assert_eq!(realtime_structure.all_polygons().len(), 1);
assert_eq!(memory_structure.all_polygons().len(), 1);
assert_eq!(precision_structure.all_polygons().len(), 1);
```

## Integration with CSG Operations

While spatial structures are independent, they integrate seamlessly with CSG operations:

```rust
use csgrs::{CSG, BspNode};

// CSG operations use spatial structures internally
let result = csg1.union(&csg2);  // Uses BSP trees for Boolean operations

// Direct spatial structure usage
let mut bsp = BspNode::from_polygons(&csg1.polygons);
let clipped = bsp.clip_polygons(&csg2.polygons);
```

## Performance Considerations

### Memory Usage

Each spatial structure has different memory characteristics:

- **BSP Trees**: Moderate memory usage, depends on tree balance
- **KD-Trees**: Excellent memory efficiency, minimal overhead
- **Octrees**: Higher memory usage due to 8-way branching, but efficient for sparse data

### Query Performance

Performance varies significantly based on data characteristics and query types:

- **Point Queries**: KD-trees > Octrees > BSP trees
- **Range Queries**: KD-trees ≈ Octrees > BSP trees
- **Volume Queries**: Octrees > KD-trees > BSP trees
- **Boolean Operations**: BSP trees >> KD-trees ≈ Octrees

### Construction Time

- **KD-Trees**: Fastest construction, O(n log n) consistently
- **BSP Trees**: Moderate construction time, depends on heuristics
- **Octrees**: Moderate to slow construction, depends on subdivision depth

### Trait Object vs Direct Usage Performance

When using polymorphic patterns with trait objects (`Box<dyn SpatialIndex<S>>`), there are small performance considerations:

**Direct Usage (Zero-Cost Abstraction)**:
```rust
let kdtree = KdTreeNode::from_polygons(&polygons);
let nearest = kdtree.nearest_neighbor(&point); // Direct method call
```

**Trait Object Usage (Small Virtual Call Overhead)**:
```rust
let structure: Box<dyn SpatialIndex<i32>> = SpatialStructureFactory::create_kdtree(&polygons);
let nearest = structure.nearest_neighbor(&point); // Virtual method call
```

**Performance Impact**:
- **Virtual Call Overhead**: ~1-3ns per method call (negligible for most applications)
- **Memory Overhead**: One additional pointer indirection
- **Optimization**: Compiler can still optimize hot paths in many cases

**Recommendations**:
- Use direct types for performance-critical inner loops
- Use trait objects for application-level structure selection and generic algorithms
- The flexibility benefits usually outweigh the minimal performance cost

### General Performance Tips

- **Parallel Processing**: Enable the `parallel` feature for multi-threaded operations
- **Memory Efficiency**: Spatial structures use lazy initialization and optimized memory layouts
- **Algorithm Selection**: Automatic selection between parallel and sequential implementations

## Contributing

When adding new spatial data structures:

1. Create a dedicated subdirectory (e.g., `src/spatial/kdtree/`)
2. Follow the modular structure pattern established by BSP
3. Provide comprehensive documentation and usage examples
4. Include both sequential and parallel implementations where applicable
5. Add appropriate re-exports in `src/spatial/mod.rs`

## Error Handling and Validation

The spatial module provides comprehensive error handling for robust applications:

```rust
use csgrs::spatial::{SpatialError, SpatialConfig, SpatialStructureFactory, QueryType};
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

// Configuration validation
let mut config = SpatialConfig::default();
config.max_depth = 0; // Invalid value

match config.validate() {
    Ok(()) => println!("Configuration is valid"),
    Err(SpatialError::ConfigurationError { parameter, value, expected }) => {
        println!("Invalid {}: '{}', expected {}", parameter, value, expected);
    },
    Err(e) => println!("Other error: {}", e),
}

// Safe structure creation with error handling
match SpatialStructureFactory::try_create_kdtree(&polygons) {
    Ok(kdtree) => {
        println!("Successfully created KD-tree with {} polygons", kdtree.all_polygons().len());
    },
    Err(SpatialError::InvalidInput { input_type, message, suggestion }) => {
        println!("Invalid input ({}): {}", input_type, message);
        if let Some(sug) = suggestion {
            println!("Suggestion: {}", sug);
        }
    },
    Err(e) => println!("Creation failed: {}", e),
}

// Validated configuration creation
match SpatialConfig::validated(10, 1, 5, 100, true, true, 0.001, 0.5, 0.5) {
    Ok(config) => {
        let structure = SpatialStructureFactory::create_kdtree_with_config(&polygons, &config);
        println!("Created structure with validated config");
    },
    Err(e) => println!("Configuration validation failed: {}", e),
}
```

**Error Types**:
- `SpatialError::ConfigurationError` - Invalid configuration parameters
- `SpatialError::InvalidInput` - Invalid input data (e.g., empty polygons)
- `SpatialError::ConstructionError` - Errors during structure construction
- `SpatialError::QueryError` - Errors during spatial queries
- `SpatialError::GeometricError` - Geometric computation failures
- `SpatialError::UnsupportedOperation` - Operations not supported by structure type

**Best Practices**:
- Use `try_*` methods for error handling in production code
- Validate configurations before use with `config.validate()`
- Handle empty input gracefully with appropriate error messages
- Use `SpatialConfig::validated()` for safe configuration creation

## Shared Geometric Utilities

The spatial module provides shared geometric utilities to eliminate code duplication:

```rust
use csgrs::spatial::utils;
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};

// Create a test polygon
let vertices = vec![
    Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(1.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, None);

// Polygon operations
let center = utils::polygon_center(&polygon);
let bounds = utils::polygon_bounds(&polygon).unwrap();
let area = utils::polygon_area(&polygon);

// Bounds operations
use csgrs::spatial::traits::Aabb;
let bounds1 = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
let bounds2 = Aabb::new(Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));

let union = utils::bounds_union(&[bounds1.clone(), bounds2.clone()]);
let intersection = utils::bounds_intersection(&bounds1, &bounds2);
let merged = utils::merge_bounds(&bounds1, &bounds2);
let volume = utils::bounds_volume(&bounds1);

// Distance calculations
let p1 = Point3::new(0.0, 0.0, 0.0);
let p2 = Point3::new(3.0, 4.0, 0.0);
let distance = utils::distance(&p1, &p2);
let distance_sq = utils::distance_squared(&p1, &p2);
```

**Available Utilities**:
- **Polygon Operations**: `polygon_center()`, `polygon_bounds()`, `polygon_area()`, `polygon_intersects_bounds()`
- **Point Operations**: `point_in_polygon()`, `closest_point_on_polygon()`, `distance()`, `distance_squared()`
- **Bounds Operations**: `bounds_intersection()`, `bounds_union()`, `merge_bounds()`, `bounds_contains_point()`, `bounds_volume()`
- **Ray Operations**: `ray_intersects_aabb()`, `ray_aabb_intersection_distance()`

**Note**: All spatial structures (BSP, KD-tree, Octree, R-tree) use these shared utilities to ensure consistent behavior and eliminate code duplication. R-trees particularly benefit from the bounds operations for minimum bounding rectangle calculations.

## See Also

- [`bsp/README.md`](bsp/README.md) - Detailed BSP tree documentation
- [`utils.rs`](utils.rs) - Shared geometric utilities documentation
- [`ARCHITECTURE.md`](../../ARCHITECTURE.md) - Overall codebase architecture
- [`docs/adr/001-bsp-spatial-module-migration.md`](../../docs/adr/001-bsp-spatial-module-migration.md) - Migration decision record
