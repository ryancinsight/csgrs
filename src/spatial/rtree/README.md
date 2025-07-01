# R-tree Spatial Data Structure

R-trees are balanced tree data structures optimized for indexing multi-dimensional spatial data. They are particularly well-suited for:

- **Dynamic spatial indexing** with efficient insertion and deletion
- **Bounding box range queries** with polygon overlap detection  
- **Real-time applications** like GIS, collision detection, and spatial databases
- **Large datasets** where bulk loading provides superior tree quality

## Algorithmic Background

R-trees organize spatial data by grouping nearby objects and representing them with their minimum bounding rectangle (MBR) in the next higher level of the tree. This hierarchical structure enables efficient spatial queries by pruning large portions of the search space.

### Key Properties

- **Balanced tree structure** - All leaf nodes are at the same level
- **Minimum bounding rectangles** - Each node contains the MBR of all its children
- **Configurable branching factor** - Typically 4-16 children per node
- **Dynamic operations** - Supports insertion/deletion without full reconstruction

### Time Complexity

- **Range Query**: O(log n) to O(n) depending on query selectivity
- **Insertion**: O(log n) average case
- **Deletion**: O(log n) average case
- **Construction**: O(n log n) with bulk loading

## Basic Usage

### Creating an R-tree

```rust
use csgrs::spatial::{rtree::Node, SpatialIndex};
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

// Build R-tree from polygons
let rtree = Node::from_polygons(&polygons);
println!("Created R-tree with {} polygons", rtree.all_polygons().len());
```

### Dynamic Operations

```rust
use csgrs::spatial::rtree::Node;

// Create empty R-tree
let mut rtree: Node<i32> = Node::new();

// Insert polygons dynamically
let vertices = vec![
    Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(3.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(2.5, 3.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, Some(42));

rtree.insert_rtree(polygon);
println!("Tree now contains {} polygons", rtree.polygon_count());

// Remove by metadata
let removed = rtree.remove_by_metadata(&Some(42));
println!("Removal successful: {}", removed);
```

### Range Queries

```rust
use csgrs::spatial::{rtree::Node, traits::Aabb, SpatialIndex};

// Create and populate R-tree
let rtree = Node::from_polygons(&polygons);

// Define query bounds
let query_bounds = Aabb::new(
    Point3::new(0.0, 0.0, 0.0), 
    Point3::new(1.0, 1.0, 1.0)
);

// Find all polygons intersecting the bounds
let results = rtree.query_range(&query_bounds);
println!("Found {} polygons in query range", results.len());
```

## Configuration and Optimization

### Split Algorithms

R-trees support different split algorithms with varying trade-offs:

```rust
use csgrs::spatial::rtree::{RTreeConfig, SplitAlgorithm};

// Linear split - fastest construction, lower quality
let fast_config = RTreeConfig {
    split_algorithm: SplitAlgorithm::Linear,
    max_children: 16,
    min_children: 8,
    ..Default::default()
};

// Quadratic split - balanced performance and quality
let balanced_config = RTreeConfig {
    split_algorithm: SplitAlgorithm::Quadratic,
    max_children: 8,
    min_children: 4,
    ..Default::default()
};

// R*-tree split - highest quality, slower construction
let quality_config = RTreeConfig {
    split_algorithm: SplitAlgorithm::RStarTree,
    max_children: 6,
    min_children: 3,
    reinsert_factor: 0.3,
    ..Default::default()
};
```

### Predefined Configurations

```rust
use csgrs::spatial::rtree::RTreeConfig;

// Optimized for fast insertion
let fast_insert = RTreeConfig::for_fast_insertion();

// Optimized for range query performance  
let range_optimized = RTreeConfig::for_range_queries();

// Optimized for memory efficiency
let memory_efficient = RTreeConfig::for_memory_efficiency();

// Optimized for large datasets
let large_data = RTreeConfig::for_large_datasets();
```

### Bulk Loading

For large datasets, bulk loading provides better tree quality than incremental insertion:

```rust
use csgrs::spatial::rtree::{Node, RTreeConfig};

// STR (Sort-Tile-Recursive) bulk loading
let config = RTreeConfig::for_range_queries();
let rtree = Node::str_bulk_load(&large_polygon_set, &config);

// Hilbert curve bulk loading (alternative)
let hilbert_rtree = Node::hilbert_bulk_load(&large_polygon_set, &config);
```

## Performance Characteristics

### Optimal Use Cases

- **Range queries** over spatial regions
- **Dynamic datasets** with frequent insertions/deletions
- **Large datasets** (>1000 polygons) with bulk loading
- **Real-time applications** requiring consistent performance

### Performance Tuning

- **Branching factor**: Higher values (8-16) reduce tree height but increase node processing cost
- **Split algorithm**: Linear for speed, R*-tree for quality
- **Bulk loading**: Use for datasets larger than `bulk_load_threshold`
- **Memory efficiency**: Lower branching factor (4-6) reduces memory usage

### Benchmarks

Typical performance characteristics (varies by data distribution):

- **Range queries**: 10-100x faster than linear search for sparse queries
- **Insertion**: ~O(log n) with occasional O(n) for tree restructuring
- **Memory usage**: ~20-40 bytes per polygon plus tree overhead
- **Construction**: Bulk loading 2-5x faster than incremental for large datasets

## Integration with Spatial System

### Polymorphic Usage

```rust
use csgrs::spatial::{SpatialStructureFactory, QueryType, SpatialIndex};

// Factory automatically selects R-tree for range queries
let structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::RangeQuery);

// Explicit R-tree creation
let rtree = SpatialStructureFactory::create_rtree(&polygons);

// Use polymorphically with other spatial structures
let structures: Vec<Box<dyn SpatialIndex<i32>>> = vec![
    SpatialStructureFactory::create_bsp(&polygons),
    SpatialStructureFactory::create_kdtree(&polygons), 
    SpatialStructureFactory::create_rtree(&polygons),
];
```

### Configuration Integration

```rust
use csgrs::spatial::{SpatialConfig, SpatialStructureFactory};

// Use unified spatial configuration
let config = SpatialConfig::for_range_queries();
let rtree = SpatialStructureFactory::create_rtree_with_config(&polygons, &config);

// Error handling
match SpatialStructureFactory::try_create_rtree(&polygons) {
    Ok(rtree) => println!("R-tree created successfully"),
    Err(e) => println!("Creation failed: {}", e),
}
```

## Implementation Details

### Tree Structure

- **Internal nodes**: Contain bounding boxes and child pointers
- **Leaf nodes**: Contain actual polygon data
- **Bounding boxes**: Automatically maintained and updated
- **Balance**: All leaves at same level, enforced during operations

### Split Algorithms

1. **Linear Split**: O(M) - picks seeds with maximum separation
2. **Quadratic Split**: O(M²) - considers all pairs for optimal seeds  
3. **R*-tree Split**: Advanced heuristics with forced reinsertion

### Memory Layout

Optimized for cache efficiency with contiguous storage of:
- Node metadata and bounding boxes
- Child pointers for internal nodes
- Polygon data for leaf nodes

## See Also

- [Spatial Module Overview](../README.md)
- [BSP Trees](../bsp/README.md) - For Boolean operations
- [KD-trees](../kdtree/README.md) - For point queries
- [Octrees](../octree/README.md) - For volume queries
- [Shared Utilities](../utils.rs) - Common geometric operations
