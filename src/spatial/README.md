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

### Future Spatial Structures

The spatial module is designed to accommodate additional spatial data structures:

- **KD-Trees**: For nearest neighbor searches and range queries
- **Octrees**: For hierarchical 3D space subdivision
- **R-Trees**: For bounding box queries and spatial indexing
- **Spatial Hashing**: For fast collision detection

## Usage Patterns

### Direct Module Access
```rust
use csgrs::spatial::bsp::Node;
let bsp_node = Node::new();
```

### Convenient Re-exports
```rust
use csgrs::spatial::BspNode;
let bsp_node = BspNode::new();
```

### Crate-level Re-exports
```rust
use csgrs::BspNode;
let bsp_node = BspNode::new();
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

## See Also

- [`bsp/README.md`](bsp/README.md) - Detailed BSP tree documentation
- [`ARCHITECTURE.md`](../../ARCHITECTURE.md) - Overall codebase architecture
- [`docs/adr/001-bsp-spatial-module-migration.md`](../../docs/adr/001-bsp-spatial-module-migration.md) - Migration decision record
