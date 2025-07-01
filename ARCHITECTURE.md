# CSGRS Architecture: The Geometric Engine Cathedral

## Overview

CSGRS is a high-performance Constructive Solid Geometry (CSG) library built in Rust, designed as a digital cathedral dedicated to computational geometry and mesh manipulation. The architecture follows Cathedral Engineering principles, emphasizing modularity, performance, and mathematical precision.

## Core Architectural Principles

### The Law of Sacred Spaces
Each module serves a distinct architectural purpose with clear boundaries and responsibilities:

- **Spatial Module**: Dedicated space for spatial data structures and algorithms
- **CSG Module**: Boolean operations and geometric transformations  
- **Geometry Module**: Fundamental geometric primitives
- **IO Module**: File format support and data interchange
- **Math Module**: Mathematical operations and algorithms

### The Law of Crystalline Composition
- **Modular Design**: Independent modules that compose cleanly
- **Feature Gates**: Optional functionality through Cargo features
- **Zero-Cost Abstractions**: High-level APIs that compile to efficient code

### The Law of Performance Sanctity
- **Optional Parallelism**: Rayon-based parallel processing for compute-intensive operations
- **Memory Efficiency**: Careful memory management and allocation strategies
- **SIMD Optimization**: Vectorized operations where applicable

## Module Architecture

### Spatial Module (`src/spatial/`)

**Purpose**: Dedicated module for spatial data structures and geometric subdivision algorithms.

**Design Philosophy**: Following the Law of Sacred Spaces, spatial structures are independent, reusable components separate from CSG operations.

```
src/spatial/
├── mod.rs              # Module declarations and re-exports
├── README.md           # Spatial module documentation
└── bsp/                # Binary Space Partitioning trees
    ├── mod.rs          # BSP module declarations
    ├── README.md       # BSP-specific documentation
    ├── core.rs         # Core Node structure and basic operations
    ├── operations.rs   # Non-parallel BSP operations
    └── parallel.rs     # Parallel BSP implementations
```

**Key Benefits**:
- **Modularity**: Spatial structures independent of CSG operations
- **Reusability**: Same structures serve multiple geometric purposes
- **Extensibility**: Clean architecture for adding new spatial algorithms (KD-trees, Octrees, R-trees)

### CSG Module (`src/csg/`)

**Purpose**: Constructive Solid Geometry operations and mesh manipulation.

**Key Components**:
- `ops.rs`: Boolean operations (union, intersection, difference, XOR)
- `mesh/`: Mesh connectivity, quality analysis, and manifold operations
- `transform.rs`: Geometric transformations and positioning
- `query.rs`: Spatial queries and geometric analysis

**Integration**: Uses spatial data structures from the spatial module for efficient Boolean operations.

### Geometry Module (`src/geometry/`)

**Purpose**: Fundamental geometric primitives and operations.

**Key Components**:
- `vertex.rs`: 3D vertices with position and normal vectors
- `polygon.rs`: Polygon representation and operations
- `plane.rs`: Plane mathematics and classification

### IO Module (`src/io/`)

**Purpose**: File format support and data interchange.

**Supported Formats**:
- STL (binary/ASCII)
- DXF (AutoCAD)
- SVG (vector graphics)
- OBJ (Wavefront)
- PLY (Stanford)
- AMF (Additive Manufacturing)

### Math Module (`src/math/`)

**Purpose**: Mathematical operations and specialized algorithms.

**Key Components**:
- `convex_hull.rs`: Convex hull computation
- `metaballs.rs`: Implicit surface generation
- `sdf.rs`: Signed distance fields
- `offset.rs`: Polygon offsetting operations

## Architectural Migration: BSP Trees

### Historical Context
Originally, BSP trees were located in `src/csg/bsp/` as part of the CSG module, tightly coupled to Boolean operations.

### Migration Rationale
Following Cathedral Engineering principles, BSP trees were migrated to `src/spatial/bsp/` to achieve:

1. **Separation of Concerns**: Spatial indexing separated from geometric operations
2. **Reusability**: BSP trees can serve purposes beyond CSG (visibility, collision detection)
3. **Modularity**: Independent spatial structures that compose cleanly
4. **Extensibility**: Foundation for additional spatial data structures

### Migration Benefits
- **Clean API**: Multiple access patterns (`csgrs::BspNode`, `csgrs::spatial::bsp::Node`)
- **Backward Compatibility**: Functionality preserved, only import paths changed
- **Future Growth**: Ready for KD-trees, Octrees, and other spatial structures

## Performance Architecture

### Parallel Processing
- **Feature Flag**: `parallel` feature enables Rayon-based parallelism
- **Automatic Selection**: Algorithms choose parallel/sequential based on input size
- **Thresholds**: Optimized switching points for different operations

### Memory Management
- **Lazy Initialization**: Child nodes created only when needed
- **Pre-allocation**: Vectors sized based on input estimates
- **Iterator Patterns**: Efficient processing without unnecessary copies

### SIMD Optimization
- **Vectorized Operations**: Geometric calculations use SIMD where possible
- **Platform Specific**: Optimizations for different CPU architectures
- **Fallback Support**: Scalar implementations for unsupported platforms

## API Design

### Public Interface
```rust
// Crate-level re-exports for convenience
use csgrs::{CSG, BspNode, Vertex};

// Direct module access for specific needs
use csgrs::spatial::bsp::Node;
use csgrs::geometry::Polygon;
```

### Feature Gates
```toml
[dependencies]
csgrs = { version = "0.19", features = ["parallel", "stl-io", "svg-io"] }
```

### Error Handling
- **Result Types**: Comprehensive error handling with descriptive messages
- **Validation**: Input validation with clear error reporting
- **Recovery**: Graceful handling of degenerate cases

## Testing Strategy

### Unit Tests
- **Mathematical Precision**: Geometric operations verified with known results
- **Edge Cases**: Degenerate inputs and boundary conditions
- **Property-Based**: Geometric invariants verified across input domains

### Integration Tests
- **Cross-Module**: Functionality validated across module boundaries
- **File Format**: Round-trip testing for all supported formats
- **Performance**: Regression testing for computational efficiency

### Documentation Tests
- **Example Validation**: All documentation examples compile and run
- **API Coverage**: Public interfaces documented with working examples

## Future Architecture

### Planned Spatial Structures
- **KD-Trees**: For nearest neighbor searches and range queries
- **Octrees**: For hierarchical 3D space subdivision
- **R-Trees**: For bounding box queries and spatial indexing
- **Spatial Hashing**: For fast collision detection

### Extensibility Points
- **Plugin Architecture**: Support for custom spatial algorithms
- **Trait System**: Common interfaces for spatial data structures
- **Feature Composition**: Mix and match capabilities through feature flags

## Quality Assurance

### Code Quality
- **Clippy Lints**: Comprehensive static analysis
- **Format Consistency**: Automated code formatting
- **Documentation**: Every public interface documented with examples

### Performance Monitoring
- **Benchmarks**: Continuous performance regression testing
- **Profiling**: Regular performance analysis and optimization
- **Memory Usage**: Monitoring for memory leaks and excessive allocation

### Architectural Integrity
- **Module Boundaries**: Clear separation of concerns enforced
- **Dependency Management**: Minimal and well-justified dependencies
- **API Stability**: Semantic versioning and deprecation policies

## Conclusion

The CSGRS architecture represents a carefully designed system that balances performance, modularity, and extensibility. The migration of BSP trees to the spatial module exemplifies the Cathedral Engineering approach, creating a foundation for future growth while maintaining the mathematical precision and performance characteristics that define the library.
