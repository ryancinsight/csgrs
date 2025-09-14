# Product Requirements Document (PRD)

## Executive Summary
csgrs is a high-performance Rust library for Constructive Solid Geometry (CSG) operations, designed to provide robust geometric processing capabilities for CAD, CNC, gaming, and scientific applications. The library emphasizes mathematical correctness, performance, and seamless integration with the Rust ecosystem.

## Product Vision
To be the premier Rust library for geometric modeling, providing mathematically rigorous CSG operations that scale from embedded systems to high-performance computing environments.

## Target Audience
1. **Rust Developers**: Seeking type-safe, zero-cost geometric operations
2. **CAD/CAM Developers**: Needing robust boolean operations and file I/O
3. **Game Developers**: Requiring mesh processing and physics integration
4. **Scientific Computing**: Demanding precise geometric algorithms
5. **Educational Users**: Learning geometric processing and CSG concepts

## Core Value Propositions
- **Performance**: Sub-second boolean operations on thousands of polygons
- **Correctness**: Mathematically rigorous algorithms with robust predicates
- **Integration**: Seamless compatibility with Rust geometry ecosystem
- **Flexibility**: Generic metadata system and extensible architecture
- **Reliability**: Comprehensive error handling and edge case management

## Requirements

### Functional Requirements

#### Boolean Operations
- **Union**: Combine multiple geometric objects
- **Difference**: Subtract one object from another
- **Intersection**: Find overlapping regions
- **XOR**: Symmetric difference between objects
- **All operations must handle coplanar faces and degenerate cases**

#### Geometric Primitives
- **3D Primitives**: Cube, sphere, cylinder, torus, polyhedron
- **2D Shapes**: Circle, rectangle, polygon, text, airfoil profiles
- **Complex Shapes**: Metaballs, TPMS surfaces, NURBS curves

#### Transformations
- **Basic**: Translation, rotation, scaling
- **Advanced**: Mirroring, shearing, matrix transformations
- **Composition**: Efficient transformation pipelines

#### File Format Support
- **Primary**: STL (ASCII/Binary), OBJ
- **Secondary**: DXF, PLY, AMF, SVG
- **Text**: TrueType fonts, Hershey fonts

#### Mesh Processing
- **Quality Analysis**: Triangle quality metrics, aspect ratios
- **Optimization**: Smoothing, refinement, decimation
- **Repair**: Hole filling, manifold correction
- **Connectivity**: Adjacency finding, topological queries, mesh traversal

#### IndexedMesh Processing
- **Vertex Deduplication**: Automatic elimination of redundant vertices
- **Face Indexing**: Efficient face representation using vertex indices
- **Connectivity Queries**: Fast adjacency finding and neighbor enumeration
- **Topology Analysis**: Manifold detection, boundary extraction, component separation
- **Memory Optimization**: Reduced memory footprint through index-based storage

### Non-Functional Requirements

#### Performance
- **Boolean Operations**: O(n log n) scaling to 10,000+ polygons
- **IndexedMesh Operations**: O(n log n) scaling to 100,000+ vertices
- **Memory Efficiency**: < 10x input size (Mesh), < 3x input size (IndexedMesh)
- **Connectivity Queries**: O(1) amortized cost for adjacency queries
- **Parallel Processing**: Automatic multi-threading for large datasets
- **Compile Time**: Minimal impact on build times

#### Accuracy & Robustness
- **Geometric Precision**: Configurable epsilon (1e-8 for f64, 1e-4 for f32)
- **Numerical Stability**: Robust predicates for orientation and intersection
- **Error Handling**: Comprehensive error types and recovery
- **Edge Cases**: Handle degenerate geometry gracefully

#### Developer Experience
- **API Design**: Ergonomic, chainable method calls
- **Documentation**: Comprehensive examples and mathematical foundations
- **Testing**: > 80% coverage with property-based testing
- **Type Safety**: Generic metadata system with compile-time guarantees

#### Compatibility
- **Rust Versions**: Support latest stable Rust (currently 1.85.1+)
- **Platforms**: x86_64, ARM64, WASM compilation targets
- **Ecosystem**: Integration with nalgebra, parry, geo, bevy
- **Dependencies**: Minimal, optional feature gating

## Technical Architecture

### Core Components
1. **Mesh System**: BSP tree-based boolean operations with polygon-based storage
2. **IndexedMesh System**: Vertex-indexed mesh representation for memory efficiency and connectivity
3. **Sketch System**: 2D geometry with extrusion capabilities
4. **NURBS System**: Advanced curve and surface representation
5. **Voxel System**: Volume-based processing
6. **I/O System**: Multi-format file import/export with indexed mesh support

### Key Technologies
- **BSP Trees**: Efficient spatial partitioning for boolean operations
- **Indexed Meshes**: Vertex deduplication and face indexing for memory efficiency
- **Geometric Predicates**: Robust floating-point computations
- **Connectivity Analysis**: Adjacency finding and topological queries
- **Rayon**: Optional parallel processing
- **nalgebra**: High-performance linear algebra
- **parry**: Collision detection and spatial queries

## Success Metrics

### Performance Benchmarks
- Boolean operations on 1,000 polygons: < 100ms
- IndexedMesh boolean operations on 10,000 vertices: < 200ms
- Memory usage: < 5x input size (Mesh), < 2x input size (IndexedMesh)
- File I/O: < 1 second for 10MB models
- IndexedMesh connectivity queries: < 10ms for 1,000 vertices
- Vertex deduplication efficiency: > 80% reduction in redundant vertices

### Quality Metrics
- Test coverage: > 85%
- Documentation completeness: 100% public API
- Clippy/clean code: Zero warnings
- Memory safety: No unsafe code

### Adoption Metrics
- Crates.io downloads: 1,000+ monthly
- GitHub stars: 500+
- Dependent crates: 50+
- Production usage in commercial projects

## Risk Assessment

### Technical Risks
- **Floating-point precision**: Mitigated by robust predicates and epsilon handling
- **Performance scaling**: Addressed through BSP optimization and parallel processing
- **Memory usage**: Managed through efficient data structures and streaming

### Market Risks
- **Competition**: Differentiated by Rust focus and mathematical rigor
- **Adoption**: Addressed through comprehensive documentation and examples
- **Maintenance**: Managed through automated testing and clear architecture

## Implementation Roadmap

### Phase 1: Core Stability (Completed)
- [x] Basic boolean operations
- [x] Geometric primitives
- [x] File I/O (STL, OBJ)
- [x] Basic transformations
- [x] Documentation and examples
- [x] Performance optimizations

### Phase 2: IndexedMesh Architecture (Next 3 months)
- [ ] IndexedMesh data structures and API
- [ ] Vertex deduplication algorithms
- [ ] Connectivity analysis and adjacency queries
- [ ] IndexedMesh boolean operations
- [ ] File I/O support for indexed meshes (STL, OBJ, PLY)
- [ ] Memory optimization and performance benchmarking
- [ ] IndexedMesh trait implementations (CSG, transformations)
- [ ] Interoperability between Mesh and IndexedMesh

### Phase 3: Advanced Features (3-6 months)
- [ ] Mesh quality analysis and optimization
- [ ] Advanced NURBS support
- [ ] GPU acceleration for indexed meshes
- [ ] Streaming processing for large models
- [ ] Topology repair and manifold correction

### Phase 4: Ecosystem Integration (6-9 months)
- [ ] Bevy integration with IndexedMesh support
- [ ] Physics engine plugins (Rapier integration)
- [ ] WebAssembly support and web-based visualization
- [ ] Advanced file format support (FBX, glTF)
- [ ] Procedural generation and mesh synthesis

## Dependencies & Constraints

### External Dependencies
- **nalgebra**: Linear algebra operations
- **parry3d**: Collision detection
- **geo**: 2D geometry processing
- **rayon**: Parallel processing (optional)

### Platform Constraints
- **Minimum Rust**: 1.85.1
- **Supported Targets**: x86_64, ARM64, WASM32
- **Memory**: 64MB minimum for typical operations

### Licensing
- **License**: MIT
- **Compatibility**: Permissive for commercial use
- **Attribution**: Credit for original CSG.js inspiration

## Support & Maintenance

### Documentation
- **API Documentation**: Generated from code comments
- **User Guide**: Comprehensive examples and tutorials
- **Mathematical Foundations**: Detailed algorithm explanations

### Testing Strategy
- **Unit Tests**: Core algorithm correctness
- **Integration Tests**: File I/O and ecosystem compatibility
- **Performance Tests**: Benchmarking and regression detection
- **Property Tests**: Edge case and invariant validation

### Community Engagement
- **GitHub Issues**: Bug tracking and feature requests
- **Discord Community**: User support and discussion
- **Contributing Guide**: Clear path for external contributions
- **Roadmap Transparency**: Public development planning
