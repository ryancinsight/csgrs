# Product Requirements Document: Sparse Voxel Octree-Embedded BSP Trees

## Executive Summary

This document outlines the requirements for implementing **Sparse Voxel Octree (SVO)**-embedded BSP trees within the `voxels` module to provide a complete alternative to the traditional `Mesh` module. The implementation maintains full compatibility with the existing `CSG` trait while providing enhanced spatial efficiency and performance through hierarchical sparse voxel representation.

**CURRENT STATUS: 75% COMPLETE** - Core SVO-BSP architecture implemented and functional. Remaining work focuses on advanced mesh processing capabilities for complete feature parity.

### Key Distinction: Sparse vs Dense Octrees

Unlike the dense octree approach described in the foundational research <mcreference link="https://arxiv.org/abs/2103.02486" index="1">1</mcreference>, this implementation uses **sparse voxel octrees** where:
- **Only occupied voxels are stored in memory** (not empty space)
- **Memory usage scales with geometric complexity**, not spatial volume
- **Empty regions are implicitly represented** through absence of nodes
- **Adaptive subdivision** occurs only where geometry exists

This sparse approach provides significant advantages for complex geometries with substantial empty space, common in CAD and manufacturing applications, while maintaining the proven octree-embedded BSP architecture for robust CSG operations.

## Gap Analysis: SvoMesh vs Mesh Capabilities

### ✅ IMPLEMENTED FEATURES (Core SVO-BSP)
- **CSG Operations**: All boolean operations (union, difference, intersection, xor)
- **Transformations**: Complete transformation suite (translate, rotate, scale, mirror)
- **SDF Meshing**: Signed distance field surface extraction with octree optimization
- **TPMS Generation**: Triply periodic minimal surfaces with mathematical precision
- **Basic Manifold Operations**: Manifold checking and basic healing
- **Conversion Layer**: Bidirectional conversion to/from traditional Mesh
- **Performance Optimization**: Parallel processing, fixed-precision arithmetic
- **Spatial Queries**: Basic ray casting and containment testing

### ❌ MISSING FEATURES (Advanced Mesh Processing)
- **Shape Generation**: No direct primitive creation (cube, sphere, cylinder, etc.)
- **Smoothing Operations**: No Laplacian, Taubin, or bilateral smoothing
- **Quality Analysis**: No triangle quality metrics or mesh quality assessment
- **Connectivity Analysis**: No vertex adjacency or edge analysis capabilities
- **Convex Hull Operations**: No convex hull generation or Minkowski sum
- **Metaballs**: No implicit surface generation from metaball fields
- **Export/Import Integration**: No direct file format support (STL, OBJ, PLY, AMF)
- **Physics Integration**: No mass properties calculation or collision detection
- **Advanced Manifold Operations**: Limited manifold repair and topology validation
- **Mesh Simplification**: No octree-guided decimation or vertex clustering

## Design Principles

This implementation adheres to the following design principles:
- **SSOT (Single Source of Truth)**: Unified data structures and APIs
- **SOLID**: Single responsibility, open/closed, Liskov substitution, interface segregation, dependency inversion
- **CUPID**: Composable, Unix philosophy, predictable, idiomatic, domain-centric
- **GRASP**: General responsibility assignment software patterns
- **ACID**: Atomicity, consistency, isolation, durability for operations
- **KISS**: Keep it simple, stupid
- **DRY**: Don't repeat yourself
- **CLEAN**: Clear, logical, efficient, actionable, nested
- **YAGNI**: You aren't gonna need it
- **ADP**: Acyclic dependencies principle
- **DIP**: Dependency inversion principle

## Implementation Priority Analysis

### CRITICAL GAPS (Immediate Priority)

**Shape Generation (Phase 6 - High Priority)**
- Direct primitive creation without Mesh conversion overhead
- Adaptive voxel subdivision for optimal memory usage
- Essential for practical SvoMesh adoption

**Export/Import Integration (New Phase)**
- Direct file format support (STL, OBJ, PLY, AMF) for SvoMesh
- Avoid conversion overhead for large sparse meshes
- Critical for workflow integration

**Basic Smoothing Operations (Phase 5 - High Priority)**
- Laplacian smoothing leveraging octree spatial locality
- Essential for mesh processing workflows
- Performance advantage through sparse representation

### IMPORTANT GAPS (Medium Priority)

**Quality Analysis (Phase 5)**
- Triangle quality metrics with voxel-aware optimization
- Mesh quality assessment using spatial coherence
- Sub-linear performance through octree structure

**Connectivity Analysis (Phase 7)**
- Vertex adjacency analysis with octree optimization
- Edge-based connectivity queries
- Foundation for advanced topology operations

### ENHANCEMENT GAPS (Lower Priority)

**Advanced Operations**
- Convex hull with octree acceleration
- Metaballs using sparse voxel sampling
- Physics integration (mass properties, collision detection)
- Advanced manifold repair and topology validation

## Current State Analysis

### ✅ COMPLETED ARCHITECTURE (75% Implementation)

**Core SVO-BSP Foundation**
- Unified BSP implementation with feature-gated parallelism
- Sparse voxel octree with embedded BSP trees
- Complete CSG trait implementation
- Bidirectional Mesh conversion
- Fixed-precision arithmetic for robustness
- Parallel processing optimization

**Research Foundation Implementation**
Successfully implements proven techniques from academic research:
- **Plane-based geometry representation** with integer arithmetic ✅
- **Octree as global acceleration structure** for local modifications ✅
- **Mesh cutting algorithms** as foundation for operations ✅
- **Fixed-precision operations** for performance optimization ✅
- **Persistent data structure** design for iterated CSG workflows ✅

## Requirements

### Functional Requirements

#### FR1: Sparse Voxel Octree-BSP Hybrid Structure
- **FR1.1**: Implement `SvoNode<S>` that embeds BSP tree functionality within sparse octree nodes
- **FR1.2**: Support adaptive subdivision **only where geometry exists** (sparse allocation)
- **FR1.3**: Maintain spatial coherence through hierarchical sparse voxel organization
- **FR1.4**: Enable **implicit empty space representation** through node absence
- **FR1.5**: Optimize memory usage by storing only occupied voxels

#### FR2: CSG Trait Compatibility
- **FR2.1**: Implement all CSG trait methods for `SvoMesh<S>`
- **FR2.2**: Maintain identical function signatures and behavior
- **FR2.3**: Support bidirectional conversion between `Mesh<S>` and `SvoMesh<S>`
- **FR2.4**: Preserve metadata throughout all operations

#### FR3: BSP Integration Strategy
- **FR3.1**: **Shared BSP Logic**: Unify `bsp.rs` and `bsp_parallel.rs` into a single implementation
- **FR3.2**: **Feature-Gated Parallelism**: Use `#[cfg(feature = "parallel")]` for parallel operations
- **FR3.3**: **Embedded BSP Nodes**: Each SVO node contains a BSP subtree for local geometry
- **FR3.4**: **Hierarchical Splitting**: Combine octree spatial splitting with BSP plane-based splitting

#### FR4: Performance Optimization
- **FR4.1**: Implement lazy evaluation for expensive operations
- **FR4.2**: Support parallel processing where beneficial
- **FR4.3**: Optimize memory usage through sparse representation
- **FR4.4**: Enable early termination for boolean operations

#### FR5: API Consistency
- **FR5.1**: Maintain existing function names and signatures
- **FR5.2**: Support in-place modifications where appropriate
- **FR5.3**: Provide seamless integration with existing codebase
- **FR5.4**: Ensure zero-cost abstractions where possible

#### FR6: Advanced Mesh Processing
- **FR6.1**: Implement smoothing operations (Laplacian, Taubin, Bilateral) using sparse voxel optimization
- **FR6.2**: Provide comprehensive mesh quality analysis with voxel-aware metrics
- **FR6.3**: Support adaptive mesh refinement using octree subdivision
- **FR6.4**: Implement manifold validation and repair operations
- **FR6.5**: Add mesh healing for degenerate geometry

#### FR7: Geometric Primitives
- **FR7.1**: Generate primitive shapes (spheres, gears) with adaptive voxel subdivision
- **FR7.2**: Implement metaballs and implicit surface generation using sparse voxel sampling
- **FR7.3**: Support SDF-based surface generation with octree acceleration
- **FR7.4**: Provide convex hull operations with spatial optimization
- **FR7.5**: Implement TPMS generation for advanced geometric structures

#### FR8: Topology Analysis
- **FR8.1**: Build connectivity analysis using sparse voxel structure
- **FR8.2**: Implement vertex adjacency analysis with octree optimization
- **FR8.3**: Support mesh simplification using octree-guided decimation
- **FR8.4**: Provide spatial queries (ray intersection, nearest neighbor) with octree acceleration
- **FR8.5**: Implement collision detection between SvoMesh objects

### Non-Functional Requirements

#### NFR1: Performance (Sparse Voxel Octree Advantages)
- **NFR1.1**: Boolean operations should scale better than O(n²) for complex meshes
- **NFR1.2**: **Memory usage proportional to geometric complexity, not spatial volume** (key sparse advantage)
- **NFR1.3**: Spatial queries should achieve O(log n) average case performance
- **NFR1.4**: **Automatic empty space culling** with zero memory overhead for unoccupied regions
- **NFR1.5**: Target performance of 1+ million mesh-plane cuts per second (research baseline: 2.5M cuts/sec <mcreference link="https://arxiv.org/abs/2103.02486" index="1">1</mcreference>)
- **NFR1.6**: **Superior performance for sparse geometries** compared to dense octree approaches
- **NFR1.7**: Support efficient iterated CSG operations for manufacturing simulation workflows

#### NFR2: Maintainability
- **NFR2.1**: Code should follow Rust idioms and best practices
- **NFR2.2**: Implementation should be well-documented with examples
- **NFR2.3**: Unit tests should cover all public APIs

#### NFR3: Compatibility
- **NFR3.1**: Must compile with existing feature flags
- **NFR3.2**: Should maintain backward compatibility with existing `Mesh<S>` usage
- **NFR3.3**: Must support all existing metadata types

#### NFR4: Advanced Processing Performance
- **NFR4.1**: Smoothing operations should achieve O(log n) complexity using octree locality
- **NFR4.2**: Quality analysis should be sub-linear using spatial coherence
- **NFR4.3**: Shape generation should use adaptive voxel subdivision for optimal memory usage
- **NFR4.4**: Topology analysis should leverage sparse representation for efficiency
- **NFR4.5**: Spatial queries should achieve O(log n) average case using octree acceleration

## Technical Architecture

### Core Data Structures

Based on the octree-embedded BSP research approach <mcreference link="https://arxiv.org/abs/2103.02486" index="1">1</mcreference>:

```rust
/// Sparse Voxel Octree node with embedded BSP functionality
/// Key difference from dense octrees: only stores nodes where geometry exists
pub struct SvoNode<S: Clone + Send + Sync + Debug> {
    /// Octree spatial bounds for this sparse voxel
    pub bounds: Aabb,
    
    /// Local BSP tree for geometry within this sparse voxel cell
    /// Only present when this voxel contains actual geometry
    pub bsp: Option<Node<S>>,
    
    /// Eight octree children (SPARSE: None = empty space)
    /// Memory is allocated only for children containing geometry
    /// Empty regions are implicitly represented by None values
    pub children: [Option<Box<SvoNode<S>>>; 8],
    
    /// Node-level metadata (only for occupied voxels)
    pub metadata: Option<S>,
    
    /// Fixed-precision arithmetic support for robust operations
    pub precision_scale: Option<i32>,
}

/// Sparse Voxel Octree mesh with embedded BSP for iterated CSG
/// Memory usage scales with geometric complexity, not spatial volume
pub struct SvoMesh<S: Clone + Send + Sync + Debug> {
    /// Root SVO node containing the sparse octree-embedded BSP structure
    /// Only subdivides and allocates memory where geometry exists
    pub root: Option<SvoNode<S>>, // None = completely empty mesh
    
    /// Global bounding box cache (computed from occupied voxels only)
    pub bounding_box: OnceLock<Aabb>,
    
    /// Mesh-level metadata
    pub metadata: Option<S>,
    
    /// Global precision settings for fixed-point arithmetic
    pub precision_config: PrecisionConfig,
}

/// Configuration for fixed-precision arithmetic
pub struct PrecisionConfig {
    pub scale_factor: i32,
    pub epsilon_scaled: i64,
}
```

### BSP Unification Strategy

Following research-proven techniques <mcreference link="https://arxiv.org/abs/2103.02486" index="1">1</mcreference>:

1. **Conditional Compilation**: Use feature gates to select implementation
2. **Trait-Based Design**: Abstract common BSP operations
3. **Iterator-Based Processing**: Leverage Rust's iterator combinators
4. **Zero-Copy Operations**: Minimize data movement during operations
5. **Mesh Cutting Foundation**: Formulate BSP operations and mesh extraction in terms of efficient mesh cutting
6. **Local BSP Construction**: Build BSPs locally within octree cells using small triangle sets
7. **Fixed-Precision Arithmetic**: Implement custom fixed-precision operations for robustness

### Integration Points

1. **Conversion Layer**: Bidirectional conversion between `Mesh<S>` and `SvoMesh<S>`
2. **CSG Operations**: Implement boolean operations using octree-embedded BSP approach
3. **Spatial Queries**: Leverage octree structure for efficient spatial operations
4. **Rendering Pipeline**: Maintain compatibility with existing output formats
5. **Iterated CSG Support**: Persistent data structure for manufacturing simulation workflows
6. **Mesh Import/Export**: Fast mesh import with octree-first construction followed by local BSP building

## Implementation Plan

### Phase 1: Foundation (Week 1-2)
- [ ] Create `SvoNode<S>` and `SvoMesh<S>` structures
- [ ] Implement basic octree operations (subdivision, traversal)
- [ ] Unify BSP implementations with feature gates
- [ ] Create conversion functions between `Mesh<S>` and `SvoMesh<S>`

### Phase 2: CSG Implementation (Week 3-4)
- [ ] Implement CSG trait for `SvoMesh<S>`
- [ ] Develop SVO-BSP hybrid boolean operations
- [ ] Add transformation and spatial operation support
- [ ] Implement bounding box calculation and caching

### Phase 3: Optimization (Week 5-6)
- [ ] Add parallel processing support
- [ ] Implement lazy evaluation strategies
- [ ] Optimize memory usage and spatial queries
- [ ] Performance testing and benchmarking

### Phase 4: Integration (Week 7-8)
- [ ] Comprehensive testing with existing codebase
- [ ] Documentation and examples
- [ ] Code review and refinement
- [ ] Final integration and validation

### Phase 5: Advanced Mesh Processing (REMAINING - High Priority)
- [ ] **CRITICAL**: Implement `SvoMesh::laplacian_smooth()` leveraging octree spatial locality
- [ ] **CRITICAL**: Implement `SvoMesh::analyze_triangle_quality()` with voxel-aware metrics
- [ ] **CRITICAL**: Implement `SvoMesh::compute_mesh_quality()` for comprehensive assessment
- [ ] Implement `SvoMesh::taubin_smooth()` with feature preservation
- [ ] Implement `SvoMesh::bilateral_smooth()` for edge-preserving smoothing
- [ ] Add adaptive mesh refinement using octree subdivision
- [ ] Enhance manifold validation and repair operations
- [ ] Implement advanced mesh healing for degenerate geometry

### Phase 6: Geometric Primitives (REMAINING - Critical Priority)
- [ ] **CRITICAL**: Implement `SvoMesh::cube()`, `SvoMesh::sphere()`, `SvoMesh::cylinder()` with adaptive subdivision
- [ ] **CRITICAL**: Implement `SvoMesh::metaballs()` using sparse voxel sampling
- [ ] **CRITICAL**: Implement `SvoMesh::convex_hull()` with octree acceleration
- [ ] Add complete primitive shape generation suite (frustum, polyhedron, etc.)
- [ ] Enhance SDF-based surface generation with octree optimization
- [ ] Implement Minkowski sum operations with spatial optimization
- [ ] Enhance TPMS generation for advanced geometric structures

### Phase 7: Topology Analysis (REMAINING - Medium Priority)
- [ ] **IMPORTANT**: Implement `SvoMesh::build_connectivity()` using sparse voxel structure
- [ ] **IMPORTANT**: Add vertex adjacency analysis with octree optimization
- [ ] Implement mesh simplification using octree-guided decimation
- [ ] Add advanced spatial queries with octree acceleration
- [ ] Implement collision detection between SvoMesh objects
- [ ] Add boundary detection and analysis
- [ ] Implement mesh repair and hole filling

### Phase 8: Export/Import Integration (NEW - Critical Priority)
- [ ] **CRITICAL**: Implement Export trait for SvoMesh (STL, OBJ, PLY, AMF)
- [ ] **CRITICAL**: Implement Import trait for SvoMesh
- [ ] Add direct file format support without Mesh conversion
- [ ] Optimize export for sparse voxel structure
- [ ] Add format-specific optimizations for large sparse meshes

## Risk Assessment

### Technical Risks
1. **Complexity**: SVO-BSP hybrid may introduce implementation complexity
   - *Mitigation*: Incremental development with extensive testing

2. **Performance**: Initial implementation may not meet performance goals
   - *Mitigation*: Profile-guided optimization and iterative improvement

3. **Memory Usage**: Octree structure may increase memory overhead
   - *Mitigation*: Sparse representation and memory pooling

### Integration Risks
1. **API Compatibility**: Changes may break existing code
   - *Mitigation*: Maintain strict API compatibility and comprehensive testing

2. **Feature Parity**: SVO implementation may miss edge cases
   - *Mitigation*: Extensive test coverage and validation against existing implementation

## Success Criteria

1. **Functional**: All CSG operations work correctly with SVO-based implementation
2. **Performance**: Boolean operations show measurable improvement for complex meshes
3. **Compatibility**: Existing code continues to work without modification
4. **Maintainability**: Code follows established design principles and is well-documented
5. **Integration**: Seamless conversion between `Mesh<S>` and `SvoMesh<S>`

## Dependencies

### Internal Dependencies
- `mesh::bsp` and `mesh::bsp_parallel` modules
- `mesh::polygon`, `mesh::vertex`, `mesh::plane` modules
- `traits::CSG` trait definition
- `float_types` module for numeric types

### External Dependencies
- `nalgebra` for linear algebra operations
- `parry3d` for bounding volume calculations
- `rayon` for parallel processing (feature-gated)

## Conclusion

This PRD outlines a comprehensive approach to implementing sparse voxel octree-embedded BSP trees while maintaining full compatibility with the existing CSG system. The design emphasizes clean architecture, performance optimization, and adherence to established design principles. The phased implementation approach ensures manageable development while minimizing integration risks.