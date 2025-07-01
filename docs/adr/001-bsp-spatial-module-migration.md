# ADR-001: BSP Tree Migration to Spatial Module

## Status
**Accepted** - Implemented on 2025-06-30

## Context

The Binary Space Partitioning (BSP) tree implementation was originally located in `src/csg/bsp/` as part of the CSG (Constructive Solid Geometry) module. While BSP trees are heavily used for CSG Boolean operations, this tight coupling created architectural limitations and violated the Cathedral Engineering principle of "The Law of Sacred Spaces."

### Problems with Original Architecture

1. **Tight Coupling**: BSP trees were conceptually bound to CSG operations, limiting their reusability
2. **Single Responsibility Violation**: The CSG module contained both geometric operations AND spatial data structures
3. **Limited Extensibility**: No clear path for adding other spatial data structures (KD-trees, Octrees, etc.)
4. **Architectural Inconsistency**: Spatial algorithms mixed with geometric operations

### Cathedral Engineering Principles

The existing architecture violated key Cathedral Engineering principles:
- **The Law of Sacred Spaces**: Each module should have a single, well-defined architectural purpose
- **The Law of Crystalline Composition**: Modules should compose cleanly without tight coupling
- **The Law of Extensible Devotion**: Architecture should accommodate future growth

## Decision

**We will migrate BSP trees from `src/csg/bsp/` to `src/spatial/bsp/` and create a dedicated spatial module for all spatial data structures.**

### Migration Scope

1. **File Migration**: Move all BSP-related files to the new spatial module
2. **Import Path Updates**: Update all references from `csg::bsp` to `spatial::bsp`
3. **Module Declarations**: Remove BSP from CSG module, add to spatial module
4. **API Enhancement**: Provide convenient re-exports for backward compatibility
5. **Documentation**: Comprehensive documentation reflecting the new architecture

### New Module Structure

```
src/spatial/
├── mod.rs              # Spatial module declarations and re-exports
├── README.md           # Spatial module documentation
└── bsp/                # Binary Space Partitioning trees
    ├── mod.rs          # BSP module declarations
    ├── README.md       # BSP-specific documentation
    ├── core.rs         # Core Node structure and basic operations
    ├── operations.rs   # Non-parallel BSP operations
    └── parallel.rs     # Parallel BSP implementations
```

## Rationale

### Architectural Benefits

1. **Separation of Concerns**: Spatial indexing is now separate from geometric operations
2. **Reusability**: BSP trees can serve multiple purposes beyond CSG (visibility, collision detection, etc.)
3. **Modularity**: Spatial structures are independent, composable components
4. **Extensibility**: Clear foundation for adding KD-trees, Octrees, R-trees, and other spatial structures
5. **API Clarity**: Multiple access patterns for different use cases

### Performance Considerations

- **No Performance Impact**: Migration is purely organizational, no algorithmic changes
- **Compilation**: All existing functionality preserved with identical performance characteristics
- **Memory Usage**: No changes to memory layout or allocation patterns

### Backward Compatibility

- **Functional Compatibility**: All BSP functionality preserved exactly
- **API Access**: Multiple convenient access patterns provided
- **Import Paths**: Only import statements need updating, no API changes

## Implementation

### Migration Process

The migration was executed in systematic phases:

1. **Phase 1: Pre-Migration Setup**
   - Created `src/spatial/` module structure
   - Added spatial module declaration to `src/lib.rs`

2. **Phase 2: File Migration**
   - Moved all files from `src/csg/bsp/` to `src/spatial/bsp/`
   - Preserved all content exactly (no modifications during migration)

3. **Phase 3: Import Path Updates**
   - Updated `src/csg/ops.rs`: `csg::bsp::Node` → `spatial::bsp::Node`
   - Updated `src/utils/flatten_slice.rs`: `csg::bsp::Node` → `spatial::bsp::Node`
   - Updated `src/tests.rs`: `csg::bsp::Node` → `spatial::bsp::Node`
   - Updated documentation links in `src/lib.rs`
   - Removed BSP module declaration from `src/csg/mod.rs`

4. **Phase 4: Module Declaration Updates**
   - Added convenient re-exports in `src/lib.rs`: `pub use spatial::bsp::Node as BspNode;`
   - Verified module accessibility and compilation

5. **Phase 5: Documentation Updates**
   - Created comprehensive `src/spatial/README.md`
   - Created detailed `src/spatial/bsp/README.md`
   - Created `ARCHITECTURE.md` for overall project architecture
   - Created this ADR documenting the decision

6. **Phase 6: Validation and Testing**
   - Verified compilation success
   - Confirmed all tests pass (98 passed, 1 ignored)
   - Validated API accessibility through multiple paths

### API Access Patterns

After migration, BSP functionality is accessible through multiple convenient patterns:

```rust
// Crate-level re-export (most convenient)
use csgrs::BspNode;
let bsp = BspNode::new();

// Spatial module re-export
use csgrs::spatial::BspNode;
let bsp = BspNode::new();

// Direct module access (most explicit)
use csgrs::spatial::bsp::Node;
let bsp = Node::new();
```

## Consequences

### Positive Consequences

1. **Architectural Clarity**: Clear separation between spatial indexing and geometric operations
2. **Future Extensibility**: Foundation ready for additional spatial data structures
3. **Code Organization**: Logical grouping of related functionality
4. **Reusability**: BSP trees can be used independently of CSG operations
5. **Documentation**: Comprehensive documentation reflecting the new architecture

### Neutral Consequences

1. **Import Path Changes**: Existing code needs import statement updates
2. **Learning Curve**: Developers need to understand the new module organization
3. **Documentation Updates**: All references to old paths need updating

### Negative Consequences

1. **Migration Effort**: One-time cost of updating import paths
2. **Temporary Disruption**: Brief period where documentation might be inconsistent

### Risk Mitigation

- **Comprehensive Testing**: All tests pass, ensuring functional preservation
- **Multiple Access Patterns**: Convenient re-exports minimize migration friction
- **Documentation**: Clear migration guidance and examples provided
- **Validation**: Systematic verification of all changes

## Future Implications

### Planned Spatial Structures

The new spatial module architecture enables future additions:

- **KD-Trees**: For nearest neighbor searches and range queries
- **Octrees**: For hierarchical 3D space subdivision  
- **R-Trees**: For bounding box queries and spatial indexing
- **Spatial Hashing**: For fast collision detection

### Architectural Evolution

This migration establishes patterns for future architectural decisions:

- **Module Boundaries**: Clear criteria for module organization
- **Migration Process**: Systematic approach for future refactoring
- **Documentation Standards**: Comprehensive documentation for architectural changes

## Alternatives Considered

### Alternative 1: Keep BSP in CSG Module
**Rejected**: Would perpetuate architectural problems and limit future extensibility.

### Alternative 2: Create Generic "Algorithms" Module
**Rejected**: Too broad and would violate single responsibility principle.

### Alternative 3: Separate Crate for Spatial Structures
**Rejected**: Unnecessary complexity for current needs, can be reconsidered if spatial module grows significantly.

## References

- [Cathedral Engineering Principles](../ARCHITECTURE.md)
- [Spatial Module Documentation](../../src/spatial/README.md)
- [BSP Tree Documentation](../../src/spatial/bsp/README.md)
- [Binary Space Partitioning (Wikipedia)](https://en.wikipedia.org/wiki/Binary_space_partitioning)

## Approval

- **Architect**: Ryan Clanton
- **Date**: 2025-06-30
- **Implementation**: Complete
- **Status**: Accepted and Implemented
