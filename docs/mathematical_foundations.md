# Mathematical Foundations of CSG Operations

This document provides the mathematical foundations and algorithmic principles underlying the Constructive Solid Geometry (CSG) operations in `csgrs`. It covers the theoretical basis for boolean operations, geometric representations, and numerical algorithms used throughout the library.

## Table of Contents

1. [Geometric Representations](#geometric-representations)
2. [Boolean Operations](#boolean-operations)
3. [BSP Tree Algorithms](#bsp-tree-algorithms)
4. [Numerical Stability](#numerical-stability)
5. [Mesh Processing](#mesh-processing)
6. [Sparse Voxel Systems](#sparse-voxel-systems)

## Geometric Representations

### Triangle Meshes

**Definition**: A triangle mesh is a piecewise linear approximation of a 2-manifold surface, represented as:

```
M = (V, F, E)
```

Where:
- **V**: Vertex set {v₁, v₂, ..., vₙ} ⊂ ℝ³
- **F**: Face set of triangles {f₁, f₂, ..., fₘ}
- **E**: Edge set connecting vertices

**Face Normal Calculation**: For triangle (v₀, v₁, v₂), the normal is:

```
n = (v₁ - v₀) × (v₂ - v₀) / |(v₁ - v₀) × (v₂ - v₀)|
```

**Winding Order**: Counter-clockwise vertex ordering ensures outward-pointing normals.

### Parametric Surfaces

**NURBS Curves**: B-spline basis functions with rational weights:

```
C(u) = ∑ᵢⁿ Nᵢ,ₚ(u) · wᵢ · Pᵢ / ∑ᵢⁿ Nᵢ,ₚ(u) · wᵢ
```

Where:
- **Nᵢ,ₚ(u)**: B-spline basis functions
- **wᵢ**: Rational weights
- **Pᵢ**: Control points

**Surface Revolution**: Parametric surface from profile curve:

```
S(u,v) = (x(u)·cos(v), y(u), x(u)·sin(v))
```

### Axis-Aligned Bounding Boxes (AABB)

**Definition**: Minimum axis-aligned box containing a geometric object:

```
AABB = [min_x, max_x] × [min_y, max_y] × [min_z, max_z]
```

**Volume Calculation**:
```
V = (max_x - min_x) × (max_y - min_y) × (max_z - min_z)
```

## Boolean Operations

### Set Theory Foundation

CSG operations are defined on solid objects as set operations:

- **Union (∪)**: A ∪ B = {x | x ∈ A ∨ x ∈ B}
- **Intersection (∩)**: A ∩ B = {x | x ∈ A ∧ x ∈ B}
- **Difference (-)**: A - B = {x | x ∈ A ∧ x ∉ B}

### Algorithmic Implementation

**Boundary Representation**: Operations on mesh boundaries using:

1. **Face Classification**: Determine which side of separating plane each face lies
2. **Edge Intersection**: Find intersection curves between meshes
3. **Polygon Splitting**: Divide faces along intersection curves
4. **Winding Number**: Classify regions using winding rules

### Topological Consistency

**Euler Characteristic**: For closed manifolds:

```
V - E + F = 2
```

Where V=vertices, E=edges, F=faces for sphere-like topology.

## BSP Tree Algorithms

### Binary Space Partitioning

**Definition**: Recursive subdivision of space using separating planes:

```
BSP(T) = (plane, front_tree, back_tree, coplanar_faces)
```

**Construction Algorithm**:
1. Select splitting plane from face set
2. Partition remaining faces into front/back/coplanar
3. Recursively build subtrees

### Polygon Splitting

**Plane-Face Intersection**: For triangle (v₀,v₁,v₂) and plane (p,n):

```
For each edge (vᵢ,vⱼ):
  If edge crosses plane: find intersection point
  Classify vertices: front/back/on-plane
```

**Splitting Logic**:
- **All front/back**: No split needed
- **Mixed classification**: Split into multiple polygons
- **Coplanar**: Handle as special case

### CSG Evaluation

**Point Classification**: Determine point membership in CSG tree:

```
classify(point, bsp_tree):
  case Empty: return Outside
  case Leaf: return Solid
  case Node:
    side = point_side_of_plane(point, node.plane)
    if side == Front: return classify(point, node.front)
    if side == Back:  return classify(point, node.back)
    // On plane: depends on CSG operation
```

## Numerical Stability

### Floating-Point Precision

**SRS NFR005**: Geometric precision tolerance:
- **f64 operations**: 1e-8 absolute tolerance
- **f32 operations**: 1e-4 absolute tolerance

### Robust Geometric Predicates

**Point-in-Triangle Test**: Barycentric coordinates:

```
For point p, triangle (a,b,c):
  Compute barycentric coordinates (u,v,w)
  Point inside if: u ≥ 0 ∧ v ≥ 0 ∧ w ≥ 0
```

**Plane Classification**: Signed distance with epsilon:

```
d = n · (p - p₀)
ε = 1e-8  // For f64 precision

Classification:
  d > ε:  Front
  d < -ε: Back
  |d| ≤ ε: On plane
```

### Numerical Error Accumulation

**Error Bounds**: For composed operations:

```
ε_total ≤ ∑ εᵢ + O(ε²)
```

Where εᵢ is error from individual operations.

## Mesh Processing

### Vertex Deduplication

**Spatial Hashing**: Efficient proximity queries using grid-based hashing:

```
hash(v) = floor(v.x / ε) + floor(v.y / ε) × M + floor(v.z / ε) × M²
```

Where ε is deduplication tolerance, M is hash modulus.

### Normal Vector Calculation

**Newell Method**: Robust normal computation for arbitrary polygons:

```
For polygon with vertices v₀, v₁, ..., vₙ:
n = ∑ᵢ (vᵢ - v₀) × (vᵢ₊₁ - v₀)
```

**Weighted Average**: For smooth shading:

```
n_smooth = ∑ adjacent_face_normals / |adjacent_faces|
```

### Mesh Validation

**Manifold Check**: Each edge shared by exactly two faces:

```
∀edges e: |{faces containing e}| = 2
```

**Orientation Check**: All face normals point outward:

```
∀faces f: f.normal · (centroid - object_center) > 0
```

## Sparse Voxel Systems

### Octree Structure

**Recursive Subdivision**: 8-way spatial partitioning:

```
Node = Leaf(occupied: bool) | Internal(children: [Node; 8])
```

**World-to-Local Coordinates**:

```
local_coord = (world_coord - origin) / voxel_size
octant_index = floor(local_coord.x) & 1 |
               (floor(local_coord.y) & 1) << 1 |
               (floor(local_coord.z) & 1) << 2
```

### DAG Compression

**Subtree Deduplication**: Eliminate redundant identical subtrees:

```
Registry: Map<Node_ID, Node_Reference>
```

**Memory Reduction**: Common patterns (empty/solid cubes) stored once.

### CSG on Voxels

**Boolean Operations**: Tree-based evaluation:

```
voxel_union(a, b, coord):
  a_occ = a.is_occupied(coord)
  b_occ = b.is_occupied(coord)
  return a_occ || b_occ

voxel_intersection(a, b, coord):
  a_occ = a.is_occupied(coord)
  b_occ = b.is_occupied(coord)
  return a_occ && b_occ

voxel_difference(a, b, coord):
  a_occ = a.is_occupied(coord)
  b_occ = b.is_occupied(coord)
  return a_occ && !b_occ
```

### Surface Extraction

**Marching Cubes**: Generate triangles from voxel occupancy:

```
For each 2×2×2 voxel cube:
  Determine 8 corner occupancy states
  Select triangulation pattern from lookup table
  Generate triangles at isosurface
```

## Algorithmic Complexity

### Time Complexity

- **BSP Construction**: O(n log n) for n faces
- **CSG Evaluation**: O(log n) per point query
- **Mesh Boolean**: O(n + m) for n,m face meshes
- **Voxel Operations**: O(log N) for N voxels

### Space Complexity

- **BSP Tree**: O(n) storage for n faces
- **Voxel Octree**: O(k) for k occupied voxels (vs O(N) dense)
- **DAG Compression**: Sublinear reduction for repetitive structures

## References

1. **Foley et al.**: *Computer Graphics: Principles and Practice*
2. **Mantyla**: *Introduction to Solid Modeling*
3. **Lorensen & Cline**: *Marching Cubes: A High Resolution 3D Surface Construction Algorithm*
4. **Snoeyink & van Kreveld**: *Triangulation of a Point Set on a Curve*
5. **Numerical Recipes**: *Floating-Point Arithmetic Considerations*

---

This document provides the mathematical foundation for all algorithms implemented in `csgrs`. All operations maintain the specified precision tolerances and topological invariants while providing efficient implementations for geometric computing applications.
