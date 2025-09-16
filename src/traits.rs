use crate::float_types::Real;
use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::mesh::plane::Plane;
use nalgebra::{Matrix3, Matrix4, Rotation3, Translation3, Vector3};

/// Boolean operations + transformations
pub trait CSG: Sized + Clone {
    fn new() -> Self;
    fn union(&self, other: &Self) -> Self;
    fn difference(&self, other: &Self) -> Self;
    fn intersection(&self, other: &Self) -> Self;
    fn xor(&self, other: &Self) -> Self;
    fn transform(&self, matrix: &Matrix4<Real>) -> Self;
    fn inverse(&self) -> Self;
    fn bounding_box(&self) -> Aabb;
    fn invalidate_bounding_box(&mut self);

    /// Returns a new Self translated by vector.
    fn translate_vector(&self, vector: Vector3<Real>) -> Self {
        self.transform(&Translation3::from(vector).to_homogeneous())
    }

    /// Returns a new Self translated by x, y, and z.
    fn translate(&self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector(Vector3::new(x, y, z))
    }

    /// Returns a new Self translated so that its bounding-box center is at the origin (0,0,0).
    fn center(&self) -> Self {
        let aabb = self.bounding_box();

        // Compute the AABB center
        let center_x = (aabb.mins.x + aabb.maxs.x) * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) * 0.5;
        let center_z = (aabb.mins.z + aabb.maxs.z) * 0.5;

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center_x, -center_y, -center_z)
    }

    /// Translates Self so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use csgrs::traits::CSG;
    /// let mesh = Mesh::<()>::cube(1.0, None).expect("Failed to create cube").translate(2.0, 1.0, -2.0);
    /// let floated = mesh.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(0.0, 0.0, -min_z)
    }

    /// Rotates Self by `x_deg` degrees, `y_deg` degrees, `z_deg` degrees
    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());

        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    /// Scales Self by `scale_x`, `scale_y`, `scale_z`
    fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }

    /// **Mathematical Foundation: Reflection Across Arbitrary Planes**
    ///
    /// Reflect (mirror) this object about an arbitrary plane `plane`.
    /// This implements the complete mathematical theory of 3D reflections:
    ///
    /// ## **Reflection Mathematics**
    ///
    /// ### **Plane Representation**
    /// The plane is specified by:
    /// - `plane.normal` = the plane's normal vector n⃗ (need not be unit)
    /// - `plane.offset` = the signed distance d from origin to plane
    /// - **Plane Equation**: n⃗·p⃗ + d = 0
    ///
    /// ### **Reflection Matrix Derivation**
    /// For a unit normal n̂ and plane through origin, the reflection matrix is:
    /// ```text
    /// R = I - 2n̂n̂ᵀ
    /// ```
    /// **Proof**: For any vector v⃗, the reflection is:
    /// - **Component parallel to n̂**: v∥ = (v⃗·n̂)n̂  → reflected to -v∥
    /// - **Component perpendicular**: v⊥ = v⃗ - v∥  → unchanged
    /// - **Result**: v'⃗ = v⊥ - v∥ = v⃗ - 2(v⃗·n̂)n̂ = (I - 2n̂n̂ᵀ)v⃗
    ///
    /// ### **General Plane Reflection Algorithm**
    /// 1. **Normalize**: n̂ = n⃗/|n⃗|, d̂ = d/|n⃗|
    /// 2. **Translate to Origin**: T₁ = translate by -d̂n̂
    /// 3. **Reflect at Origin**: R = I - 2n̂n̂ᵀ
    /// 4. **Translate Back**: T₂ = translate by +d̂n̂
    /// 5. **Compose**: M = T₂ · R · T₁
    ///
    /// ### **Normal Vector Transformation**
    /// Normals transform by the inverse transpose: n'⃗ = (M⁻¹)ᵀn⃗
    /// For reflections, this simplifies to the same matrix M.
    ///
    /// ## **Geometric Properties**
    /// - **Isometry**: Preserves distances and angles
    /// - **Orientation Reversal**: Changes handedness (det(M) = -1)
    /// - **Involution**: M² = I (reflecting twice gives identity)
    /// - **Plane Invariance**: Points on the plane remain fixed
    ///
    /// **Note**: The result is inverted (`.inverse()`) because reflection reverses
    /// the orientation of polygons, affecting inside/outside semantics in CSG.
    ///
    /// Returns a new Self whose geometry is mirrored accordingly.
    fn mirror(&self, plane: Plane) -> Self {
        // Normal might not be unit, so compute its length:
        let len = plane.normal().norm();
        if len.abs() < crate::float_types::EPSILON {
            // Degenerate plane? Just return clone (no transform)
            return self.clone();
        }

        // Unit normal:
        let n = plane.normal() / len;
        // Adjusted offset = w / ||n||
        let w = plane.offset() / len;

        // Translate so the plane crosses the origin
        // The plane’s offset vector from origin is (w * n).
        let offset = n * w;
        let t1 = Translation3::from(-offset).to_homogeneous(); // push the plane to origin

        // Build the reflection matrix about a plane normal n at the origin
        // R = I - 2 n n^T
        let mut reflect_4 = Matrix4::identity();
        let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Translate back
        let t2 = Translation3::from(offset).to_homogeneous(); // pull the plane back out

        // Combine into a single 4×4
        let mirror_mat = t2 * reflect_4 * t1;

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }

    /// **Mathematical Foundation: Arc Distribution Algorithm**
    ///
    /// Distribute Self `count` times around a circular arc in the XY plane with specified radius,
    /// from `start_angle_deg` to `end_angle_deg`. This implements angular interpolation with
    /// uniform spacing along the arc.
    ///
    /// ## **Algorithm Overview**
    /// 1. **Angular Interpolation**: Linear interpolation in angle space
    /// 2. **Coordinate Transformation**: Polar to Cartesian conversion
    /// 3. **Batch Processing**: Collect all transformations before union
    /// 4. **Memory Optimization**: Pre-allocated vector with exact capacity
    ///
    /// ## **Mathematical Properties**
    /// - **Arc Length**: Uniform angular spacing, not linear spacing
    /// - **Coordinate System**: XY-plane rotation around Z-axis
    /// - **Radius Preservation**: All copies lie on circle of given radius
    /// - **Angle Range**: [`start_angle_deg`, `end_angle_deg`] in degrees
    /// - **Interpolation**: Linear in angle, resulting in uniform angular spacing
    ///
    /// ## **Performance Characteristics**
    /// - **Time Complexity**: O(count × T) where T is transform cost
    /// - **Space Complexity**: O(count) for intermediate storage
    /// - **Memory Efficiency**: Single union operation vs. incremental growth
    /// - **Numerical Stability**: Robust angle interpolation with proper boundary handling
    ///
    /// Returns a new Self containing all arc-distributed copies.
    fn distribute_arc(
        &self,
        count: usize,
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }

        let start_rad = start_angle_deg.to_radians();
        let end_rad = end_angle_deg.to_radians();
        let sweep = end_rad - start_rad;

        // Pre-allocate vector with exact capacity for better performance
        let mut meshes = Vec::with_capacity(count);

        for i in 0..count {
            // Linear interpolation in angle space for uniform angular distribution
            let t = if count == 1 {
                0.5
            } else {
                (i as Real) / ((count.saturating_sub(1).max(1)) as Real)
            };

            let angle = start_rad + t * sweep;

            // Compose rotation and translation transformations
            let rot =
                nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), angle)
                    .to_homogeneous();
            let trans = nalgebra::Translation3::new(radius, 0.0, 0.0).to_homogeneous();

            let mat = rot * trans;
            let transformed = self.transform(&mat);
            meshes.push(transformed);
        }

        // Perform single union operation for optimal performance
        if meshes.is_empty() {
            self.clone()
        } else {
            let mut iter = meshes.into_iter();
            if let Some(first) = iter.next() {
                iter.fold(first, |acc, mesh| acc.union(&mesh))
            } else {
                self.clone()
            }
        }
    }

    /// **Mathematical Foundation: Linear Distribution Algorithm**
    ///
    /// Distribute Self `count` times along a straight line defined by direction vector `dir`,
    /// with uniform spacing `spacing` between copies. This implements linear interpolation
    /// along an arbitrary 3D direction vector.
    ///
    /// ## **Algorithm Overview**
    /// 1. **Vector Normalization**: Ensure direction vector has unit length
    /// 2. **Step Calculation**: Compute displacement vector for each step
    /// 3. **Linear Interpolation**: Position = origin + i × `step_vector`
    /// 4. **Batch Processing**: Collect all transformations before union
    ///
    /// ## **Mathematical Properties**
    /// - **Direction Vector**: Arbitrary 3D direction (auto-normalized)
    /// - **Uniform Spacing**: Constant distance between consecutive copies
    /// - **Coordinate System**: 3D linear distribution along arbitrary axis
    /// - **Origin Preservation**: First copy remains at original position
    /// - **Degenerate Handling**: Zero-length direction vectors handled gracefully
    ///
    /// ## **Performance Characteristics**
    /// - **Time Complexity**: O(count × T) where T is transform cost
    /// - **Space Complexity**: O(count) for intermediate storage
    /// - **Memory Efficiency**: Single union operation vs. incremental growth
    /// - **Numerical Stability**: Proper handling of near-zero direction vectors
    ///
    /// ## **Edge Cases**
    /// - **Zero Direction**: Returns original mesh unchanged
    /// - **Single Count**: Returns original mesh unchanged
    /// - **Near-Zero Direction**: Uses epsilon comparison for robustness
    ///
    /// Returns a new Self containing all linearly distributed copies.
    fn distribute_linear(
        &self,
        count: usize,
        dir: nalgebra::Vector3<Real>,
        spacing: Real,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }

        // Check for zero-length direction vector to prevent division by zero
        let dir_norm = dir.norm();
        if dir_norm < crate::float_types::EPSILON {
            return self.clone();
        }

        // Compute normalized step vector for uniform spacing
        let step = (dir / dir_norm) * spacing;

        // Pre-allocate vector with exact capacity for better performance
        let mut meshes = Vec::with_capacity(count);

        for i in 0..count {
            // Linear interpolation along direction vector
            let offset = step * (i as Real);
            let trans = nalgebra::Translation3::from(offset).to_homogeneous();
            let transformed = self.transform(&trans);
            meshes.push(transformed);
        }

        // Perform single union operation for optimal performance
        if meshes.is_empty() {
            self.clone()
        } else {
            let mut iter = meshes.into_iter();
            if let Some(first) = iter.next() {
                iter.fold(first, |acc, mesh| acc.union(&mesh))
            } else {
                self.clone()
            }
        }
    }

    /// **Mathematical Foundation: Grid Distribution Algorithm**
    ///
    /// Distribute Self in a uniform grid of `rows × cols`, with spacing `dx, dy` in XY plane.
    /// This implements an optimized grid generation algorithm with O(rows × cols) complexity.
    ///
    /// ## **Algorithm Overview**
    /// 1. **Grid Generation**: Create transformation matrices for each grid position
    /// 2. **Batch Processing**: Collect all transformations before applying unions
    /// 3. **Memory Optimization**: Pre-allocate vectors and avoid redundant cloning
    /// 4. **Performance**: Single union operation on collected meshes vs. incremental unions
    ///
    /// ## **Mathematical Properties**
    /// - **Uniform Spacing**: Regular grid with constant dx, dy intervals
    /// - **Coordinate System**: XY-plane distribution with Z=constant
    /// - **Origin Placement**: First copy at original position (0,0,0)
    /// - **Grid Bounds**: Extends from (0,0) to ((cols-1)×dx, (rows-1)×dy)
    ///
    /// ## **Performance Characteristics**
    /// - **Time Complexity**: O(rows × cols × T) where T is transform cost
    /// - **Space Complexity**: O(rows × cols) for intermediate storage
    /// - **Memory Efficiency**: Single union operation vs. incremental growth
    /// - **Optimization**: Iterator-based mesh collection for better cache locality
    ///
    /// Returns a new Self containing all grid-distributed copies.
    fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> Self {
        if rows < 1 || cols < 1 {
            return self.clone();
        }

        let step_x = nalgebra::Vector3::new(dx, 0.0, 0.0);
        let step_y = nalgebra::Vector3::new(0.0, dy, 0.0);
        let total_copies = rows * cols;

        // Pre-allocate vector with exact capacity for better performance
        let mut meshes = Vec::with_capacity(total_copies);

        // Generate all grid positions and transformations
        for r in 0..rows {
            for c in 0..cols {
                let offset = step_x * (c as Real) + step_y * (r as Real);
                let trans = nalgebra::Translation3::from(offset).to_homogeneous();
                let transformed = self.transform(&trans);
                meshes.push(transformed);
            }
        }

        // Perform single union operation for optimal performance
        // Use fold to efficiently combine all meshes
        if meshes.is_empty() {
            self.clone()
        } else {
            let mut iter = meshes.into_iter();
            if let Some(first) = iter.next() {
                iter.fold(first, |acc, mesh| acc.union(&mesh))
            } else {
                self.clone()
            }
        }
    }
}
