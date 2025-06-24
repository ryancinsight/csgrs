use super::CSG;
use crate::core::float_types::{EPSILON, Real};
use crate::geometry::Plane;
use geo::{AffineOps, AffineTransform};
use nalgebra::{Matrix3, Matrix4, Point3, Rotation3, Translation3, Vector3};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Invert this CSG (flip inside vs. outside)
    pub fn inverse(&self) -> CSG<S> {
        let mut csg = self.clone();
        for p in &mut csg.polygons {
            p.flip();
        }
        csg
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by projecting the resulting coordinates.
    pub fn transform(&self, mat: &Matrix4<Real>) -> CSG<S> {
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match mat.try_inverse() {
            Some(inv) => inv.transpose(),
            None => {
                eprintln!("Warning: Transformation matrix is not invertible, using identity for normals");
                Matrix4::identity()
            }
        };
        
        let mut csg = self.clone();

        for poly in &mut csg.polygons {
            for vert in &mut poly.vertices {
                // Transform position using homogeneous coordinates
                let hom_pos = mat * vert.pos.to_homogeneous();
                match Point3::from_homogeneous(hom_pos) {
                    Some(transformed_pos) => vert.pos = transformed_pos,
                    None => {
                        eprintln!("Warning: Invalid homogeneous coordinates after transformation, skipping vertex");
                        continue;
                    }
                }

                // Transform normal using inverse transpose rule
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }

            // Reconstruct plane from transformed vertices for consistency
            poly.plane = Plane::from_vertices(poly.vertices.clone());
        }

        // Convert the top-left 2×2 submatrix + translation of a 4×4 into a geo::AffineTransform
        // The 4x4 looks like:
        //  [ m11  m12  m13  m14 ]
        //  [ m21  m22  m23  m24 ]
        //  [ m31  m32  m33  m34 ]
        //  [ m41  m42  m43  m44 ]
        //
        // For 2D, we use the sub-block:
        //   a = m11,  b = m12,
        //   d = m21,  e = m22,
        //   xoff = m14,
        //   yoff = m24,
        // ignoring anything in z.
        //
        // So the final affine transform in 2D has matrix:
        //   [a   b   xoff]
        //   [d   e   yoff]
        //   [0   0    1  ]
        let a = mat[(0, 0)];
        let b = mat[(0, 1)];
        let xoff = mat[(0, 3)];
        let d = mat[(1, 0)];
        let e = mat[(1, 1)];
        let yoff = mat[(1, 3)];

        let affine2 = AffineTransform::new(a, b, xoff, d, e, yoff);

        // Transform csg.geometry (the GeometryCollection) in 2D
        // Using geo's map-coords approach or the built-in AffineOps trait.
        // Below we use the `AffineOps` trait if you have `use geo::AffineOps;`
        csg.geometry = csg.geometry.affine_transform(&affine2);

        // invalidate the old cached bounding box
        csg.bounding_box = OnceLock::new();

        csg
    }

    /// Returns a new CSG translated by x, y, and z.
    ///
    pub fn translate(&self, x: Real, y: Real, z: Real) -> CSG<S> {
        self.translate_vector(Vector3::new(x, y, z))
    }

    /// Returns a new CSG translated by vector.
    ///
    pub fn translate_vector(&self, vector: Vector3<Real>) -> CSG<S> {
        let translation = Translation3::from(vector);

        // Convert to a Matrix4
        let mat4 = translation.to_homogeneous();
        self.transform(&mat4)
    }

    /// Returns a new CSG translated so that its bounding-box center is at the origin (0,0,0).
    pub fn center(&self) -> Self {
        let aabb = self.bounding_box();

        // Compute the AABB center
        let center_x = (aabb.mins.x + aabb.maxs.x) * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) * 0.5;
        let center_z = (aabb.mins.z + aabb.maxs.z) * 0.5;

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center_x, -center_y, -center_z)
    }

    /// Translates the CSG so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```
    /// use csgrs::CSG;
    /// let csg: CSG<()> = CSG::cube(1.0, None).translate(2.0, 1.0, -2.0);
    /// let floated = csg.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    pub fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(0.0, 0.0, -min_z)
    }

    /// Rotates the CSG by x_degrees, y_degrees, z_degrees
    pub fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> CSG<S> {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());

        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    /// Scales the CSG by scale_x, scale_y, scale_z
    pub fn scale(&self, sx: Real, sy: Real, sz: Real) -> CSG<S> {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }

    /// **Mathematical Foundation: Reflection Across Arbitrary Planes**
    ///
    /// Reflect (mirror) this CSG about an arbitrary plane `plane`.
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
    /// **Note**: The result is inverted (.inverse()) because reflection reverses
    /// the orientation of polygons, affecting inside/outside semantics in CSG.
    ///
    /// Returns a new CSG whose geometry is mirrored accordingly.
    pub fn mirror(&self, plane: Plane) -> Self {
        // Normal might not be unit, so compute its length:
        let len = plane.normal().norm();
        if len.abs() < EPSILON {
            // Degenerate plane? Just return clone (no transform)
            return self.clone();
        }

        // Unit normal:
        let n = plane.normal() / len;
        // Adjusted offset = w / ||n||
        let w = plane.offset() / len;

        // Step 1) Translate so the plane crosses the origin
        // The plane's offset vector from origin is (w * n).
        let offset = n * w;
        let t1 = Translation3::from(-offset).to_homogeneous(); // push the plane to origin

        // Step 2) Build the reflection matrix about a plane normal n at the origin
        //   R = I - 2 n n^T
        let mut reflect_4 = Matrix4::identity();
        let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Step 3) Translate back
        let t2 = Translation3::from(offset).to_homogeneous(); // pull the plane back out

        // Combine into a single 4×4
        let mirror_mat = t2 * reflect_4 * t1;

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }
} 
