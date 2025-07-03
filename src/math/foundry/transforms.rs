//! **Canonical Transformation Utilities (The Mind)**
//!
//! This module provides the authoritative implementations of geometric transformation
//! operations used throughout the codebase. It consolidates transformation utilities
//! into optimized, mathematically correct implementations.
//!
//! ## **Mathematical Foundation**
//!
//! ### **Homogeneous Coordinates**
//! 3D transformations are represented using 4×4 matrices in homogeneous coordinates:
//! ```text
//! [x']   [m00 m01 m02 m03] [x]
//! [y'] = [m10 m11 m12 m13] [y]
//! [z']   [m20 m21 m22 m23] [z]
//! [1 ]   [0   0   0   1  ] [1]
//! ```
//!
//! ### **Transformation Composition**
//! Transformations are applied right-to-left: T₃(T₂(T₁(p))) = (T₃ × T₂ × T₁) × p

use crate::core::float_types::Real;
use nalgebra::{Matrix4, Vector3, Point3, Rotation3, Translation3, UnitQuaternion, Unit};

/// **Create a translation transformation matrix**
///
/// This function creates a 4×4 homogeneous transformation matrix that translates
/// points by the specified offset vector.
///
/// ## **Mathematical Definition**
/// For translation vector t = (tx, ty, tz):
/// ```text
/// T = [1  0  0  tx]
///     [0  1  0  ty]
///     [0  0  1  tz]
///     [0  0  0  1 ]
/// ```
///
/// # Arguments
/// * `translation` - The translation vector
///
/// # Returns
/// * `Matrix4<Real>` - The translation transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::translation_matrix;
/// use nalgebra::Vector3;
///
/// let translation = Vector3::new(1.0, 2.0, 3.0);
/// let matrix = translation_matrix(&translation);
/// // Apply to origin point to verify
/// let origin = nalgebra::Point3::new(0.0, 0.0, 0.0);
/// let transformed = matrix.transform_point(&origin);
/// assert_eq!(transformed, nalgebra::Point3::new(1.0, 2.0, 3.0));
/// ```
#[inline]
pub fn translation_matrix(translation: &Vector3<Real>) -> Matrix4<Real> {
    Translation3::from(*translation).to_homogeneous()
}

/// **Create a rotation transformation matrix from axis and angle**
///
/// This function creates a 4×4 homogeneous transformation matrix that rotates
/// points around the specified axis by the specified angle.
///
/// ## **Mathematical Foundation**
/// Uses Rodrigues' rotation formula to construct the rotation matrix.
/// The rotation is performed counter-clockwise when looking down the axis
/// toward the origin (right-hand rule).
///
/// # Arguments
/// * `axis` - The rotation axis (will be normalized)
/// * `angle_radians` - The rotation angle in radians
///
/// # Returns
/// * `Matrix4<Real>` - The rotation transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::rotation_matrix;
/// use nalgebra::Vector3;
/// use std::f64::consts::PI;
///
/// // 90-degree rotation around Z-axis
/// let axis = Vector3::new(0.0, 0.0, 1.0);
/// let matrix = rotation_matrix(&axis, PI / 2.0);
/// ```
#[inline]
pub fn rotation_matrix(axis: &Vector3<Real>, angle_radians: Real) -> Matrix4<Real> {
    let rotation = Rotation3::from_axis_angle(&Unit::new_normalize(*axis), angle_radians);
    rotation.to_homogeneous()
}

/// **Create a rotation transformation matrix from Euler angles**
///
/// This function creates a rotation matrix from Euler angles using the
/// ZYX (yaw-pitch-roll) convention.
///
/// # Arguments
/// * `roll_x` - Rotation around X-axis (radians)
/// * `pitch_y` - Rotation around Y-axis (radians)  
/// * `yaw_z` - Rotation around Z-axis (radians)
///
/// # Returns
/// * `Matrix4<Real>` - The rotation transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::euler_rotation_matrix;
/// use std::f64::consts::PI;
///
/// // 90-degree rotation around each axis
/// let matrix = euler_rotation_matrix(PI / 2.0, PI / 2.0, PI / 2.0);
/// ```
#[inline]
pub fn euler_rotation_matrix(roll_x: Real, pitch_y: Real, yaw_z: Real) -> Matrix4<Real> {
    let rotation = Rotation3::from_euler_angles(roll_x, pitch_y, yaw_z);
    rotation.to_homogeneous()
}

/// **Create a rotation transformation matrix from a quaternion**
///
/// This function creates a rotation matrix from a unit quaternion representation.
///
/// # Arguments
/// * `quaternion` - The unit quaternion representing the rotation
///
/// # Returns
/// * `Matrix4<Real>` - The rotation transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::quaternion_rotation_matrix;
/// use nalgebra::{Vector3, UnitQuaternion, Unit};
/// use std::f64::consts::PI;
///
/// let axis = Vector3::new(0.0, 0.0, 1.0);
/// let quaternion = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), PI / 2.0);
/// let matrix = quaternion_rotation_matrix(&quaternion);
/// ```
#[inline]
pub fn quaternion_rotation_matrix(quaternion: &UnitQuaternion<Real>) -> Matrix4<Real> {
    quaternion.to_homogeneous()
}

/// **Create a scaling transformation matrix**
///
/// This function creates a 4×4 homogeneous transformation matrix that scales
/// points by the specified factors along each axis.
///
/// ## **Mathematical Definition**
/// For scale factors (sx, sy, sz):
/// ```text
/// S = [sx 0  0  0]
///     [0  sy 0  0]
///     [0  0  sz 0]
///     [0  0  0  1]
/// ```
///
/// # Arguments
/// * `scale` - The scale factors for each axis
///
/// # Returns
/// * `Matrix4<Real>` - The scaling transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::scaling_matrix;
/// use nalgebra::Vector3;
///
/// let scale = Vector3::new(2.0, 3.0, 4.0);
/// let matrix = scaling_matrix(&scale);
/// ```
#[inline]
pub fn scaling_matrix(scale: &Vector3<Real>) -> Matrix4<Real> {
    Matrix4::new_nonuniform_scaling(scale)
}

/// **Create a uniform scaling transformation matrix**
///
/// This function creates a scaling matrix that scales uniformly in all directions.
///
/// # Arguments
/// * `scale` - The uniform scale factor
///
/// # Returns
/// * `Matrix4<Real>` - The uniform scaling transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::uniform_scaling_matrix;
///
/// let matrix = uniform_scaling_matrix(2.0);
/// ```
#[inline]
pub fn uniform_scaling_matrix(scale: Real) -> Matrix4<Real> {
    Matrix4::new_scaling(scale)
}

/// **Compose multiple transformation matrices**
///
/// This function multiplies a sequence of transformation matrices to create
/// a single composite transformation. Transformations are applied in the
/// order they appear in the slice (left-to-right application).
///
/// ## **Mathematical Foundation**
/// For transformations T₁, T₂, ..., Tₙ:
/// ```text
/// Composite = Tₙ × ... × T₂ × T₁
/// ```
/// Applied to point p: result = Composite × p = Tₙ(Tₙ₋₁(...T₂(T₁(p))...))
///
/// # Arguments
/// * `transforms` - Slice of transformation matrices to compose
///
/// # Returns
/// * `Matrix4<Real>` - The composite transformation matrix
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::{compose_transforms, translation_matrix, scaling_matrix};
/// use nalgebra::Vector3;
///
/// let translate = translation_matrix(&Vector3::new(1.0, 0.0, 0.0));
/// let scale = scaling_matrix(&Vector3::new(2.0, 2.0, 2.0));
/// let composite = compose_transforms(&[translate, scale]);
/// ```
pub fn compose_transforms(transforms: &[Matrix4<Real>]) -> Matrix4<Real> {
    if transforms.is_empty() {
        return Matrix4::identity();
    }

    let mut result = transforms[0];
    for transform in &transforms[1..] {
        result = transform * result;
    }
    result
}

/// **Create a transformation matrix that looks at a target**
///
/// This function creates a transformation matrix that orients an object to
/// look at a specific target point, similar to a camera look-at transformation.
///
/// # Arguments
/// * `eye` - The position of the observer
/// * `target` - The point to look at
/// * `up` - The up direction vector
///
/// # Returns
/// * `Option<Matrix4<Real>>` - The look-at transformation matrix, or None if invalid
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::look_at_matrix;
/// use nalgebra::{Point3, Vector3};
///
/// let eye = Point3::new(0.0, 0.0, 5.0);
/// let target = Point3::new(0.0, 0.0, 0.0);
/// let up = Vector3::new(0.0, 1.0, 0.0);
/// let matrix = look_at_matrix(&eye, &target, &up).unwrap();
/// ```
pub fn look_at_matrix(
    eye: &Point3<Real>,
    target: &Point3<Real>,
    up: &Vector3<Real>
) -> Option<Matrix4<Real>> {
    let forward = (target - eye).normalize();
    let right = forward.cross(up);
    
    // Check for degenerate case (forward and up are parallel)
    if right.norm_squared() < Real::EPSILON {
        return None;
    }
    
    let right = right.normalize();
    let up_corrected = right.cross(&forward);
    
    let rotation = Matrix4::new(
        right.x, up_corrected.x, -forward.x, 0.0,
        right.y, up_corrected.y, -forward.y, 0.0,
        right.z, up_corrected.z, -forward.z, 0.0,
        0.0, 0.0, 0.0, 1.0,
    );
    
    let translation = translation_matrix(&(-eye.coords));
    Some(rotation * translation)
}

/// **Extract translation component from a transformation matrix**
///
/// This function extracts the translation vector from a 4×4 transformation matrix.
///
/// # Arguments
/// * `matrix` - The transformation matrix
///
/// # Returns
/// * `Vector3<Real>` - The translation component
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::{translation_matrix, extract_translation};
/// use nalgebra::Vector3;
///
/// let original_translation = Vector3::new(1.0, 2.0, 3.0);
/// let matrix = translation_matrix(&original_translation);
/// let extracted = extract_translation(&matrix);
/// assert_eq!(extracted, original_translation);
/// ```
#[inline]
pub fn extract_translation(matrix: &Matrix4<Real>) -> Vector3<Real> {
    Vector3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)])
}

/// **Extract scale factors from a transformation matrix**
///
/// This function extracts the scale factors from a transformation matrix
/// by computing the lengths of the basis vectors.
///
/// # Arguments
/// * `matrix` - The transformation matrix
///
/// # Returns
/// * `Vector3<Real>` - The scale factors for each axis
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::{scaling_matrix, extract_scale};
/// use nalgebra::Vector3;
///
/// let original_scale = Vector3::new(2.0, 3.0, 4.0);
/// let matrix = scaling_matrix(&original_scale);
/// let extracted = extract_scale(&matrix);
/// // Note: extracted values should be approximately equal due to floating-point precision
/// ```
#[inline]
pub fn extract_scale(matrix: &Matrix4<Real>) -> Vector3<Real> {
    let x_axis = Vector3::new(matrix[(0, 0)], matrix[(1, 0)], matrix[(2, 0)]);
    let y_axis = Vector3::new(matrix[(0, 1)], matrix[(1, 1)], matrix[(2, 1)]);
    let z_axis = Vector3::new(matrix[(0, 2)], matrix[(1, 2)], matrix[(2, 2)]);
    
    Vector3::new(x_axis.norm(), y_axis.norm(), z_axis.norm())
}

/// **Create an inverse transformation matrix**
///
/// This function computes the inverse of a transformation matrix, with
/// proper error handling for non-invertible matrices.
///
/// # Arguments
/// * `matrix` - The transformation matrix to invert
///
/// # Returns
/// * `Option<Matrix4<Real>>` - The inverse matrix, or None if not invertible
///
/// # Examples
/// ```rust
/// use csgrs::math::foundry::transforms::{translation_matrix, inverse_transform};
/// use nalgebra::Vector3;
///
/// let matrix = translation_matrix(&Vector3::new(1.0, 2.0, 3.0));
/// let inverse = inverse_transform(&matrix).unwrap();
/// let identity = matrix * inverse;
/// // identity should be approximately the identity matrix
/// ```
#[inline]
pub fn inverse_transform(matrix: &Matrix4<Real>) -> Option<Matrix4<Real>> {
    matrix.try_inverse()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::float_types::EPSILON;
    use std::f64::consts::PI;

    #[test]
    fn test_translation_matrix() {
        let translation = Vector3::new(1.0, 2.0, 3.0);
        let matrix = translation_matrix(&translation);
        
        let origin = Point3::new(0.0, 0.0, 0.0);
        let transformed = matrix.transform_point(&origin);
        assert_eq!(transformed, Point3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_rotation_matrix() {
        let axis = Vector3::new(0.0, 0.0, 1.0);
        let matrix = rotation_matrix(&axis, PI / 2.0);
        
        let point = Point3::new(1.0, 0.0, 0.0);
        let transformed = matrix.transform_point(&point);
        
        // Should rotate (1,0,0) to approximately (0,1,0)
        assert!((transformed.x - 0.0).abs() < EPSILON);
        assert!((transformed.y - 1.0).abs() < EPSILON);
        assert!((transformed.z - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_scaling_matrix() {
        let scale = Vector3::new(2.0, 3.0, 4.0);
        let matrix = scaling_matrix(&scale);
        
        let point = Point3::new(1.0, 1.0, 1.0);
        let transformed = matrix.transform_point(&point);
        assert_eq!(transformed, Point3::new(2.0, 3.0, 4.0));
    }

    #[test]
    fn test_compose_transforms() {
        let translate = translation_matrix(&Vector3::new(1.0, 0.0, 0.0));
        let scale = scaling_matrix(&Vector3::new(2.0, 2.0, 2.0));
        let composite = compose_transforms(&[translate, scale]);
        
        let point = Point3::new(1.0, 1.0, 1.0);
        let transformed = composite.transform_point(&point);
        
        // First translate (1,1,1) -> (2,1,1), then scale -> (4,2,2)
        assert_eq!(transformed, Point3::new(4.0, 2.0, 2.0));
    }

    #[test]
    fn test_extract_translation() {
        let original_translation = Vector3::new(1.0, 2.0, 3.0);
        let matrix = translation_matrix(&original_translation);
        let extracted = extract_translation(&matrix);
        assert_eq!(extracted, original_translation);
    }

    #[test]
    fn test_inverse_transform() {
        let matrix = translation_matrix(&Vector3::new(1.0, 2.0, 3.0));
        let inverse = inverse_transform(&matrix).unwrap();
        let identity = matrix * inverse;
        
        // Check that result is approximately identity matrix
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((identity[(i, j)] - expected).abs() < EPSILON);
            }
        }
    }
}
