//! SVG transform parsing and application utilities
//!
//! This module handles parsing of SVG transform attributes and applying
//! transformations to geometric data structures.

use crate::float_types::Real;
use crate::io::IoError;
use nalgebra::Matrix3;

/// Parse SVG transform attribute into a transformation matrix
/// Supports: translate, rotate, scale, skewX, skewY, matrix
pub fn parse_svg_transform(transform_str: &str) -> Result<Matrix3<Real>, IoError> {
    let transform_str = transform_str.trim();

    // Handle matrix transform: matrix(a,b,c,d,e,f)
    if let Some(matrix_part) = transform_str.strip_prefix("matrix(") {
        if let Some(matrix_part) = matrix_part.strip_suffix(")") {
            let values: Vec<&str> = matrix_part.split(',').map(|s| s.trim()).collect();
            if values.len() == 6 {
                let a: Real = values[0].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid matrix transform values".to_string())
                })?;
                let b: Real = values[1].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid matrix transform values".to_string())
                })?;
                let c: Real = values[2].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid matrix transform values".to_string())
                })?;
                let d: Real = values[3].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid matrix transform values".to_string())
                })?;
                let e: Real = values[4].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid matrix transform values".to_string())
                })?;
                let f: Real = values[5].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid matrix transform values".to_string())
                })?;

                // Create 3x3 transformation matrix [a c e; b d f; 0 0 1]
                return Ok(Matrix3::new(a, c, e, b, d, f, 0.0, 0.0, 1.0));
            }
        }
    }

    // Handle translate transform: translate(x,y) or translate(x)
    if let Some(translate_part) = transform_str.strip_prefix("translate(") {
        if let Some(translate_part) = translate_part.strip_suffix(")") {
            let values: Vec<&str> = translate_part.split(',').map(|s| s.trim()).collect();
            if !values.is_empty() {
                let tx: Real = values[0].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid translate transform values".to_string())
                })?;
                let ty: Real = if values.len() > 1 {
                    values[1].parse().map_err(|_| {
                        IoError::MalformedInput(
                            "Invalid translate transform values".to_string(),
                        )
                    })?
                } else {
                    0.0
                };

                return Ok(Matrix3::new(1.0, 0.0, tx, 0.0, 1.0, ty, 0.0, 0.0, 1.0));
            }
        }
    }

    // Handle scale transform: scale(x,y) or scale(x)
    if let Some(scale_part) = transform_str.strip_prefix("scale(") {
        if let Some(scale_part) = scale_part.strip_suffix(")") {
            let values: Vec<&str> = scale_part.split(',').map(|s| s.trim()).collect();
            if !values.is_empty() {
                let sx: Real = values[0].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid scale transform values".to_string())
                })?;
                let sy: Real = if values.len() > 1 {
                    values[1].parse().map_err(|_| {
                        IoError::MalformedInput("Invalid scale transform values".to_string())
                    })?
                } else {
                    sx
                };

                return Ok(Matrix3::new(sx, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 1.0));
            }
        }
    }

    // Handle rotate transform: rotate(angle,x,y) or rotate(angle)
    if let Some(rotate_part) = transform_str.strip_prefix("rotate(") {
        if let Some(rotate_part) = rotate_part.strip_suffix(")") {
            let values: Vec<&str> = rotate_part.split(',').map(|s| s.trim()).collect();
            if !values.is_empty() {
                let angle_deg: Real = values[0].parse().map_err(|_| {
                    IoError::MalformedInput("Invalid rotate transform values".to_string())
                })?;
                let angle_rad = angle_deg.to_radians();

                let cos_a = angle_rad.cos();
                let sin_a = angle_rad.sin();

                let mut matrix =
                    Matrix3::new(cos_a, -sin_a, 0.0, sin_a, cos_a, 0.0, 0.0, 0.0, 1.0);

                // Handle rotation around a point (x,y)
                if values.len() >= 3 {
                    let cx: Real = values[1].parse().map_err(|_| {
                        IoError::MalformedInput("Invalid rotate transform values".to_string())
                    })?;
                    let cy: Real = values[2].parse().map_err(|_| {
                        IoError::MalformedInput("Invalid rotate transform values".to_string())
                    })?;

                    // Translate to origin, rotate, translate back
                    let translate_to_origin =
                        Matrix3::new(1.0, 0.0, -cx, 0.0, 1.0, -cy, 0.0, 0.0, 1.0);
                    let translate_back =
                        Matrix3::new(1.0, 0.0, cx, 0.0, 1.0, cy, 0.0, 0.0, 1.0);

                    matrix = translate_back * matrix * translate_to_origin;
                }

                return Ok(matrix);
            }
        }
    }

    // Handle skewX transform: skewX(angle)
    if let Some(skew_part) = transform_str.strip_prefix("skewX(") {
        if let Some(skew_part) = skew_part.strip_suffix(")") {
            let angle_deg: Real = skew_part.trim().parse().map_err(|_| {
                IoError::MalformedInput("Invalid skewX transform values".to_string())
            })?;
            let tan_a = angle_deg.to_radians().tan();

            return Ok(Matrix3::new(1.0, tan_a, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0));
        }
    }

    // Handle skewY transform: skewY(angle)
    if let Some(skew_part) = transform_str.strip_prefix("skewY(") {
        if let Some(skew_part) = skew_part.strip_suffix(")") {
            let angle_deg: Real = skew_part.trim().parse().map_err(|_| {
                IoError::MalformedInput("Invalid skewY transform values".to_string())
            })?;
            let tan_a = angle_deg.to_radians().tan();

            return Ok(Matrix3::new(1.0, 0.0, 0.0, tan_a, 1.0, 0.0, 0.0, 0.0, 1.0));
        }
    }

    Err(IoError::MalformedInput(format!(
        "Unsupported transform: {}",
        transform_str
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_svg_transform_parsing() {
        use nalgebra::Matrix3;

        // Test translate
        let matrix = parse_svg_transform("translate(10,20)")
            .expect("Failed to parse translate transform");
        let expected = Matrix3::new(1.0, 0.0, 10.0, 0.0, 1.0, 20.0, 0.0, 0.0, 1.0);
        assert_eq!(matrix, expected);

        // Test translate with single value
        let matrix = parse_svg_transform("translate(15)")
            .expect("Failed to parse single-value translate transform");
        let expected = Matrix3::new(1.0, 0.0, 15.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(matrix, expected);

        // Test scale
        let matrix =
            parse_svg_transform("scale(2,3)").expect("Failed to parse scale transform");
        let expected = Matrix3::new(2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(matrix, expected);

        // Test scale with single value
        let matrix = parse_svg_transform("scale(2)")
            .expect("Failed to parse single-value scale transform");
        let expected = Matrix3::new(2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(matrix, expected);

        // Test rotate
        let matrix =
            parse_svg_transform("rotate(45)").expect("Failed to parse rotate transform");
        let cos45 = 45f64.to_radians().cos();
        let sin45 = 45f64.to_radians().sin();
        let expected = Matrix3::new(cos45, -sin45, 0.0, sin45, cos45, 0.0, 0.0, 0.0, 1.0);
        for i in 0..9 {
            assert!((matrix[i] - expected[i]).abs() < 1e-10);
        }

        // Test matrix
        let matrix = parse_svg_transform("matrix(1,2,3,4,5,6)")
            .expect("Failed to parse matrix transform");
        let expected = Matrix3::new(1.0, 3.0, 5.0, 2.0, 4.0, 6.0, 0.0, 0.0, 1.0);
        assert_eq!(matrix, expected);

        // Test skewX
        let matrix =
            parse_svg_transform("skewX(30)").expect("Failed to parse skewX transform");
        let tan30 = 30f64.to_radians().tan();
        let expected = Matrix3::new(1.0, tan30, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        for i in 0..9 {
            assert!((matrix[i] - expected[i]).abs() < 1e-10);
        }
    }
}
