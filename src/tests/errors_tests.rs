#[cfg(test)]
mod validation_error_tests {
    use crate::errors::ValidationError;
    use crate::float_types::Real;
    use nalgebra::Point3;

    #[test]
    fn test_validation_error_display() {
        // Test all error variants have proper display formatting
        let point = Point3::new(1.0 as Real, 2.0 as Real, 3.0 as Real);

        let errors = vec![
            ValidationError::RepeatedPoint(point),
            ValidationError::HoleOutsideShell(point),
            ValidationError::NestedHoles(point),
            ValidationError::DisconnectedInterior(point),
            ValidationError::SelfIntersection(point),
            ValidationError::RingSelfIntersection(point),
            ValidationError::NestedShells(point),
            ValidationError::TooFewPoints(point),
            ValidationError::InvalidCoordinate(point),
            ValidationError::RingNotClosed(point),
            ValidationError::MismatchedVertices,
            ValidationError::IndexOutOfRange,
            ValidationError::InvalidArguments,
            ValidationError::InvalidDimension("width".to_string(), 0.0 as Real),
            ValidationError::InvalidShapeParameter("segments".to_string(), "must be >= 3".to_string()),
            ValidationError::Other("Test error".to_string(), Some(point)),
            ValidationError::Other("Generic error".to_string(), None),
        ];

        for error in errors {
            // Just verify that Display works without panicking
            let _display = format!("{}", error);
            assert!(!error.to_string().is_empty());
        }
    }

    #[test]
    fn test_validation_error_debug() {
        let point = Point3::new(1.0 as Real, 2.0 as Real, 3.0 as Real);
        let error = ValidationError::InvalidCoordinate(point);

        // Verify Debug formatting works
        let debug_str = format!("{:?}", error);
        assert!(debug_str.contains("InvalidCoordinate"));
        assert!(debug_str.contains("1"));
    }

    #[test]
    fn test_validation_error_clone() {
        let point = Point3::new(1.0 as Real, 2.0 as Real, 3.0 as Real);
        let error = ValidationError::RepeatedPoint(point);
        let cloned_error = error.clone();

        assert_eq!(error, cloned_error);
        assert!(matches!(cloned_error, ValidationError::RepeatedPoint(_)));
    }

    #[test]
    fn test_validation_error_partial_eq() {
        let point1 = Point3::new(1.0 as Real, 2.0 as Real, 3.0 as Real);
        let point2 = Point3::new(4.0 as Real, 5.0 as Real, 6.0 as Real);

        // Same variants with same points should be equal
        assert_eq!(
            ValidationError::RepeatedPoint(point1),
            ValidationError::RepeatedPoint(point1)
        );

        // Same variants with different points should not be equal
        assert_ne!(
            ValidationError::RepeatedPoint(point1),
            ValidationError::RepeatedPoint(point2)
        );

        // Different variants should not be equal
        assert_ne!(
            ValidationError::RepeatedPoint(point1),
            ValidationError::HoleOutsideShell(point1)
        );

        // Test dimensionless errors
        assert_eq!(
            ValidationError::MismatchedVertices,
            ValidationError::MismatchedVertices
        );
        assert_ne!(
            ValidationError::MismatchedVertices,
            ValidationError::IndexOutOfRange
        );
    }

    #[test]
    fn test_validation_error_std_error_trait() {
        let point = Point3::new(1.0 as Real, 2.0 as Real, 3.0 as Real);
        let error = ValidationError::InvalidCoordinate(point);

        // Verify it implements std::error::Error
        let _error_trait: &dyn std::error::Error = &error;
    }

    #[test]
    fn test_validation_error_specific_messages() {
        // Test specific error message formats
        let point = Point3::new(1.0 as Real, 2.0 as Real, 3.0 as Real);

        let repeated_point_error = ValidationError::RepeatedPoint(point);
        assert!(repeated_point_error.to_string().contains("Repeated point"));

        let invalid_dim_error = ValidationError::InvalidDimension("radius".to_string(), -1.0 as Real);
        assert!(invalid_dim_error.to_string().contains("Invalid radius dimension"));
        assert!(invalid_dim_error.to_string().contains("must be positive and finite"));

        let invalid_param_error = ValidationError::InvalidShapeParameter(
            "segments".to_string(),
            "must be positive integer".to_string()
        );
        assert!(invalid_param_error.to_string().contains("Invalid segments parameter"));
        assert!(invalid_param_error.to_string().contains("must be positive integer"));

        let other_error_with_point = ValidationError::Other("Custom error".to_string(), Some(point));
        assert!(other_error_with_point.to_string().contains("Custom error"));
        assert!(other_error_with_point.to_string().contains("1"));

        let other_error_no_point = ValidationError::Other("Generic error".to_string(), None);
        assert!(other_error_no_point.to_string().contains("Generic error"));
        assert!(!other_error_no_point.to_string().contains("at"));
    }

    #[test]
    fn test_validation_error_edge_cases() {
        // Test edge cases in error formatting
        let nan_point = Point3::new(f32::NAN as Real, 1.0 as Real, 2.0 as Real);
        let inf_point = Point3::new(f32::INFINITY as Real, 1.0 as Real, 2.0 as Real);
        let neg_inf_point = Point3::new(f32::NEG_INFINITY as Real, 1.0 as Real, 2.0 as Real);

        let errors_with_extreme_values = vec![
            ValidationError::InvalidCoordinate(nan_point),
            ValidationError::InvalidCoordinate(inf_point),
            ValidationError::InvalidCoordinate(neg_inf_point),
        ];

        for error in errors_with_extreme_values {
            // These should not panic even with extreme values
            let _display = format!("{}", error);
        }

        // Test very long parameter names
        let long_param_name = "very_long_parameter_name_that_exceeds_normal_length".to_string();
        let long_param_error = ValidationError::InvalidDimension(long_param_name, 0.0 as Real);
        assert!(long_param_error.to_string().contains("very_long_parameter_name_that_exceeds_normal_length"));

        // Test empty string parameters
        let empty_param_error = ValidationError::InvalidShapeParameter("".to_string(), "".to_string());
        let display = empty_param_error.to_string();
        assert!(display.contains("Invalid  parameter"));
    }
}
