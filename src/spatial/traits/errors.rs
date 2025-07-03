//! **Spatial Error Types (The Immune System)**
//!
//! This module defines comprehensive error handling for spatial data structure operations,
//! following Cathedral Engineering principles where error types represent the
//! "immune system" that protects against invalid states and operations.
//!
//! ## **Error Taxonomy**
//!
//! The error types are organized hierarchically to provide precise diagnostic
//! information while maintaining ergonomic error handling patterns.

/// **Result type alias for spatial operations**
///
/// This type alias provides a convenient shorthand for Result types
/// used throughout the spatial module, following Rust conventions.
pub type SpatialResult<T> = Result<T, SpatialError>;

/// **Comprehensive error type for spatial data structure operations**
///
/// This enum provides consistent error handling across all spatial structures
/// (BSP, KD-tree, Octree) with detailed error information for debugging.
///
/// # Examples
///
/// ```rust
/// use csgrs::spatial::traits::errors::{SpatialError, SpatialResult};
/// use csgrs::spatial::traits::factory::SpatialStructureFactory;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// // Example of handling construction errors
/// let empty_polygons: Vec<Polygon<i32>> = vec![];
///
/// // This would typically succeed, but demonstrates error handling pattern
/// let result = std::panic::catch_unwind(|| {
///     SpatialStructureFactory::create_kdtree(&empty_polygons)
/// });
///
/// // In practice, most operations don't fail, but when they do,
/// // SpatialError provides detailed information
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum SpatialError {
    /// **Error during spatial structure construction**
    ///
    /// These errors occur when building spatial data structures from polygon data.
    ConstructionError {
        /// The type of structure being constructed
        structure_type: String,
        /// Detailed error message
        message: String,
        /// Optional source error
        source: Option<String>,
    },

    /// **Error during spatial query operations**
    ///
    /// These errors occur when performing queries on spatial structures.
    QueryError {
        /// The type of query being performed
        query_type: String,
        /// Detailed error message
        message: String,
        /// Optional context information
        context: Option<String>,
    },

    /// **Configuration validation error**
    ///
    /// These errors occur when spatial configuration parameters are invalid.
    ConfigurationError {
        /// The configuration parameter that failed validation
        parameter: String,
        /// The invalid value
        value: String,
        /// Expected value or range
        expected: String,
    },

    /// **Geometric computation error**
    ///
    /// These errors occur during geometric calculations within spatial operations.
    GeometricError {
        /// The geometric operation that failed
        operation: String,
        /// Detailed error message
        message: String,
        /// Optional geometric context
        context: Option<String>,
    },

    /// **Memory allocation or capacity error**
    ///
    /// These errors occur when spatial operations exceed memory limits.
    MemoryError {
        /// The operation that caused the memory error
        operation: String,
        /// Requested size or capacity
        requested: usize,
        /// Available size or capacity
        available: Option<usize>,
    },

    /// **Invalid input data error**
    ///
    /// These errors occur when input data doesn't meet spatial operation requirements.
    InvalidInput {
        /// Description of the invalid input
        input_type: String,
        /// Detailed error message
        message: String,
        /// Suggestion for fixing the input
        suggestion: Option<String>,
    },

    /// **Feature not supported error**
    ///
    /// These errors occur when attempting unsupported operations on spatial structures.
    UnsupportedOperation {
        /// The operation that is not supported
        operation: String,
        /// The structure type that doesn't support it
        structure_type: String,
        /// Suggested alternative
        alternative: Option<String>,
    },
}

impl std::fmt::Display for SpatialError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpatialError::ConstructionError { structure_type, message, source } => {
                write!(f, "Construction error in {}: {}", structure_type, message)?;
                if let Some(src) = source {
                    write!(f, " (source: {})", src)?;
                }
                Ok(())
            },
            SpatialError::QueryError { query_type, message, context } => {
                write!(f, "Query error in {}: {}", query_type, message)?;
                if let Some(ctx) = context {
                    write!(f, " (context: {})", ctx)?;
                }
                Ok(())
            },
            SpatialError::ConfigurationError { parameter, value, expected } => {
                write!(f, "Configuration error: parameter '{}' has invalid value '{}', expected {}",
                       parameter, value, expected)
            },
            SpatialError::GeometricError { operation, message, context } => {
                write!(f, "Geometric error in {}: {}", operation, message)?;
                if let Some(ctx) = context {
                    write!(f, " (context: {})", ctx)?;
                }
                Ok(())
            },
            SpatialError::MemoryError { operation, requested, available } => {
                write!(f, "Memory error in {}: requested {} bytes", operation, requested)?;
                if let Some(avail) = available {
                    write!(f, ", available {} bytes", avail)?;
                }
                Ok(())
            },
            SpatialError::InvalidInput { input_type, message, suggestion } => {
                write!(f, "Invalid input ({}): {}", input_type, message)?;
                if let Some(sug) = suggestion {
                    write!(f, " (suggestion: {})", sug)?;
                }
                Ok(())
            },
            SpatialError::UnsupportedOperation { operation, structure_type, alternative } => {
                write!(f, "Unsupported operation '{}' for {}", operation, structure_type)?;
                if let Some(alt) = alternative {
                    write!(f, " (try: {})", alt)?;
                }
                Ok(())
            },
        }
    }
}

impl std::error::Error for SpatialError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None // We store source as String for simplicity
    }
}

/// **Utility functions for creating common spatial errors**
impl SpatialError {
    /// **Create a construction error**
    pub fn construction(structure_type: &str, message: &str) -> Self {
        Self::ConstructionError {
            structure_type: structure_type.to_string(),
            message: message.to_string(),
            source: None,
        }
    }

    /// **Create a construction error with source**
    pub fn construction_with_source(structure_type: &str, message: &str, source: &str) -> Self {
        Self::ConstructionError {
            structure_type: structure_type.to_string(),
            message: message.to_string(),
            source: Some(source.to_string()),
        }
    }

    /// **Create a query error**
    pub fn query(query_type: &str, message: &str) -> Self {
        Self::QueryError {
            query_type: query_type.to_string(),
            message: message.to_string(),
            context: None,
        }
    }

    /// **Create a query error with context**
    pub fn query_with_context(query_type: &str, message: &str, context: &str) -> Self {
        Self::QueryError {
            query_type: query_type.to_string(),
            message: message.to_string(),
            context: Some(context.to_string()),
        }
    }

    /// **Create a configuration error**
    pub fn configuration(parameter: &str, value: &str, expected: &str) -> Self {
        Self::ConfigurationError {
            parameter: parameter.to_string(),
            value: value.to_string(),
            expected: expected.to_string(),
        }
    }

    /// **Create a geometric error**
    pub fn geometric(operation: &str, message: &str) -> Self {
        Self::GeometricError {
            operation: operation.to_string(),
            message: message.to_string(),
            context: None,
        }
    }

    /// **Create a geometric error with context**
    pub fn geometric_with_context(operation: &str, message: &str, context: &str) -> Self {
        Self::GeometricError {
            operation: operation.to_string(),
            message: message.to_string(),
            context: Some(context.to_string()),
        }
    }

    /// **Create a memory error**
    pub fn memory(operation: &str, requested: usize) -> Self {
        Self::MemoryError {
            operation: operation.to_string(),
            requested,
            available: None,
        }
    }

    /// **Create a memory error with available capacity**
    pub fn memory_with_available(operation: &str, requested: usize, available: usize) -> Self {
        Self::MemoryError {
            operation: operation.to_string(),
            requested,
            available: Some(available),
        }
    }

    /// **Create an invalid input error**
    pub fn invalid_input(input_type: &str, message: &str) -> Self {
        Self::InvalidInput {
            input_type: input_type.to_string(),
            message: message.to_string(),
            suggestion: None,
        }
    }

    /// **Create an invalid input error with suggestion**
    pub fn invalid_input_with_suggestion(input_type: &str, message: &str, suggestion: &str) -> Self {
        Self::InvalidInput {
            input_type: input_type.to_string(),
            message: message.to_string(),
            suggestion: Some(suggestion.to_string()),
        }
    }

    /// **Create an unsupported operation error**
    pub fn unsupported(operation: &str, structure_type: &str) -> Self {
        Self::UnsupportedOperation {
            operation: operation.to_string(),
            structure_type: structure_type.to_string(),
            alternative: None,
        }
    }

    /// **Create an unsupported operation error with alternative**
    pub fn unsupported_with_alternative(operation: &str, structure_type: &str, alternative: &str) -> Self {
        Self::UnsupportedOperation {
            operation: operation.to_string(),
            structure_type: structure_type.to_string(),
            alternative: Some(alternative.to_string()),
        }
    }

    /// **Check if this error is recoverable**
    ///
    /// Returns true if the error might be resolved by adjusting parameters
    /// or retrying with different inputs.
    pub fn is_recoverable(&self) -> bool {
        match self {
            SpatialError::ConstructionError { .. } => true,
            SpatialError::QueryError { .. } => true,
            SpatialError::ConfigurationError { .. } => true,
            SpatialError::GeometricError { .. } => true,
            SpatialError::MemoryError { .. } => false,
            SpatialError::InvalidInput { .. } => true,
            SpatialError::UnsupportedOperation { .. } => false,
        }
    }

    /// **Get recovery suggestions for this error**
    ///
    /// Returns a vector of human-readable suggestions for resolving the error.
    pub fn recovery_suggestions(&self) -> Vec<String> {
        match self {
            SpatialError::ConstructionError { .. } => {
                vec![
                    "Check input polygon validity".to_string(),
                    "Verify polygon count is reasonable".to_string(),
                    "Try different spatial structure type".to_string(),
                ]
            },
            SpatialError::QueryError { .. } => {
                vec![
                    "Validate query parameters".to_string(),
                    "Check spatial structure is properly built".to_string(),
                    "Try different query method".to_string(),
                ]
            },
            SpatialError::ConfigurationError { expected, .. } => {
                vec![format!("Use value in range: {}", expected)]
            },
            SpatialError::GeometricError { .. } => {
                vec![
                    "Check for degenerate geometry".to_string(),
                    "Increase numerical precision tolerance".to_string(),
                    "Validate input coordinates".to_string(),
                ]
            },
            SpatialError::MemoryError { .. } => {
                vec![
                    "Reduce polygon count".to_string(),
                    "Use streaming processing".to_string(),
                    "Increase available memory".to_string(),
                ]
            },
            SpatialError::InvalidInput { suggestion, .. } => {
                if let Some(s) = suggestion {
                    vec![s.clone()]
                } else {
                    vec!["Validate input data format and content".to_string()]
                }
            },
            SpatialError::UnsupportedOperation { alternative, .. } => {
                if let Some(alt) = alternative {
                    vec![format!("Use alternative: {}", alt)]
                } else {
                    vec!["Use different spatial structure type".to_string()]
                }
            },
        }
    }
}
