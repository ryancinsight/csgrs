//! **Minkowski Operation Error Types (The Immune System)**
//!
//! This module defines comprehensive error handling for Minkowski operations,
//! following Cathedral Engineering principles where error types represent the
//! "immune system" that protects against invalid states and operations.
//!
//! ## **Error Taxonomy**
//!
//! The error types are organized hierarchically to provide precise diagnostic
//! information while maintaining ergonomic error handling patterns.

use std::fmt;
use std::error::Error;

/// **Result type alias for Minkowski operations**
///
/// This type alias provides a convenient shorthand for Result types
/// used throughout the Minkowski module, following Rust conventions.
pub type MinkowskiResult<T> = Result<T, MinkowskiError>;

/// **Comprehensive error type for Minkowski operations**
///
/// This enum captures all possible failure modes in Minkowski operations,
/// providing detailed diagnostic information for debugging and error recovery.
#[derive(Debug, Clone, PartialEq)]
pub enum MinkowskiError {
    /// **Input validation errors**
    ///
    /// These errors occur when the input geometries don't meet the
    /// mathematical preconditions for Minkowski operations.
    InvalidInput {
        /// Human-readable description of the validation failure
        reason: String,
        /// Optional suggestion for fixing the input
        suggestion: Option<String>,
    },

    /// **Empty geometry errors**
    ///
    /// These errors occur when one or both operands contain no geometric data.
    EmptyGeometry {
        /// Which operand is empty ("first", "second", or "both")
        operand: String,
    },

    /// **Convex hull computation errors**
    ///
    /// These errors occur when the underlying convex hull algorithm fails,
    /// typically due to degenerate point configurations.
    ConvexHullFailure {
        /// Detailed error message from the convex hull library
        hull_error: String,
        /// Number of points that were processed
        point_count: usize,
    },

    /// **Numerical precision errors**
    ///
    /// These errors occur when floating-point precision issues prevent
    /// reliable computation of the Minkowski operation.
    NumericalInstability {
        /// Description of the precision issue
        issue: String,
        /// Suggested epsilon value for retry
        suggested_epsilon: Option<f64>,
    },

    /// **Memory allocation errors**
    ///
    /// These errors occur when the operation requires more memory than available,
    /// typically for very large geometric objects.
    InsufficientMemory {
        /// Estimated memory requirement in bytes
        required_bytes: usize,
        /// Available memory in bytes (if known)
        available_bytes: Option<usize>,
    },

    /// **Algorithmic complexity errors**
    ///
    /// These errors occur when the operation would exceed reasonable
    /// computational limits due to input size.
    ComplexityLimit {
        /// Estimated number of operations required
        estimated_operations: usize,
        /// Maximum allowed operations
        max_operations: usize,
    },

    /// **Feature availability errors**
    ///
    /// These errors occur when required features are not enabled
    /// in the current build configuration.
    FeatureNotEnabled {
        /// Name of the required feature
        feature_name: String,
        /// Instructions for enabling the feature
        enable_instructions: String,
    },
}

impl MinkowskiError {
    /// **Create an invalid input error with detailed context**
    pub fn invalid_input<S: Into<String>>(reason: S) -> Self {
        MinkowskiError::InvalidInput {
            reason: reason.into(),
            suggestion: None,
        }
    }

    /// **Create an invalid input error with a suggested fix**
    pub fn invalid_input_with_suggestion<S: Into<String>, T: Into<String>>(
        reason: S, 
        suggestion: T
    ) -> Self {
        MinkowskiError::InvalidInput {
            reason: reason.into(),
            suggestion: Some(suggestion.into()),
        }
    }

    /// **Create an empty geometry error**
    pub fn empty_geometry<S: Into<String>>(operand: S) -> Self {
        MinkowskiError::EmptyGeometry {
            operand: operand.into(),
        }
    }

    /// **Create a convex hull failure error**
    pub fn convex_hull_failure<S: Into<String>>(hull_error: S, point_count: usize) -> Self {
        MinkowskiError::ConvexHullFailure {
            hull_error: hull_error.into(),
            point_count,
        }
    }

    /// **Create a numerical instability error**
    pub fn numerical_instability<S: Into<String>>(issue: S) -> Self {
        MinkowskiError::NumericalInstability {
            issue: issue.into(),
            suggested_epsilon: None,
        }
    }

    /// **Create a numerical instability error with epsilon suggestion**
    pub fn numerical_instability_with_epsilon<S: Into<String>>(
        issue: S, 
        suggested_epsilon: f64
    ) -> Self {
        MinkowskiError::NumericalInstability {
            issue: issue.into(),
            suggested_epsilon: Some(suggested_epsilon),
        }
    }

    /// **Create an insufficient memory error**
    pub fn insufficient_memory(required_bytes: usize) -> Self {
        MinkowskiError::InsufficientMemory {
            required_bytes,
            available_bytes: None,
        }
    }

    /// **Create a complexity limit error**
    pub fn complexity_limit(estimated_operations: usize, max_operations: usize) -> Self {
        MinkowskiError::ComplexityLimit {
            estimated_operations,
            max_operations,
        }
    }

    /// **Create a feature not enabled error**
    pub fn feature_not_enabled<S: Into<String>, T: Into<String>>(
        feature_name: S, 
        enable_instructions: T
    ) -> Self {
        MinkowskiError::FeatureNotEnabled {
            feature_name: feature_name.into(),
            enable_instructions: enable_instructions.into(),
        }
    }

    /// **Check if this error is recoverable**
    ///
    /// Returns true if the error might be resolved by adjusting parameters
    /// or retrying with different inputs.
    pub fn is_recoverable(&self) -> bool {
        match self {
            MinkowskiError::InvalidInput { .. } => true,
            MinkowskiError::EmptyGeometry { .. } => false,
            MinkowskiError::ConvexHullFailure { .. } => true,
            MinkowskiError::NumericalInstability { .. } => true,
            MinkowskiError::InsufficientMemory { .. } => false,
            MinkowskiError::ComplexityLimit { .. } => true,
            MinkowskiError::FeatureNotEnabled { .. } => false,
        }
    }

    /// **Get recovery suggestions for this error**
    ///
    /// Returns a vector of human-readable suggestions for resolving the error.
    pub fn recovery_suggestions(&self) -> Vec<String> {
        match self {
            MinkowskiError::InvalidInput { suggestion, .. } => {
                if let Some(s) = suggestion {
                    vec![s.clone()]
                } else {
                    vec!["Validate input geometry before operation".to_string()]
                }
            },
            MinkowskiError::EmptyGeometry { .. } => {
                vec!["Ensure both operands contain valid geometric data".to_string()]
            },
            MinkowskiError::ConvexHullFailure { .. } => {
                vec![
                    "Try simplifying the input geometry".to_string(),
                    "Remove duplicate or collinear points".to_string(),
                    "Increase numerical precision tolerance".to_string(),
                ]
            },
            MinkowskiError::NumericalInstability { suggested_epsilon, .. } => {
                let mut suggestions = vec!["Increase numerical precision tolerance".to_string()];
                if let Some(epsilon) = suggested_epsilon {
                    suggestions.push(format!("Try using epsilon value: {}", epsilon));
                }
                suggestions
            },
            MinkowskiError::InsufficientMemory { .. } => {
                vec![
                    "Simplify input geometry to reduce memory requirements".to_string(),
                    "Process geometry in smaller chunks".to_string(),
                ]
            },
            MinkowskiError::ComplexityLimit { .. } => {
                vec![
                    "Reduce input geometry complexity".to_string(),
                    "Use convex hull approximation".to_string(),
                    "Increase complexity limit if appropriate".to_string(),
                ]
            },
            MinkowskiError::FeatureNotEnabled { enable_instructions, .. } => {
                vec![enable_instructions.clone()]
            },
        }
    }
}

impl fmt::Display for MinkowskiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MinkowskiError::InvalidInput { reason, .. } => {
                write!(f, "Invalid input for Minkowski operation: {}", reason)
            },
            MinkowskiError::EmptyGeometry { operand } => {
                write!(f, "Empty geometry in {} operand", operand)
            },
            MinkowskiError::ConvexHullFailure { hull_error, point_count } => {
                write!(f, "Convex hull computation failed with {} points: {}", point_count, hull_error)
            },
            MinkowskiError::NumericalInstability { issue, .. } => {
                write!(f, "Numerical instability in Minkowski operation: {}", issue)
            },
            MinkowskiError::InsufficientMemory { required_bytes, .. } => {
                write!(f, "Insufficient memory for Minkowski operation: {} bytes required", required_bytes)
            },
            MinkowskiError::ComplexityLimit { estimated_operations, max_operations } => {
                write!(f, "Operation complexity limit exceeded: {} > {} operations", 
                       estimated_operations, max_operations)
            },
            MinkowskiError::FeatureNotEnabled { feature_name, .. } => {
                write!(f, "Required feature '{}' is not enabled", feature_name)
            },
        }
    }
}

impl Error for MinkowskiError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        // Most Minkowski errors are leaf errors without underlying causes
        None
    }
}
