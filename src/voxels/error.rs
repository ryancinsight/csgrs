//! Error handling for voxels module
//! 
//! Following ACID principles where applicable:
//! - Atomicity: Operations either complete fully or fail completely
//! - Consistency: Data structures remain in valid states
//! - Isolation: Operations don't interfere with each other
//! - Durability: Results persist after operations complete

use std::fmt;

/// Comprehensive error type for voxel operations
#[derive(Debug, Clone, PartialEq)]
pub enum VoxelError {
    /// Invalid resolution (must be > 0 and <= 16)
    InvalidResolution(u8),
    
    /// Invalid bounds (min must be < max in all dimensions)
    InvalidBounds { min: [f64; 3], max: [f64; 3] },
    
    /// SVO depth exceeded maximum allowed
    DepthExceeded { current: u8, max: u8 },
    
    /// BSP tree construction failed
    BspConstructionFailed(String),
    
    /// Surface extraction failed
    SurfaceExtractionFailed(String),
    
    /// Memory allocation failed
    OutOfMemory,
    
    /// Invalid child index (must be 0-7)
    InvalidChildIndex(u8),
    
    /// Operation on empty SVO
    EmptySvo,
    
    /// Inconsistent SVO state
    InconsistentState(String),
}

impl fmt::Display for VoxelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VoxelError::InvalidResolution(res) => {
                write!(f, "Invalid resolution {}: must be > 0 and <= 16", res)
            }
            VoxelError::InvalidBounds { min, max } => {
                write!(f, "Invalid bounds: min {:?} must be < max {:?} in all dimensions", min, max)
            }
            VoxelError::DepthExceeded { current, max } => {
                write!(f, "SVO depth {} exceeds maximum {}", current, max)
            }
            VoxelError::BspConstructionFailed(msg) => {
                write!(f, "BSP tree construction failed: {}", msg)
            }
            VoxelError::SurfaceExtractionFailed(msg) => {
                write!(f, "Surface extraction failed: {}", msg)
            }
            VoxelError::OutOfMemory => {
                write!(f, "Out of memory during voxel operation")
            }
            VoxelError::InvalidChildIndex(idx) => {
                write!(f, "Invalid child index {}: must be 0-7", idx)
            }
            VoxelError::EmptySvo => {
                write!(f, "Operation attempted on empty SVO")
            }
            VoxelError::InconsistentState(msg) => {
                write!(f, "Inconsistent SVO state: {}", msg)
            }
        }
    }
}

impl std::error::Error for VoxelError {}

/// Result type for voxel operations
pub type VoxelResult<T> = Result<T, VoxelError>;

/// Validation utilities following ACID consistency principle
pub struct VoxelValidator;

impl VoxelValidator {
    /// Validate resolution parameter
    pub fn validate_resolution(resolution: u8) -> VoxelResult<()> {
        if resolution == 0 || resolution > 16 {
            Err(VoxelError::InvalidResolution(resolution))
        } else {
            Ok(())
        }
    }
    
    /// Validate bounds
    pub fn validate_bounds(min: &[f64; 3], max: &[f64; 3]) -> VoxelResult<()> {
        for i in 0..3 {
            if min[i] >= max[i] {
                return Err(VoxelError::InvalidBounds { min: *min, max: *max });
            }
        }
        Ok(())
    }
    
    /// Validate child index
    pub fn validate_child_index(index: u8) -> VoxelResult<()> {
        if index > 7 {
            Err(VoxelError::InvalidChildIndex(index))
        } else {
            Ok(())
        }
    }
    
    /// Validate SVO depth
    pub fn validate_depth(current: u8, max: u8) -> VoxelResult<()> {
        if current > max {
            Err(VoxelError::DepthExceeded { current, max })
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_resolution_validation() {
        assert!(VoxelValidator::validate_resolution(0).is_err());
        assert!(VoxelValidator::validate_resolution(17).is_err());
        assert!(VoxelValidator::validate_resolution(8).is_ok());
    }
    
    #[test]
    fn test_bounds_validation() {
        let valid_min = [0.0, 0.0, 0.0];
        let valid_max = [1.0, 1.0, 1.0];
        assert!(VoxelValidator::validate_bounds(&valid_min, &valid_max).is_ok());
        
        let invalid_min = [1.0, 0.0, 0.0];
        let invalid_max = [0.0, 1.0, 1.0];
        assert!(VoxelValidator::validate_bounds(&invalid_min, &invalid_max).is_err());
    }
    
    #[test]
    fn test_child_index_validation() {
        assert!(VoxelValidator::validate_child_index(8).is_err());
        assert!(VoxelValidator::validate_child_index(7).is_ok());
        assert!(VoxelValidator::validate_child_index(0).is_ok());
    }
}
