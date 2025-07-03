//! **Minkowski Operation Contracts (The Soul)**
//!
//! This module defines the trait interfaces and contracts for Minkowski operations,
//! following Cathedral Engineering principles where traits represent the "soul" of
//! the architectural space.
//!
//! ## **Design Philosophy**
//!
//! These traits establish the fundamental contracts that govern how Minkowski operations
//! behave across different geometric types, ensuring consistency and enabling generic
//! programming patterns.

use super::errors::MinkowskiResult;
use std::fmt::Debug;

/// **Enumeration of supported Minkowski operations**
///
/// This enum provides a type-safe way to specify which Minkowski operation
/// to perform, enabling generic dispatch patterns.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MinkowskiOperation {
    /// Minkowski sum: A ⊕ B = {a + b | a ∈ A, b ∈ B}
    Sum,
    /// Minkowski difference: A ⊖ B = {a - b | a ∈ A, b ∈ B}
    Difference,
}

impl MinkowskiOperation {
    /// Returns a human-readable description of the operation
    pub fn description(&self) -> &'static str {
        match self {
            MinkowskiOperation::Sum => "Minkowski sum (shape expansion)",
            MinkowskiOperation::Difference => "Minkowski difference (shape contraction)",
        }
    }

    /// Returns the mathematical symbol for the operation
    pub fn symbol(&self) -> &'static str {
        match self {
            MinkowskiOperation::Sum => "⊕",
            MinkowskiOperation::Difference => "⊖",
        }
    }
}

/// **Primary trait for types that support Minkowski operations**
///
/// This trait defines the core interface for computing Minkowski operations
/// between geometric objects. It follows the principle of abstract forces,
/// allowing different implementations while maintaining consistent behavior.
///
/// ## **Mathematical Foundation**
///
/// Implementors must ensure that:
/// 1. **Associativity**: (A ⊕ B) ⊕ C = A ⊕ (B ⊕ C) for convex sets
/// 2. **Commutativity**: A ⊕ B = B ⊕ A for Minkowski sum
/// 3. **Identity Element**: A ⊕ {0} = A (point at origin)
/// 4. **Monotonicity**: If A ⊆ B, then A ⊕ C ⊆ B ⊕ C
pub trait MinkowskiComputable<S> 
where 
    S: Clone + Debug + Send + Sync 
{
    /// Compute the Minkowski sum of self with other
    ///
    /// **Mathematical Definition**: self ⊕ other = {s + o | s ∈ self, o ∈ other}
    ///
    /// # Arguments
    /// * `other` - The second operand for the Minkowski sum
    ///
    /// # Returns
    /// * `MinkowskiResult<Self>` - The resulting Minkowski sum or an error
    ///
    /// # Examples
    /// ```rust
    /// use csgrs::CSG;
    /// use csgrs::math::minkowski::contracts::MinkowskiComputable;
    ///
    /// let cube: CSG<()> = CSG::cube(1.0, None);
    /// let sphere: CSG<()> = CSG::sphere(0.5, 16, 8, None);
    /// // Note: minkowski_sum_trait is not implemented for CSG yet
/// // let result = cube.minkowski_sum_trait(&sphere).unwrap();
    /// ```
    fn minkowski_sum_trait(&self, other: &Self) -> MinkowskiResult<Self>
    where 
        Self: Sized;

    /// Compute the Minkowski difference of self with other
    ///
    /// **Mathematical Definition**: self ⊖ other = {s - o | s ∈ self, o ∈ other}
    ///
    /// # Arguments
    /// * `other` - The second operand for the Minkowski difference
    ///
    /// # Returns
    /// * `MinkowskiResult<Self>` - The resulting Minkowski difference or an error
    fn minkowski_difference_trait(&self, other: &Self) -> MinkowskiResult<Self>
    where 
        Self: Sized;

    /// Generic Minkowski operation dispatcher
    ///
    /// This method provides a unified interface for all Minkowski operations,
    /// enabling generic programming patterns and operation chaining.
    ///
    /// # Arguments
    /// * `other` - The second operand
    /// * `operation` - The type of Minkowski operation to perform
    ///
    /// # Returns
    /// * `MinkowskiResult<Self>` - The result of the specified operation
    fn apply_minkowski_operation(&self, other: &Self, operation: MinkowskiOperation) -> MinkowskiResult<Self>
    where 
        Self: Sized
    {
        match operation {
            MinkowskiOperation::Sum => self.minkowski_sum_trait(other),
            MinkowskiOperation::Difference => self.minkowski_difference_trait(other),
        }
    }
}

/// **Trait for validating Minkowski operation preconditions**
///
/// This trait provides methods to validate that geometric objects meet
/// the mathematical requirements for Minkowski operations.
pub trait MinkowskiValidation<S> 
where 
    S: Clone + Debug + Send + Sync 
{
    /// Check if the object is suitable for Minkowski operations
    ///
    /// # Returns
    /// * `MinkowskiResult<()>` - Ok(()) if valid, Err with details if invalid
    fn validate_for_minkowski(&self) -> MinkowskiResult<()>;

    /// Check if two objects are compatible for Minkowski operations
    ///
    /// # Arguments
    /// * `other` - The second operand to check compatibility with
    ///
    /// # Returns
    /// * `MinkowskiResult<()>` - Ok(()) if compatible, Err with details if incompatible
    fn validate_minkowski_compatibility(&self, other: &Self) -> MinkowskiResult<()>;
}

/// **Trait for Minkowski operation optimization hints**
///
/// This trait allows implementors to provide optimization hints that can
/// improve the performance of Minkowski operations for specific geometric
/// configurations.
pub trait MinkowskiOptimization<S> 
where 
    S: Clone + Debug + Send + Sync 
{
    /// Estimate the computational complexity of a Minkowski operation
    ///
    /// # Arguments
    /// * `other` - The second operand
    /// * `operation` - The type of operation
    ///
    /// # Returns
    /// * `usize` - Estimated number of operations (for algorithm selection)
    fn estimate_complexity(&self, other: &Self, operation: MinkowskiOperation) -> usize;

    /// Suggest whether to use convex hull optimization
    ///
    /// # Arguments
    /// * `other` - The second operand
    ///
    /// # Returns
    /// * `bool` - True if convex hull pre-processing is recommended
    fn should_use_convex_hull_optimization(&self, other: &Self) -> bool;
}
