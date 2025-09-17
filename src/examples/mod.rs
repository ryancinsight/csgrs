//! Modular examples for csgrs demonstrating various features
//!
//! This module provides organized, focused examples that can be run independently
//! or as part of a comprehensive demonstration of csgrs capabilities.

pub mod advanced_features;
pub mod basic_shapes;
pub mod boolean_ops;
pub mod indexed_mesh;
pub mod sparse_voxels_demo;
pub mod transformations;
#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
pub mod wasm_example;

/// Run all example demonstrations
pub fn run_all_examples() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CSGRS Comprehensive Examples ===\n");

    // Basic shapes
    basic_shapes::run_basic_shapes_demo()?;
    basic_shapes::run_2d_shapes_demo()?;

    // Voxel operations
    basic_shapes::run_all_voxel_demos()?;

    // Sparse voxel demonstration
    sparse_voxels_demo::run_sparse_voxels_demo()?;

    // Transformations
    transformations::run_basic_transformations_demo()?;
    transformations::run_mirroring_demo()?;
    transformations::run_centering_demo()?;

    // Boolean operations
    boolean_ops::run_boolean_operations_demo()?;
    boolean_ops::run_complex_boolean_demo()?;
    boolean_ops::run_inversion_demo()?;

    // IndexedMesh features
    indexed_mesh::run_indexed_mesh_demo()?;

    // Advanced features
    advanced_features::run_extrusion_demo()?;
    advanced_features::run_mesh_processing_demo()?;
    advanced_features::run_metaballs_demo()?;
    advanced_features::run_sdf_demo()?;
    advanced_features::run_2d_boolean_demo()?;

    println!("\n=== All Examples Completed Successfully! ===");
    println!("STL files have been generated in the 'stl/' directory.");
    println!("You can view these files with any STL viewer (e.g., f3d, Meshlab, etc.)");

    Ok(())
}

/// Run specific example categories
pub fn run_basic_examples() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Basic CSGRS Examples ===\n");

    basic_shapes::run_basic_shapes_demo()?;
    transformations::run_basic_transformations_demo()?;
    boolean_ops::run_boolean_operations_demo()?;

    println!("\n=== Basic Examples Completed! ===");
    Ok(())
}

/// Run IndexedMesh example features
pub fn run_indexed_mesh_examples() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== IndexedMesh CSGRS Examples ===\n");

    indexed_mesh::run_basic_shapes_demo()?;
    indexed_mesh::run_boolean_operations_demo()?;
    indexed_mesh::run_connectivity_demo()?;
    indexed_mesh::run_memory_optimization_demo()?;

    println!("\n=== IndexedMesh Examples Completed! ===");
    Ok(())
}

/// Run advanced example features
pub fn run_advanced_examples() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Advanced CSGRS Examples ===\n");

    indexed_mesh::run_indexed_mesh_demo()?;
    advanced_features::run_extrusion_demo()?;
    advanced_features::run_mesh_processing_demo()?;
    advanced_features::run_metaballs_demo()?;
    advanced_features::run_sdf_demo()?;

    println!("\n=== Advanced Examples Completed! ===");
    Ok(())
}
