//! Geometric transformation examples for csgrs
//!
//! This module demonstrates various affine transformations including
//! translation, rotation, scaling, and mirroring operations.

use crate::mesh::Mesh;
use crate::mesh::plane::Plane;
use crate::traits::CSG;
use nalgebra::Vector3;
use std::fs;

type MeshType = Mesh<()>;

/// Demonstrate basic transformations: translate, rotate, scale
pub fn run_basic_transformations_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running basic transformations demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    // Start with a cube
    let cube = MeshType::cube(2.0, None).expect("Failed to create cube");

    #[cfg(feature = "stl-io")]
    {
        // Translate
        let translated = cube.translate(3.0, 0.0, 0.0);
        let stl_data = translated.to_stl_binary("cube_translated")?;
        fs::write("stl/cube_translated.stl", stl_data)?;
        println!("✓ Created cube_translated.stl");

        // Rotate
        let rotated = cube.rotate(0.0, 45.0, 0.0);
        let stl_data = rotated.to_stl_binary("cube_rotated")?;
        fs::write("stl/cube_rotated.stl", stl_data)?;
        println!("✓ Created cube_rotated.stl");

        // Scale
        let scaled = cube.scale(1.0, 2.0, 0.5);
        let stl_data = scaled.to_stl_binary("cube_scaled")?;
        fs::write("stl/cube_scaled.stl", stl_data)?;
        println!("✓ Created cube_scaled.stl");

        // Combined transformation
        let transformed = cube
            .translate(1.0, 0.0, 0.0)
            .rotate(0.0, 45.0, 0.0)
            .scale(1.0, 0.5, 2.0);
        let stl_data = transformed.to_stl_binary("cube_transformed")?;
        fs::write("stl/cube_transformed.stl", stl_data)?;
        println!("✓ Created cube_transformed.stl");
    }

    println!("Basic transformations demonstration completed successfully!");
    Ok(())
}

/// Demonstrate mirroring across planes
pub fn run_mirroring_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running mirroring demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    let cube = MeshType::cube(2.0, None).expect("Failed to create cube");

    #[cfg(feature = "stl-io")]
    {
        // Mirror across X=0 plane
        let plane_x = Plane::from_normal(Vector3::x(), 0.0);
        let mirrored_x = cube.mirror(plane_x);
        let stl_data = mirrored_x.to_stl_binary("cube_mirrored_x")?;
        fs::write("stl/cube_mirrored_x.stl", stl_data)?;
        println!("✓ Created cube_mirrored_x.stl");

        // Mirror across Y=0 plane
        let plane_y = Plane::from_normal(Vector3::y(), 0.0);
        let mirrored_y = cube.mirror(plane_y);
        let stl_data = mirrored_y.to_stl_binary("cube_mirrored_y")?;
        fs::write("stl/cube_mirrored_y.stl", stl_data)?;
        println!("✓ Created cube_mirrored_y.stl");

        // Mirror across Z=0 plane
        let plane_z = Plane::from_normal(Vector3::z(), 0.0);
        let mirrored_z = cube.mirror(plane_z);
        let stl_data = mirrored_z.to_stl_binary("cube_mirrored_z")?;
        fs::write("stl/cube_mirrored_z.stl", stl_data)?;
        println!("✓ Created cube_mirrored_z.stl");
    }

    println!("Mirroring demonstration completed successfully!");
    Ok(())
}

/// Demonstrate center() and float() operations
pub fn run_centering_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running centering demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    // Create an off-center shape
    let off_center_cube = MeshType::cube(2.0, None).expect("Failed to create cube").translate(5.0, 3.0, 1.0);

    #[cfg(feature = "stl-io")]
    {
        // Show original position
        let stl_data = off_center_cube.to_stl_binary("cube_off_center")?;
        fs::write("stl/cube_off_center.stl", stl_data)?;
        println!("✓ Created cube_off_center.stl");

        // Center the shape
        let centered = off_center_cube.center();
        let stl_data = centered.to_stl_binary("cube_centered")?;
        fs::write("stl/cube_centered.stl", stl_data)?;
        println!("✓ Created cube_centered.stl");

        // Float to Z=0
        let sphere_low = MeshType::sphere(1.0, 16, 8, None).expect("Failed to create sphere").translate(0.0, 0.0, -2.0);
        let floated = sphere_low.float();
        let stl_data = floated.to_stl_binary("sphere_floated")?;
        fs::write("stl/sphere_floated.stl", stl_data)?;
        println!("✓ Created sphere_floated.stl");
    }

    println!("Centering demonstration completed successfully!");
    Ok(())
}
