//! Basic geometric shape examples for csgrs
//!
//! This module demonstrates creation and export of fundamental 3D shapes
//! including cubes, spheres, and cylinders.

use crate::mesh::Mesh;
use std::fs;

type MeshType = Mesh<()>;

/// Demonstrate basic 3D shape creation and STL export
pub fn run_basic_shapes_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running basic shapes demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    // 1) Basic shapes: cube, sphere, cylinder
    let cube = MeshType::cube(2.0, None).expect("Failed to create cube");

    #[cfg(feature = "stl-io")]
    {
        let stl_data = cube.to_stl_binary("cube")?;
        fs::write("stl/cube.stl", stl_data)?;
        println!("✓ Created cube.stl");

        let sphere = MeshType::sphere(1.0, 16, 8, None).expect("Failed to create sphere");
        let stl_data = sphere.to_stl_binary("sphere")?;
        fs::write("stl/sphere.stl", stl_data)?;
        println!("✓ Created sphere.stl");

        let cylinder = MeshType::cylinder(1.0, 2.0, 32, None).expect("Failed to create cylinder");
        let stl_data = cylinder.to_stl_binary("cylinder")?;
        fs::write("stl/cylinder.stl", stl_data)?;
        println!("✓ Created cylinder.stl");
    }

    println!("Basic shapes demonstration completed successfully!");
    Ok(())
}

/// Demonstrate 2D shape creation and export
pub fn run_2d_shapes_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running 2D shapes demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    use crate::sketch::Sketch;
    type SketchType = Sketch<()>;

    #[cfg(feature = "stl-io")]
    {
        let square_2d = SketchType::square(2.0, None);
        let stl_data = square_2d.to_stl_ascii("square_2d");
        fs::write("stl/square_2d.stl", stl_data)?;
        println!("✓ Created square_2d.stl");

        let circle_2d = SketchType::circle(1.0, 32, None);
        let stl_data = circle_2d.to_stl_binary("circle_2d")?;
        fs::write("stl/circle_2d.stl", stl_data)?;
        println!("✓ Created circle_2d.stl");

        let star_2d = SketchType::star(5, 2.0, 0.8, None);
        let stl_data = star_2d.to_stl_ascii("star_2d");
        fs::write("stl/star_2d.stl", stl_data)?;
        println!("✓ Created star_2d.stl");
    }

    println!("2D shapes demonstration completed successfully!");
    Ok(())
}
