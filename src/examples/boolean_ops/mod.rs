//! Boolean operations examples for csgrs
//!
//! This module demonstrates constructive solid geometry operations:
//! union, difference, intersection, and XOR operations on meshes.

use crate::mesh::Mesh;
use crate::traits::CSG;
use std::fs;

type MeshType = Mesh<()>;

/// Demonstrate basic boolean operations: union, difference, intersection
pub fn run_boolean_operations_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running boolean operations demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    // Create base shapes
    let cube = MeshType::cube(2.0, None).expect("Failed to create cube");
    let sphere = MeshType::sphere(1.25, 16, 8, None)
        .expect("Failed to create sphere")
        .translate(1.0, 1.0, 1.0);

    #[cfg(feature = "stl-io")]
    {
        // Union
        let union_result = cube.union(&sphere);
        let stl_data = union_result.to_stl_binary("union_cube_sphere")?;
        fs::write("stl/union_cube_sphere.stl", stl_data)?;
        println!("✓ Created union_cube_sphere.stl");

        // Difference
        let difference_result = cube.difference(&sphere);
        let stl_data = difference_result.to_stl_binary("difference_cube_sphere")?;
        fs::write("stl/difference_cube_sphere.stl", stl_data)?;
        println!("✓ Created difference_cube_sphere.stl");

        // Intersection
        let intersection_result = cube.intersection(&sphere);
        let stl_data = intersection_result.to_stl_binary("intersection_cube_sphere")?;
        fs::write("stl/intersection_cube_sphere.stl", stl_data)?;
        println!("✓ Created intersection_cube_sphere.stl");

        // XOR
        let xor_result = cube.xor(&sphere);
        let stl_data = xor_result.to_stl_binary("xor_cube_sphere")?;
        fs::write("stl/xor_cube_sphere.stl", stl_data)?;
        println!("✓ Created xor_cube_sphere.stl");
    }

    println!("Boolean operations demonstration completed successfully!");
    Ok(())
}

/// Demonstrate complex boolean operations with multiple shapes
pub fn run_complex_boolean_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running complex boolean operations demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    // Create multiple shapes for complex operations
    let cube1 = MeshType::cube(3.0, None).expect("Failed to create cube");
    let cube2 = MeshType::cube(3.0, None)
        .expect("Failed to create cube")
        .translate(1.0, 1.0, 1.0);
    let sphere = MeshType::sphere(2.0, 16, 8, None).expect("Failed to create sphere");

    #[cfg(feature = "stl-io")]
    {
        // Complex union
        let complex_union = cube1.union(&cube2).union(&sphere);
        let stl_data = complex_union.to_stl_binary("complex_union")?;
        fs::write("stl/complex_union.stl", stl_data)?;
        println!("✓ Created complex_union.stl");

        // Complex difference (sphere cut from union of cubes)
        let complex_diff = cube1.union(&cube2).difference(&sphere);
        let stl_data = complex_diff.to_stl_binary("complex_difference")?;
        fs::write("stl/complex_difference.stl", stl_data)?;
        println!("✓ Created complex_difference.stl");

        // Intersection of multiple shapes
        let complex_intersect = cube1.intersection(&cube2).intersection(&sphere);
        let stl_data = complex_intersect.to_stl_binary("complex_intersection")?;
        fs::write("stl/complex_intersection.stl", stl_data)?;
        println!("✓ Created complex_intersection.stl");
    }

    println!("Complex boolean operations demonstration completed successfully!");
    Ok(())
}

/// Demonstrate boolean operations with inverted shapes
pub fn run_inversion_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running inversion demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    let cube = MeshType::cube(3.0, None).expect("Failed to create cube");
    let sphere = MeshType::sphere(1.5, 16, 8, None).expect("Failed to create sphere");

    #[cfg(feature = "stl-io")]
    {
        // Normal difference
        let normal_diff = cube.difference(&sphere);
        let stl_data = normal_diff.to_stl_binary("normal_difference")?;
        fs::write("stl/normal_difference.stl", stl_data)?;
        println!("✓ Created normal_difference.stl");

        // Difference with inverted sphere (creates a cavity)
        let inverted_sphere = sphere.inverse();
        let cavity_diff = cube.difference(&inverted_sphere);
        let stl_data = cavity_diff.to_stl_binary("cavity_difference")?;
        fs::write("stl/cavity_difference.stl", stl_data)?;
        println!("✓ Created cavity_difference.stl");

        // Union with inverted shape
        let inverted_cube = cube.inverse();
        let union_inverted = sphere.union(&inverted_cube);
        let stl_data = union_inverted.to_stl_binary("union_with_inverted")?;
        fs::write("stl/union_with_inverted.stl", stl_data)?;
        println!("✓ Created union_with_inverted.stl");
    }

    println!("Inversion demonstration completed successfully!");
    Ok(())
}
