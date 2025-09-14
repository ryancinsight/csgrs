//! Advanced features examples for csgrs
//!
//! This module demonstrates advanced geometric operations including
//! extrusions, mesh processing, and complex shape generation.

use crate::mesh::Mesh;
use crate::sketch::Sketch;
use crate::traits::CSG;
use nalgebra::Vector3;
use std::fs;
use std::num::NonZeroU32;

#[cfg(feature = "metaballs")]
use crate::mesh::metaballs::MetaBall;

type MeshType = Mesh<()>;
type SketchType = Sketch<()>;

/// Demonstrate extrusion and revolution operations
pub fn run_extrusion_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running extrusion demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    #[cfg(feature = "stl-io")]
    {
        // Basic extrusion
        let square = SketchType::square(2.0, None);
        let prism = square.extrude(3.0);
        let stl_data = prism.to_stl_binary("prism")?;
        fs::write("stl/prism.stl", stl_data)?;
        println!("✓ Created prism.stl");

        // Vector extrusion (angled)
        let vector_extruded = square.extrude_vector(Vector3::new(2.0, 1.0, 3.0));
        let stl_data = vector_extruded.to_stl_binary("vector_extruded")?;
        fs::write("stl/vector_extruded.stl", stl_data)?;
        println!("✓ Created vector_extruded.stl");

        // Revolution
        let circle = SketchType::circle(1.0, 32, None);
        let revolved = circle.revolve(360.0, 32)?;
        let stl_data = revolved.to_stl_binary("revolved_circle")?;
        fs::write("stl/revolved_circle.stl", stl_data)?;
        println!("✓ Created revolved_circle.stl");

        // Partial revolution
        let partial_revolved = circle.revolve(180.0, 32)?;
        let stl_data = partial_revolved.to_stl_binary("partial_revolution")?;
        fs::write("stl/partial_revolution.stl", stl_data)?;
        println!("✓ Created partial_revolution.stl");
    }

    println!("Extrusion demonstration completed successfully!");
    Ok(())
}

/// Demonstrate mesh processing operations
pub fn run_mesh_processing_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running mesh processing demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    let sphere = MeshType::sphere(1.0, 16, 8, None).expect("Failed to create sphere");

    #[cfg(feature = "stl-io")]
    {
        // Subdivide triangles for smoother surface
        let subdivided =
            sphere.subdivide_triangles(NonZeroU32::try_from(1u32).expect("1 is not zero"));
        let stl_data = subdivided.to_stl_binary("subdivided_sphere")?;
        fs::write("stl/subdivided_sphere.stl", stl_data)?;
        println!("✓ Created subdivided_sphere.stl");

        // Renormalize (recompute normals)
        let mut sphere_clone = sphere.clone();
        sphere_clone.renormalize();
        let stl_data = sphere_clone.to_stl_binary("renormalized_sphere")?;
        fs::write("stl/renormalized_sphere.stl", stl_data)?;
        println!("✓ Created renormalized_sphere.stl");

        // Triangulate (force all polygons to triangles)
        let triangulated = sphere.triangulate();
        let stl_data = triangulated.to_stl_binary("triangulated_sphere")?;
        fs::write("stl/triangulated_sphere.stl", stl_data)?;
        println!("✓ Created triangulated_sphere.stl");
    }

    println!("Mesh processing demonstration completed successfully!");
    Ok(())
}

/// Demonstrate metaballs if feature is enabled
#[cfg(feature = "metaballs")]
pub fn run_metaballs_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running metaballs demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    use nalgebra::Point3;

    #[cfg(feature = "stl-io")]
    {
        // Create metaballs
        let balls = vec![
            MetaBall::new(Point3::origin(), 1.0),
            MetaBall::new(Point3::new(1.5, 0.0, 0.0), 1.0),
            MetaBall::new(Point3::new(0.75, 1.0, 0.5), 0.8),
        ];

        let resolution = (40, 40, 40);
        let iso_value = 1.0;
        let padding = 1.0;

        let metaball_mesh = MeshType::metaballs(&balls, resolution, iso_value, padding, None);
        let stl_data = metaball_mesh.to_stl_binary("metaballs")?;
        fs::write("stl/metaballs.stl", stl_data)?;
        println!("✓ Created metaballs.stl");
    }

    println!("Metaballs demonstration completed successfully!");
    Ok(())
}

#[cfg(not(feature = "metaballs"))]
pub fn run_metaballs_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Metaballs feature not enabled, skipping metaballs demonstration");
    Ok(())
}

/// Demonstrate signed distance field operations
#[cfg(feature = "sdf")]
pub fn run_sdf_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running SDF demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    use nalgebra::Point3;

    #[cfg(feature = "stl-io")]
    {
        // SDF for sphere
        let sphere_sdf = |p: &Point3<f64>| p.coords.norm() - 1.5;

        let resolution = (40, 40, 40);
        let min_pt = Point3::new(-2.0, -2.0, -2.0);
        let max_pt = Point3::new(2.0, 2.0, 2.0);
        let iso_value = 0.0;

        let sdf_mesh = MeshType::sdf(sphere_sdf, resolution, min_pt, max_pt, iso_value, None);
        let stl_data = sdf_mesh.to_stl_binary("sdf_sphere")?;
        fs::write("stl/sdf_sphere.stl", stl_data)?;
        println!("✓ Created sdf_sphere.stl");
    }

    println!("SDF demonstration completed successfully!");
    Ok(())
}

#[cfg(not(feature = "sdf"))]
pub fn run_sdf_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("SDF feature not enabled, skipping SDF demonstration");
    Ok(())
}

/// Demonstrate 2D boolean operations
pub fn run_2d_boolean_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running 2D boolean operations demonstration...");

    // Ensure output directory exists
    fs::create_dir_all("stl")?;

    #[cfg(feature = "stl-io")]
    {
        let circle1 = SketchType::circle(2.0, 32, None);
        let circle2 = SketchType::circle(2.0, 32, None).translate(1.0, 0.0, 0.0);

        // 2D union
        let union_2d = circle1.union(&circle2);
        let stl_data = union_2d.to_stl_ascii("union_2d");
        fs::write("stl/union_2d.stl", stl_data)?;
        println!("✓ Created union_2d.stl");

        // 2D difference
        let difference_2d = circle1.difference(&circle2);
        let stl_data = difference_2d.to_stl_ascii("difference_2d");
        fs::write("stl/difference_2d.stl", stl_data)?;
        println!("✓ Created difference_2d.stl");

        // 2D intersection
        let intersection_2d = circle1.intersection(&circle2);
        let stl_data = intersection_2d.to_stl_ascii("intersection_2d");
        fs::write("stl/intersection_2d.stl", stl_data)?;
        println!("✓ Created intersection_2d.stl");
    }

    println!("2D boolean operations demonstration completed successfully!");
    Ok(())
}
