//! Basic geometric shape examples for csgrs
//!
//! This module demonstrates creation and export of fundamental 3D shapes
//! including cubes, spheres, and cylinders.

use crate::mesh::Mesh;
use crate::traits::CSG;
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

        let cylinder =
            MeshType::cylinder(1.0, 2.0, 32, None).expect("Failed to create cylinder");
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

/// Demonstrate dense voxel grid operations and export
pub fn run_voxel_grid_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running dense voxel grid demonstration...");

    // Ensure output directories exist
    fs::create_dir_all("stl")?;
    fs::create_dir_all("stl/voxels")?;
    fs::create_dir_all("voxels")?;

    use crate::voxels::{VoxelData, VoxelGrid, VoxelizationConfig, VoxelizationMode};
    use nalgebra::Point3;

    // 1) Create a voxel grid and set some voxels manually
    let mut grid = VoxelGrid::new(Point3::new(0.0, 0.0, 0.0), 0.5, (10, 10, 10), None);

    // Create a simple pattern - a diagonal line
    for i in 0..10 {
        grid.set_voxel(i, i, i, VoxelData::Occupied { metadata: Some(()) });
    }

    // Convert to mesh and export
    let mesh = grid.to_mesh(None);

    #[cfg(feature = "stl-io")]
    {
        let stl_data = mesh.to_stl_binary("voxel_grid_manual")?;
        fs::write("stl/voxels/voxel_grid_manual.stl", stl_data)?;
        println!("✓ Created voxel_grid_manual.stl");

        // Save grid statistics
        let stats = format!(
            "Voxel Grid Statistics:\n\
             Dimensions: {}x{}x{}\n\
             Voxel Size: {}\n\
             Occupied Voxels: {}\n\
             Occupancy Ratio: {:.2}%\n\
             Memory Usage: {} bytes",
            grid.dimensions.0,
            grid.dimensions.1,
            grid.dimensions.2,
            grid.voxel_size,
            grid.occupied_voxels(),
            grid.occupancy_ratio() * 100.0,
            grid.voxels.len() * std::mem::size_of::<VoxelData<()>>()
        );
        fs::write("voxels/grid_stats.txt", stats)?;
        println!("✓ Saved voxel grid statistics");
    }

    // 2) Voxelize meshes (if shape creation works)
    if let Ok(cube_mesh) = MeshType::cube(2.0, None) {
        let mut voxelized_grid = VoxelGrid::from_mesh_bounds(&cube_mesh, 0.25, 0.1, None);

        let config = VoxelizationConfig {
            mode: VoxelizationMode::Solid,
            default_metadata: None,
            parallel: false,
        };

        let occupied_count = voxelized_grid.voxelize_mesh(&cube_mesh, &config);
        let voxelized_mesh = voxelized_grid.to_mesh(None);

        #[cfg(feature = "stl-io")]
        {
            let stl_data = voxelized_mesh.to_stl_binary("voxelized_cube")?;
            fs::write("stl/voxels/voxelized_cube.stl", stl_data)?;
            println!(
                "✓ Created voxelized_cube.stl ({} occupied voxels)",
                occupied_count
            );

            // Save voxelization statistics
            let voxel_stats = format!(
                "Cube Voxelization Statistics:\n\
                 Original Mesh Polygons: {}\n\
                 Voxel Grid Size: {}x{}x{}\n\
                 Voxel Size: {}\n\
                 Occupied Voxels: {}\n\
                 Occupancy Ratio: {:.2}%",
                cube_mesh.polygons.len(),
                voxelized_grid.dimensions.0,
                voxelized_grid.dimensions.1,
                voxelized_grid.dimensions.2,
                voxelized_grid.voxel_size,
                occupied_count,
                voxelized_grid.occupancy_ratio() * 100.0
            );
            fs::write("voxels/cube_voxelization_stats.txt", voxel_stats)?;
            println!("✓ Saved cube voxelization statistics");
        }
    }

    // 3) Voxelize a sphere mesh for comparison
    if let Ok(sphere_mesh) = MeshType::sphere(1.5, 16, 8, None) {
        println!("Sphere mesh created with {} polygons", sphere_mesh.polygons.len());

        // Check if sphere mesh vertices are actually spherical
        let mut min_radius = f64::INFINITY;
        let mut max_radius = 0.0f64;
        for polygon in &sphere_mesh.polygons {
            for vertex in &polygon.vertices {
                let radius = vertex.pos.coords.norm();
                min_radius = min_radius.min(radius);
                max_radius = max_radius.max(radius);
            }
        }
        println!("Sphere vertex radii: min={:.3}, max={:.3} (expected ~1.5)", min_radius, max_radius);

        let bbox = sphere_mesh.bounding_box();
        println!("Sphere bounding box: [{:.2}, {:.2}, {:.2}] to [{:.2}, {:.2}, {:.2}]",
            bbox.mins.x, bbox.mins.y, bbox.mins.z,
            bbox.maxs.x, bbox.maxs.y, bbox.maxs.z);

        let mut sphere_voxelized_grid =
            VoxelGrid::from_mesh_bounds(&sphere_mesh, 0.2, 0.1, None);

        println!("Voxel grid dimensions: {}x{}x{} (size: {:.2})",
            sphere_voxelized_grid.dimensions.0,
            sphere_voxelized_grid.dimensions.1,
            sphere_voxelized_grid.dimensions.2,
            sphere_voxelized_grid.voxel_size);

        let config = VoxelizationConfig {
            mode: VoxelizationMode::Solid,
            default_metadata: None,
            parallel: false,
        };

        // Debug: Check some basic properties
        println!("Sphere has {} polygons", sphere_mesh.polygons.len());

        let sphere_occupied_count = sphere_voxelized_grid.voxelize_mesh(&sphere_mesh, &config);
        println!("Sphere voxelization completed: {} occupied voxels out of {} total",
            sphere_occupied_count,
            sphere_voxelized_grid.dimensions.0 * sphere_voxelized_grid.dimensions.1 * sphere_voxelized_grid.dimensions.2);

        let sphere_voxelized_mesh = sphere_voxelized_grid.to_mesh(None);

        #[cfg(feature = "stl-io")]
        {
            let stl_data = sphere_voxelized_mesh.to_stl_binary("voxelized_sphere")?;
            fs::write("stl/voxels/voxelized_sphere.stl", stl_data)?;
            println!(
                "✓ Created voxelized_sphere.stl ({} occupied voxels)",
                sphere_occupied_count
            );

            // Save sphere voxelization statistics
            let sphere_voxel_stats = format!(
                "Sphere Voxelization Statistics:\n\
                 Original Mesh Polygons: {}\n\
                 Voxel Grid Size: {}x{}x{}\n\
                 Voxel Size: {}\n\
                 Occupied Voxels: {}\n\
                 Occupancy Ratio: {:.2}%",
                sphere_mesh.polygons.len(),
                sphere_voxelized_grid.dimensions.0,
                sphere_voxelized_grid.dimensions.1,
                sphere_voxelized_grid.dimensions.2,
                sphere_voxelized_grid.voxel_size,
                sphere_occupied_count,
                sphere_voxelized_grid.occupancy_ratio() * 100.0
            );
            fs::write("voxels/sphere_voxelization_stats.txt", sphere_voxel_stats)?;
            println!("✓ Saved sphere voxelization statistics");
        }
    }

    println!("Dense voxel grid demonstration completed successfully!");
    Ok(())
}

/// Demonstrate sparse voxel octree operations and export
pub fn run_sparse_voxel_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running sparse voxel octree demonstration...");

    // Ensure output directories exist
    fs::create_dir_all("stl/voxels")?;
    fs::create_dir_all("voxels")?;

    use crate::voxels::SparseVoxelOctree;
    use nalgebra::Point3;

    // 1) Create a sparse voxel octree and add some voxels
    let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(
        Point3::new(0.0, 0.0, 0.0),
        8.0,
        6, // Increased max depth for finer resolution
        None,
    );

    // Create a high-resolution sphere using proper voxelization
    let radius = 3.0;
    let center = Point3::new(4.0, 4.0, 4.0);

    // Use much finer sampling for complete sphere filling
    let step = 0.05; // Much finer step size for dense sphere

    // Calculate the bounds for efficient sampling - only sample within sphere's bounding box
    let min_coord = (center.coords - nalgebra::Vector3::new(radius, radius, radius))
        .map(|c: f64| c.max(0.0));
    let max_coord = center.coords + nalgebra::Vector3::new(radius, radius, radius);

    // Convert to grid coordinates with larger bounds for complete coverage
    let min_x = ((min_coord.x / step) as usize).saturating_sub(2);
    let max_x = ((max_coord.x / step) as usize + 2).min(200);
    let min_y = ((min_coord.y / step) as usize).saturating_sub(2);
    let max_y = ((max_coord.y / step) as usize + 2).min(200);
    let min_z = ((min_coord.z / step) as usize).saturating_sub(2);
    let max_z = ((max_coord.z / step) as usize + 2).min(200);

    // Sample densely within the sphere's bounding box for proper filling
    for x in min_x..max_x {
        for y in min_y..max_y {
            for z in min_z..max_z {
                let point = Point3::new(x as f64 * step, y as f64 * step, z as f64 * step);
                let distance = (point - center).norm();
                if distance <= radius {
                    octree.set_voxel(&point, true, None);
                }
            }
        }
    }

    // Convert to mesh and export
    let mesh = octree.to_mesh();

    #[cfg(feature = "stl-io")]
    {
        let stl_data = mesh.to_stl_binary("sparse_voxel_sphere")?;
        fs::write("stl/voxels/sparse_voxel_sphere.stl", stl_data)?;
        println!("✓ Created sparse_voxel_sphere.stl");

        // Get and save memory statistics
        let memory_stats = octree.memory_stats();
        let stats = format!(
            "Sparse Voxel Octree Statistics:\n\
             Octree Size: {}\n\
             Max Depth: {}\n\
             Occupied Leaves: {}\n\
             Total Nodes: {}\n\
             Memory Usage: {} bytes\n\
             Compression Ratio: {}\n\
             Compression Savings: {}%",
            octree.size,
            octree.max_depth,
            octree.occupied_leaves,
            octree.total_nodes,
            memory_stats.memory_usage_bytes,
            memory_stats
                .compression_ratio
                .map_or("N/A".to_string(), |r| format!("{:.2}", r)),
            memory_stats
                .compression_savings_percent()
                .map_or("N/A".to_string(), |s| format!("{:.1}", s))
        );
        fs::write("voxels/sparse_octree_stats.txt", stats)?;
        println!("✓ Saved sparse octree statistics");
    }

    // 2) Demonstrate compression (if available)
    let mut compressed_octree: SparseVoxelOctree<()> =
        SparseVoxelOctree::new_compressed(Point3::new(0.0, 0.0, 0.0), 8.0, 6, None);

    // Add the same pattern to compressed octree (using same sampling density as uncompressed)
    for x in min_x..max_x {
        for y in min_y..max_y {
            for z in min_z..max_z {
                let point = Point3::new(x as f64 * step, y as f64 * step, z as f64 * step);
                let distance = (point - center).norm();
                if distance <= radius {
                    compressed_octree.set_voxel(&point, true, None);
                }
            }
        }
    }

    compressed_octree.compress_existing();

    let compressed_mesh = compressed_octree.to_mesh();

    #[cfg(feature = "stl-io")]
    {
        let stl_data = compressed_mesh.to_stl_binary("compressed_voxel_sphere")?;
        fs::write("stl/voxels/compressed_voxel_sphere.stl", stl_data)?;
        println!("✓ Created compressed_voxel_sphere.stl");

        // Compare memory usage
        let compressed_stats = compressed_octree.memory_stats();
        let comparison = format!(
            "Compression Comparison:\n\
             Uncompressed - Memory: {} bytes, Nodes: {}\n\
             Compressed - Memory: {} bytes, Nodes: {}\n\
             Space Savings: {}%",
            octree.memory_stats().memory_usage_bytes,
            octree.total_nodes,
            compressed_stats.memory_usage_bytes,
            compressed_octree.total_nodes,
            compressed_stats
                .compression_savings_percent()
                .map_or("N/A".to_string(), |s| format!("{:.1}", s))
        );
        fs::write("voxels/compression_comparison.txt", comparison)?;
        println!("✓ Saved compression comparison");
    }

    // Validate that the sphere has a reasonable number of occupied voxels
    // A sphere with radius 3.0 sampled at 0.05 intervals should have thousands of voxels
    assert!(
        octree.occupied_leaves > 2000,
        "High-density sphere voxelization should produce more than 2000 occupied voxels, got {}",
        octree.occupied_leaves
    );

    println!("Sparse voxel octree demonstration completed successfully!");
    Ok(())
}

/// Demonstrate CSG operations on sparse voxels
pub fn run_voxel_csg_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running voxel CSG operations demonstration...");

    // Ensure output directories exist
    fs::create_dir_all("stl/voxels")?;
    fs::create_dir_all("voxels")?;

    use crate::voxels::SparseVoxelOctree;
    use nalgebra::Point3;

    // 1) Create two voxel shapes for CSG operations
    let mut sphere1: SparseVoxelOctree<()> =
        SparseVoxelOctree::new(Point3::new(0.0, 0.0, 0.0), 8.0, 6, None);

    let mut sphere2: SparseVoxelOctree<()> =
        SparseVoxelOctree::new(Point3::new(0.0, 0.0, 0.0), 8.0, 6, None);

    // Create overlapping spheres with high-density sampling
    let center1 = Point3::new(3.0, 3.0, 3.0);
    let center2 = Point3::new(5.0, 5.0, 5.0);
    let radius = 2.5;
    let step = 0.1; // Much finer step size for dense spheres

    // Sample densely within the spheres' bounding boxes
    let min_bound = 0.0;
    let max_bound = 8.0;
    let steps = ((max_bound - min_bound) / step) as usize;

    for x in 0..steps {
        for y in 0..steps {
            for z in 0..steps {
                let point = Point3::new(
                    min_bound + x as f64 * step,
                    min_bound + y as f64 * step,
                    min_bound + z as f64 * step,
                );

                let dist1 = (point - center1).norm();
                let dist2 = (point - center2).norm();

                if dist1 <= radius {
                    sphere1.set_voxel(&point, true, None);
                }
                if dist2 <= radius {
                    sphere2.set_voxel(&point, true, None);
                }
            }
        }
    }

    // 2) Perform CSG operations
    println!("Performing CSG operations...");

    let union_result = sphere1.csg_union(&sphere2);
    let intersection_result = sphere1.csg_intersection(&sphere2);
    let difference_result = sphere1.csg_difference(&sphere2);
    let xor_result = sphere1.csg_symmetric_difference(&sphere2);

    // Validate that spheres have reasonable voxel counts with high-density sampling
    assert!(
        sphere1.occupied_leaves > 500,
        "Sphere 1 should have more than 500 occupied voxels with dense sampling, got {}",
        sphere1.occupied_leaves
    );
    assert!(
        sphere2.occupied_leaves > 500,
        "Sphere 2 should have more than 500 occupied voxels with dense sampling, got {}",
        sphere2.occupied_leaves
    );

    // 3) Export results
    #[cfg(feature = "stl-io")]
    {
        // Export individual spheres
        let sphere1_mesh = sphere1.to_mesh();
        let stl_data = sphere1_mesh.to_stl_binary("voxel_sphere1")?;
        fs::write("stl/voxels/voxel_sphere1.stl", stl_data)?;
        println!("✓ Created voxel_sphere1.stl");

        let sphere2_mesh = sphere2.to_mesh();
        let stl_data = sphere2_mesh.to_stl_binary("voxel_sphere2")?;
        fs::write("stl/voxels/voxel_sphere2.stl", stl_data)?;
        println!("✓ Created voxel_sphere2.stl");

        // Export CSG results
        let union_mesh = union_result.to_mesh();
        let stl_data = union_mesh.to_stl_binary("voxel_union")?;
        fs::write("stl/voxels/voxel_union.stl", stl_data)?;
        println!("✓ Created voxel_union.stl");

        let intersection_mesh = intersection_result.to_mesh();
        let stl_data = intersection_mesh.to_stl_binary("voxel_intersection")?;
        fs::write("stl/voxels/voxel_intersection.stl", stl_data)?;
        println!("✓ Created voxel_intersection.stl");

        let difference_mesh = difference_result.to_mesh();
        let stl_data = difference_mesh.to_stl_binary("voxel_difference")?;
        fs::write("stl/voxels/voxel_difference.stl", stl_data)?;
        println!("✓ Created voxel_difference.stl");

        let xor_mesh = xor_result.to_mesh();
        let stl_data = xor_mesh.to_stl_binary("voxel_xor")?;
        fs::write("stl/voxels/voxel_xor.stl", stl_data)?;
        println!("✓ Created voxel_xor.stl");

        // Save CSG statistics
        let csg_stats = format!(
            "Voxel CSG Operations Statistics:\n\
             Sphere 1 - Occupied: {}\n\
             Sphere 2 - Occupied: {}\n\
             Union - Occupied: {}\n\
             Intersection - Occupied: {}\n\
             Difference - Occupied: {}\n\
             XOR - Occupied: {}",
            sphere1.occupied_leaves,
            sphere2.occupied_leaves,
            union_result.occupied_leaves,
            intersection_result.occupied_leaves,
            difference_result.occupied_leaves,
            xor_result.occupied_leaves
        );
        fs::write("voxels/csg_operations_stats.txt", csg_stats)?;
        println!("✓ Saved CSG operations statistics");
    }

    println!("Voxel CSG operations demonstration completed successfully!");
    Ok(())
}

/// Demonstrate voxel transformations and export
pub fn run_voxel_transform_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("Running voxel transformation demonstration...");

    // Ensure output directories exist
    fs::create_dir_all("stl/voxels")?;
    fs::create_dir_all("voxels")?;

    use crate::traits::CSG;
    use crate::voxels::SparseVoxelOctree;
    use nalgebra::{Point3, Rotation3, Translation3, Vector3};

    // 1) Create a base voxel shape
    let mut octree: SparseVoxelOctree<()> =
        SparseVoxelOctree::new(Point3::new(0.0, 0.0, 0.0), 8.0, 4, None);

    // Create a cube pattern
    for x in 2..6 {
        for y in 2..6 {
            for z in 2..6 {
                let point = Point3::new(x as f64, y as f64, z as f64);
                octree.set_voxel(&point, true, None);
            }
        }
    }

    // 2) Apply transformations using the CSG trait
    let translate_matrix = Translation3::new(3.0, 0.0, 0.0).to_homogeneous();
    let translated = octree.transform(&translate_matrix);

    let rotate_matrix =
        Rotation3::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI / 4.0)
            .to_homogeneous();
    let rotated = octree.transform(&rotate_matrix);

    let scale_matrix = nalgebra::Matrix4::new_nonuniform_scaling(&Vector3::new(1.5, 0.5, 2.0));
    let scaled = octree.transform(&scale_matrix);

    // 3) Export results
    #[cfg(feature = "stl-io")]
    {
        // Export original
        let original_mesh = octree.to_mesh();
        let stl_data = original_mesh.to_stl_binary("voxel_original")?;
        fs::write("stl/voxels/voxel_original.stl", stl_data)?;
        println!("✓ Created voxel_original.stl");

        // Export translated
        let translated_mesh = translated.to_mesh();
        let stl_data = translated_mesh.to_stl_binary("voxel_translated")?;
        fs::write("stl/voxels/voxel_translated.stl", stl_data)?;
        println!("✓ Created voxel_translated.stl");

        // Export rotated
        let rotated_mesh = rotated.to_mesh();
        let stl_data = rotated_mesh.to_stl_binary("voxel_rotated")?;
        fs::write("stl/voxels/voxel_rotated.stl", stl_data)?;
        println!("✓ Created voxel_rotated.stl");

        // Export scaled
        let scaled_mesh = scaled.to_mesh();
        let stl_data = scaled_mesh.to_stl_binary("voxel_scaled")?;
        fs::write("stl/voxels/voxel_scaled.stl", stl_data)?;
        println!("✓ Created voxel_scaled.stl");

        // Save transformation statistics
        let transform_stats = format!(
            "Voxel Transformation Statistics:\n\
             Original - Occupied: {}\n\
             Translated - Occupied: {}\n\
             Rotated - Occupied: {}\n\
             Scaled - Occupied: {}\n\
             Transformations applied via CSG trait methods",
            octree.occupied_leaves,
            translated.occupied_leaves,
            rotated.occupied_leaves,
            scaled.occupied_leaves
        );
        fs::write("voxels/transformation_stats.txt", transform_stats)?;
        println!("✓ Saved transformation statistics");
    }

    println!("Voxel transformation demonstration completed successfully!");
    Ok(())
}

/// Demonstrate surface quality differences between mesh and voxel representations
pub fn run_surface_quality_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Surface Quality Comparison Demonstration ===");

    // Ensure output directories exist
    std::fs::create_dir_all("stl")?;
    std::fs::create_dir_all("stl/voxels")?;

    // Create a smooth mesh sphere
    let smooth_sphere: crate::mesh::Mesh<()> =
        crate::mesh::Mesh::sphere(1.0, 32, 16, None).expect("Failed to create smooth sphere");

    println!("Mesh Sphere (Smooth Surface):");
    println!("  - Polygons: {}", smooth_sphere.polygons.len());
    println!("  - Mathematical surface with smooth curvature");
    println!("  - Precise geometric calculations");
    println!("  - Ideal for rendering and smooth surfaces");

    // Export smooth mesh
    #[cfg(feature = "stl-io")]
    {
        let stl_data = smooth_sphere.to_stl_ascii("smooth_sphere");
        std::fs::write("stl/smooth_sphere.stl", stl_data)?;
        println!("✓ Created smooth_sphere.stl");
    }

    // Convert to sparse voxels and back to mesh
    let voxel_sphere =
        crate::voxels::octree::SparseVoxelOctree::from_mesh(&smooth_sphere, 0.1, None);

    // Convert back to mesh to show surface quality
    let voxel_mesh = voxel_sphere.to_mesh();

    println!("\nVoxel Sphere (Blocky Surface):");
    println!("  - Original mesh polygons: {}", smooth_sphere.polygons.len());
    println!("  - Voxel mesh polygons: {}", voxel_mesh.polygons.len());
    println!("  - Blocky, faceted surface from discrete sampling");
    println!("  - Each occupied voxel becomes a cube");
    println!("  - Surface quality depends on voxel resolution");
    println!("  - Ideal for volume operations and CSG");

    // Export voxel mesh
    #[cfg(feature = "stl-io")]
    {
        let stl_data = voxel_mesh.to_stl_ascii("voxel_sphere");
        std::fs::write("stl/voxels/voxel_sphere.stl", stl_data)?;
        println!("✓ Created voxel_sphere.stl");
    }

    println!("\nKey Differences:");
    println!("  • Mesh: Smooth mathematical surfaces, precise geometry");
    println!("  • Voxels: Blocky discrete representations, efficient volume operations");
    println!("  • Mesh: Better for visualization and smooth surfaces");
    println!("  • Voxels: Better for boolean operations on large volumes");
    println!("  • Mesh: Sub-pixel precision, mathematical accuracy");
    println!("  • Voxels: Grid-based sampling, memory efficient for sparse data");

    println!("\nCompare the STL files to see the surface quality difference!");
    println!("Surface quality demonstration completed successfully!");
    Ok(())
}

/// Run all voxel demonstrations
pub fn run_all_voxel_demos() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Running All Voxel Demonstrations ===\n");

    run_surface_quality_demo()?;
    println!();

    run_voxel_grid_demo()?;
    println!();

    run_sparse_voxel_demo()?;
    println!();

    run_voxel_csg_demo()?;
    println!();

    run_voxel_transform_demo()?;

    println!("\n=== All Voxel Demonstrations Completed Successfully ===");
    println!("Check the following directories for results:");
    println!("  - stl/         : Mesh STL files for visualization");
    println!("  - stl/voxels/  : Voxel STL files for visualization");
    println!("  - voxels/      : Statistics and analysis files");

    Ok(())
}
