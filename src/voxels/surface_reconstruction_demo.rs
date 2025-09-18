//! # Surface Reconstruction Demo
//!
//! This module demonstrates the improvement from naive voxel-to-mesh conversion
//! to proper surface reconstruction with face culling.

use crate::voxels::conversion::SurfaceReconstructionMode;
use crate::voxels::octree::SparseVoxelOctree;
use nalgebra::{Point3, Vector3};

/// Create a simple test voxel structure and compare mesh outputs
pub fn demonstrate_surface_reconstruction() {
    println!("=== Surface Reconstruction Demo ===");

    // Create a simple 2x2x2 voxel cube
    let origin = Point3::new(0.0, 0.0, 0.0);
    let size = 2.0; // 2 voxels of size 1 each
    let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(origin, size, 1, None);

    // Fill two adjacent voxels - this will have some faces culled
    let voxel1 = Point3::new(0.5, 0.5, 0.5); // Center of first voxel
    let voxel2 = Point3::new(1.5, 0.5, 0.5); // Center of second voxel (adjacent in X direction)
    octree.set_voxel(&voxel1, true, None);
    octree.set_voxel(&voxel2, true, None);

    println!("Created two adjacent voxels (2 occupied voxels)");

    // Test naive approach (original pixelated method)
    let naive_mesh = octree.to_mesh_with_reconstruction(SurfaceReconstructionMode::Naive);
    println!("Naive approach: {} polygons", naive_mesh.polygons.len());

    // Test face culling approach (improved method)
    let surface_mesh =
        octree.to_mesh_with_reconstruction(SurfaceReconstructionMode::FaceCulling);
    println!(
        "Face culling approach: {} polygons",
        surface_mesh.polygons.len()
    );

    // Test marching cubes approach (best method)
    let marching_cubes_mesh =
        octree.to_mesh_with_reconstruction(SurfaceReconstructionMode::MarchingCubes);
    println!(
        "Marching cubes approach: {} polygons",
        marching_cubes_mesh.polygons.len()
    );

    // Test dual contouring approach (sharp features)
    let dual_contouring_mesh =
        octree.to_mesh_with_reconstruction(SurfaceReconstructionMode::DualContouring);
    println!(
        "Dual contouring approach: {} polygons",
        dual_contouring_mesh.polygons.len()
    );

    // Debug: Check a few voxel positions
    let test_point = Point3::new(0.5, 0.5, 0.5);
    println!(
        "Voxel at (0.5,0.5,0.5) occupied: {:?}",
        octree.get_voxel(&test_point)
    );

    let test_point2 = Point3::new(1.5, 0.5, 0.5);
    println!(
        "Voxel at (1.5,0.5,0.5) occupied: {:?}",
        octree.get_voxel(&test_point2)
    );

    // Debug: Check adjacent positions for face culling
    let center_voxel = Point3::new(1.0, 0.5, 0.5); // Center between the two voxels
    println!(
        "Center voxel at (1.0,0.5,0.5) occupied: {:?}",
        octree.get_voxel(&center_voxel)
    );

    // Check all 6 adjacent positions
    let adjacent_positions = [
        center_voxel + Vector3::new(0.0, 0.0, -1.0), // bottom
        center_voxel + Vector3::new(0.0, 0.0, 1.0),  // top
        center_voxel + Vector3::new(0.0, -1.0, 0.0), // front
        center_voxel + Vector3::new(0.0, 1.0, 0.0),  // back
        center_voxel + Vector3::new(-1.0, 0.0, 0.0), // left
        center_voxel + Vector3::new(1.0, 0.0, 0.0),  // right
    ];

    for (i, pos) in adjacent_positions.iter().enumerate() {
        let occupied = octree.get_voxel(pos).unwrap_or(false);
        println!("Adjacent position {}: {:?} -> {:?}", i, pos, occupied);
    }

    // Calculate improvement
    let naive_count = naive_mesh.polygons.len();
    let surface_count = surface_mesh.polygons.len();
    let marching_cubes_count = marching_cubes_mesh.polygons.len();
    let dual_contouring_count = dual_contouring_mesh.polygons.len();

    let face_culling_reduction =
        ((naive_count - surface_count) as f64 / naive_count as f64) * 100.0;
    let marching_cubes_reduction =
        ((naive_count - marching_cubes_count) as f64 / naive_count as f64) * 100.0;
    let dual_contouring_reduction =
        ((naive_count - dual_contouring_count) as f64 / naive_count as f64) * 100.0;

    println!(
        "Face culling reduction: {:.1}% ({} -> {})",
        face_culling_reduction, naive_count, surface_count
    );
    println!(
        "Marching cubes reduction: {:.1}% ({} -> {})",
        marching_cubes_reduction, naive_count, marching_cubes_count
    );
    println!(
        "Dual contouring reduction: {:.1}% ({} -> {})",
        dual_contouring_reduction, naive_count, dual_contouring_count
    );

    // Expected results:
    // - Naive: 2 voxels * 6 faces = 12 polygons
    // - Face culling: 2 faces culled (shared face) = 10 polygons
    // - Marching cubes: 2 faces culled (shared face) = 10 polygons
    // - Dual contouring: 2 faces culled (shared face) = 10 polygons

    // For two adjacent voxels, the shared face should be culled
    assert_eq!(naive_count, 12, "Two voxels should have 12 faces");
    assert!(
        surface_count < naive_count,
        "Face culling should reduce polygon count for adjacent voxels"
    );
    assert!(
        marching_cubes_count < naive_count,
        "Marching cubes should reduce polygon count for adjacent voxels"
    );
    assert!(
        dual_contouring_count < naive_count,
        "Dual contouring should reduce polygon count for adjacent voxels"
    );

    println!("✅ Surface reconstruction demo completed successfully!");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_surface_reconstruction_improvement() {
        demonstrate_surface_reconstruction();
    }
}
