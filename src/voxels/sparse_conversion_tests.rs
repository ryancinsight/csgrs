//! # Sparse Voxel Conversion Tests
//!
//! This module provides comprehensive tests for sparse voxel conversion capabilities.

use crate::float_types::Real;
use crate::voxels::conversion::SurfaceReconstructionMode;
use crate::voxels::formats::SparseVoxelFormat;
use crate::voxels::octree::SparseVoxelOctree;
use nalgebra::Point3;

/// Test sparse voxel to dense grid conversion
pub fn test_sparse_to_dense_conversion() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Testing Sparse to Dense Grid Conversion ===");
    
    // Create a simple sparse voxel octree
    let origin = Point3::new(0.0, 0.0, 0.0);
    let size = 4.0;
    let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(origin, size, 3, None);
    
    // Add some voxels
    let voxel_positions = [
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(3.0, 1.0, 1.0),
        Point3::new(1.0, 3.0, 1.0),
        Point3::new(3.0, 3.0, 1.0),
    ];
    
    for pos in &voxel_positions {
        octree.set_voxel(pos, true, None);
    }
    
    println!("Created sparse octree with {} occupied voxels", voxel_positions.len());
    
    // Convert to dense grid
    let target_voxel_size = 0.5;
    let dense_grid = octree.to_dense_grid(target_voxel_size);
    
    println!("Converted to dense grid with dimensions: {:?}", dense_grid.dimensions);
    println!("Dense grid voxel size: {}", dense_grid.voxel_size);
    
    // Verify conversion
    let mut occupied_count = 0;
    for x in 0..dense_grid.dimensions.0 {
        for y in 0..dense_grid.dimensions.1 {
            for z in 0..dense_grid.dimensions.2 {
                if let Some(voxel) = dense_grid.get_voxel(x, y, z) {
                    if matches!(voxel, crate::voxels::grid::VoxelData::Occupied { .. }) {
                        occupied_count += 1;
                    }
                }
            }
        }
    }
    
    println!("Dense grid contains {} occupied voxels", occupied_count);
    
    // Verify that all original voxels are present in dense grid
    for pos in &voxel_positions {
        let (gx, gy, gz) = dense_grid.world_to_voxel(pos);
        if let Some(voxel) = dense_grid.get_voxel(gx, gy, gz) {
            assert!(matches!(voxel, crate::voxels::grid::VoxelData::Occupied { .. }), 
                "Original voxel at {:?} should be occupied in dense grid", pos);
        } else {
            panic!("Original voxel at {:?} not found in dense grid", pos);
        }
    }
    
    println!("✅ Sparse to dense conversion test passed");
    Ok(())
}

/// Test sparse voxel format export/import
pub fn test_sparse_voxel_formats() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Testing Sparse Voxel Format Export/Import ===");
    
    // Create test sparse voxel octree
    let origin = Point3::new(0.0, 0.0, 0.0);
    let size = 2.0;
    let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(origin, size, 2, None);
    
    // Add test voxels
    let test_voxels = [
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(1.5, 0.5, 0.5),
        Point3::new(0.5, 1.5, 0.5),
    ];
    
    for pos in &test_voxels {
        octree.set_voxel(pos, true, None);
    }
    
    println!("Created test octree with {} voxels", test_voxels.len());
    
    // Test each format
    let formats = [
        SparseVoxelFormat::Raw,
        SparseVoxelFormat::Compressed,
        SparseVoxelFormat::Vdb,
    ];
    
    for format in &formats {
        println!("Testing format: {:?}", format);
        
        // Export
        let exported_data = octree.export_to_format(*format)?;
        println!("Exported {} bytes", exported_data.len());
        
        // Import
        let imported_octree: SparseVoxelOctree<()> = SparseVoxelOctree::import_from_format(&exported_data, *format, 0.5)?;
        println!("Imported octree with {} occupied leaves", imported_octree.occupied_leaves);
        
        // Verify voxels are preserved
        for pos in &test_voxels {
            let original_occupied = octree.get_voxel(pos).unwrap_or(false);
            let imported_occupied = imported_octree.get_voxel(pos).unwrap_or(false);
            assert_eq!(original_occupied, imported_occupied, 
                "Voxel at {:?} should have same occupancy in original and imported octrees", pos);
        }
        
        println!("✅ Format {:?} test passed", format);
    }
    
    println!("✅ All format tests passed");
    Ok(())
}

/// Test sparse voxel surface reconstruction
pub fn test_sparse_voxel_surface_reconstruction() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Testing Sparse Voxel Surface Reconstruction ===");
    
    // Create sparse voxel octree
    let origin = Point3::new(0.0, 0.0, 0.0);
    let size = 4.0;
    let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(origin, size, 3, None);
    
    // Create a 2x2x2 cube pattern
    for x in 0..2 {
        for y in 0..2 {
            for z in 0..2 {
                let pos = Point3::new(
                    x as Real * 2.0 + 1.0,
                    y as Real * 2.0 + 1.0,
                    z as Real * 2.0 + 1.0,
                );
                octree.set_voxel(&pos, true, None);
            }
        }
    }
    
    println!("Created 2x2x2 voxel cube (8 voxels)");
    
    // Test all surface reconstruction modes
    let modes = [
        SurfaceReconstructionMode::Naive,
        SurfaceReconstructionMode::FaceCulling,
        SurfaceReconstructionMode::MarchingCubes,
        SurfaceReconstructionMode::DualContouring,
    ];
    
    for mode in &modes {
        let mesh = octree.to_mesh_with_reconstruction(*mode);
        println!("Mode {:?}: {} polygons", mode, mesh.polygons.len());
        
        // Verify mesh is valid
        assert!(!mesh.polygons.is_empty(), "Mesh should not be empty for mode {:?}", mode);
        
        // Verify all polygons are valid
        for (i, polygon) in mesh.polygons.iter().enumerate() {
            assert!(polygon.vertices.len() >= 3, 
                "Polygon {} should have at least 3 vertices", i);
            
            // Verify vertices have valid positions and normals
            for (j, vertex) in polygon.vertices.iter().enumerate() {
                assert!(vertex.normal.norm() > 0.0, 
                    "Vertex {} in polygon {} should have valid normal", j, i);
            }
        }
    }
    
    println!("✅ Surface reconstruction tests passed");
    Ok(())
}

/// Test sparse voxel data structure
pub fn test_sparse_voxel_data_structure() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Testing Sparse Voxel Data Structure ===");
    
    // Create test octree
    let origin = Point3::new(0.0, 0.0, 0.0);
    let size = 3.0;
    let mut octree: SparseVoxelOctree<()> = SparseVoxelOctree::new(origin, size, 2, None);
    
    // Add test voxels
    let test_positions = [
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(1.5, 0.5, 0.5),
        Point3::new(0.5, 1.5, 0.5),
        Point3::new(1.5, 1.5, 0.5),
    ];
    
    for pos in &test_positions {
        octree.set_voxel(pos, true, None);
    }
    
    // Convert to sparse voxel data
    let sparse_data = octree.to_sparse_voxel_data();
    
    println!("Sparse data contains {} voxels", sparse_data.positions.len());
    println!("Bounding box: {:?} to {:?}", sparse_data.bounding_box.0, sparse_data.bounding_box.1);
    
    // Verify data integrity
    assert_eq!(sparse_data.positions.len(), test_positions.len());
    assert_eq!(sparse_data.sizes.len(), test_positions.len());
    assert_eq!(sparse_data.metadata.len(), test_positions.len());
    
    // Verify all original positions are present
    for original_pos in &test_positions {
        let found = sparse_data.positions.iter().any(|pos| {
            (pos - original_pos).norm() < 1e-6
        });
        assert!(found, "Original position {:?} should be present in sparse data", original_pos);
    }
    
    // Test round-trip conversion
    let reconstructed_octree = SparseVoxelOctree::from_sparse_voxel_data(&sparse_data, 0.5)?;
    
    // Verify voxels are preserved
    for pos in &test_positions {
        let original_occupied = octree.get_voxel(pos).unwrap_or(false);
        let reconstructed_occupied = reconstructed_octree.get_voxel(pos).unwrap_or(false);
        assert_eq!(original_occupied, reconstructed_occupied,
            "Voxel at {:?} should have same occupancy after round-trip conversion", pos);
    }
    
    println!("✅ Sparse voxel data structure tests passed");
    Ok(())
}

/// Run all sparse voxel conversion tests
pub fn run_all_sparse_conversion_tests() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Running All Sparse Voxel Conversion Tests ===");
    
    test_sparse_to_dense_conversion()?;
    test_sparse_voxel_formats()?;
    test_sparse_voxel_surface_reconstruction()?;
    test_sparse_voxel_data_structure()?;
    
    println!("✅ All sparse voxel conversion tests passed!");
    Ok(())
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_sparse_to_dense_conversion() -> Result<(), Box<dyn std::error::Error>> {
        super::test_sparse_to_dense_conversion()
    }

    #[test]
    fn test_sparse_voxel_formats() -> Result<(), Box<dyn std::error::Error>> {
        super::test_sparse_voxel_formats()
    }

    #[test]
    fn test_sparse_voxel_surface_reconstruction() -> Result<(), Box<dyn std::error::Error>> {
        super::test_sparse_voxel_surface_reconstruction()
    }

    #[test]
    fn test_sparse_voxel_data_structure() -> Result<(), Box<dyn std::error::Error>> {
        super::test_sparse_voxel_data_structure()
    }

    #[test]
    fn test_all_sparse_conversion_tests() -> Result<(), Box<dyn std::error::Error>> {
        super::run_all_sparse_conversion_tests()
    }
}
