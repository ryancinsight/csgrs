//! Example: Sphere - Cube Difference
//!
//! This example demonstrates the difference boolean operation in reverse,
//! subtracting a cube from a sphere to create a spherical object with
//! a cubic cavity or cutout.

use csgrs::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a sphere with 30mm radius centered at origin
    let sphere_radius = 30.0;
    let sphere: CSG<()> = CSG::sphere(sphere_radius, 32, 16, None);
    
    // Create a 35x35x35mm cube, positioned to intersect the sphere
    let cube_size = 35.0;
    let cube: CSG<()> = CSG::cube(cube_size, None)
        .translate(-cube_size/2.0, -cube_size/2.0, -cube_size/2.0)  // Center the cube
        .translate(5.0, 5.0, 0.0);  // Offset to create interesting intersection
    
    // Perform difference operation: sphere - cube  
    let difference_result = sphere.difference(&cube);
    
    // Export to STL file
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = difference_result.to_stl_binary("sphere_cube_difference")?;
        std::fs::write("sphere_cube_difference.stl", stl_bytes)?;
        println!("Created sphere_cube_difference.stl");
    }
    
    // Print information about the operation
    println!("Boolean Operation: DIFFERENCE (A - B) [Sphere - Cube]");
    println!("======================================================");
    println!("Operation: Removes space of cube from sphere");
    println!("Result: Contains space that was in the sphere BUT NOT in the cube");
    println!();
    
    let sphere_bbox = sphere.bounding_box();
    let cube_bbox = cube.bounding_box();
    let result_bbox = difference_result.bounding_box();
    
    println!("Input geometries:");
    println!("  Sphere (A): radius {}mm centered at origin", sphere_radius);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             sphere_bbox.mins.x, sphere_bbox.mins.y, sphere_bbox.mins.z,
             sphere_bbox.maxs.x, sphere_bbox.maxs.y, sphere_bbox.maxs.z);
    
    println!("  Cube (B): {}x{}x{}mm at offset (5, 5, 0)", cube_size, cube_size, cube_size);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             cube_bbox.mins.x, cube_bbox.mins.y, cube_bbox.mins.z,
             cube_bbox.maxs.x, cube_bbox.maxs.y, cube_bbox.maxs.z);
    
    println!();
    println!("Difference result (A - B):");
    println!("  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             result_bbox.mins.x, result_bbox.mins.y, result_bbox.mins.z,
             result_bbox.maxs.x, result_bbox.maxs.y, result_bbox.maxs.z);
    println!("  Note: The cube has created a cubic cavity in the sphere");
    
    // Additional analysis
    println!();
    println!("Analysis:");
    println!("  This operation is the reverse of cube - sphere");
    println!("  Creates a spherical shell with a cubic bite taken out");
    println!("  Useful for creating complex hollow geometries");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sphere_minus_cube() {
        let sphere: CSG<()> = CSG::sphere(25.0, 16, 8, None);
        let cube: CSG<()> = CSG::cube(30.0, None).center();
        let result = sphere.difference(&cube);
        
        // Result should fit within the original sphere's bounding box
        let result_bbox = result.bounding_box();
        let sphere_bbox = sphere.bounding_box();
        
        // Result should not exceed sphere bounds (with small tolerance)
        let tolerance = 0.1;
        assert!(result_bbox.mins.x >= sphere_bbox.mins.x - tolerance);
        assert!(result_bbox.mins.y >= sphere_bbox.mins.y - tolerance);
        assert!(result_bbox.mins.z >= sphere_bbox.mins.z - tolerance);
        
        assert!(result_bbox.maxs.x <= sphere_bbox.maxs.x + tolerance);
        assert!(result_bbox.maxs.y <= sphere_bbox.maxs.y + tolerance);
        assert!(result_bbox.maxs.z <= sphere_bbox.maxs.z + tolerance);
    }
    
    #[test]
    fn test_difference_not_commutative() {
        // Test that A - B ≠ B - A
        let sphere: CSG<()> = CSG::sphere(20.0, 16, 8, None);
        let cube: CSG<()> = CSG::cube(25.0, None).center();
        
        let sphere_minus_cube = sphere.difference(&cube);
        let cube_minus_sphere = cube.difference(&sphere);
        
        let bbox_sc = sphere_minus_cube.bounding_box();
        let bbox_cs = cube_minus_sphere.bounding_box();
        
        // These should be different operations with different results
        // At minimum, the bounding boxes should differ
        let x_diff = (bbox_sc.maxs.x - bbox_sc.mins.x) - (bbox_cs.maxs.x - bbox_cs.mins.x);
        assert!(x_diff.abs() > 1.0); // Should have meaningfully different sizes
    }
    
    #[test]
    fn test_sphere_minus_small_cube() {
        // Small cube inside sphere should create cavity while preserving sphere bounds
        let sphere: CSG<()> = CSG::sphere(30.0, 16, 8, None);
        let small_cube: CSG<()> = CSG::cube(10.0, None).center();
        let result = sphere.difference(&small_cube);
        
        let sphere_bbox = sphere.bounding_box();
        let result_bbox = result.bounding_box();
        
        // Sphere bounds should be approximately preserved
        let tolerance = 0.5;
        assert!((sphere_bbox.mins.x - result_bbox.mins.x).abs() < tolerance);
        assert!((sphere_bbox.maxs.x - result_bbox.maxs.x).abs() < tolerance);
    }
} 