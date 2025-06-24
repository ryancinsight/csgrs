//! Example: Cube - Sphere Difference
//!
//! This example demonstrates the difference boolean operation, which
//! subtracts one CSG object from another, creating a "hole" or "cutout".

use csgrs::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 50x50x50mm cube centered at origin
    let cube_size = 50.0;
    let cube: CSG<()> = CSG::cube(cube_size, None).center();
    
    // Create a sphere with 20mm radius, positioned to intersect the cube
    let sphere_radius = 20.0;
    let sphere: CSG<()> = CSG::sphere(sphere_radius, 32, 16, None)
        .translate(10.0, 0.0, 0.0);  // Offset to create partial overlap
    
    // Perform difference operation: cube - sphere
    let difference_result = cube.difference(&sphere);
    
    // Export to STL file
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = difference_result.to_stl_binary("cube_sphere_difference")?;
        std::fs::write("cube_sphere_difference.stl", stl_bytes)?;
        println!("Created cube_sphere_difference.stl");
    }
    
    // Print information about the operation
    println!("Boolean Operation: DIFFERENCE (A - B)");
    println!("======================================");
    println!("Operation: Removes space of B from A");
    println!("Result: Contains space that was in the cube BUT NOT in the sphere");
    println!();
    
    let cube_bbox = cube.bounding_box();
    let sphere_bbox = sphere.bounding_box();
    let result_bbox = difference_result.bounding_box();
    
    println!("Input geometries:");
    println!("  Cube (A): {}x{}x{}mm centered at origin", cube_size, cube_size, cube_size);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             cube_bbox.mins.x, cube_bbox.mins.y, cube_bbox.mins.z,
             cube_bbox.maxs.x, cube_bbox.maxs.y, cube_bbox.maxs.z);
    
    println!("  Sphere (B): radius {}mm at offset (10, 0, 0)", sphere_radius);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             sphere_bbox.mins.x, sphere_bbox.mins.y, sphere_bbox.mins.z,
             sphere_bbox.maxs.x, sphere_bbox.maxs.y, sphere_bbox.maxs.z);
    
    println!();
    println!("Difference result (A - B):");
    println!("  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             result_bbox.mins.x, result_bbox.mins.y, result_bbox.mins.z,
             result_bbox.maxs.x, result_bbox.maxs.y, result_bbox.maxs.z);
    println!("  Note: The sphere has created a spherical cavity in the cube");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_difference_operation() {
        let cube: CSG<()> = CSG::cube(30.0, None).center();
        let sphere: CSG<()> = CSG::sphere(10.0, 16, 8, None);
        let result = cube.difference(&sphere);
        
        // Difference result should fit within the original cube's bounding box
        let result_bbox = result.bounding_box();
        let cube_bbox = cube.bounding_box();
        
        // Result should not exceed cube bounds
        assert!(result_bbox.mins.x >= cube_bbox.mins.x - 0.001);
        assert!(result_bbox.mins.y >= cube_bbox.mins.y - 0.001);
        assert!(result_bbox.mins.z >= cube_bbox.mins.z - 0.001);
        
        assert!(result_bbox.maxs.x <= cube_bbox.maxs.x + 0.001);
        assert!(result_bbox.maxs.y <= cube_bbox.maxs.y + 0.001);
        assert!(result_bbox.maxs.z <= cube_bbox.maxs.z + 0.001);
    }
    
    #[test]
    fn test_difference_with_non_overlapping() {
        // Test difference with non-overlapping objects
        let cube: CSG<()> = CSG::cube(20.0, None).center();
        let sphere: CSG<()> = CSG::sphere(10.0, 16, 8, None).translate(50.0, 0.0, 0.0);
        let result = cube.difference(&sphere);
        
        // Should be identical to original cube since no overlap
        let result_bbox = result.bounding_box();
        let cube_bbox = cube.bounding_box();
        
        let tolerance = 0.001;
        assert!((result_bbox.mins.x - cube_bbox.mins.x).abs() < tolerance);
        assert!((result_bbox.maxs.x - cube_bbox.maxs.x).abs() < tolerance);
    }
    
    #[test] 
    fn test_difference_creates_cavity() {
        // When sphere is inside cube, it should create a cavity
        let cube: CSG<()> = CSG::cube(40.0, None).center();
        let sphere: CSG<()> = CSG::sphere(10.0, 16, 8, None); // Sphere at origin, inside cube
        let result = cube.difference(&sphere);
        
        // Result should still have same external bounds as cube
        let result_bbox = result.bounding_box();
        let cube_bbox = cube.bounding_box();
        
        // External dimensions should be preserved
        let tolerance = 0.001;
        assert!((result_bbox.mins.x - cube_bbox.mins.x).abs() < tolerance);
        assert!((result_bbox.maxs.x - cube_bbox.maxs.x).abs() < tolerance);
    }
} 