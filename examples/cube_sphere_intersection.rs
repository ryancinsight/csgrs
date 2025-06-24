//! Example: Cube ∩ Sphere Intersection
//!
//! This example demonstrates the intersection boolean operation, which
//! creates a new object containing only the space that exists in BOTH
//! input objects simultaneously.

use csgrs::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 40x40x40mm cube centered at origin
    let cube_size = 40.0;
    let cube: CSG<()> = CSG::cube(cube_size, None).center();
    
    // Create a sphere with 25mm radius, positioned to partially overlap
    let sphere_radius = 25.0;
    let sphere: CSG<()> = CSG::sphere(sphere_radius, 32, 16, None)
        .translate(10.0, 10.0, 0.0);  // Offset to create partial overlap
    
    // Perform intersection operation: cube ∩ sphere
    let intersection_result = cube.intersection(&sphere);
    
    // Export to STL file
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = intersection_result.to_stl_binary("cube_sphere_intersection")?;
        std::fs::write("cube_sphere_intersection.stl", stl_bytes)?;
        println!("Created cube_sphere_intersection.stl");
    }
    
    // Print information about the operation
    println!("Boolean Operation: INTERSECTION (A ∩ B)");
    println!("========================================");
    println!("Operation: Keeps only the overlapping space");
    println!("Result: Contains space that was in BOTH the cube AND the sphere");
    println!();
    
    let cube_bbox = cube.bounding_box();
    let sphere_bbox = sphere.bounding_box();
    let result_bbox = intersection_result.bounding_box();
    
    println!("Input geometries:");
    println!("  Cube (A): {}x{}x{}mm centered at origin", cube_size, cube_size, cube_size);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             cube_bbox.mins.x, cube_bbox.mins.y, cube_bbox.mins.z,
             cube_bbox.maxs.x, cube_bbox.maxs.y, cube_bbox.maxs.z);
    
    println!("  Sphere (B): radius {}mm at offset (10, 10, 0)", sphere_radius);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             sphere_bbox.mins.x, sphere_bbox.mins.y, sphere_bbox.mins.z,
             sphere_bbox.maxs.x, sphere_bbox.maxs.y, sphere_bbox.maxs.z);
    
    println!();
    println!("Intersection result (A ∩ B):");
    println!("  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             result_bbox.mins.x, result_bbox.mins.y, result_bbox.mins.z,
             result_bbox.maxs.x, result_bbox.maxs.y, result_bbox.maxs.z);
    println!("  Dimensions: {:.1} x {:.1} x {:.1} mm", 
             result_bbox.maxs.x - result_bbox.mins.x,
             result_bbox.maxs.y - result_bbox.mins.y,
             result_bbox.maxs.z - result_bbox.mins.z);
    
    // Additional analysis
    println!();
    println!("Analysis:");
    println!("  The result has flat faces (from the cube) and curved faces (from the sphere)");
    println!("  This creates a unique geometry that's part cubic, part spherical");
    println!("  Intersection is smaller than either input object");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_intersection_operation() {
        let cube: CSG<()> = CSG::cube(30.0, None).center();
        let sphere: CSG<()> = CSG::sphere(20.0, 16, 8, None);
        let result = cube.intersection(&sphere);
        
        // Intersection should fit within both input objects
        let result_bbox = result.bounding_box();
        let cube_bbox = cube.bounding_box();
        let sphere_bbox = sphere.bounding_box();
        
        let tolerance = 0.001;
        
        // Result should be within cube bounds
        assert!(result_bbox.mins.x >= cube_bbox.mins.x - tolerance);
        assert!(result_bbox.mins.y >= cube_bbox.mins.y - tolerance);
        assert!(result_bbox.mins.z >= cube_bbox.mins.z - tolerance);
        assert!(result_bbox.maxs.x <= cube_bbox.maxs.x + tolerance);
        assert!(result_bbox.maxs.y <= cube_bbox.maxs.y + tolerance);
        assert!(result_bbox.maxs.z <= cube_bbox.maxs.z + tolerance);
        
        // Result should be within sphere bounds  
        assert!(result_bbox.mins.x >= sphere_bbox.mins.x - tolerance);
        assert!(result_bbox.mins.y >= sphere_bbox.mins.y - tolerance);
        assert!(result_bbox.mins.z >= sphere_bbox.mins.z - tolerance);
        assert!(result_bbox.maxs.x <= sphere_bbox.maxs.x + tolerance);
        assert!(result_bbox.maxs.y <= sphere_bbox.maxs.y + tolerance);
        assert!(result_bbox.maxs.z <= sphere_bbox.maxs.z + tolerance);
    }
    
    #[test]
    fn test_intersection_is_commutative() {
        let cube: CSG<()> = CSG::cube(25.0, None).center();
        let sphere: CSG<()> = CSG::sphere(15.0, 16, 8, None);
        
        let intersection_a_b = cube.intersection(&sphere);
        let intersection_b_a = sphere.intersection(&cube);
        
        // Intersection should be commutative: A ∩ B = B ∩ A
        let bbox_ab = intersection_a_b.bounding_box();
        let bbox_ba = intersection_b_a.bounding_box();
        
        let tolerance = 0.001;
        assert!((bbox_ab.mins.x - bbox_ba.mins.x).abs() < tolerance);
        assert!((bbox_ab.maxs.x - bbox_ba.maxs.x).abs() < tolerance);
        assert!((bbox_ab.mins.y - bbox_ba.mins.y).abs() < tolerance);
        assert!((bbox_ab.maxs.y - bbox_ba.maxs.y).abs() < tolerance);
    }
    
    #[test]
    fn test_intersection_with_no_overlap() {
        // Test intersection with non-overlapping objects
        let cube: CSG<()> = CSG::cube(20.0, None).center();
        let sphere: CSG<()> = CSG::sphere(10.0, 16, 8, None).translate(50.0, 0.0, 0.0);
        let result = cube.intersection(&sphere);
        
        // Should result in empty or very small geometry
        let result_bbox = result.bounding_box();
        let volume = (result_bbox.maxs.x - result_bbox.mins.x) * 
                    (result_bbox.maxs.y - result_bbox.mins.y) * 
                    (result_bbox.maxs.z - result_bbox.mins.z);
        
        // Volume should be essentially zero for non-overlapping objects
        assert!(volume < 1.0);
    }
    
    #[test]
    fn test_intersection_bounded_by_inputs() {
        let cube: CSG<()> = CSG::cube(30.0, None).center();
        let sphere: CSG<()> = CSG::sphere(20.0, 16, 8, None);
        let result = cube.intersection(&sphere);
        
        let cube_bbox = cube.bounding_box();
        let sphere_bbox = sphere.bounding_box();
        let result_bbox = result.bounding_box();
        
        // Intersection should be bounded by both input objects
        let tolerance = 0.1;
        
        // Result should not exceed either input's bounds
        assert!(result_bbox.mins.x >= cube_bbox.mins.x - tolerance);
        assert!(result_bbox.mins.y >= cube_bbox.mins.y - tolerance);
        assert!(result_bbox.mins.z >= cube_bbox.mins.z - tolerance);
        assert!(result_bbox.maxs.x <= cube_bbox.maxs.x + tolerance);
        assert!(result_bbox.maxs.y <= cube_bbox.maxs.y + tolerance);
        assert!(result_bbox.maxs.z <= cube_bbox.maxs.z + tolerance);
        
        assert!(result_bbox.mins.x >= sphere_bbox.mins.x - tolerance);
        assert!(result_bbox.mins.y >= sphere_bbox.mins.y - tolerance);
        assert!(result_bbox.mins.z >= sphere_bbox.mins.z - tolerance);
        assert!(result_bbox.maxs.x <= sphere_bbox.maxs.x + tolerance);
        assert!(result_bbox.maxs.y <= sphere_bbox.maxs.y + tolerance);
        assert!(result_bbox.maxs.z <= sphere_bbox.maxs.z + tolerance);
    }
} 