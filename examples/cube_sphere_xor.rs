//! Example: Cube ⊕ Sphere XOR (Symmetric Difference)
//!
//! This example demonstrates the XOR boolean operation, which creates
//! a new object containing space that exists in either input object
//! but NOT in both simultaneously. This is the "symmetric difference".

use csgrs::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 35x35x35mm cube centered at origin
    let cube_size = 35.0;
    let cube: CSG<()> = CSG::cube(cube_size, None).center();
    
    // Create a sphere with 22mm radius, positioned to overlap significantly
    let sphere_radius = 22.0;
    let sphere: CSG<()> = CSG::sphere(sphere_radius, 32, 16, None)
        .translate(8.0, 8.0, 0.0);  // Offset to create meaningful overlap
    
    // Perform XOR operation: cube ⊕ sphere
    let xor_result = cube.xor(&sphere);
    
    // Export to STL file
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = xor_result.to_stl_binary("cube_sphere_xor")?;
        std::fs::write("cube_sphere_xor.stl", stl_bytes)?;
        println!("Created cube_sphere_xor.stl");
    }
    
    // Print information about the operation
    println!("Boolean Operation: XOR / SYMMETRIC DIFFERENCE (A ⊕ B)");
    println!("=====================================================");
    println!("Operation: (A ∪ B) - (A ∩ B) = (A - B) ∪ (B - A)");
    println!("Result: Contains space in either object but NOT in both");
    println!();
    
    let cube_bbox = cube.bounding_box();
    let sphere_bbox = sphere.bounding_box();
    let result_bbox = xor_result.bounding_box();
    
    println!("Input geometries:");
    println!("  Cube (A): {}x{}x{}mm centered at origin", cube_size, cube_size, cube_size);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             cube_bbox.mins.x, cube_bbox.mins.y, cube_bbox.mins.z,
             cube_bbox.maxs.x, cube_bbox.maxs.y, cube_bbox.maxs.z);
    
    println!("  Sphere (B): radius {}mm at offset (8, 8, 0)", sphere_radius);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             sphere_bbox.mins.x, sphere_bbox.mins.y, sphere_bbox.mins.z,
             sphere_bbox.maxs.x, sphere_bbox.maxs.y, sphere_bbox.maxs.z);
    
    println!();
    println!("XOR result (A ⊕ B):");
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
    println!("  XOR creates a 'donut' or 'shell' effect");
    println!("  The overlapping region becomes a hole/cavity");
    println!("  Useful for creating complex hollow structures");
    println!("  Result = Union - Intersection");
    
    // Demonstrate the mathematical relationship
    let union_result = cube.union(&sphere);
    let intersection_result = cube.intersection(&sphere);
    let manual_xor = union_result.difference(&intersection_result);
    
    let manual_bbox = manual_xor.bounding_box();
    println!();
    println!("Verification (Union - Intersection):");
    println!("  Manual XOR bbox: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             manual_bbox.mins.x, manual_bbox.mins.y, manual_bbox.mins.z,
             manual_bbox.maxs.x, manual_bbox.maxs.y, manual_bbox.maxs.z);
    println!("  Should match XOR result above");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_xor_operation() {
        let cube: CSG<()> = CSG::cube(30.0, None).center();
        let sphere: CSG<()> = CSG::sphere(20.0, 16, 8, None);
        let result = cube.xor(&sphere);
        
        // XOR result should encompass both input objects  
        let result_bbox = result.bounding_box();
        let cube_bbox = cube.bounding_box();
        let sphere_bbox = sphere.bounding_box();
        
        let tolerance = 0.1;
        
        // Should encompass the union of both objects
        assert!(result_bbox.mins.x <= cube_bbox.mins.x.min(sphere_bbox.mins.x) + tolerance);
        assert!(result_bbox.mins.y <= cube_bbox.mins.y.min(sphere_bbox.mins.y) + tolerance);
        assert!(result_bbox.mins.z <= cube_bbox.mins.z.min(sphere_bbox.mins.z) + tolerance);
        
        assert!(result_bbox.maxs.x >= cube_bbox.maxs.x.max(sphere_bbox.maxs.x) - tolerance);
        assert!(result_bbox.maxs.y >= cube_bbox.maxs.y.max(sphere_bbox.maxs.y) - tolerance);
        assert!(result_bbox.maxs.z >= cube_bbox.maxs.z.max(sphere_bbox.maxs.z) - tolerance);
    }
    
    #[test]
    fn test_xor_is_commutative() {
        let cube: CSG<()> = CSG::cube(25.0, None).center();
        let sphere: CSG<()> = CSG::sphere(15.0, 16, 8, None);
        
        let xor_a_b = cube.xor(&sphere);
        let xor_b_a = sphere.xor(&cube);
        
        // XOR should be commutative: A ⊕ B = B ⊕ A
        let bbox_ab = xor_a_b.bounding_box();
        let bbox_ba = xor_b_a.bounding_box();
        
        let tolerance = 0.001;
        assert!((bbox_ab.mins.x - bbox_ba.mins.x).abs() < tolerance);
        assert!((bbox_ab.maxs.x - bbox_ba.maxs.x).abs() < tolerance);
        assert!((bbox_ab.mins.y - bbox_ba.mins.y).abs() < tolerance);
        assert!((bbox_ab.maxs.y - bbox_ba.maxs.y).abs() < tolerance);
    }
    
    #[test]
    fn test_xor_mathematical_identity() {
        // Test that XOR = (A ∪ B) - (A ∩ B)
        let cube: CSG<()> = CSG::cube(20.0, None).center();
        let sphere: CSG<()> = CSG::sphere(15.0, 16, 8, None);
        
        let xor_direct = cube.xor(&sphere);
        
        let union_ab = cube.union(&sphere);
        let intersection_ab = cube.intersection(&sphere);
        let xor_manual = union_ab.difference(&intersection_ab);
        
        // The results should be very similar
        let bbox_direct = xor_direct.bounding_box();
        let bbox_manual = xor_manual.bounding_box();
        
        let tolerance = 0.1;
        assert!((bbox_direct.mins.x - bbox_manual.mins.x).abs() < tolerance);
        assert!((bbox_direct.maxs.x - bbox_manual.maxs.x).abs() < tolerance);
    }
    
    #[test]
    fn test_xor_with_identical_objects() {
        // XOR of identical objects should be empty
        let cube1: CSG<()> = CSG::cube(20.0, None).center();
        let cube2: CSG<()> = CSG::cube(20.0, None).center();
        let result = cube1.xor(&cube2);
        
        let result_bbox = result.bounding_box();
        let volume = (result_bbox.maxs.x - result_bbox.mins.x) * 
                    (result_bbox.maxs.y - result_bbox.mins.y) * 
                    (result_bbox.maxs.z - result_bbox.mins.z);
        
        // Volume should be very small (ideally zero) for identical objects
        assert!(volume < 1.0);
    }
    
    #[test]
    fn test_xor_with_no_overlap() {
        // XOR with no overlap should equal union
        let cube: CSG<()> = CSG::cube(20.0, None).center();
        let sphere: CSG<()> = CSG::sphere(10.0, 16, 8, None).translate(50.0, 0.0, 0.0);
        
        let xor_result = cube.xor(&sphere);
        let union_result = cube.union(&sphere);
        
        let xor_bbox = xor_result.bounding_box();
        let union_bbox = union_result.bounding_box();
        
        // Should be approximately equal
        let tolerance = 0.1;
        assert!((xor_bbox.mins.x - union_bbox.mins.x).abs() < tolerance);
        assert!((xor_bbox.maxs.x - union_bbox.maxs.x).abs() < tolerance);
    }
} 