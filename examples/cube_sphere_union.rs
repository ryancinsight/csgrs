//! Example: Cube + Sphere Union
//!
//! This example demonstrates the union boolean operation, which combines
//! two CSG objects into a single object containing all the space that 
//! was in either object.

use csgrs::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 40x40x40mm cube centered at origin
    let cube_size = 40.0;
    let cube: CSG<()> = CSG::cube(cube_size, None).center();
    
    // Create a sphere with 25mm radius, offset to partially overlap
    let sphere_radius = 25.0;
    let sphere: CSG<()> = CSG::sphere(sphere_radius, 32, 16, None)
        .translate(15.0, 15.0, 15.0);  // Offset to create interesting overlap
    
    // Perform union operation: cube ∪ sphere
    let union_result = cube.union(&sphere);
    
    // Export to STL file
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = union_result.to_stl_binary("cube_sphere_union")?;
        std::fs::write("cube_sphere_union.stl", stl_bytes)?;
        println!("Created cube_sphere_union.stl");
    }
    
    // Print information about the operation
    println!("Boolean Operation: UNION (A ∪ B)");
    println!("=====================================");
    println!("Operation: Combines all space from both objects");
    println!("Result: Contains everything that was in either the cube OR the sphere");
    println!();
    
    let cube_bbox = cube.bounding_box();
    let sphere_bbox = sphere.bounding_box();
    let result_bbox = union_result.bounding_box();
    
    println!("Input geometries:");
    println!("  Cube: {}x{}x{}mm centered at origin", cube_size, cube_size, cube_size);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             cube_bbox.mins.x, cube_bbox.mins.y, cube_bbox.mins.z,
             cube_bbox.maxs.x, cube_bbox.maxs.y, cube_bbox.maxs.z);
    
    println!("  Sphere: radius {}mm at offset (15, 15, 15)", sphere_radius);
    println!("    Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             sphere_bbox.mins.x, sphere_bbox.mins.y, sphere_bbox.mins.z,
             sphere_bbox.maxs.x, sphere_bbox.maxs.y, sphere_bbox.maxs.z);
    
    println!();
    println!("Union result:");
    println!("  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             result_bbox.mins.x, result_bbox.mins.y, result_bbox.mins.z,
             result_bbox.maxs.x, result_bbox.maxs.y, result_bbox.maxs.z);
    println!("  Dimensions: {:.1} x {:.1} x {:.1} mm", 
             result_bbox.maxs.x - result_bbox.mins.x,
             result_bbox.maxs.y - result_bbox.mins.y,
             result_bbox.maxs.z - result_bbox.mins.z);
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_union_operation() {
        let cube: CSG<()> = CSG::cube(40.0, None).center();
        let sphere: CSG<()> = CSG::sphere(25.0, 16, 8, None).translate(15.0, 15.0, 15.0);
        let result = cube.union(&sphere);
        
        // Union should contain both original objects
        let result_bbox = result.bounding_box();
        let cube_bbox = cube.bounding_box();
        let sphere_bbox = sphere.bounding_box();
        
        // Result bounding box should encompass both input objects
        assert!(result_bbox.mins.x <= cube_bbox.mins.x.min(sphere_bbox.mins.x));
        assert!(result_bbox.mins.y <= cube_bbox.mins.y.min(sphere_bbox.mins.y));
        assert!(result_bbox.mins.z <= cube_bbox.mins.z.min(sphere_bbox.mins.z));
        
        assert!(result_bbox.maxs.x >= cube_bbox.maxs.x.max(sphere_bbox.maxs.x));
        assert!(result_bbox.maxs.y >= cube_bbox.maxs.y.max(sphere_bbox.maxs.y));
        assert!(result_bbox.maxs.z >= cube_bbox.maxs.z.max(sphere_bbox.maxs.z));
    }
    
    #[test]
    fn test_union_is_commutative() {
        let cube: CSG<()> = CSG::cube(20.0, None).center();
        let sphere: CSG<()> = CSG::sphere(15.0, 16, 8, None);
        
        let union_a_b = cube.union(&sphere);
        let union_b_a = sphere.union(&cube);
        
        // Union should be commutative: A ∪ B = B ∪ A
        let bbox_ab = union_a_b.bounding_box();
        let bbox_ba = union_b_a.bounding_box();
        
        let tolerance = 0.001;
        assert!((bbox_ab.mins.x - bbox_ba.mins.x).abs() < tolerance);
        assert!((bbox_ab.maxs.x - bbox_ba.maxs.x).abs() < tolerance);
    }
} 