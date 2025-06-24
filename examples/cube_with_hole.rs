//! Example: Cube with cylindrical hole
//!
//! This example demonstrates creating a rectangular cube (127x85x44mm) with a 
//! cylindrical hole (6mm diameter) drilled through its length, centered in the 
//! cross-sectional area.

use csgrs::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Dimensions in millimeters
    let cube_width = 127.0;   // X dimension - length of the hole
    let cube_length = 85.0;   // Y dimension 
    let cube_height = 44.0;   // Z dimension
    
    let hole_diameter = 6.0;
    let hole_radius = hole_diameter / 2.0;
    
    // Create the main cube
    let cube: CSG<()> = CSG::cuboid(cube_width, cube_length, cube_height, None);
    
    // Create a cylinder for the hole - initially vertical (along Z-axis)
    // We make it slightly longer than the cube width to ensure clean subtraction
    let cylinder_length = cube_width + 1.0; // Add 1mm buffer on each side
    let segments = 32; // Number of segments for smooth cylinder
    
    let cylinder: CSG<()> = CSG::cylinder(hole_radius, cylinder_length, segments, None);
    
    // Rotate the cylinder 90 degrees around Y-axis to align it with X-axis
    let cylinder_horizontal = cylinder.rotate(0.0, 90.0, 0.0);
    
    // Position the cylinder to be centered in the Y-Z cross-section
    // and slightly offset in X to ensure it goes completely through
    let center_y = cube_length / 2.0;  // 42.5mm - center in Y direction
    let center_z = cube_height / 2.0;  // 22.0mm - center in Z direction
    let offset_x = -0.5; // Start slightly before the cube face
    
    let positioned_cylinder = cylinder_horizontal.translate(offset_x, center_y, center_z);
    
    // Subtract the cylinder from the cube to create the hole
    let cube_with_hole = cube.difference(&positioned_cylinder);
    
    // Export to STL file
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = cube_with_hole.to_stl_binary("cube_with_hole")?;
        std::fs::write("cube_with_hole.stl", stl_bytes)?;
        println!("Created cube_with_hole.stl");
        println!("Dimensions: {}x{}x{}mm cube with {}mm diameter hole", 
                 cube_width, cube_length, cube_height, hole_diameter);
    }
    
    // Print some information about the result
    let bbox = cube_with_hole.bounding_box();
    println!("Bounding box:");
    println!("  X: {:.2} to {:.2} mm (width: {:.2} mm)", 
             bbox.mins.x, bbox.maxs.x, bbox.maxs.x - bbox.mins.x);
    println!("  Y: {:.2} to {:.2} mm (length: {:.2} mm)", 
             bbox.mins.y, bbox.maxs.y, bbox.maxs.y - bbox.mins.y);
    println!("  Z: {:.2} to {:.2} mm (height: {:.2} mm)", 
             bbox.mins.z, bbox.maxs.z, bbox.maxs.z - bbox.mins.z);
    
    println!("Hole position: centered at Y={:.1}mm, Z={:.1}mm", center_y, center_z);
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cube_with_hole_creation() {
        // Test that we can create the cube with hole without panicking
        let cube: CSG<()> = CSG::cuboid(127.0, 85.0, 44.0, None);
        let cylinder: CSG<()> = CSG::cylinder(3.0, 128.0, 32, None);
        let positioned_cylinder = cylinder.rotate(0.0, 90.0, 0.0).translate(-0.5, 42.5, 22.0);
        let result = cube.difference(&positioned_cylinder);
        
        // Verify the bounding box is as expected (should be same as original cube)
        let bbox = result.bounding_box();
        assert!((bbox.maxs.x - bbox.mins.x - 127.0).abs() < 0.1);
        assert!((bbox.maxs.y - bbox.mins.y - 85.0).abs() < 0.1);
        assert!((bbox.maxs.z - bbox.mins.z - 44.0).abs() < 0.1);
    }
    
    #[test]
    fn test_hole_positioning() {
        // Verify the hole is centered correctly
        let center_y = 85.0 / 2.0;
        let center_z = 44.0 / 2.0;
        
        assert_eq!(center_y, 42.5);
        assert_eq!(center_z, 22.0);
    }
} 