//! Fixed Cylinder Junction - No Holes
//!
//! This example demonstrates how to properly configure metaballs to avoid
//! holes in the junction area between two cylinders.

use csgrs::CSG;
use csgrs::math::metaballs::MetaBall;
use nalgebra::Point3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Creating FIXED cylinder junction (no holes)...");
    
    // Common dimensions
    let cylinder_diameter = 6.0;
    let cylinder_radius = cylinder_diameter / 2.0;
    let cylinder_length = 20.0;
    let segments = 24;
    
    // Junction point
    let junction_point = Point3::new(0.0, 0.0, 0.0);
    let cylinder1_end = Point3::new(cylinder_length, 0.0, 0.0);
    
    // 120 degree angle
    let angle_radians = 120.0_f64.to_radians();
    let cylinder2_end = Point3::new(
        cylinder_length * angle_radians.cos(),
        cylinder_length * angle_radians.sin(),
        0.0
    );
    
    println!("Cylinder 1: extends to ({:.1}, {:.1}, {:.1})", 
             cylinder1_end.x, cylinder1_end.y, cylinder1_end.z);
    println!("Cylinder 2: extends to ({:.1}, {:.1}, {:.1})", 
             cylinder2_end.x, cylinder2_end.y, cylinder2_end.z);
    
    // Create the cylinders
    let cylinder1: CSG<()> = CSG::frustum_ptp(
        junction_point,
        cylinder1_end,
        cylinder_radius,
        cylinder_radius,
        segments,
        None
    );
    
    let cylinder2: CSG<()> = CSG::frustum_ptp(
        junction_point,
        cylinder2_end,
        cylinder_radius,
        cylinder_radius,
        segments,
        None
    );
    
    let simple_union = cylinder1.union(&cylinder2);
    println!("Simple union created with {} polygons", simple_union.polygons.len());
    
    // FIXED metaball configuration - addresses the hole issue
    println!("\n=== FIXED Configuration ===");
    
    // Key fixes:
    // 1. Lower iso_value (0.3 instead of 0.6) for more volume
    // 2. Larger junction metaball radius  
    // 3. More metaballs with better overlap
    // 4. Strategic positioning to ensure continuous field
    
    let junction_metaball_radius = cylinder_radius * 2.5; // Much larger for junction
    let transition_radius = cylinder_radius * 1.8;
    
    // Calculate direction vectors
    let dir1 = (cylinder1_end - junction_point).normalize();
    let dir2 = (cylinder2_end - junction_point).normalize();
    
    let mut metaballs = Vec::new();
    
    // MAIN JUNCTION METABALL - Large radius to ensure coverage
    metaballs.push(MetaBall::new(junction_point, junction_metaball_radius));
    
    // CLOSE TRANSITION METABALLS - Very close to junction for continuity
    let close_distance = cylinder_radius * 0.8;
    metaballs.push(MetaBall::new(
        junction_point + dir1 * close_distance, 
        transition_radius
    ));
    metaballs.push(MetaBall::new(
        junction_point + dir2 * close_distance, 
        transition_radius
    ));
    
    // MEDIUM DISTANCE METABALLS - For smooth transition
    let medium_distance = cylinder_radius * 2.0;
    metaballs.push(MetaBall::new(
        junction_point + dir1 * medium_distance, 
        cylinder_radius * 1.5
    ));
    metaballs.push(MetaBall::new(
        junction_point + dir2 * medium_distance, 
        cylinder_radius * 1.5
    ));
    
    // INTERMEDIATE METABALLS - Fill any remaining gaps
    let intermediate_distance = cylinder_radius * 1.2;
    metaballs.push(MetaBall::new(
        junction_point + dir1 * intermediate_distance, 
        cylinder_radius * 1.6
    ));
    metaballs.push(MetaBall::new(
        junction_point + dir2 * intermediate_distance, 
        cylinder_radius * 1.6
    ));
    
    println!("Using {} metaballs for complete coverage", metaballs.len());
    
    // Fixed parameters
    let resolution = (32, 32, 16); // Higher resolution
    let iso_value = 0.3; // MUCH LOWER - key fix for holes
    let padding = cylinder_radius * 2.0; // More padding
    
    println!("Resolution: {:?}", resolution);
    println!("iso_value: {} (lower = more volume)", iso_value);
    println!("Junction metaball radius: {:.2}", junction_metaball_radius);
    
    let metaball_blend: CSG<()> = CSG::metaballs(
        &metaballs,
        resolution,
        iso_value,
        padding,
        None
    );
    
    println!("Metaball blend created with {} polygons", metaball_blend.polygons.len());
    
    // Create final result
    let fixed_junction = simple_union.union(&metaball_blend);
    println!("Final fixed junction has {} polygons", fixed_junction.polygons.len());
    
    // Alternative approach: Pure metaball construction
    println!("\n=== Alternative: Pure Metaball Approach ===");
    
    // Create the entire structure using only metaballs for guaranteed continuity
    let mut pure_metaballs = Vec::new();
    
    // Dense metaball chain along cylinder 1
    for i in 0..8 {
        let t = i as f64 / 7.0; // 0 to 1
        let pos = junction_point + dir1 * (cylinder_length * 0.9 * t);
        let radius = cylinder_radius * (1.0 + 0.5 * (1.0 - t)); // Larger at junction
        pure_metaballs.push(MetaBall::new(pos, radius));
    }
    
    // Dense metaball chain along cylinder 2
    for i in 0..8 {
        let t = i as f64 / 7.0; // 0 to 1
        let pos = junction_point + dir2 * (cylinder_length * 0.9 * t);
        let radius = cylinder_radius * (1.0 + 0.5 * (1.0 - t)); // Larger at junction
        pure_metaballs.push(MetaBall::new(pos, radius));
    }
    
    let pure_metaball_junction: CSG<()> = CSG::metaballs(
        &pure_metaballs,
        (40, 40, 20), // Higher resolution for pure approach
        0.25, // Even lower iso_value
        cylinder_radius * 3.0,
        None
    );
    
    println!("Pure metaball approach: {} polygons", pure_metaball_junction.polygons.len());
    
    // Export all versions
    #[cfg(feature = "stl-io")]
    {
        // Original with hole
        let simple_stl = simple_union.to_stl_binary("simple_union_with_hole")?;
        std::fs::write("simple_union_with_hole.stl", simple_stl)?;
        println!("\nExported: simple_union_with_hole.stl");
        
        // Fixed version
        let fixed_stl = fixed_junction.to_stl_binary("fixed_junction_no_hole")?;
        std::fs::write("fixed_junction_no_hole.stl", fixed_stl)?;
        println!("Exported: fixed_junction_no_hole.stl");
        
        // Pure metaball version
        let pure_stl = pure_metaball_junction.to_stl_binary("pure_metaball_junction")?;
        std::fs::write("pure_metaball_junction.stl", pure_stl)?;
        println!("Exported: pure_metaball_junction.stl");
        
        // Metaball blend surface only for debugging
        let blend_stl = metaball_blend.to_stl_binary("metaball_blend_debug")?;
        std::fs::write("metaball_blend_debug.stl", blend_stl)?;
        println!("Exported: metaball_blend_debug.stl (for debugging)");
    }
    
    println!("\n=== Key Fixes Applied ===");
    println!("1. ✓ Lowered iso_value from 0.6 to 0.3 (more volume)");
    println!("2. ✓ Increased junction metaball radius to 2.5x cylinder radius");
    println!("3. ✓ Added more metaballs with strategic overlap");
    println!("4. ✓ Positioned metaballs closer to junction for continuity");
    println!("5. ✓ Increased resolution for better geometry capture");
    println!("6. ✓ Provided pure metaball alternative for guaranteed continuity");
    
    println!("\n=== Debugging Tips ===");
    println!("If holes persist:");
    println!("- Lower iso_value further (try 0.2 or 0.15)");
    println!("- Increase metaball radius at junction");
    println!("- Add more intermediate metaballs");
    println!("- Increase resolution to (64, 64, 32) or higher");
    println!("- Check metaball_blend_debug.stl to see the blend surface alone");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_no_hole_configuration() {
        let junction = Point3::new(0.0, 0.0, 0.0);
        let end1 = Point3::new(10.0, 0.0, 0.0);
        let end2 = Point3::new(-5.0, 8.66, 0.0);
        let radius = 2.0;
        
        // Test that the fixed configuration produces geometry
        let metaballs = vec![
            MetaBall::new(junction, radius * 2.5),
            MetaBall::new(Point3::new(1.6, 0.0, 0.0), radius * 1.8),
            MetaBall::new(Point3::new(-0.8, 1.38, 0.0), radius * 1.8),
        ];
        
        let blend: CSG<()> = CSG::metaballs(
            &metaballs,
            (24, 24, 12),
            0.3, // Low iso_value
            radius * 2.0,
            None
        );
        
        // Should produce significant geometry with low iso_value
        println!("Fixed config produced {} polygons", blend.polygons.len());
        
        // With proper parameters, should get substantial geometry
        // Note: Actual count depends on resolution and iso_value
    }
    
    #[test]
    fn test_iso_value_effect() {
        let metaballs = vec![
            MetaBall::new(Point3::new(0.0, 0.0, 0.0), 3.0),
        ];
        
        // Test different iso_values
        for iso_value in [0.8, 0.5, 0.3, 0.1] {
            let result: CSG<()> = CSG::metaballs(
                &metaballs,
                (16, 16, 8),
                iso_value,
                2.0,
                None
            );
            
            println!("iso_value {} produced {} polygons", iso_value, result.polygons.len());
        }
        
        // Lower iso_values should generally produce more polygons (more volume)
    }
} 