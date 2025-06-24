//! Configurable Cylinder Fillet System
//!
//! This example demonstrates a reusable system for creating smooth fillets
//! between cylinders with configurable parameters for different applications.

use csgrs::CSG;
use csgrs::math::metaballs::MetaBall;
use nalgebra::Point3;

/// Configuration for metaball-based filleting
#[derive(Debug, Clone)]
pub struct FilletConfig {
    /// Quality level affects resolution and computation time
    pub quality: FilletQuality,
    /// Fillet radius as a multiple of cylinder radius
    pub radius_multiplier: f64,
    /// Strength of the blending effect (0.3-1.0)
    pub blend_strength: f64,
    /// Number of metaballs along each cylinder axis
    pub metaball_count: usize,
}

#[derive(Debug, Clone)]
pub enum FilletQuality {
    Fast,      // For prototyping and real-time applications
    Balanced,  // Good quality/speed tradeoff
    High,      // For final production
    Ultra,     // Maximum quality for rendering
}

impl FilletConfig {
    pub fn fast() -> Self {
        Self {
            quality: FilletQuality::Fast,
            radius_multiplier: 1.5,
            blend_strength: 0.6,
            metaball_count: 2,
        }
    }
    
    pub fn balanced() -> Self {
        Self {
            quality: FilletQuality::Balanced,
            radius_multiplier: 1.8,
            blend_strength: 0.5,
            metaball_count: 3,
        }
    }
    
    pub fn high_quality() -> Self {
        Self {
            quality: FilletQuality::High,
            radius_multiplier: 2.0,
            blend_strength: 0.4,
            metaball_count: 4,
        }
    }
    
    pub fn ultra_quality() -> Self {
        Self {
            quality: FilletQuality::Ultra,
            radius_multiplier: 2.2,
            blend_strength: 0.35,
            metaball_count: 6,
        }
    }
    
    fn get_resolution(&self) -> (usize, usize, usize) {
        match self.quality {
            FilletQuality::Fast => (16, 16, 8),
            FilletQuality::Balanced => (24, 24, 12),
            FilletQuality::High => (32, 32, 16),
            FilletQuality::Ultra => (48, 48, 24),
        }
    }
}

/// Create a smooth fillet between two cylinders using metaballs
pub fn create_cylinder_fillet(
    junction_point: Point3<f64>,
    cylinder1_end: Point3<f64>,
    cylinder2_end: Point3<f64>,
    cylinder_radius: f64,
    config: &FilletConfig,
) -> CSG<()> {
    // Calculate directions from junction to cylinder ends
    let dir1 = (cylinder1_end - junction_point).normalize();
    let dir2 = (cylinder2_end - junction_point).normalize();
    
    // Metaball radius based on configuration
    let metaball_radius = cylinder_radius * config.radius_multiplier;
    
    let mut metaballs = Vec::new();
    
    // Main junction metaball - largest for primary blending
    metaballs.push(MetaBall::new(junction_point, metaball_radius));
    
    // Create metaballs along each cylinder axis
    for i in 1..=config.metaball_count {
        let distance = cylinder_radius * (i as f64 * 1.5);
        let radius_falloff = 1.0 - (i as f64 * 0.15); // Gradual radius reduction
        let current_radius = cylinder_radius * radius_falloff;
        
        // Along cylinder 1
        let pos1 = junction_point + dir1 * distance;
        metaballs.push(MetaBall::new(pos1, current_radius));
        
        // Along cylinder 2
        let pos2 = junction_point + dir2 * distance;
        metaballs.push(MetaBall::new(pos2, current_radius));
    }
    
    // Create the metaball surface
    let resolution = config.get_resolution();
    let padding = cylinder_radius * 1.5;
    
    CSG::metaballs(
        &metaballs,
        resolution,
        config.blend_strength,
        padding,
        None
    )
}

/// Create two cylinders with a smooth fillet junction
pub fn create_filleted_cylinder_junction(
    junction_point: Point3<f64>,
    cylinder1_end: Point3<f64>,
    cylinder2_end: Point3<f64>,
    cylinder_radius: f64,
    segments: usize,
    config: &FilletConfig,
) -> (CSG<()>, CSG<()>, CSG<()>) {
    // Create the base cylinders
    let cylinder1 = CSG::frustum_ptp(
        junction_point,
        cylinder1_end,
        cylinder_radius,
        cylinder_radius,
        segments,
        None
    );
    
    let cylinder2 = CSG::frustum_ptp(
        junction_point,
        cylinder2_end,
        cylinder_radius,
        cylinder_radius,
        segments,
        None
    );
    
    // Create simple union
    let simple_union = cylinder1.union(&cylinder2);
    
    // Create fillet surface
    let fillet_surface = create_cylinder_fillet(
        junction_point,
        cylinder1_end,
        cylinder2_end,
        cylinder_radius,
        config
    );
    
    // Create final filleted result
    let filleted_result = simple_union.union(&fillet_surface);
    
    (simple_union, fillet_surface, filleted_result)
}

/// Demonstration of different fillet configurations
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Configurable Cylinder Fillet System Demo");
    println!("========================================");
    
    // Common parameters
    let cylinder_diameter = 6.0;
    let cylinder_radius = cylinder_diameter / 2.0;
    let cylinder_length = 30.0;
    let segments = 24;
    
    let junction_point = Point3::new(0.0, 0.0, 0.0);
    let cylinder1_end = Point3::new(cylinder_length, 0.0, 0.0);
    
    // 120 degree angle
    let angle_radians = 120.0_f64.to_radians();
    let cylinder2_end = Point3::new(
        cylinder_length * angle_radians.cos(),
        cylinder_length * angle_radians.sin(),
        0.0
    );
    
    // Test different configurations
    let configs = vec![
        ("fast", FilletConfig::fast()),
        ("balanced", FilletConfig::balanced()),
        ("high_quality", FilletConfig::high_quality()),
    ];
    
    for (name, config) in configs {
        println!("\nCreating {} fillet...", name);
        println!("  Quality: {:?}", config.quality);
        println!("  Resolution: {:?}", config.get_resolution());
        println!("  Radius multiplier: {:.1}", config.radius_multiplier);
        println!("  Blend strength: {:.2}", config.blend_strength);
        
        let (simple, fillet, result) = create_filleted_cylinder_junction(
            junction_point,
            cylinder1_end,
            cylinder2_end,
            cylinder_radius,
            segments,
            &config
        );
        
        println!("  Simple union: {} polygons", simple.polygons.len());
        println!("  Fillet surface: {} polygons", fillet.polygons.len());
        println!("  Final result: {} polygons", result.polygons.len());
        
        // Export files
        #[cfg(feature = "stl-io")]
        {
            let filename = format!("cylinder_fillet_{}.stl", name);
            let stl_bytes = result.to_stl_binary(&format!("cylinder_fillet_{}", name))?;
            std::fs::write(&filename, stl_bytes)?;
            println!("  Exported: {}", filename);
        }
    }
    
    // Demonstrate custom configuration
    println!("\nCreating custom fillet configuration...");
    let custom_config = FilletConfig {
        quality: FilletQuality::Balanced,
        radius_multiplier: 2.5,  // Extra large fillet
        blend_strength: 0.3,     // Strong blending
        metaball_count: 5,       // More metaballs for smoother transition
    };
    
    let (_, _, custom_result) = create_filleted_cylinder_junction(
        junction_point,
        cylinder1_end,
        cylinder2_end,
        cylinder_radius,
        segments,
        &custom_config
    );
    
    println!("  Custom result: {} polygons", custom_result.polygons.len());
    
    #[cfg(feature = "stl-io")]
    {
        let stl_bytes = custom_result.to_stl_binary("cylinder_fillet_custom")?;
        std::fs::write("cylinder_fillet_custom.stl", stl_bytes)?;
        println!("  Exported: cylinder_fillet_custom.stl");
    }
    
    // Performance comparison
    println!("\n=== Performance Guide ===");
    println!("Fast:      Best for prototyping and real-time applications");
    println!("Balanced:  Good quality/speed tradeoff for most uses");
    println!("High:      Production quality for manufacturing");
    println!("Ultra:     Maximum quality for visualization/rendering");
    
    println!("\n=== Parameter Tuning Guide ===");
    println!("radius_multiplier: 1.2-3.0 (larger = bigger fillet)");
    println!("blend_strength: 0.3-0.8 (lower = more blending)");
    println!("metaball_count: 2-8 (more = smoother but slower)");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_fillet_config_presets() {
        let fast = FilletConfig::fast();
        let balanced = FilletConfig::balanced();
        let high = FilletConfig::high_quality();
        let ultra = FilletConfig::ultra_quality();
        
        // Verify quality progression
        assert!(matches!(fast.quality, FilletQuality::Fast));
        assert!(matches!(balanced.quality, FilletQuality::Balanced));
        assert!(matches!(high.quality, FilletQuality::High));
        assert!(matches!(ultra.quality, FilletQuality::Ultra));
        
        // Verify resolution progression
        let (fx, fy, fz) = fast.get_resolution();
        let (bx, by, bz) = balanced.get_resolution();
        let (hx, hy, hz) = high.get_resolution();
        let (ux, uy, uz) = ultra.get_resolution();
        
        assert!(fx < bx && bx < hx && hx < ux);
        assert!(fy < by && by < hy && hy < uy);
        assert!(fz < bz && bz < hz && hz < uz);
    }
    
    #[test]
    fn test_fillet_creation() {
        let junction = Point3::new(0.0, 0.0, 0.0);
        let end1 = Point3::new(10.0, 0.0, 0.0);
        let end2 = Point3::new(-5.0, 8.66, 0.0); // 120 degrees
        let radius = 2.0;
        let config = FilletConfig::fast();
        
        let (simple, fillet, result) = create_filleted_cylinder_junction(
            junction, end1, end2, radius, 12, &config
        );
        
        // All should produce geometry
        assert!(simple.polygons.len() > 0, "Simple union should have polygons");
        assert!(result.polygons.len() > 0, "Result should have polygons");
        
        // Result should have more polygons than simple (due to fillet addition)
        // Note: Might not be true if fillet is empty due to parameters
        println!("Simple: {}, Fillet: {}, Result: {}", 
                simple.polygons.len(), fillet.polygons.len(), result.polygons.len());
    }
    
    #[test]
    fn test_custom_config() {
        let mut config = FilletConfig::balanced();
        config.radius_multiplier = 3.0;
        config.blend_strength = 0.2;
        config.metaball_count = 8;
        
        // Should not panic with extreme values
        let junction = Point3::new(0.0, 0.0, 0.0);
        let end1 = Point3::new(5.0, 0.0, 0.0);
        let end2 = Point3::new(-2.5, 4.33, 0.0);
        
        let fillet = create_cylinder_fillet(junction, end1, end2, 1.0, &config);
        
        // Should complete without error
        println!("Custom config produced {} polygons", fillet.polygons.len());
    }
} 