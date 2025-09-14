// main.rs
//
// Modular demonstration of csgrs capabilities using organized example modules.
// This replaces the monolithic example with focused, maintainable demonstrations.

use csgrs::examples;
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse command line arguments
    let args: Vec<String> = env::args().collect();

    // Ensure the /stl folder exists
    std::fs::create_dir_all("stl")?;

    // Default to running all examples if no arguments provided
    if args.len() == 1 {
        println!("CSGRS Examples Runner");
        println!(
            "Usage: {} [all|basic|advanced|shapes|transform|boolean|extrude]",
            args[0]
        );
        println!("Running all examples by default...\n");
        return examples::run_all_examples();
    }

    // Handle command line arguments
    match args[1].as_str() {
        "all" => examples::run_all_examples(),
        "basic" => examples::run_basic_examples(),
        "advanced" => examples::run_advanced_examples(),
        "shapes" => {
            csgrs::examples::basic_shapes::run_basic_shapes_demo()?;
            csgrs::examples::basic_shapes::run_2d_shapes_demo()
        },
        "transform" => {
            csgrs::examples::transformations::run_basic_transformations_demo()?;
            csgrs::examples::transformations::run_mirroring_demo()?;
            csgrs::examples::transformations::run_centering_demo()
        },
        "boolean" => {
            csgrs::examples::boolean_ops::run_boolean_operations_demo()?;
            csgrs::examples::boolean_ops::run_complex_boolean_demo()?;
            csgrs::examples::boolean_ops::run_inversion_demo()
        },
        "extrude" => {
            csgrs::examples::advanced_features::run_extrusion_demo()?;
            csgrs::examples::advanced_features::run_2d_boolean_demo()
        },
        _ => {
            eprintln!("Unknown argument: {}", args[1]);
            eprintln!(
                "Usage: {} [all|basic|advanced|shapes|transform|boolean|extrude]",
                args[0]
            );
            std::process::exit(1);
        },
    }
}
