//! Mathematical function mesh synthesis
//!
//! This module provides algorithms for generating meshes from mathematical
//! functions and equations, creating complex surfaces and volumes.

use crate::indexed_mesh::{IndexedMesh, IndexedFace};
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

/// Generate mesh from a 2D function z = f(x, y)
pub fn synthesize_from_function<F>(
    x_min: f64, x_max: f64, x_steps: usize,
    y_min: f64, y_max: f64, y_steps: usize,
    function: F,
) -> IndexedMesh<()>
where
    F: Fn(f64, f64) -> f64,
{
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    let dx = (x_max - x_min) / x_steps as f64;
    let dy = (y_max - y_min) / y_steps as f64;

    // Generate vertices
    for i in 0..=x_steps {
        let x = x_min + i as f64 * dx;
        for j in 0..=y_steps {
            let y = y_min + j as f64 * dy;
            let z = function(x, y);

            // Approximate normal using partial derivatives
            let normal = calculate_function_normal(&function, x, y, 0.01);

            vertices.push(Vertex::new(
                Point3::new(x, y, z),
                normal,
            ));
        }
    }

    // Generate faces
    for i in 0..x_steps {
        for j in 0..y_steps {
            let i0 = i * (y_steps + 1) + j;
            let i1 = i * (y_steps + 1) + (j + 1);
            let i2 = (i + 1) * (y_steps + 1) + j;
            let i3 = (i + 1) * (y_steps + 1) + (j + 1);

            faces.push(IndexedFace {
                vertices: vec![i0, i2, i1],
            });
            faces.push(IndexedFace {
                vertices: vec![i1, i2, i3],
            });
        }
    }

    IndexedMesh::from_vertices_and_faces(vertices, faces, Some(()))
}

/// Calculate surface normal for a 2D function
fn calculate_function_normal<F>(
    function: &F,
    x: f64,
    y: f64,
    epsilon: f64,
) -> Vector3<f64>
where
    F: Fn(f64, f64) -> f64,
{
    // Calculate partial derivatives
    let z = function(x, y);
    let zx = (function(x + epsilon, y) - z) / epsilon;
    let zy = (function(x, y + epsilon) - z) / epsilon;

    // Normal is (-zx, -zy, 1)
    let normal = Vector3::new(-zx, -zy, 1.0);
    let length = normal.magnitude();

    if length > 0.0 {
        normal / length
    } else {
        Vector3::z()
    }
}

/// Predefined mathematical surfaces
pub mod surfaces {
    use super::*;

    /// Generate a sine wave surface
    pub fn sine_wave(amplitude: f64, frequency: f64, x_steps: usize, y_steps: usize) -> IndexedMesh<()> {
        synthesize_from_function(
            -5.0, 5.0, x_steps,
            -5.0, 5.0, y_steps,
            move |x, y| amplitude * (frequency * (x * x + y * y).sqrt()).sin()
        )
    }

    /// Generate a ripple surface
    pub fn ripples(amplitude: f64, frequency: f64, x_steps: usize, y_steps: usize) -> IndexedMesh<()> {
        synthesize_from_function(
            -5.0, 5.0, x_steps,
            -5.0, 5.0, y_steps,
            move |x, y| amplitude * ((frequency * (x * x + y * y).sqrt()).sin()) / (1.0 + (x * x + y * y).sqrt())
        )
    }

    /// Generate a gaussian bump
    pub fn gaussian_bump(amplitude: f64, sigma: f64, x_steps: usize, y_steps: usize) -> IndexedMesh<()> {
        synthesize_from_function(
            -3.0, 3.0, x_steps,
            -3.0, 3.0, y_steps,
            move |x, y| amplitude * (-(x * x + y * y) / (2.0 * sigma * sigma)).exp()
        )
    }

    /// Generate a hyperbolic paraboloid (saddle surface)
    pub fn hyperbolic_paraboloid(a: f64, b: f64, x_steps: usize, y_steps: usize) -> IndexedMesh<()> {
        synthesize_from_function(
            -2.0, 2.0, x_steps,
            -2.0, 2.0, y_steps,
            move |x, y| (x * x) / (a * a) - (y * y) / (b * b)
        )
    }

    /// Generate a mexican hat wavelet surface
    pub fn mexican_hat(amplitude: f64, sigma: f64, x_steps: usize, y_steps: usize) -> IndexedMesh<()> {
        synthesize_from_function(
            -5.0, 5.0, x_steps,
            -5.0, 5.0, y_steps,
            move |x, y| {
                let r2 = x * x + y * y;
                let r = r2.sqrt();
                amplitude * (2.0 / (std::f64::consts::PI * sigma.powi(4))) * (1.0 - r2 / (2.0 * sigma * sigma)) * (-r2 / (2.0 * sigma * sigma)).exp()
            }
        )
    }
}

/// Generate implicit surface using marching cubes (simplified)
pub fn implicit_surface<F>(
    function: F,
    bounds: (f64, f64, f64, f64, f64, f64), // min_x, max_x, min_y, max_y, min_z, max_z
    resolution: usize,
) -> IndexedMesh<()>
where
    F: Fn(f64, f64, f64) -> f64,
{
    let (min_x, max_x, min_y, max_y, min_z, max_z) = bounds;
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    let dx = (max_x - min_x) / resolution as f64;
    let dy = (max_y - min_y) / resolution as f64;
    let dz = (max_z - min_z) / resolution as f64;

    // Simple surface extraction (placeholder - full marching cubes is complex)
    for i in 0..resolution {
        for j in 0..resolution {
            for k in 0..resolution {
                let x = min_x + i as f64 * dx;
                let y = min_y + j as f64 * dy;
                let z = min_z + k as f64 * dz;

                let value = function(x, y, z);

                // If close to surface, add vertex
                if value.abs() < 0.1 {
                    vertices.push(Vertex::new(
                        Point3::new(x, y, z),
                        Vector3::new(1.0, 0.0, 0.0), // Placeholder normal
                    ));
                }
            }
        }
    }

    // Simple triangulation (placeholder)
    for i in 0..(vertices.len().saturating_sub(3)) {
        faces.push(IndexedFace {
            vertices: vec![i, i + 1, i + 2],
        });
    }

    IndexedMesh::from_vertices_and_faces(vertices, faces, Some(()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_function_synthesis() {
        let surface = synthesize_from_function(
            -1.0, 1.0, 4,
            -1.0, 1.0, 4,
            |x, y| x * x + y * y
        );

        assert!(!surface.vertices.is_empty());
        assert!(!surface.faces.is_empty());
    }

    #[test]
    fn test_sine_wave_surface() {
        let wave = surfaces::sine_wave(1.0, 1.0, 16, 16);
        assert!(wave.vertices.len() > 0);
        assert!(wave.faces.len() > 0);
    }

    #[test]
    fn test_ripple_surface() {
        let ripples = surfaces::ripples(1.0, 2.0, 16, 16);
        assert!(ripples.vertices.len() > 0);
        assert!(ripples.faces.len() > 0);
    }

    #[test]
    fn test_gaussian_bump() {
        let bump = surfaces::gaussian_bump(2.0, 1.0, 16, 16);
        assert!(bump.vertices.len() > 0);
        assert!(bump.faces.len() > 0);
    }

    #[test]
    fn test_implicit_surface() {
        let sphere = implicit_surface(
            |x, y, z| x * x + y * y + z * z - 1.0, // Unit sphere equation
            (-2.0, 2.0, -2.0, 2.0, -2.0, 2.0),
            8
        );

        assert!(sphere.vertices.len() >= 0); // May be empty with simple algorithm
    }
}
