//! Parametric surface generation
//!
//! This module provides algorithms for generating complex surfaces using
//! parametric equations, including mathematical surfaces and custom functions.

use crate::indexed_mesh::{IndexedMesh, IndexedFace};
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::f64::consts::PI;

/// Generate a parametric surface from a function
pub fn generate_parametric_surface<F>(
    u_min: f64, u_max: f64, u_steps: usize,
    v_min: f64, v_max: f64, v_steps: usize,
    surface_function: F,
) -> IndexedMesh<()>
where
    F: Fn(f64, f64) -> (f64, f64, f64),
{
    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    let du = (u_max - u_min) / u_steps as f64;
    let dv = (v_max - v_min) / v_steps as f64;

    // Generate vertices
    for i in 0..=u_steps {
        let u = u_min + i as f64 * du;
        for j in 0..=v_steps {
            let v = v_min + j as f64 * dv;

            let (x, y, z) = surface_function(u, v);

            // Approximate normal using partial derivatives
            let normal = calculate_surface_normal(&surface_function, u, v, 0.01);

            vertices.push(Vertex::new(
                Point3::new(x, y, z),
                normal,
            ));
        }
    }

    // Generate faces
    for i in 0..u_steps {
        for j in 0..v_steps {
            let i0 = i * (v_steps + 1) + j;
            let i1 = i * (v_steps + 1) + (j + 1);
            let i2 = (i + 1) * (v_steps + 1) + j;
            let i3 = (i + 1) * (v_steps + 1) + (j + 1);

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

/// Calculate surface normal using partial derivatives
fn calculate_surface_normal<F>(
    surface_fn: &F,
    u: f64,
    v: f64,
    epsilon: f64,
) -> Vector3<f64>
where
    F: Fn(f64, f64) -> (f64, f64, f64),
{
    // Calculate partial derivatives
    let p0 = surface_fn(u, v);
    let pu = surface_fn(u + epsilon, v);
    let pv = surface_fn(u, v + epsilon);

    let du = (
        pu.0 - p0.0,
        pu.1 - p0.1,
        pu.2 - p0.2,
    );

    let dv = (
        pv.0 - p0.0,
        pv.1 - p0.1,
        pv.2 - p0.2,
    );

    // Cross product gives normal
    let normal = Vector3::new(
        du.1 * dv.2 - du.2 * dv.1,
        du.2 * dv.0 - du.0 * dv.2,
        du.0 * dv.1 - du.1 * dv.0,
    );

    let length = normal.magnitude();
    if length > 0.0 {
        normal / length
    } else {
        Vector3::z() // Default normal
    }
}

/// Predefined parametric surfaces
pub mod presets {
    use super::*;

    /// Generate a torus (doughnut shape)
    pub fn torus(major_radius: f64, minor_radius: f64, u_steps: usize, v_steps: usize) -> IndexedMesh<()> {
        generate_parametric_surface(
            0.0, 2.0 * PI, u_steps,
            0.0, 2.0 * PI, v_steps,
            move |u, v| {
                let x = (major_radius + minor_radius * v.cos()) * u.cos();
                let y = (major_radius + minor_radius * v.cos()) * u.sin();
                let z = minor_radius * v.sin();
                (x, y, z)
            }
        )
    }

    /// Generate a mobius strip
    pub fn mobius_strip(radius: f64, width: f64, u_steps: usize, v_steps: usize) -> IndexedMesh<()> {
        generate_parametric_surface(
            0.0, 2.0 * PI, u_steps,
            -width/2.0, width/2.0, v_steps,
            move |u, v| {
                let x = (radius + v * (u * 0.5).cos()) * u.cos();
                let y = (radius + v * (u * 0.5).cos()) * u.sin();
                let z = v * (u * 0.5).sin();
                (x, y, z)
            }
        )
    }

    /// Generate a Klein bottle (simplified parametric approximation)
    pub fn klein_bottle(u_steps: usize, v_steps: usize) -> IndexedMesh<()> {
        generate_parametric_surface(
            0.0, 2.0 * PI, u_steps,
            0.0, 2.0 * PI, v_steps,
            |u, v| {
                let r = 4.0 * (1.0 - (u / (2.0 * PI)).cos() / 2.0);
                let x = 6.0 * (u / (2.0 * PI)).cos() * (1.0 + (v / (2.0 * PI)).sin());
                let y = 16.0 * (v / (2.0 * PI)).sin();
                let z = 4.0 * (u / (2.0 * PI)).sin() * (1.0 + (v / (2.0 * PI)).sin());
                (x, y, z)
            }
        )
    }

    /// Generate a seashell spiral
    pub fn seashell(a: f64, b: f64, c: f64, n: f64, u_steps: usize, v_steps: usize) -> IndexedMesh<()> {
        generate_parametric_surface(
            0.0, 4.0 * PI, u_steps,
            0.0, 2.0 * PI, v_steps,
            move |u, v| {
                let r = a + b * (n * u).powf(2.0/3.0);
                let x = r * u.cos() * v.cos();
                let y = r * u.sin() * v.cos();
                let z = c * n * u + r * v.sin();
                (x, y, z)
            }
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parametric_surface_generation() {
        let surface = generate_parametric_surface(
            0.0, PI, 8,
            0.0, 2.0 * PI, 8,
            |u, v| (u.cos() * v.cos(), u.cos() * v.sin(), u.sin())
        );

        assert!(surface.vertices.len() > 0);
        assert!(surface.faces.len() > 0);
        assert_eq!(surface.vertices.len(), 81); // (8+1) * (8+1)
    }

    #[test]
    fn test_torus_generation() {
        let torus = presets::torus(3.0, 1.0, 16, 16);
        assert!(torus.vertices.len() > 0);
        assert!(torus.faces.len() > 0);
    }

    #[test]
    fn test_mobius_strip_generation() {
        let mobius = presets::mobius_strip(2.0, 0.5, 16, 8);
        assert!(mobius.vertices.len() > 0);
        assert!(mobius.faces.len() > 0);
    }

    #[test]
    fn test_klein_bottle_generation() {
        let klein = presets::klein_bottle(16, 16);
        assert!(klein.vertices.len() > 0);
        assert!(klein.faces.len() > 0);
    }

    #[test]
    fn test_seashell_generation() {
        let shell = presets::seashell(1.0, 0.5, 0.1, 2.0, 16, 16);
        assert!(shell.vertices.len() > 0);
        assert!(shell.faces.len() > 0);
    }
}
