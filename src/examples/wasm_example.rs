//! WebAssembly example demonstrating CSG operations in the browser
//!
//! This module provides a WebAssembly-compatible interface for csgrs,
//! allowing geometric operations to run in web browsers and other
//! WebAssembly environments.

use crate::indexed_mesh::IndexedMesh;
use crate::mesh::Mesh;
use crate::traits::CSG;

#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
use wasm_bindgen::prelude::*;

/// WebAssembly interface for basic CSG operations
#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
#[wasm_bindgen]
pub struct WasmCsg {
    mesh: Mesh<()>,
}

#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
#[wasm_bindgen]
impl WasmCsg {
    /// Create a new cube mesh
    #[wasm_bindgen(constructor)]
    pub fn new_cube(size: f64) -> WasmCsg {
        let mesh = Mesh::cube(size, None).expect("Failed to create cube mesh");
        WasmCsg { mesh }
    }

    /// Create a new sphere mesh
    #[wasm_bindgen]
    pub fn new_sphere(radius: f64, segments: usize, stacks: usize) -> WasmCsg {
        let mesh = Mesh::sphere(radius, segments, stacks, None)
            .expect("Failed to create sphere mesh");
        WasmCsg { mesh }
    }

    /// Create a new cylinder mesh
    #[wasm_bindgen]
    pub fn new_cylinder(radius: f64, height: f64, segments: usize) -> WasmCsg {
        let mesh = Mesh::cylinder(radius, height, segments, None)
            .expect("Failed to create cylinder mesh");
        WasmCsg { mesh }
    }

    /// Perform union operation with another mesh
    #[wasm_bindgen]
    pub fn union(&self, other: &WasmCsg) -> WasmCsg {
        let result = self.mesh.union(&other.mesh);
        WasmCsg { mesh: result }
    }

    /// Perform difference operation with another mesh
    #[wasm_bindgen]
    pub fn difference(&self, other: &WasmCsg) -> WasmCsg {
        let result = self.mesh.difference(&other.mesh);
        WasmCsg { mesh: result }
    }

    /// Perform intersection operation with another mesh
    #[wasm_bindgen]
    pub fn intersection(&self, other: &WasmCsg) -> WasmCsg {
        let result = self.mesh.intersection(&other.mesh);
        WasmCsg { mesh: result }
    }

    /// Translate the mesh
    #[wasm_bindgen]
    pub fn translate(&self, x: f64, y: f64, z: f64) -> WasmCsg {
        let result = self.mesh.translate(x, y, z);
        WasmCsg { mesh: result }
    }

    /// Rotate the mesh (degrees)
    #[wasm_bindgen]
    pub fn rotate(&self, x_deg: f64, y_deg: f64, z_deg: f64) -> WasmCsg {
        let result = self.mesh.rotate(x_deg, y_deg, z_deg);
        WasmCsg { mesh: result }
    }

    /// Scale the mesh
    #[wasm_bindgen]
    pub fn scale(&self, x: f64, y: f64, z: f64) -> WasmCsg {
        let result = self.mesh.scale(x, y, z);
        WasmCsg { mesh: result }
    }

    /// Export as ASCII STL string
    #[wasm_bindgen]
    pub fn to_stl_ascii(&self, solid_name: &str) -> String {
        self.mesh.to_stl_ascii(solid_name)
    }

    /// Get vertex count
    #[wasm_bindgen(getter)]
    pub fn vertex_count(&self) -> usize {
        self.mesh.vertices().len()
    }

    /// Get face count
    #[wasm_bindgen(getter)]
    pub fn face_count(&self) -> usize {
        self.mesh.polygons.len()
    }
}

/// WebAssembly interface for IndexedMesh operations
#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
#[wasm_bindgen]
pub struct WasmIndexedMesh {
    mesh: IndexedMesh<()>,
}

#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
#[wasm_bindgen]
impl WasmIndexedMesh {
    /// Create a new indexed cube
    #[wasm_bindgen(constructor)]
    pub fn new_cube(size: f64) -> WasmIndexedMesh {
        let mesh = crate::indexed_mesh::shapes::cube(size, None);
        WasmIndexedMesh { mesh }
    }

    /// Create a new indexed sphere
    #[wasm_bindgen]
    pub fn new_sphere(radius: f64, segments: usize, stacks: usize) -> WasmIndexedMesh {
        let mesh = crate::indexed_mesh::shapes::sphere(radius, segments, stacks, None);
        WasmIndexedMesh { mesh }
    }

    /// Perform union operation with another indexed mesh
    #[wasm_bindgen]
    pub fn union(&self, other: &WasmIndexedMesh) -> WasmIndexedMesh {
        let result = self.mesh.union(&other.mesh);
        WasmIndexedMesh { mesh: result }
    }

    /// Perform difference operation with another indexed mesh
    #[wasm_bindgen]
    pub fn difference(&self, other: &WasmIndexedMesh) -> WasmIndexedMesh {
        let result = self.mesh.difference(&other.mesh);
        WasmIndexedMesh { mesh: result }
    }

    /// Translate the indexed mesh
    #[wasm_bindgen]
    pub fn translate(&self, x: f64, y: f64, z: f64) -> WasmIndexedMesh {
        let result = self.mesh.translate(x, y, z);
        WasmIndexedMesh { mesh: result }
    }

    /// Export as ASCII STL string
    #[wasm_bindgen]
    pub fn to_stl_ascii(&self, solid_name: &str) -> String {
        self.mesh.to_stl_ascii(solid_name)
    }

    /// Get vertex count
    #[wasm_bindgen(getter)]
    pub fn vertex_count(&self) -> usize {
        self.mesh.vertices.len()
    }

    /// Get face count
    #[wasm_bindgen(getter)]
    pub fn face_count(&self) -> usize {
        self.mesh.faces.len()
    }

    /// Check if mesh is manifold
    #[wasm_bindgen]
    pub fn is_manifold(&self) -> bool {
        self.mesh.is_manifold()
    }
}

/// Run a simple WASM demonstration
#[cfg(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
))]
pub fn run_wasm_demo() -> Result<(), Box<dyn std::error::Error>> {
    // Create basic shapes
    let cube = WasmCsg::new_cube(2.0);
    let sphere = WasmCsg::new_sphere(1.25, 16, 8);

    // Perform boolean operations
    let difference = cube.difference(&sphere);

    println!("WebAssembly CSG Demo:");
    println!(
        "- Cube: {} vertices, {} faces",
        cube.vertex_count(),
        cube.face_count()
    );
    println!(
        "- Sphere: {} vertices, {} faces",
        sphere.vertex_count(),
        sphere.face_count()
    );
    println!(
        "- Difference: {} vertices, {} faces",
        difference.vertex_count(),
        difference.face_count()
    );

    // Test IndexedMesh
    let indexed_cube = WasmIndexedMesh::new_cube(2.0);
    let indexed_sphere = WasmIndexedMesh::new_sphere(1.25, 16, 8);
    let indexed_diff = indexed_cube.difference(&indexed_sphere);

    println!("\nIndexedMesh Demo:");
    println!(
        "- Cube: {} vertices, {} faces",
        indexed_cube.vertex_count(),
        indexed_cube.face_count()
    );
    println!(
        "- Sphere: {} vertices, {} faces",
        indexed_sphere.vertex_count(),
        indexed_sphere.face_count()
    );
    println!(
        "- Difference: {} vertices, {} faces, manifold: {}",
        indexed_diff.vertex_count(),
        indexed_diff.face_count(),
        indexed_diff.is_manifold()
    );

    println!("\nWebAssembly support successfully demonstrated!");
    Ok(())
}

#[cfg(not(all(
    feature = "wasm-bindgen",
    any(target_arch = "wasm32", target_arch = "wasm64")
)))]
pub fn run_wasm_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "WebAssembly demo requires the 'wasm-bindgen' feature to be enabled and WASM target architecture."
    );
    println!(
        "Run with: cargo run --features wasm --target wasm32-unknown-unknown --bin csgrs-examples -- wasm"
    );
    Ok(())
}
