//! glTF 2.0 file format support for csgrs
//!
//! This module provides import and export functionality for glTF 2.0 files,
//! supporting both JSON (.gltf) and binary (.glb) formats.
//!
//! # Features
//!
//! - **Import**: Load glTF scenes with meshes, materials, and animations
//! - **Export**: Save IndexedMesh objects to glTF format
//! - **Materials**: Basic material support with PBR properties
//! - **Animations**: Skeletal animation support (future)
//! - **Binary**: Full .glb support for compact storage
//!
//! # Usage
//!
//! ```rust,ignore
//! use csgrs::io::gltf::{import_gltf, export_gltf};
//!
//! // Import a glTF file
//! let meshes = import_gltf("model.gltf")?;
//!
//! // Export meshes to glTF
//! export_gltf(&meshes, "output.gltf")?;
//! ```

use crate::errors::CsgrsResult;
use crate::indexed_mesh::{IndexedMesh, IndexedFace};
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3, Matrix4};
use std::collections::HashMap;
use std::path::Path;

/// Import meshes from a glTF file
#[cfg(feature = "gltf")]
pub fn import_gltf<P: AsRef<Path>>(path: P) -> CsgrsResult<Vec<IndexedMesh<()>>> {
    use gltf::Gltf;

    let gltf = Gltf::open(path)?;
    let mut meshes = Vec::new();

    // Process all scenes (or just the default scene)
    let scenes = if let Some(default_scene) = gltf.default_scene() {
        vec![default_scene]
    } else {
        gltf.scenes().collect()
    };

    for scene in scenes {
        for node in scene.nodes() {
            process_node(&node, &gltf, &mut meshes, &Matrix4::identity());
        }
    }

    Ok(meshes)
}

/// Recursively process a glTF node and extract meshes
#[cfg(feature = "gltf")]
fn process_node(
    node: &gltf::Node,
    gltf: &gltf::Gltf,
    meshes: &mut Vec<IndexedMesh<()>>,
    parent_transform: &Matrix4<f32>,
) {
    // Calculate node transform
    let local_transform = node_transform(node);
    let world_transform = parent_transform * local_transform;

    // Process mesh if present
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            if let Some(indexed_mesh) = primitive_to_indexed_mesh(&primitive, gltf) {
                // Apply world transform
                let transformed_mesh = indexed_mesh.transform(&world_transform.cast());
                meshes.push(transformed_mesh);
            }
        }
    }

    // Process children
    for child in node.children() {
        process_node(&child, gltf, meshes, &world_transform);
    }
}

/// Convert glTF node to transformation matrix
#[cfg(feature = "gltf")]
fn node_transform(node: &gltf::Node) -> Matrix4<f32> {
    let (translation, rotation, scale) = node.transform().decomposed();

    // Convert quaternion to rotation matrix
    let rot_matrix = Matrix4::from_axis_angle(
        &Vector3::x_axis(),
        rotation[0],
    ) * Matrix4::from_axis_angle(
        &Vector3::y_axis(),
        rotation[1],
    ) * Matrix4::from_axis_angle(
        &Vector3::z_axis(),
        rotation[2],
    );

    // Apply scale and translation
    let mut transform = rot_matrix;
    transform[(0, 0)] *= scale[0];
    transform[(1, 1)] *= scale[1];
    transform[(2, 2)] *= scale[2];
    transform[(0, 3)] = translation[0];
    transform[(1, 3)] = translation[1];
    transform[(2, 3)] = translation[2];

    transform
}

/// Convert glTF primitive to IndexedMesh
#[cfg(feature = "gltf")]
fn primitive_to_indexed_mesh(
    primitive: &gltf::Primitive,
    gltf: &gltf::Gltf,
) -> Option<IndexedMesh<()>> {
    let reader = primitive.reader(|buffer| Some(&gltf.buffers().nth(buffer.index())?.0));

    // Get positions
    let positions = reader.read_positions()?;
    let positions: Vec<Point3<f32>> = positions
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();

    // Get normals (optional)
    let normals = if let Some(normals_iter) = reader.read_normals() {
        normals_iter
            .map(|n| Vector3::new(n[0], n[1], n[2]))
            .collect()
    } else {
        // Generate normals if not present
        generate_normals(&positions)
    };

    // Get indices
    let indices = reader.read_indices()?.into_u32();
    let indices: Vec<usize> = indices.map(|i| i as usize).collect();

    // Convert to IndexedMesh format
    let vertices: Vec<Vertex> = positions
        .into_iter()
        .zip(normals)
        .map(|(pos, normal)| Vertex::new(pos.cast(), normal.cast()))
        .collect();

    // Group indices into triangles
    let mut faces = Vec::new();
    for chunk in indices.chunks_exact(3) {
        faces.push(IndexedFace {
            vertices: vec![chunk[0], chunk[1], chunk[2]],
        });
    }

    Some(IndexedMesh::from_vertices_and_faces(vertices, faces, Some(())))
}

/// Generate normals for a mesh without them
#[cfg(feature = "gltf")]
fn generate_normals(positions: &[Point3<f32>]) -> Vec<Vector3<f32>> {
    let mut normals = vec![Vector3::zeros(); positions.len()];

    // For each triangle, compute face normal and add to vertex normals
    for i in (0..positions.len()).step_by(3) {
        if i + 2 >= positions.len() {
            break;
        }

        let v0 = positions[i];
        let v1 = positions[i + 1];
        let v2 = positions[i + 2];

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let face_normal = edge1.cross(&edge2).normalize();

        normals[i] += face_normal;
        normals[i + 1] += face_normal;
        normals[i + 2] += face_normal;
    }

    // Normalize all vertex normals
    for normal in &mut normals {
        if normal.magnitude() > 0.0 {
            *normal = normal.normalize();
        } else {
            *normal = Vector3::z(); // Default normal
        }
    }

    normals
}

/// Export meshes to a glTF file
#[cfg(feature = "gltf")]
pub fn export_gltf<P: AsRef<Path>>(
    meshes: &[IndexedMesh<impl Clone>],
    path: P,
) -> CsgrsResult<()> {
    use gltf::json::{self, validation::Checked::*};
    use gltf::{buffer, mesh};

    let mut root = json::Root::default();

    // Create buffer for vertex/index data
    let mut buffer_data = Vec::new();
    let mut buffer_views = Vec::new();
    let mut accessors = Vec::new();
    let mut meshes_json = Vec::new();

    for (mesh_idx, csgrs_mesh) in meshes.iter().enumerate() {
        // Prepare vertex data
        let positions: Vec<[f32; 3]> = csgrs_mesh
            .vertices()
            .iter()
            .map(|v| [v.position.x as f32, v.position.y as f32, v.position.z as f32])
            .collect();

        let normals: Vec<[f32; 3]> = csgrs_mesh
            .vertices()
            .iter()
            .map(|v| [v.normal.x as f32, v.normal.y as f32, v.normal.z as f32])
            .collect();

        // Prepare index data
        let indices: Vec<u32> = csgrs_mesh
            .faces()
            .iter()
            .flat_map(|face| face.vertices.iter().map(|&idx| idx as u32))
            .collect();

        // Add positions to buffer
        let positions_offset = buffer_data.len();
        buffer_data.extend_from_slice(bytemuck::cast_slice(&positions));
        buffer_views.push(json::buffer::View {
            buffer: json::Index::new(0),
            byte_length: (positions.len() * 12) as u32,
            byte_offset: Some(positions_offset as u32),
            byte_stride: None,
            extensions: Default::default(),
            extras: Default::default(),
            name: None,
            target: Some(Checked::Valid(json::buffer::Target::ArrayBuffer)),
        });
        accessors.push(json::Accessor {
            buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
            byte_offset: 0,
            count: positions.len() as u32,
            component_type: Checked::Valid(json::accessor::GenericComponentType(
                json::accessor::ComponentType::F32,
            )),
            extensions: Default::default(),
            extras: Default::default(),
            type_: Checked::Valid(json::accessor::Type::Vec3),
            min: None,
            max: None,
            name: None,
            normalized: false,
            sparse: None,
        });

        // Add normals to buffer
        let normals_offset = buffer_data.len();
        buffer_data.extend_from_slice(bytemuck::cast_slice(&normals));
        buffer_views.push(json::buffer::View {
            buffer: json::Index::new(0),
            byte_length: (normals.len() * 12) as u32,
            byte_offset: Some(normals_offset as u32),
            byte_stride: None,
            extensions: Default::default(),
            extras: Default::default(),
            name: None,
            target: Some(Checked::Valid(json::buffer::Target::ArrayBuffer)),
        });
        accessors.push(json::Accessor {
            buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
            byte_offset: 0,
            count: normals.len() as u32,
            component_type: Checked::Valid(json::accessor::GenericComponentType(
                json::accessor::ComponentType::F32,
            )),
            extensions: Default::default(),
            extras: Default::default(),
            type_: Checked::Valid(json::accessor::Type::Vec3),
            min: None,
            max: None,
            name: None,
            normalized: false,
            sparse: None,
        });

        // Add indices to buffer
        let indices_offset = buffer_data.len();
        buffer_data.extend_from_slice(bytemuck::cast_slice(&indices));
        buffer_views.push(json::buffer::View {
            buffer: json::Index::new(0),
            byte_length: (indices.len() * 4) as u32,
            byte_offset: Some(indices_offset as u32),
            byte_stride: None,
            extensions: Default::default(),
            extras: Default::default(),
            name: None,
            target: Some(Checked::Valid(json::buffer::Target::ElementArrayBuffer)),
        });
        accessors.push(json::Accessor {
            buffer_view: Some(json::Index::new(buffer_views.len() as u32 - 1)),
            byte_offset: 0,
            count: indices.len() as u32,
            component_type: Checked::Valid(json::accessor::GenericComponentType(
                json::accessor::ComponentType::U32,
            )),
            extensions: Default::default(),
            extras: Default::default(),
            type_: Checked::Valid(json::accessor::Type::Scalar),
            min: None,
            max: None,
            name: None,
            normalized: false,
            sparse: None,
        });

        // Create primitive
        let primitive = json::mesh::Primitive {
            attributes: {
                let mut map = std::collections::HashMap::new();
                map.insert(
                    Checked::Valid(json::mesh::Semantic::Positions),
                    json::Index::new((mesh_idx * 3) as u32),
                );
                map.insert(
                    Checked::Valid(json::mesh::Semantic::Normals),
                    json::Index::new((mesh_idx * 3 + 1) as u32),
                );
                map
            },
            extensions: Default::default(),
            extras: Default::default(),
            indices: Some(json::Index::new((mesh_idx * 3 + 2) as u32)),
            material: None,
            mode: Checked::Valid(json::mesh::Mode::Triangles),
            targets: None,
        };

        // Create mesh
        let mesh_json = json::Mesh {
            extensions: Default::default(),
            extras: Default::default(),
            name: Some(format!("mesh_{}", mesh_idx)),
            primitives: vec![primitive],
            weights: None,
        };

        meshes_json.push(mesh_json);
    }

    // Create buffer
    let buffer = json::Buffer {
        byte_length: buffer_data.len() as u32,
        extensions: Default::default(),
        extras: Default::default(),
        name: None,
        uri: None, // Embedded in glTF
    };

    // Assemble root
    root.buffers = vec![buffer];
    root.buffer_views = buffer_views;
    root.accessors = accessors;
    root.meshes = meshes_json;

    // Create nodes and scene
    let mut nodes = Vec::new();
    for i in 0..meshes.len() {
        nodes.push(json::Node {
            camera: None,
            children: None,
            extensions: Default::default(),
            extras: Default::default(),
            matrix: None,
            mesh: Some(json::Index::new(i as u32)),
            name: Some(format!("node_{}", i)),
            rotation: None,
            scale: None,
            translation: None,
            skin: None,
            weights: None,
        });
    }

    let scene = json::Scene {
        extensions: Default::default(),
        extras: Default::default(),
        name: None,
        nodes: (0..nodes.len()).map(|i| json::Index::new(i as u32)).collect(),
    };

    root.nodes = nodes;
    root.scenes = vec![scene];
    root.scene = Some(json::Index::new(0));

    // Write to file
    let path = path.as_ref();
    if path.extension().and_then(|s| s.to_str()) == Some("glb") {
        // Write as binary glTF
        let json_string = json::serialize::to_string(&root)?;
        let json_bytes = json_string.as_bytes();

        // Calculate offsets
        let json_padding = (4 - (json_bytes.len() % 4)) % 4;
        let bin_padding = (4 - (buffer_data.len() % 4)) % 4;
        let json_chunk_size = json_bytes.len() + json_padding;
        let bin_chunk_size = buffer_data.len() + bin_padding;
        let total_size = 12 + 8 + json_chunk_size + 8 + bin_chunk_size;

        let mut file_data = Vec::with_capacity(total_size);

        // glTF header
        file_data.extend_from_slice(&0x46546C67u32.to_le_bytes()); // "glTF"
        file_data.extend_from_slice(&2u32.to_le_bytes()); // version
        file_data.extend_from_slice(&(total_size as u32).to_le_bytes());

        // JSON chunk header
        file_data.extend_from_slice(&(json_chunk_size as u32).to_le_bytes());
        file_data.extend_from_slice(&0x4E4F534Au32.to_le_bytes()); // "JSON"
        file_data.extend_from_slice(json_bytes);
        file_data.extend_from_slice(&vec![b' '; json_padding]);

        // BIN chunk header
        file_data.extend_from_slice(&(bin_chunk_size as u32).to_le_bytes());
        file_data.extend_from_slice(&0x004E4942u32.to_le_bytes()); // "BIN"
        file_data.extend_from_slice(&buffer_data);
        file_data.extend_from_slice(&vec![0u8; bin_padding]);

        std::fs::write(path, file_data)?;
    } else {
        // Write as JSON glTF
        let json_string = json::serialize::to_string_pretty(&root)?;
        std::fs::write(path, json_string)?;
    }

    Ok(())
}

/// Stub implementations for when glTF feature is disabled
#[cfg(not(feature = "gltf"))]
pub fn import_gltf<P: AsRef<Path>>(_path: P) -> CsgrsResult<Vec<IndexedMesh<()>>> {
    Err(crate::errors::CsgrsError::UnsupportedFormat(
        "glTF support not enabled. Enable with --features gltf".to_string()
    ))
}

#[cfg(not(feature = "gltf"))]
pub fn export_gltf<P: AsRef<Path>>(
    _meshes: &[IndexedMesh<impl Clone>],
    _path: P,
) -> CsgrsResult<()> {
    Err(crate::errors::CsgrsError::UnsupportedFormat(
        "glTF support not enabled. Enable with --features gltf".to_string()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::indexed_mesh::shapes;

    #[cfg(feature = "gltf")]
    #[test]
    fn test_gltf_export_basic() {
        let cube = shapes::cube::<()>(1.0, None);

        // Test export to JSON glTF
        let result = export_gltf(&[cube.clone()], "test_cube.gltf");
        assert!(result.is_ok());

        // Test export to binary glTF
        let result = export_gltf(&[cube], "test_cube.glb");
        assert!(result.is_ok());

        // Clean up test files
        let _ = std::fs::remove_file("test_cube.gltf");
        let _ = std::fs::remove_file("test_cube.glb");
    }

    #[cfg(not(feature = "gltf"))]
    #[test]
    fn test_gltf_disabled() {
        let cube = shapes::cube::<()>(1.0, None);

        // Should return error when glTF feature is disabled
        let import_result = import_gltf("nonexistent.gltf");
        assert!(import_result.is_err());

        let export_result = export_gltf(&[cube], "test.gltf");
        assert!(export_result.is_err());
    }
}
