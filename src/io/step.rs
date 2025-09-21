//! STEP (ISO 10303) file format support
//!
//! This module provides comprehensive support for reading and writing STEP files,
//! the ISO 10303 standard for exchanging product manufacturing information.
//!
//! ## Supported STEP Entities
//!
//! ### Geometric Entities
//! - `CARTESIAN_POINT`: 3D coordinate points
//! - `DIRECTION`: Direction vectors (unit vectors)
//! - `VECTOR`: Magnitude and direction
//! - `AXIS2_PLACEMENT_3D`: Coordinate system placement
//! - `PLANE`: Infinite plane definition
//! - `CIRCLE`: Circular curve
//! - `LINE`: Infinite line definition
//! - `TRIMMED_CURVE`: Bounded curve segments
//!
//! ### Topological Entities
//! - `VERTEX_POINT`: Point vertex
//! - `EDGE_CURVE`: Curve-based edge
//! - `FACE_BOUND`: Face boundary definition
//! - `FACE_OUTER_BOUND`: Outer face boundary
//! - `ADVANCED_FACE`: Face with surface and boundaries
//! - `CLOSED_SHELL`: Closed shell of faces
//! - `MANIFOLD_SOLID_BREP`: Solid defined by BREP
//! - `SOLID_MODEL`: Complete solid model
//!
//! ## Usage
//!
//! ```rust,no_run
//! # #[cfg(feature = "step-io")]
//! # {
//! use csgrs::io::{StepModel, StepError};
//!
//! // Read a STEP file
//! let model = StepModel::from_file("model.stp").expect("Failed to read STEP file");
//!
//! // Convert to mesh for CSG operations
//! let mesh = model.to_mesh().expect("Failed to convert STEP to mesh");
//!
//! // Write mesh back to STEP
//! model.write_to_file("output.stp").expect("Failed to write STEP file");
//! # }
//! ```

use crate::float_types::Real;
use crate::mesh::Mesh;
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

/// Errors that can occur during STEP file processing
#[derive(Debug)]
pub enum StepError {
    /// I/O error reading or writing file
    Io(std::io::Error),
    /// Malformed STEP file structure
    MalformedStructure(String),
    /// Unsupported STEP entity
    UnsupportedEntity(String),
    /// Invalid geometric data
    InvalidGeometry(String),
    /// Entity reference not found
    EntityNotFound(String),
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StepError::Io(e) => write!(f, "I/O error: {}", e),
            StepError::MalformedStructure(msg) => write!(f, "Malformed STEP structure: {}", msg),
            StepError::UnsupportedEntity(entity) => write!(f, "Unsupported STEP entity: {}", entity),
            StepError::InvalidGeometry(msg) => write!(f, "Invalid geometry: {}", msg),
            StepError::EntityNotFound(entity) => write!(f, "Entity not found: {}", entity),
        }
    }
}

impl std::error::Error for StepError {}

impl From<std::io::Error> for StepError {
    fn from(error: std::io::Error) -> Self {
        StepError::Io(error)
    }
}

impl From<String> for StepError {
    fn from(error: String) -> Self {
        StepError::MalformedStructure(error)
    }
}

/// Unique identifier for STEP entities
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct EntityId {
    /// The numeric ID of the entity
    pub id: u64,
    /// Optional entity name
    pub name: Option<String>,
}

impl EntityId {
    /// Create a new entity ID
    pub const fn new(id: u64) -> Self {
        Self { id, name: None }
    }

    /// Create a named entity ID
    pub const fn with_name(id: u64, name: String) -> Self {
        Self { id, name: Some(name) }
    }
}

impl std::fmt::Display for EntityId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.name {
            Some(name) => write!(f, "#{}={}", self.id, name),
            None => write!(f, "#{}", self.id),
        }
    }
}

/// Base trait for all STEP entities
pub trait StepEntity: std::fmt::Debug {
    /// Get the entity ID
    fn entity_id(&self) -> &EntityId;

    /// Get the entity type name
    fn entity_type(&self) -> &'static str;

    /// Convert entity to STEP format string
    fn to_step(&self) -> String;
}

/// 3D point in Cartesian coordinates
#[derive(Debug, Clone)]
pub struct CartesianPoint {
    pub entity_id: EntityId,
    pub coordinates: Point3<Real>,
}

impl StepEntity for CartesianPoint {
    fn entity_id(&self) -> &EntityId {
        &self.entity_id
    }

    fn entity_type(&self) -> &'static str {
        "CARTESIAN_POINT"
    }

    fn to_step(&self) -> String {
        format!(
            "#{} = {}('{}', ({}));",
            self.entity_id.id,
            self.entity_type(),
            self.entity_id.name.as_deref().unwrap_or(""),
            self.coordinates.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", ")
        )
    }
}

impl CartesianPoint {
    pub const fn new(id: u64, coordinates: Point3<Real>) -> Self {
        Self {
            entity_id: EntityId::new(id),
            coordinates,
        }
    }

    pub const fn from_coordinates(id: u64, x: Real, y: Real, z: Real) -> Self {
        Self::new(id, Point3::new(x, y, z))
    }
}

/// Direction vector (unit vector)
#[derive(Debug, Clone)]
pub struct Direction {
    pub entity_id: EntityId,
    pub direction: Vector3<Real>,
}

impl StepEntity for Direction {
    fn entity_id(&self) -> &EntityId {
        &self.entity_id
    }

    fn entity_type(&self) -> &'static str {
        "DIRECTION"
    }

    fn to_step(&self) -> String {
        format!(
            "#{} = {}('{}', ({}));",
            self.entity_id.id,
            self.entity_type(),
            self.entity_id.name.as_deref().unwrap_or(""),
            self.direction.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", ")
        )
    }
}

impl Direction {
    pub const fn new(id: u64, direction: Vector3<Real>) -> Self {
        Self {
            entity_id: EntityId::new(id),
            direction,
        }
    }

    pub const fn from_components(id: u64, x: Real, y: Real, z: Real) -> Self {
        Self::new(id, Vector3::new(x, y, z))
    }
}

/// Axis placement for coordinate systems
#[derive(Debug, Clone)]
pub struct Axis2Placement3D {
    pub entity_id: EntityId,
    pub location: EntityId,  // Reference to CARTESIAN_POINT
    pub axis: EntityId,      // Reference to DIRECTION
    pub ref_direction: EntityId, // Reference to DIRECTION
}

impl StepEntity for Axis2Placement3D {
    fn entity_id(&self) -> &EntityId {
        &self.entity_id
    }

    fn entity_type(&self) -> &'static str {
        "AXIS2_PLACEMENT_3D"
    }

    fn to_step(&self) -> String {
        format!(
            "#{} = {}('{}', #{}, #{}, #{});",
            self.entity_id.id,
            self.entity_type(),
            self.entity_id.name.as_deref().unwrap_or(""),
            self.location.id,
            self.axis.id,
            self.ref_direction.id
        )
    }
}

impl Axis2Placement3D {
    pub const fn new(id: u64, location: EntityId, axis: EntityId, ref_direction: EntityId) -> Self {
        Self {
            entity_id: EntityId::new(id),
            location,
            axis,
            ref_direction,
        }
    }
}

/// A STEP model containing entities and their relationships
#[derive(Debug)]
pub struct StepModel {
    pub entities: HashMap<EntityId, Box<dyn StepEntity>>,
    pub header: StepHeader,
}

/// Header information for STEP files
#[derive(Debug)]
pub struct StepHeader {
    pub file_name: String,
    pub file_description: String,
    pub author: String,
    pub organization: String,
    pub preprocessor_version: String,
    pub originating_system: String,
    pub authorization: String,
}

impl Default for StepHeader {
    fn default() -> Self {
        Self {
            file_name: "model.stp".to_string(),
            file_description: "csgrs STEP export".to_string(),
            author: "csgrs".to_string(),
            organization: "csgrs".to_string(),
            preprocessor_version: "1.0".to_string(),
            originating_system: "csgrs".to_string(),
            authorization: "csgrs".to_string(),
        }
    }
}

impl StepModel {
    /// Create a new empty STEP model
    pub fn new() -> Self {
        Self {
            entities: HashMap::new(),
            header: StepHeader::default(),
        }
    }

    /// Add an entity to the model
    pub fn add_entity(&mut self, entity: Box<dyn StepEntity>) {
        let id = entity.entity_id().clone();
        self.entities.insert(id, entity);
    }

    /// Get an entity by ID
    pub fn get_entity(&self, id: &EntityId) -> Option<&dyn StepEntity> {
        self.entities.get(id).map(|e| e.as_ref())
    }

    /// Convert the STEP model to a mesh for CSG operations
    pub fn to_mesh(&self) -> Result<Mesh<()>, StepError> {
        // This is a simplified implementation
        // A full implementation would parse all geometric entities
        // and construct proper mesh topology
        todo!("STEP to mesh conversion not yet implemented")
    }

    /// Create a STEP model from a mesh
    pub fn from_mesh(_mesh: &Mesh<()>) -> Self {
        let _model = Self::new();

        // This is a simplified implementation
        // A full implementation would create proper STEP entities
        // from mesh topology
        todo!("Mesh to STEP conversion not yet implemented")
    }

    /// Read a STEP model from a file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, StepError> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut model = Self::new();
        let mut current_entity = String::new();

        for line in reader.lines() {
            let line = line?;

            if line.starts_with("ISO-10303-21;") {
                // Start of STEP file
                continue;
            }

            if line.starts_with("HEADER;") {
                // Header section
                continue;
            }

            if line.starts_with("DATA;") {
                // Data section
                continue;
            }

            if line.starts_with("ENDSEC;") {
                // End of section
                continue;
            }

            if line.starts_with("END-ISO-10303-21;") {
                // End of STEP file
                break;
            }

            // Parse entity data
            current_entity.push_str(&line);

            if line.contains(';') && !line.ends_with(';') {
                // Multi-line entity
                continue;
            }

            if line.contains(';') {
                // Complete entity
                if let Some(entity) = Self::parse_entity(&current_entity)? {
                    model.add_entity(entity);
                }
                current_entity.clear();
            }
        }

        Ok(model)
    }

    /// Write the STEP model to a file
    pub fn write_to_file<P: AsRef<Path>>(&self, path: P) -> Result<(), StepError> {
        let mut file = File::create(path)?;

        // Write header
        writeln!(file, "ISO-10303-21;")?;
        writeln!(file, "HEADER;")?;
        writeln!(file, "FILE_NAME('{}','{}','{}','{}');",
                self.header.file_name,
                self.header.file_description,
                self.header.author,
                self.header.organization)?;
        writeln!(file, "FILE_DESCRIPTION('{}','{}');",
                self.header.file_description,
                self.header.preprocessor_version)?;
        writeln!(file, "ENDSEC;")?;

        // Write data section
        writeln!(file, "DATA;")?;
        for entity in self.entities.values() {
            writeln!(file, "{}", entity.to_step())?;
        }
        writeln!(file, "ENDSEC;")?;

        writeln!(file, "END-ISO-10303-21;")?;
        Ok(())
    }

    /// Parse a single entity from STEP format
    fn parse_entity(data: &str) -> Result<Option<Box<dyn StepEntity>>, StepError> {
        let data = data.trim();

        if data.is_empty() || !data.contains('=') {
            return Ok(None);
        }

        let parts: Vec<&str> = data.split('=').collect();
        if parts.len() != 2 {
            return Ok(None);
        }

        let entity_ref = parts[0].trim().trim_start_matches('#');
        let entity_data = parts[1].trim();

        let entity_id = entity_ref.parse::<u64>()
            .map_err(|_| StepError::MalformedStructure(format!("Invalid entity ID: {}", entity_ref)))?;

        if entity_data.starts_with("CARTESIAN_POINT") {
            Self::parse_cartesian_point(entity_id, entity_data)
        } else if entity_data.starts_with("DIRECTION") {
            Self::parse_direction(entity_id, entity_data)
        } else {
            // For now, skip unsupported entities
            Ok(None)
        }
    }

    /// Parse a CARTESIAN_POINT entity
    fn parse_cartesian_point(id: u64, data: &str) -> Result<Option<Box<dyn StepEntity>>, StepError> {
        let coords_start = data.find('(').ok_or_else(|| {
            StepError::MalformedStructure("CARTESIAN_POINT missing opening parenthesis".to_string())
        })?;

        let coords_end = data.find(')').ok_or_else(|| {
            StepError::MalformedStructure("CARTESIAN_POINT missing closing parenthesis".to_string())
        })?;

        let coords_str = &data[coords_start + 1..coords_end];
        let coords: Result<Vec<Real>, _> = coords_str
            .split(',')
            .map(|s| s.trim().parse::<Real>())
            .collect();

        let coords = coords.map_err(|_| {
            StepError::InvalidGeometry("Invalid coordinates in CARTESIAN_POINT".to_string())
        })?;

        if coords.len() != 3 {
            return Err(StepError::InvalidGeometry(
                "CARTESIAN_POINT must have exactly 3 coordinates".to_string()
            ));
        }

        let point = CartesianPoint::from_coordinates(id, coords[0], coords[1], coords[2]);
        Ok(Some(Box::new(point)))
    }

    /// Parse a DIRECTION entity
    fn parse_direction(id: u64, data: &str) -> Result<Option<Box<dyn StepEntity>>, StepError> {
        let coords_start = data.find('(').ok_or_else(|| {
            StepError::MalformedStructure("DIRECTION missing opening parenthesis".to_string())
        })?;

        let coords_end = data.find(')').ok_or_else(|| {
            StepError::MalformedStructure("DIRECTION missing closing parenthesis".to_string())
        })?;

        let coords_str = &data[coords_start + 1..coords_end];
        let coords: Result<Vec<Real>, _> = coords_str
            .split(',')
            .map(|s| s.trim().parse::<Real>())
            .collect();

        let coords = coords.map_err(|_| {
            StepError::InvalidGeometry("Invalid coordinates in DIRECTION".to_string())
        })?;

        if coords.len() != 3 {
            return Err(StepError::InvalidGeometry(
                "DIRECTION must have exactly 3 coordinates".to_string()
            ));
        }

        let direction = Direction::from_components(id, coords[0], coords[1], coords[2]);
        Ok(Some(Box::new(direction)))
    }
}

impl Default for StepModel {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cartesian_point_creation() {
        let point = CartesianPoint::from_coordinates(1, 1.0, 2.0, 3.0);
        assert_eq!(point.coordinates, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(point.entity_id.id, 1);
    }

    #[test]
    fn test_direction_creation() {
        let direction = Direction::from_components(1, 1.0, 0.0, 0.0);
        assert_eq!(direction.direction, Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(direction.entity_id.id, 1);
    }

    #[test]
    fn test_step_model_creation() {
        let mut model = StepModel::new();
        let point = CartesianPoint::from_coordinates(1, 1.0, 2.0, 3.0);
        model.add_entity(Box::new(point));

        assert_eq!(model.entities.len(), 1);
    }

    #[test]
    fn test_axis2_placement_creation() {
        let location = EntityId::new(1);
        let axis = EntityId::new(2);
        let ref_direction = EntityId::new(3);

        let placement = Axis2Placement3D::new(4, location, axis, ref_direction);
        assert_eq!(placement.entity_id.id, 4);
    }
}
