#[cfg(feature = "svg-io")]
pub mod svg;

#[cfg(feature = "stl-io")]
mod stl;

#[cfg(feature = "stl-io")]
pub use stl::indexed_mesh_stl;

#[cfg(feature = "dxf-io")]
mod dxf;

#[cfg(feature = "obj-io")]
mod obj;

#[cfg(feature = "obj-io")]
pub use obj::indexed_mesh_obj;

#[cfg(feature = "ply-io")]
mod ply;

#[cfg(feature = "ply-io")]
pub use ply::indexed_mesh_ply;

#[cfg(feature = "amf-io")]
mod amf;

#[cfg(feature = "step-io")]
mod step;

#[cfg(feature = "step-io")]
pub use step::{StepModel, StepError, StepEntity, CartesianPoint, Direction, Axis2Placement3D, EntityId};

// #[cfg(feature = "gltf")]
// mod gltf;

/// Generic I/O and format‑conversion errors.
///
/// Many I/O features are behind cargo feature‑flags.  
/// When a feature is disabled the corresponding variant is *not*
/// constructed in user code.
#[derive(Debug)]
pub enum IoError {
    StdIo(std::io::Error),
    ParseFloat(std::num::ParseFloatError),

    MalformedInput(String),
    MalformedPath(String),
    Unimplemented(String),

    #[cfg(feature = "svg-io")]
    /// Error bubbled up from the `svg` crate during parsing.
    SvgParsing(::svg::parser::Error),

    #[cfg(feature = "obj-io")]
    /// Error during OBJ file processing.
    ObjParsing(String),

    #[cfg(feature = "ply-io")]
    /// Error during PLY file processing.
    PlyParsing(String),

    #[cfg(feature = "amf-io")]
    /// Error during AMF file processing.
    AmfParsing(String),

    #[cfg(feature = "step-io")]
    /// Error during STEP file processing.
    StepParsing(String),

    #[cfg(feature = "gltf")]
    /// Error during glTF file processing.
    GltfParsing(String),
}

impl std::fmt::Display for IoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use IoError::*;

        match self {
            StdIo(error) => write!(f, "std::io::Error: {error}"),
            ParseFloat(error) => write!(f, "Could not parse float: {error}"),

            MalformedInput(msg) => write!(f, "Input is malformed: {msg}"),
            MalformedPath(msg) => write!(f, "The path is malformed: {msg}"),
            Unimplemented(msg) => write!(f, "Feature is not implemented: {msg}"),

            #[cfg(feature = "svg-io")]
            SvgParsing(error) => write!(f, "SVG Parsing error: {error}"),

            #[cfg(feature = "obj-io")]
            ObjParsing(error) => write!(f, "OBJ Parsing error: {error}"),

            #[cfg(feature = "ply-io")]
            PlyParsing(error) => write!(f, "PLY Parsing error: {error}"),

            #[cfg(feature = "amf-io")]
            AmfParsing(error) => write!(f, "AMF Parsing error: {error}"),

            #[cfg(feature = "step-io")]
            StepParsing(error) => write!(f, "STEP Parsing error: {error}"),

            #[cfg(feature = "gltf")]
            GltfParsing(error) => write!(f, "glTF Parsing error: {error}"),
        }
    }
}

impl std::error::Error for IoError {}

impl From<std::io::Error> for IoError {
    fn from(value: std::io::Error) -> Self {
        Self::StdIo(value)
    }
}

impl From<std::num::ParseFloatError> for IoError {
    fn from(value: std::num::ParseFloatError) -> Self {
        Self::ParseFloat(value)
    }
}

#[cfg(feature = "svg-io")]
impl From<::svg::parser::Error> for IoError {
    fn from(value: ::svg::parser::Error) -> Self {
        Self::SvgParsing(value)
    }
}

// Generic From<String> implementation removed to avoid conflicts
// Use specific error types instead: IoError::ObjParsing(String) or IoError::StepParsing(String)
