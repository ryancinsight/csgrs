use crate::float_types::Real;
use nalgebra::Point3;
use std::fmt;

/// All the possible validation issues we might encounter,
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationError {
    /// (RepeatedPoint) Two consecutive coords are identical
    RepeatedPoint(Point3<Real>),
    /// (HoleOutsideShell) A hole is *not* contained by its outer shell
    HoleOutsideShell(Point3<Real>),
    /// (NestedHoles) A hole is nested inside another hole
    NestedHoles(Point3<Real>),
    /// (DisconnectedInterior) The interior is disconnected
    DisconnectedInterior(Point3<Real>),
    /// (SelfIntersection) A polygon self‐intersects
    SelfIntersection(Point3<Real>),
    /// (RingSelfIntersection) A linear ring has a self‐intersection
    RingSelfIntersection(Point3<Real>),
    /// (NestedShells) Two outer shells are nested incorrectly
    NestedShells(Point3<Real>),
    /// (TooFewPoints) A ring or line has fewer than the minimal #points
    TooFewPoints(Point3<Real>),
    /// (InvalidCoordinate) The coordinate has a NaN or infinite
    InvalidCoordinate(Point3<Real>),
    /// (RingNotClosed) The ring's first/last points differ
    RingNotClosed(Point3<Real>),
    /// (MismatchedVertices) operation requires polygons with same number of vertices
    MismatchedVertices,
    /// (IndexOutOfRange) operation requires polygons with same number of vertices
    IndexOutOfRange,
    /// (InvalidArguments) operation requires polygons with same number of vertices
    InvalidArguments,
    /// Invalid dimension for shape construction (negative, zero, or infinite)
    InvalidDimension(String, Real),
    /// Invalid parameter for shape construction (wrong range, type, etc.)
    InvalidShapeParameter(String, String),
    /// In general, anything else
    Other(String, Option<Point3<Real>>),
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValidationError::RepeatedPoint(p) => write!(f, "Repeated point at {:?}", p),
            ValidationError::HoleOutsideShell(p) => write!(f, "Hole outside shell at {:?}", p),
            ValidationError::NestedHoles(p) => write!(f, "Nested holes at {:?}", p),
            ValidationError::DisconnectedInterior(p) => {
                write!(f, "Disconnected interior at {:?}", p)
            },
            ValidationError::SelfIntersection(p) => write!(f, "Self-intersection at {:?}", p),
            ValidationError::RingSelfIntersection(p) => {
                write!(f, "Ring self-intersection at {:?}", p)
            },
            ValidationError::NestedShells(p) => write!(f, "Nested shells at {:?}", p),
            ValidationError::TooFewPoints(p) => write!(f, "Too few points at {:?}", p),
            ValidationError::InvalidCoordinate(p) => {
                write!(f, "Invalid coordinate at {:?}", p)
            },
            ValidationError::RingNotClosed(p) => write!(f, "Ring not closed at {:?}", p),
            ValidationError::MismatchedVertices => write!(f, "Mismatched vertices"),
            ValidationError::IndexOutOfRange => write!(f, "Index out of range"),
            ValidationError::InvalidArguments => write!(f, "Invalid arguments"),
            ValidationError::InvalidDimension(param, value) => {
                write!(
                    f,
                    "Invalid {} dimension: {} (must be positive and finite)",
                    param, value
                )
            },
            ValidationError::InvalidShapeParameter(param, reason) => {
                write!(f, "Invalid {} parameter: {}", param, reason)
            },
            ValidationError::Other(msg, p) => {
                if let Some(point) = p {
                    write!(f, "{} at {:?}", msg, point)
                } else {
                    write!(f, "{}", msg)
                }
            },
        }
    }
}

impl std::error::Error for ValidationError {}

// Plane::from_points "Degenerate polygon: vertices do not define a plane"
// Mesh::polyhedron "Face index {} is out of range (points.len = {})."
// Sketch::rotate_extrude "rotate_extrude requires at least 2 segments"
// Sketch::extrude_between "extrude_between: both polygons must have the same number of vertices"
