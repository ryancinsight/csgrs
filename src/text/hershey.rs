//! Create `CSG`s using Hershey fonts

use crate::core::float_types::Real;
use crate::csg::CSG;
use hershey::{Font, Glyph as HersheyGlyph, Vector as HersheyVector};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Creates **2D line-stroke text** in the XY plane using a Hershey font.
    ///
    /// Each glyph’s strokes become one or more `LineString<Real>` entries in `geometry`.
    /// If you need them filled or thickened, you can later offset or extrude these lines.
    ///
    /// # Parameters
    /// - `text`: The text to render
    /// - `font`: The Hershey font (e.g., `hershey::fonts::GOTHIC_ENG_SANS`)
    /// - `size`: Scale factor for glyphs
    /// - `metadata`: Optional user data to store in the resulting CSG
    ///
    /// # Returns
    /// A new `CSG` where each glyph stroke is a `Geometry::LineString` in `geometry`.
    pub fn from_hershey(text: &str, font: &Font, size: Real, metadata: Option<S>) -> CSG<S> {
        use geo::{Geometry, GeometryCollection};

        let mut all_strokes = Vec::new();
        let mut cursor_x: Real = 0.0;

        // Process characters using iterator patterns with parallel processing for large text
        #[cfg(feature = "parallel")]
        let character_strokes: Vec<(Vec<LineString<Real>>, Real)> = {
            if text.len() > 50 {
                use rayon::prelude::*;

                text.chars()
                    .filter(|ch| !ch.is_control())
                    .collect::<Vec<_>>()
                    .par_iter()
                    .enumerate()
                    .map(|(i, &ch)| {
                        let char_cursor_x = (i as Real) * size * 6.0; // Approximate spacing
                        process_hershey_character(&font, ch, size, char_cursor_x)
                    })
                    .collect()
            } else {
                text.chars()
                    .filter(|ch| !ch.is_control())
                    .enumerate()
                    .map(|(i, ch)| {
                        let char_cursor_x = (i as Real) * size * 6.0; // Approximate spacing
                        process_hershey_character(&font, ch, size, char_cursor_x)
                    })
                    .collect()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let character_strokes: Vec<(Vec<LineString<Real>>, Real)> = text.chars()
            .filter(|ch| !ch.is_control())
            .enumerate()
            .map(|(i, ch)| {
                let char_cursor_x = (i as Real) * size * 6.0; // Approximate spacing
                process_hershey_character(&font, ch, size, char_cursor_x)
            })
            .collect();

        // Accumulate strokes and calculate final cursor position using fold()
        let (final_cursor_x, _) = character_strokes
            .into_iter()
            .fold((0.0, &mut all_strokes), |(mut cursor_x, strokes), (char_strokes, advance)| {
                strokes.extend(char_strokes);
                cursor_x += advance;
                (cursor_x, strokes)
            });

        // Insert each stroke using iterator patterns
        let geo_coll = GeometryCollection(
            all_strokes
                .into_iter()
                .map(|line_str| Geometry::LineString(line_str))
                .collect()
        );

        // Return a new CSG that has no 3D polygons, but has these lines in geometry.
        CSG {
            polygons: Vec::new(),
            geometry: geo_coll,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }
}

/// Helper for building open polygons from a single Hershey `Glyph`.
fn build_hershey_glyph_lines(
    glyph: &HersheyGlyph,
    scale: Real,
    offset_x: Real,
    offset_y: Real,
) -> Vec<geo::LineString<Real>> {
    use geo::{LineString, coord};

    let mut strokes = Vec::new();

    // We'll accumulate each stroke’s points in `current_coords`,
    // resetting whenever Hershey issues a "MoveTo"
    let mut current_coords = Vec::new();

    // Process vector commands using iterator patterns with fold()
    let (final_strokes, final_coords) = glyph.vectors
        .iter()
        .fold((strokes, current_coords), |(mut strokes, mut current_coords), vector_cmd| {
            match vector_cmd {
                HersheyVector::MoveTo { x, y } => {
                    // If we already had 2+ points, that stroke is complete:
                    if current_coords.len() >= 2 {
                        strokes.push(LineString::from(current_coords));
                    }
                    // Start a new stroke
                    current_coords = Vec::new();
                    let px = offset_x + (*x as Real) * scale;
                    let py = offset_y + (*y as Real) * scale;
                    current_coords.push(coord! { x: px, y: py });
                },
                HersheyVector::LineTo { x, y } => {
                    let px = offset_x + (*x as Real) * scale;
                    let py = offset_y + (*y as Real) * scale;
                    current_coords.push(coord! { x: px, y: py });
                },
            }
            (strokes, current_coords)
        });

    strokes = final_strokes;
    current_coords = final_coords;

    // End-of-glyph: if our final stroke has 2+ points, convert to a line string
    if current_coords.len() >= 2 {
        strokes.push(LineString::from(current_coords));
    }

    strokes
}

/// **Helper function to process individual Hershey characters**
///
/// Processes a single character using the Hershey font, returning the strokes
/// and the advance width for cursor positioning.
fn process_hershey_character(
    font: &HersheyFont,
    ch: char,
    size: Real,
    cursor_x: Real,
) -> (Vec<LineString<Real>>, Real) {
    match font.glyph(ch) {
        Ok(glyph) => {
            // Convert the Hershey lines to geo::LineString objects
            let glyph_width = (glyph.max_x - glyph.min_x) as Real;
            let strokes = build_hershey_glyph_lines(&glyph, size, cursor_x, 0.0);
            let advance = glyph_width * size * 0.8;
            (strokes, advance)
        },
        Err(_) => {
            // Missing glyph => return empty strokes with default advance
            let advance = 6.0 * size;
            (Vec::new(), advance)
        },
    }
}
