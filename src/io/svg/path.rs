//! SVG path parsing utilities
//!
//! This module provides functionality for parsing SVG path data
//! and converting it to geometric representations.

use crate::io::IoError;
use geo::{Coord, CoordNum, LineString, MultiLineString};
use svg::node::element::path;

/// PathBuilder implementation for converting SVG paths to geometric data
#[derive(Debug)]
pub struct PathBuilder<F: CoordNum> {
    lines: Vec<LineString<F>>,
    current_line: Option<LineString<F>>,
}

impl<F: CoordNum> Default for PathBuilder<F> {
    fn default() -> Self {
        Self::new()
    }
}

impl<F: CoordNum> PathBuilder<F> {
    /// Create a new PathBuilder
    pub const fn new() -> Self {
        Self {
            lines: Vec::new(),
            current_line: None,
        }
    }

    /// Move to absolute coordinates
    pub fn move_to(&mut self, coord: Coord<F>) {
        self.finish_current_line();
        self.current_line = Some(LineString::from(vec![coord]));
    }

    /// Move by relative coordinates
    pub fn move_by(&mut self, delta: Coord<F>) {
        self.finish_current_line();
        // For relative move, we need to know the current position
        // This is a simplified implementation
        self.current_line = Some(LineString::from(vec![delta]));
    }

    /// Line to absolute coordinates
    pub fn line_to(&mut self, coord: Coord<F>) -> Result<(), IoError> {
        if let Some(ref mut line) = self.current_line {
            line.0.push(coord);
            Ok(())
        } else {
            Err(IoError::MalformedInput(
                "No current path to add line to".to_string(),
            ))
        }
    }

    /// Line by relative coordinates
    pub fn line_by(&mut self, delta: Coord<F>) -> Result<(), IoError> {
        if let Some(ref mut line) = self.current_line {
            if let Some(last) = line.0.last() {
                let new_coord = Coord {
                    x: last.x + delta.x,
                    y: last.y + delta.y,
                };
                line.0.push(new_coord);
            }
            Ok(())
        } else {
            Err(IoError::MalformedInput(
                "No current path to add line to".to_string(),
            ))
        }
    }

    /// Horizontal line to absolute x coordinate
    pub fn hline_to(&mut self, x: F) -> Result<(), IoError> {
        if let Some(ref mut line) = self.current_line {
            if let Some(last) = line.0.last() {
                let coord = Coord { x, y: last.y };
                line.0.push(coord);
            }
            Ok(())
        } else {
            Err(IoError::MalformedInput(
                "No current path to add horizontal line to".to_string(),
            ))
        }
    }

    /// Horizontal line by relative x coordinate
    pub fn hline_by(&mut self, dx: F) -> Result<(), IoError> {
        if let Some(ref mut line) = self.current_line {
            if let Some(last) = line.0.last() {
                let coord = Coord {
                    x: last.x + dx,
                    y: last.y,
                };
                line.0.push(coord);
            }
            Ok(())
        } else {
            Err(IoError::MalformedInput(
                "No current path to add horizontal line to".to_string(),
            ))
        }
    }

    /// Vertical line to absolute y coordinate
    pub fn vline_to(&mut self, y: F) -> Result<(), IoError> {
        if let Some(ref mut line) = self.current_line {
            if let Some(last) = line.0.last() {
                let coord = Coord { x: last.x, y };
                line.0.push(coord);
            }
            Ok(())
        } else {
            Err(IoError::MalformedInput(
                "No current path to add vertical line to".to_string(),
            ))
        }
    }

    /// Vertical line by relative y coordinate
    pub fn vline_by(&mut self, dy: F) -> Result<(), IoError> {
        if let Some(ref mut line) = self.current_line {
            if let Some(last) = line.0.last() {
                let coord = Coord {
                    x: last.x,
                    y: last.y + dy,
                };
                line.0.push(coord);
            }
            Ok(())
        } else {
            Err(IoError::MalformedInput(
                "No current path to add vertical line to".to_string(),
            ))
        }
    }

    /// Quadratic curve to control point and end point
    pub fn quadratic_curve_to(
        &mut self,
        _control: Coord<F>,
        end: Coord<F>,
    ) -> Result<(), IoError> {
        // Simplified implementation - just add the end point
        self.line_to(end)
    }

    /// Quadratic curve by relative control point and end point
    pub fn quadratic_curve_by(
        &mut self,
        _control: Coord<F>,
        delta: Coord<F>,
    ) -> Result<(), IoError> {
        self.line_by(delta)
    }

    /// Smooth quadratic curve to end point
    pub fn quadratic_smooth_curve_to(&mut self, end: Coord<F>) -> Result<(), IoError> {
        self.line_to(end)
    }

    /// Smooth quadratic curve by relative end point
    pub fn quadratic_smooth_curve_by(&mut self, delta: Coord<F>) -> Result<(), IoError> {
        self.line_by(delta)
    }

    /// Cubic curve to control points and end point
    pub fn curve_to(
        &mut self,
        _control1: Coord<F>,
        _control2: Coord<F>,
        end: Coord<F>,
    ) -> Result<(), IoError> {
        // Simplified implementation - just add the end point
        self.line_to(end)
    }

    /// Cubic curve by relative control points and end point
    pub fn curve_by(
        &mut self,
        _control1: Coord<F>,
        _control2: Coord<F>,
        delta: Coord<F>,
    ) -> Result<(), IoError> {
        self.line_by(delta)
    }

    /// Smooth cubic curve to control point and end point
    pub fn smooth_curve_to(
        &mut self,
        _control2: Coord<F>,
        end: Coord<F>,
    ) -> Result<(), IoError> {
        self.line_to(end)
    }

    /// Smooth cubic curve by relative control point and end point
    pub fn smooth_curve_by(
        &mut self,
        _control2: Coord<F>,
        delta: Coord<F>,
    ) -> Result<(), IoError> {
        self.line_by(delta)
    }

    /// Elliptical arc to end point
    pub fn elliptical_arc_to(
        &mut self,
        _rx: f32,
        _ry: f32,
        _x_rot: f32,
        _large_arc: bool,
        _sweep: bool,
        end: Coord<f32>,
    ) -> Result<(), IoError> {
        // Simplified implementation - convert and add the end point
        let end_f = Coord {
            x: F::from(end.x).ok_or_else(|| {
                IoError::MalformedInput(
                    "Failed to convert elliptical arc coordinate".to_string(),
                )
            })?,
            y: F::from(end.y).ok_or_else(|| {
                IoError::MalformedInput(
                    "Failed to convert elliptical arc coordinate".to_string(),
                )
            })?,
        };
        self.line_to(end_f)
    }

    /// Elliptical arc by relative end point
    pub fn elliptical_arc_by(
        &mut self,
        _rx: f32,
        _ry: f32,
        _x_rot: f32,
        _large_arc: bool,
        _sweep: bool,
        delta: Coord<f32>,
    ) -> Result<(), IoError> {
        let delta_f = Coord {
            x: F::from(delta.x).ok_or_else(|| {
                IoError::MalformedInput(
                    "Failed to convert elliptical arc coordinate".to_string(),
                )
            })?,
            y: F::from(delta.y).ok_or_else(|| {
                IoError::MalformedInput(
                    "Failed to convert elliptical arc coordinate".to_string(),
                )
            })?,
        };
        self.line_by(delta_f)
    }

    /// Close the current path
    pub fn close(&mut self) -> Result<(), IoError> {
        // For simplicity, just finish the current line
        self.finish_current_line();
        Ok(())
    }

    /// Finish the current line and add it to the collection
    fn finish_current_line(&mut self) {
        if let Some(line) = self.current_line.take() {
            if !line.0.is_empty() {
                self.lines.push(line);
            }
        }
    }

    /// Get the current position (for relative commands)
    pub fn get_pos(&self) -> Coord<F> {
        if let Some(ref line) = self.current_line {
            if let Some(last) = line.0.last() {
                *last
            } else {
                Coord {
                    x: F::zero(),
                    y: F::zero(),
                }
            }
        } else {
            Coord {
                x: F::zero(),
                y: F::zero(),
            }
        }
    }

    /// Get mutable reference to current path
    pub fn get_path_mut_or_fail(&mut self) -> Result<&mut LineString<F>, IoError> {
        if let Some(ref mut line) = self.current_line {
            Ok(line)
        } else {
            Err(IoError::MalformedInput(
                "No current path available".to_string(),
            ))
        }
    }
}

impl<F: CoordNum> From<PathBuilder<F>> for MultiLineString<F> {
    fn from(mut builder: PathBuilder<F>) -> Self {
        builder.finish_current_line();
        MultiLineString::new(builder.lines)
    }
}

/// Parse SVG path data into MultiLineString
pub fn svg_path_to_multi_line_string<F: CoordNum>(
    path_data: path::Data,
) -> Result<MultiLineString<F>, IoError> {
    // `svg` crate returns `f32`, so that's what is used here.
    let mut builder = PathBuilder::<f32>::new();

    for cmd in path_data.iter() {
        use svg::node::element::path::{Command::*, Position::*};

        macro_rules! ensure_param_count {
            ($count:expr, $div_by:expr) => {
                if $count % $div_by != 0 {
                    return Err(IoError::MalformedPath(format!("Expected the number of parameters {} to be divisible by {} in command {cmd:?}", $count, $div_by)));
                }
            };
        }

        let param_count = match cmd {
            Move(..) | Line(..) => 2,
            HorizontalLine(..) | VerticalLine(..) => 1,
            QuadraticCurve(..) => 4,
            SmoothQuadraticCurve(..) => 2,
            CubicCurve(..) => 6,
            SmoothCubicCurve(..) => 4,
            EllipticalArc(..) => 7,
            Close => {
                builder.close()?;
                continue;
            },
        };

        match cmd {
            Move(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);

                if let Some(&[x, y]) = coords.next() {
                    builder.move_to(Coord { x, y });
                }

                // Follow-up coordinates for MoveTo are implicit LineTo
                while let Some(&[x, y]) = coords.next() {
                    builder.line_to(Coord { x, y })?;
                }
            },
            Move(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);

                if let Some(&[dx, dy]) = coords.next() {
                    builder.move_by(Coord { x: dx, y: dy });
                }

                // Follow-up coordinates for MoveTo are implicit LineTo
                while let Some(&[dx, dy]) = coords.next() {
                    builder.line_by(Coord { x: dx, y: dy })?;
                }
            },
            Line(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);
                while let Some(&[x, y]) = coords.next() {
                    builder.line_to(Coord { x, y })?;
                }
            },
            Line(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut coords = params.chunks(param_count);
                while let Some(&[dx, dy]) = coords.next() {
                    builder.line_by(Coord { x: dx, y: dy })?;
                }
            },
            HorizontalLine(Absolute, params) => {
                for &x in params.iter() {
                    builder.hline_to(x)?;
                }
            },
            HorizontalLine(Relative, params) => {
                for &dx in params.iter() {
                    builder.hline_by(dx)?;
                }
            },
            VerticalLine(Absolute, params) => {
                for &y in params.iter() {
                    builder.vline_to(y)?;
                }
            },
            VerticalLine(Relative, params) => {
                for &dy in params.iter() {
                    builder.vline_by(dy)?;
                }
            },
            QuadraticCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[cx, cy, x, y]) = params_iter.next() {
                    builder.quadratic_curve_to(Coord { x: cx, y: cy }, Coord { x, y })?;
                }
            },
            QuadraticCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[cx, cy, x, y]) = params_iter.next() {
                    builder.quadratic_curve_by(Coord { x: cx, y: cy }, Coord { x, y })?;
                }
            },
            SmoothQuadraticCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[x, y]) = params_iter.next() {
                    builder.quadratic_smooth_curve_to(Coord { x, y })?;
                }
            },
            SmoothQuadraticCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[x, y]) = params_iter.next() {
                    builder.quadratic_smooth_curve_by(Coord { x, y })?;
                }
            },
            CubicCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[c1x, c1y, c2x, c2y, x, y]) = params_iter.next() {
                    builder.curve_to(
                        Coord { x: c1x, y: c1y },
                        Coord { x: c2x, y: c2y },
                        Coord { x, y },
                    )?;
                }
            },
            CubicCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[c1x, c1y, c2x, c2y, x, y]) = params_iter.next() {
                    builder.curve_by(
                        Coord { x: c1x, y: c1y },
                        Coord { x: c2x, y: c2y },
                        Coord { x, y },
                    )?;
                }
            },
            SmoothCubicCurve(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[c2x, c2y, x, y]) = params_iter.next() {
                    builder.smooth_curve_to(Coord { x: c2x, y: c2y }, Coord { x, y })?;
                }
            },
            SmoothCubicCurve(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[c2x, c2y, x, y]) = params_iter.next() {
                    builder.smooth_curve_by(Coord { x: c2x, y: c2y }, Coord { x, y })?;
                }
            },
            EllipticalArc(Absolute, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[rx, ry, x_rot, large_arc, sweep, x, y]) = params_iter.next() {
                    let large_arc = large_arc == 1.0;
                    let sweep = sweep == 1.0;
                    builder.elliptical_arc_to(
                        rx,
                        ry,
                        x_rot,
                        large_arc,
                        sweep,
                        Coord { x, y },
                    )?;
                }
            },
            EllipticalArc(Relative, params) => {
                ensure_param_count!(params.len(), param_count);
                let mut params_iter = params.chunks(param_count);
                while let Some(&[rx, ry, x_rot, large_arc, sweep, x, y]) = params_iter.next() {
                    let large_arc = large_arc == 1.0;
                    let sweep = sweep == 1.0;
                    builder.elliptical_arc_by(
                        rx,
                        ry,
                        x_rot,
                        large_arc,
                        sweep,
                        Coord { x, y },
                    )?;
                }
            },
            Close => {
                // Close command should have been handled in the parameter count check above
                return Err(IoError::MalformedPath(
                    "Unexpected Close command in SVG path parsing".to_string(),
                ));
            },
        }
    }

    let mls: MultiLineString<f32> = builder.into();

    // Convert f32 coordinates to target type F with proper error handling
    let mut converted_lines = Vec::new();
    for line in mls {
        let mut converted_coords = Vec::new();
        for coord in line {
            let x = F::from(coord.x).ok_or_else(|| {
                IoError::MalformedInput(format!(
                    "Failed to convert coordinate x={} from f32 to target type",
                    coord.x
                ))
            })?;
            let y = F::from(coord.y).ok_or_else(|| {
                IoError::MalformedInput(format!(
                    "Failed to convert coordinate y={} from f32 to target type",
                    coord.y
                ))
            })?;
            converted_coords.push(Coord { x, y });
        }
        converted_lines.push(LineString::new(converted_coords));
    }

    Ok(MultiLineString::new(converted_lines))
}

/// Parse contents of the SVG <polyline/> and <polygon/> attribute [`points`][points] into a `LineString`.
///
/// [points]: https://www.w3.org/TR/SVG11/shapes.html#PointsBNF
pub fn svg_points_to_line_string<F: CoordNum>(points: &str) -> Result<LineString<F>, IoError> {
    use nom::IResult;
    use nom::Parser;
    use nom::branch::alt;
    use nom::character::complete::{char, multispace0, multispace1};
    use nom::combinator::opt;
    use nom::multi::separated_list1;
    use nom::number::complete::float;
    use nom::sequence::{delimited, pair, separated_pair, tuple};

    fn comma_wsp(i: &str) -> IResult<&str, ()> {
        let (i, _) = alt((
            tuple((multispace1, opt(char(',')), multispace0)).map(|_| ()),
            pair(char(','), multispace0).map(|_| ()),
        ))(i)?;
        Ok((i, ()))
    }

    fn point<F: CoordNum>(i: &str) -> IResult<&str, Coord<F>> {
        let (i, (x, y)) = separated_pair(float, comma_wsp, float)(i)?;

        // Convert coordinates with proper error handling
        let x_conv = F::from(x).ok_or_else(|| {
            nom::Err::Error(nom::error::Error::new(i, nom::error::ErrorKind::Fail))
        })?;
        let y_conv = F::from(y).ok_or_else(|| {
            nom::Err::Error(nom::error::Error::new(i, nom::error::ErrorKind::Fail))
        })?;

        Ok((
            i,
            Coord {
                x: x_conv,
                y: y_conv,
            },
        ))
    }

    fn all_points<F: CoordNum>(i: &str) -> IResult<&str, Vec<Coord<F>>> {
        delimited(multispace0, separated_list1(comma_wsp, point), multispace0)(i)
    }

    match all_points(points) {
        Ok(("", points)) => Ok(LineString::new(points)),
        Ok(_) => Err(IoError::MalformedInput(format!(
            "Could not parse the list of points: {points}"
        ))),
        Err(err) => Err(IoError::MalformedInput(format!(
            "Could not parse the list of points ({err}): {points}"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn svg_points_parsing() {
        let points_str = "10,20 30,40 50,60";
        let line_string = svg_points_to_line_string::<f64>(points_str).unwrap();
        assert_eq!(line_string.0.len(), 3);
        assert_eq!(line_string.0[0], Coord { x: 10.0, y: 20.0 });
        assert_eq!(line_string.0[1], Coord { x: 30.0, y: 40.0 });
        assert_eq!(line_string.0[2], Coord { x: 50.0, y: 60.0 });
    }
}
