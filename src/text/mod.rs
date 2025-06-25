//! Text rendering using various font formats

#[cfg(feature = "hershey-text")]
pub mod hershey;

#[cfg(feature = "truetype-text")]
pub mod truetype;
