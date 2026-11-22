//! Native, read-only Fusion archive reader. No Python or Fusion runtime required.
//!
//! ASM entities retain every token, including unsupported spline definitions.
//! Analytic geometry has typed evaluation and bounded, partial face tessellation.
//! The crate does not regenerate the parametric timeline. See `README.md`.
//!
//! ```no_run
//! let file = f3d::FusionFile::open("model.f3d")?;
//! for document in file.documents() {
//!     for brep in &document.breps {
//!         println!("{}: {} faces", brep.path, brep.model.census().faces());
//!         for body in brep.model.bodies() {
//!             for face in body.faces() {
//!                 println!("face {}", face.index());
//!             }
//!         }
//!     }
//! }
//! # Ok::<(), f3d::Error>(())
//! ```

pub mod archive;
pub mod asm;
mod binary;
pub mod document;
pub mod geometry;
pub mod math;
pub mod mesh;
pub mod streams;

pub use document::{Document, FusionFile};
pub use error::{Error, Result};

mod error {
    #[derive(Debug, thiserror::Error)]
    pub enum Error {
        #[error("I/O: {0}")]
        Io(#[from] std::io::Error),
        #[error("ZIP: {0}")]
        Zip(#[from] zip::result::ZipError),
        #[error("JSON: {0}")]
        Json(#[from] serde_json::Error),
        #[error("truncated data at byte {offset}: need {needed}, have {remaining}")]
        Truncated {
            offset: usize,
            needed: usize,
            remaining: usize,
        },
        #[error("invalid data at byte {offset}: {reason}")]
        Invalid { offset: usize, reason: String },
        #[error("resource limit: {0}")]
        Limit(String),
        #[error("unsupported: {0}")]
        Unsupported(String),
        #[error(
            "unsupported {geometry:?} {source_kind:?} at entity {entity_index}; inspect its retained tokens"
        )]
        UnsupportedGeometry {
            geometry: crate::geometry::GeometryKind,
            entity_index: usize,
            source_kind: String,
        },
        #[error("malformed geometry {source_kind:?} at entity {entity_index}: {reason}")]
        MalformedGeometry {
            entity_index: usize,
            source_kind: String,
            reason: String,
        },
        #[error("entry not found: {0}")]
        MissingEntry(String),
        #[error("{path}: {source}")]
        Entry { path: String, source: Box<Error> },
    }
    pub type Result<T> = std::result::Result<T, Error>;
}

/// Bounds apply before payload allocation and during binary parsing.
#[derive(Debug, Clone)]
pub struct Limits {
    pub max_entries: usize,
    pub max_entry_bytes: u64,
    pub max_total_bytes: u64,
    pub max_string_bytes: usize,
    pub max_records: usize,
    pub max_tokens: usize,
    pub max_subtype_depth: usize,
    pub max_package_depth: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_entries: 20_000,
            max_entry_bytes: 256 * 1024 * 1024,
            max_total_bytes: 1024 * 1024 * 1024,
            max_string_bytes: 16 * 1024 * 1024,
            max_records: 2_000_000,
            max_tokens: 10_000_000,
            max_subtype_depth: 256,
            max_package_depth: 4,
        }
    }
}

pub mod spline;
