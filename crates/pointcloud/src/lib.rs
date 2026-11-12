//! Point cloud reading, writing and conversion.
//!
//! Reads E57 (ASTM E2807), LAS/LAZ (ASPRS LAS 1.0–1.4, LASzip), Leica-style PTS, XYZ/TXT/CSV and
//! PLY; writes LAS/LAZ, E57 and XYZ. Every format comes from its public specification.
//!
//! Autodesk ReCap's RCP/RCS files are *not* read: their format is undocumented, and CADCraft only
//! implements published formats. ReCap itself exports a project to E57 (or PTS), which this crate
//! then converts ([`Format::from_path`] says so when handed an RCP/RCS file).
//!
//! Conversion streams: points go from the reader to the writer one at a time, so a cloud of any
//! size converts in constant memory. Several inputs (or the several scans of an E57 file) can be
//! merged into one output.
#![forbid(unsafe_code)]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod e57io;
mod lasio;
mod ply;
mod text;

use serde::Serialize;
use std::path::{Path, PathBuf};

/// Everything that can go wrong reading or writing a point cloud.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    Io(#[from] std::io::Error),
    #[error("E57: {0}")]
    E57(#[from] e57::Error),
    #[error("LAS: {0}")]
    Las(#[from] las::Error),
    /// The file's contents are not what its format requires.
    #[error("{0}")]
    Invalid(String),
    /// The format (or this flavour of it) is not supported.
    #[error("{0}")]
    Unsupported(String),
    /// An error with the file it concerns.
    #[error("{}: {source}", path.display())]
    File { path: PathBuf, source: Box<Error> },
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    fn in_file(self, path: &Path) -> Error {
        match self {
            Error::File { .. } => self,
            e => Error::File { path: path.to_path_buf(), source: Box::new(e) },
        }
    }
}

/// One point, in a format-neutral form.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Intensity normalised to 0..=1.
    pub intensity: Option<f32>,
    /// Colour, 16 bits per channel (8-bit sources are scaled by 257).
    pub color: Option<[u16; 3]>,
    /// ASPRS classification code (LAS sources only).
    pub classification: Option<u8>,
}

/// Which attributes a cloud carries besides its coordinates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct Attributes {
    pub intensity: bool,
    pub color: bool,
    pub classification: bool,
}

impl Attributes {
    fn union(self, o: Attributes) -> Attributes {
        Attributes { intensity: self.intensity || o.intensity, color: self.color || o.color, classification: self.classification || o.classification }
    }
}

/// One scan (station / point cloud) of a source file.
#[derive(Clone, Debug, Serialize)]
pub struct ScanInfo {
    pub name: String,
    /// Point count when the file states it up front.
    pub points: Option<u64>,
    pub attributes: Attributes,
    /// How the reader interpreted the file (detected columns, intensity range, ...).
    pub notes: Vec<String>,
}

/// Axis-aligned bounds of the points written.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Bounds {
    fn of(p: &Point) -> Bounds {
        Bounds { min: [p.x, p.y, p.z], max: [p.x, p.y, p.z] }
    }
    fn grow(&mut self, p: &Point) {
        for (i, v) in [p.x, p.y, p.z].into_iter().enumerate() {
            if let (Some(lo), Some(hi)) = (self.min.get_mut(i), self.max.get_mut(i)) {
                *lo = lo.min(v);
                *hi = hi.max(v);
            }
        }
    }
}

fn grow(b: &mut Option<Bounds>, p: &Point) {
    match b {
        Some(b) => b.grow(p),
        None => *b = Some(Bounds::of(p)),
    }
}

/// A point cloud file format.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Format {
    E57,
    Las,
    Laz,
    Pts,
    /// XYZ, TXT, CSV, ASC: one point per line.
    Xyz,
    Ply,
}

impl Format {
    /// The format named by a file extension (`las`, `.LAZ`, `csv`, ...).
    pub fn from_extension(ext: &str) -> Result<Format> {
        let e = ext.trim_start_matches('.').to_ascii_lowercase();
        Ok(match e.as_str() {
            "e57" => Format::E57,
            "las" => Format::Las,
            "laz" => Format::Laz,
            "pts" => Format::Pts,
            "xyz" | "txt" | "csv" | "asc" => Format::Xyz,
            "ply" => Format::Ply,
            "rcp" | "rcs" => {
                return Err(Error::Unsupported(
                    "RCP/RCS is Autodesk ReCap's undocumented format and cannot be read directly. \
                     Open the project in ReCap, export it as E57 (or PTS), then convert that file."
                        .into(),
                ));
            }
            other => {
                return Err(Error::Unsupported(format!(
                    "unknown point cloud format `.{other}` (supported: e57, las, laz, pts, xyz, txt, csv, asc, ply)"
                )));
            }
        })
    }

    /// The format of a file, from its extension.
    pub fn from_path(path: &Path) -> Result<Format> {
        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
        Format::from_extension(ext).map_err(|e| e.in_file(path))
    }

    /// The extension written for this format.
    pub fn extension(self) -> &'static str {
        match self {
            Format::E57 => "e57",
            Format::Las => "las",
            Format::Laz => "laz",
            Format::Pts => "pts",
            Format::Xyz => "xyz",
            Format::Ply => "ply",
        }
    }

    /// Whether this crate can write the format.
    pub fn writable(self) -> bool {
        matches!(self, Format::E57 | Format::Las | Format::Laz | Format::Xyz)
    }
}

/// How sources are read.
#[derive(Clone, Debug, Default)]
pub struct ReadOptions {
    /// Raw intensity range of text/PLY sources, mapped to 0..=1. `None`: detected from the data.
    pub intensity_range: Option<(f64, f64)>,
    /// Column names of text sources (`x`, `y`, `z`, `i`, `r`, `g`, `b`, `_` to skip).
    /// `None`: from the file's header line, else from the number of columns.
    pub columns: Option<Vec<String>>,
}

/// LAS version written.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LasVersion {
    /// LAS 1.2, point formats 0/2: read by virtually every tool.
    #[default]
    V1_2,
    /// LAS 1.4, point formats 6/7.
    V1_4,
}

/// How the output is written.
#[derive(Clone, Debug)]
pub struct WriteOptions {
    /// Coordinate resolution in drawing units (LAS scale, XYZ decimals). Default 0.001.
    pub precision: f64,
    pub las_version: LasVersion,
    /// Keep one point in `every` (1 = all).
    pub every: u64,
}

impl Default for WriteOptions {
    fn default() -> Self {
        WriteOptions { precision: 0.001, las_version: LasVersion::V1_2, every: 1 }
    }
}

/// What a conversion did.
#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub inputs: Vec<PathBuf>,
    pub output: PathBuf,
    pub format: Format,
    pub scans: Vec<ScanInfo>,
    pub points_read: u64,
    pub points_written: u64,
    /// Points dropped because they had no valid coordinates (or a malformed line).
    pub points_invalid: u64,
    pub attributes: Attributes,
    pub bounds: Option<Bounds>,
}

/// An open point cloud file.
pub enum Source {
    E57(Box<e57io::E57Source>),
    Las(lasio::LasSource),
    Text(text::TextSource),
    Ply(ply::PlySource),
}

/// Point callback used while streaming a scan.
pub type PointSink<'a> = dyn FnMut(Point) -> Result<()> + 'a;

impl Source {
    /// Open a file and read its header(s).
    pub fn open(path: &Path, opts: &ReadOptions) -> Result<Source> {
        let r = match Format::from_path(path)? {
            Format::E57 => e57io::E57Source::open(path).map(|s| Source::E57(Box::new(s))),
            Format::Las | Format::Laz => lasio::LasSource::open(path).map(Source::Las),
            Format::Pts => text::TextSource::open(path, true, opts).map(Source::Text),
            Format::Xyz => text::TextSource::open(path, false, opts).map(Source::Text),
            Format::Ply => ply::PlySource::open(path, opts).map(Source::Ply),
        };
        r.map_err(|e| e.in_file(path))
    }

    pub fn scans(&self) -> &[ScanInfo] {
        match self {
            Source::E57(s) => s.scans(),
            Source::Las(s) => s.scans(),
            Source::Text(s) => s.scans(),
            Source::Ply(s) => s.scans(),
        }
    }

    pub fn path(&self) -> &Path {
        match self {
            Source::E57(s) => s.path(),
            Source::Las(s) => s.path(),
            Source::Text(s) => s.path(),
            Source::Ply(s) => s.path(),
        }
    }

    /// Stream scan `i`'s points to `sink`; returns the number of points dropped as invalid.
    pub fn read_scan(&mut self, i: usize, sink: &mut PointSink<'_>) -> Result<u64> {
        let path = self.path().to_path_buf();
        let r = match self {
            Source::E57(s) => s.read_scan(i, sink),
            Source::Las(s) => s.read_scan(i, sink),
            Source::Text(s) => s.read_scan(i, sink),
            Source::Ply(s) => s.read_scan(i, sink),
        };
        r.map_err(|e| e.in_file(&path))
    }
}

/// Summary of a file without converting it (headers only, no points read).
pub fn info(path: &Path, opts: &ReadOptions) -> Result<Vec<ScanInfo>> {
    Ok(Source::open(path, opts)?.scans().to_vec())
}

/// Convert (and merge) `inputs` into `output`, whose extension picks the format.
pub fn convert(inputs: &[PathBuf], output: &Path, ropts: &ReadOptions, wopts: &WriteOptions) -> Result<Report> {
    if inputs.is_empty() {
        return Err(Error::Invalid("no input file".into()));
    }
    if !(wopts.precision.is_finite() && wopts.precision > 0.0) {
        return Err(Error::Invalid(format!("precision must be a positive number, got {}", wopts.precision)));
    }
    let format = Format::from_path(output)?;
    if !format.writable() {
        return Err(Error::Unsupported(format!("cannot write .{} (writable: las, laz, e57, xyz)", format.extension())).in_file(output));
    }
    for i in inputs {
        if same_file(i, output) {
            return Err(Error::Invalid("the output would overwrite an input".into()).in_file(output));
        }
    }
    let mut sources = inputs.iter().map(|p| Source::open(p, ropts)).collect::<Result<Vec<_>>>()?;
    let scans: Vec<ScanInfo> = sources.iter().flat_map(|s| s.scans().iter().cloned()).collect();
    let attributes = scans.iter().fold(Attributes::default(), |a, s| a.union(s.attributes));
    let mut report = Report {
        inputs: inputs.to_vec(),
        output: output.to_path_buf(),
        format,
        scans,
        points_read: 0,
        points_written: 0,
        points_invalid: 0,
        attributes,
        bounds: None,
    };
    let r = match format {
        Format::Las | Format::Laz => lasio::write(&mut sources, output, attributes, wopts, &mut report),
        Format::E57 => e57io::write(&mut sources, output, attributes, wopts, &mut report),
        _ => text::write_xyz(&mut sources, output, attributes, wopts, &mut report),
    };
    r.map_err(|e| e.in_file(output))?;
    Ok(report)
}

/// Whether two paths name the same existing file.
fn same_file(a: &Path, b: &Path) -> bool {
    match (a.canonicalize(), b.canonicalize()) {
        (Ok(a), Ok(b)) => a == b,
        _ => false,
    }
}

/// Subsampling and statistics shared by every writer: returns whether to write `p`.
struct Filter {
    every: u64,
    seen: u64,
}

impl Filter {
    fn new(every: u64) -> Filter {
        Filter { every: every.max(1), seen: 0 }
    }
    fn keep(&mut self, report: &mut Report, p: &Point) -> bool {
        report.points_read = report.points_read.saturating_add(1);
        let keep = self.seen.is_multiple_of(self.every);
        self.seen = self.seen.wrapping_add(1);
        if keep {
            report.points_written = report.points_written.saturating_add(1);
            grow(&mut report.bounds, p);
        }
        keep
    }
}

/// 0..=1 to 0..=65535, NaN to 0.
fn unit_to_u16(v: f64) -> u16 {
    if v.is_nan() { 0 } else { (v.clamp(0.0, 1.0) * 65535.0).round() as u16 }
}

/// A unique-enough GUID string for E57 (time, process and a counter hashed).
fn guid(salt: &str) -> String {
    use std::hash::{Hash, Hasher};
    let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    let h = |k: u64| {
        let mut s = std::collections::hash_map::DefaultHasher::new();
        (t, std::process::id(), salt, k).hash(&mut s);
        s.finish()
    };
    let (a, b) = (h(1), h(2));
    format!("{{{:08x}-{:04x}-4{:03x}-{:04x}-{:012x}}}", a >> 32, (a >> 16) & 0xffff, a & 0x0fff, 0x8000 | (b >> 48 & 0x3fff), b & 0xffff_ffff_ffff)
}

#[cfg(test)]
mod tests;
