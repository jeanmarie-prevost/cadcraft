//! LAS / LAZ (ASPRS LAS, LASzip) through the `las` crate.

use crate::{Attributes, Error, Filter, LasVersion, Point, PointSink, Report, Result, ScanInfo, Source, WriteOptions};
use las::point::{Classification, Format as PointFormat};
use las::{Builder, Color, Transform, Vector, Writer};
use std::path::{Path, PathBuf};

/// Points read per chunk.
const CHUNK: u64 = 65_536;

pub struct LasSource {
    path: PathBuf,
    scans: Vec<ScanInfo>,
}

impl LasSource {
    pub fn open(path: &Path) -> Result<LasSource> {
        let r = las::Reader::from_path(path)?;
        let h = r.header();
        let v = h.version();
        let pf = h.point_format();
        let mut notes = vec![format!("LAS {}.{}, point format {}", v.major, v.minor, pf.to_u8().unwrap_or(0))];
        if pf.is_compressed {
            notes.push("LAZ compressed".into());
        }
        if h.has_crs_vlrs() {
            notes.push("has a coordinate reference system (not carried to other formats)".into());
        }
        let name = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        let scans = vec![ScanInfo {
            name,
            points: Some(h.number_of_points()),
            attributes: Attributes { intensity: true, color: pf.has_color, classification: true },
            notes,
        }];
        Ok(LasSource { path: path.to_path_buf(), scans })
    }

    pub fn scans(&self) -> &[ScanInfo] {
        &self.scans
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn read_scan(&mut self, i: usize, sink: &mut PointSink<'_>) -> Result<u64> {
        if i != 0 {
            return Err(Error::Invalid(format!("no scan {i}")));
        }
        let mut r = las::Reader::from_path(&self.path)?;
        let mut invalid = 0u64;
        loop {
            let data = r.read_points(CHUNK)?;
            if data.is_empty() {
                break;
            }
            for p in data.points() {
                let p = p?;
                if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
                    invalid = invalid.saturating_add(1);
                    continue;
                }
                sink(Point {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                    intensity: Some(f32::from(p.intensity) / 65535.0),
                    color: p.color.map(|c| [c.red, c.green, c.blue]),
                    classification: Some(u8::from(p.classification)),
                })?;
            }
        }
        Ok(invalid)
    }
}

/// Merge every scan of every source into one LAS/LAZ file (compressed when `out` ends in `.laz`).
pub fn write(sources: &mut [Source], out: &Path, attrs: Attributes, opts: &WriteOptions, report: &mut Report) -> Result<()> {
    let pf = match (opts.las_version, attrs.color) {
        (LasVersion::V1_2, false) => 0,
        (LasVersion::V1_2, true) => 2,
        (LasVersion::V1_4, false) => 6,
        (LasVersion::V1_4, true) => 7,
    };
    let mut filter = Filter::new(opts.every);
    let mut writer: Option<Writer<std::io::BufWriter<std::fs::File>>> = None;
    for src in sources.iter_mut() {
        let n = src.scans().len();
        for i in 0..n {
            let invalid = src.read_scan(i, &mut |p: Point| {
                if !filter.keep(report, &p) {
                    return Ok(());
                }
                let w = match &mut writer {
                    Some(w) => w,
                    None => writer.insert(create(out, opts, pf, [p.x, p.y, p.z])?),
                };
                w.write_point(las_point(&p, pf, attrs))?;
                Ok(())
            })?;
            report.points_invalid = report.points_invalid.saturating_add(invalid);
        }
    }
    let mut w = match writer {
        Some(w) => w,
        None => create(out, opts, pf, [0.0; 3])?,
    };
    w.close()?;
    Ok(())
}

/// A LAS writer whose offset sits at `origin` (rounded to the precision grid), so coordinates far
/// from zero (national grids) keep full resolution in LAS's 32-bit integers.
fn create(out: &Path, opts: &WriteOptions, pf: u8, origin: [f64; 3]) -> Result<Writer<std::io::BufWriter<std::fs::File>>> {
    let version = match opts.las_version {
        LasVersion::V1_2 => (1, 2),
        LasVersion::V1_4 => (1, 4),
    };
    let mut b = Builder::from(version);
    b.point_format = PointFormat::new(pf)?;
    b.generating_software = "CADCraft".into();
    b.system_identifier = "CONVERSION".into();
    let t = |o: f64| {
        let unit = (opts.precision * 1000.0).max(1.0);
        Transform { scale: opts.precision, offset: if o.is_finite() { (o / unit).round() * unit } else { 0.0 } }
    };
    b.transforms = Vector { x: t(origin[0]), y: t(origin[1]), z: t(origin[2]) };
    let header = b.into_header()?;
    Ok(Writer::from_path(out, header)?)
}

fn las_point(p: &Point, pf: u8, attrs: Attributes) -> las::Point {
    let classification = p.classification.and_then(|c| Classification::new(c).ok()).filter(|c| pf >= 6 || u8::from(*c) < 32).unwrap_or_default();
    las::Point {
        x: p.x,
        y: p.y,
        z: p.z,
        intensity: if attrs.intensity { crate::unit_to_u16(f64::from(p.intensity.unwrap_or(0.0))) } else { 0 },
        return_number: 1,
        number_of_returns: 1,
        classification,
        gps_time: if pf >= 6 { Some(0.0) } else { None },
        color: if pf == 2 || pf == 7 {
            let c = p.color.unwrap_or([0; 3]);
            Some(Color { red: c[0], green: c[1], blue: c[2] })
        } else {
            None
        },
        ..Default::default()
    }
}
