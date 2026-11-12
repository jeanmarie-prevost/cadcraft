//! E57 (ASTM E2807) through the `e57` crate.

use crate::{Attributes, Error, Filter, Point, PointSink, Report, Result, ScanInfo, Source, WriteOptions, unit_to_u16};
use e57::{CartesianCoordinate, ColorLimits, E57Reader, E57Writer, IntensityLimits, PointCloud, Record, RecordDataType, RecordName, RecordValue};
use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

pub struct E57Source {
    path: PathBuf,
    reader: E57Reader<BufReader<File>>,
    clouds: Vec<PointCloud>,
    scans: Vec<ScanInfo>,
}

impl E57Source {
    pub fn open(path: &Path) -> Result<E57Source> {
        let reader = E57Reader::from_file(path)?;
        let clouds = reader.pointclouds();
        let crs = reader.coordinate_metadata().map(str::trim).filter(|s| !s.is_empty()).map(str::to_string);
        let scans = clouds
            .iter()
            .enumerate()
            .map(|(i, pc)| {
                let mut notes = Vec::new();
                if pc.transform.is_some() {
                    notes.push("scan pose applied (points in the file's common coordinate system)".into());
                }
                if !pc.has_cartesian() && pc.has_spherical() {
                    notes.push("spherical coordinates converted to Cartesian".into());
                }
                if let Some(c) = &crs {
                    notes.push(format!("coordinate metadata: {}", c.chars().take(120).collect::<String>()));
                }
                ScanInfo {
                    name: pc.name.clone().filter(|n| !n.trim().is_empty()).unwrap_or_else(|| format!("scan {}", i + 1)),
                    points: Some(pc.records),
                    attributes: Attributes { intensity: pc.has_intensity(), color: pc.has_color(), classification: false },
                    notes,
                }
            })
            .collect();
        Ok(E57Source { path: path.to_path_buf(), reader, clouds, scans })
    }

    pub fn scans(&self) -> &[ScanInfo] {
        &self.scans
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn read_scan(&mut self, i: usize, sink: &mut PointSink<'_>) -> Result<u64> {
        let pc = self.clouds.get(i).ok_or_else(|| Error::Invalid(format!("no scan {i}")))?.clone();
        let mut it = self.reader.pointcloud_simple(&pc)?;
        it.apply_pose(true);
        it.spherical_to_cartesian(true);
        it.intensity_to_color(false);
        it.normalize_intensity(true);
        it.normalize_color(true);
        let mut invalid = 0u64;
        for p in it {
            let p = p?;
            let CartesianCoordinate::Valid { x, y, z } = p.cartesian else {
                invalid = invalid.saturating_add(1);
                continue;
            };
            if !(x.is_finite() && y.is_finite() && z.is_finite()) {
                invalid = invalid.saturating_add(1);
                continue;
            }
            let color = p.color.map(|c| [unit_to_u16(f64::from(c.red)), unit_to_u16(f64::from(c.green)), unit_to_u16(f64::from(c.blue))]);
            let intensity = p.intensity.filter(|v| v.is_finite()).map(|v| v.clamp(0.0, 1.0));
            sink(Point { x, y, z, intensity, color, classification: None })?;
        }
        Ok(invalid)
    }
}

/// Write every scan of every source as its own E57 point cloud, in world coordinates.
pub fn write(sources: &mut [Source], out: &Path, attrs: Attributes, opts: &WriteOptions, report: &mut Report) -> Result<()> {
    let mut w = E57Writer::from_file(out, &crate::guid(&out.to_string_lossy()))?;
    let mut prototype = vec![
        Record { name: RecordName::CartesianX, data_type: RecordDataType::Double { min: None, max: None } },
        Record { name: RecordName::CartesianY, data_type: RecordDataType::Double { min: None, max: None } },
        Record { name: RecordName::CartesianZ, data_type: RecordDataType::Double { min: None, max: None } },
    ];
    if attrs.intensity {
        prototype.push(Record { name: RecordName::Intensity, data_type: RecordDataType::Integer { min: 0, max: 65535 } });
    }
    if attrs.color {
        for name in [RecordName::ColorRed, RecordName::ColorGreen, RecordName::ColorBlue] {
            prototype.push(Record { name, data_type: RecordDataType::Integer { min: 0, max: 255 } });
        }
    }
    let mut filter = Filter::new(opts.every);
    let mut n = 0usize;
    for src in sources.iter_mut() {
        let scans = src.scans().to_vec();
        for (i, scan) in scans.iter().enumerate() {
            n = n.saturating_add(1);
            let mut pc = w.add_pointcloud(&crate::guid(&format!("{}#{n}", out.display())), prototype.clone())?;
            pc.set_name(Some(scan.name.clone()));
            pc.set_description(Some(format!(
                "converted by CADCraft from {}",
                src.path().file_name().map(|f| f.to_string_lossy()).unwrap_or_default()
            )));
            if attrs.intensity {
                pc.set_intensity_limits(Some(IntensityLimits {
                    intensity_min: Some(RecordValue::Integer(0)),
                    intensity_max: Some(RecordValue::Integer(65535)),
                }));
            }
            if attrs.color {
                let (lo, hi) = (Some(RecordValue::Integer(0)), Some(RecordValue::Integer(255)));
                pc.set_color_limits(Some(ColorLimits {
                    red_min: lo.clone(),
                    red_max: hi.clone(),
                    green_min: lo.clone(),
                    green_max: hi.clone(),
                    blue_min: lo,
                    blue_max: hi,
                }));
            }
            let invalid = src.read_scan(i, &mut |p: Point| {
                if !filter.keep(report, &p) {
                    return Ok(());
                }
                let mut v = vec![RecordValue::Double(p.x), RecordValue::Double(p.y), RecordValue::Double(p.z)];
                if attrs.intensity {
                    v.push(RecordValue::Integer(i64::from(unit_to_u16(f64::from(p.intensity.unwrap_or(0.0))))));
                }
                if attrs.color {
                    let c = p.color.unwrap_or([0; 3]);
                    v.extend(c.iter().map(|&c| RecordValue::Integer(i64::from(c / 257))));
                }
                pc.add_point(v).map_err(Error::from)
            })?;
            report.points_invalid = report.points_invalid.saturating_add(invalid);
            pc.finalize()?;
        }
    }
    w.finalize()?;
    Ok(())
}
