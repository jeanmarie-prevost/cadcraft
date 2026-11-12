//! Text point clouds: Leica-style PTS (point-count lines, `x y z intensity r g b`) and
//! XYZ/TXT/CSV/ASC (one point per line, optional header naming the columns).
//!
//! Separators: spaces, tabs, commas. A line with semicolons splits on semicolons only and reads a
//! comma as the decimal separator (`12,5;3,25;1,0`, as French spreadsheets write CSV).

use crate::{Attributes, Error, Filter, Point, PointSink, ReadOptions, Report, Result, ScanInfo, Source, WriteOptions};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

/// Longest accepted line (bytes).
const MAX_LINE: u64 = 64 * 1024;
/// Lines looked at to find the first point and the column layout.
const SNIFF_LINES: usize = 1000;
/// Data lines looked at to detect the intensity range.
const SNIFF_POINTS: usize = 10_000;

/// Which column holds what.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Layout {
    xyz: [usize; 3],
    intensity: Option<usize>,
    rgb: Option<[usize; 3]>,
}

impl Layout {
    /// The layout from a column count (no header): `x y z`, `x y z i`, `x y z r g b`,
    /// `x y z i r g b` (+ ignored extra columns such as normals).
    fn from_count(n: usize) -> Option<Layout> {
        let base = Layout { xyz: [0, 1, 2], ..Default::default() };
        Some(match n {
            0..=2 => return None,
            3 => base,
            4 | 5 => Layout { intensity: Some(3), ..base },
            6 => Layout { rgb: Some([3, 4, 5]), ..base },
            _ => Layout { intensity: Some(3), rgb: Some([4, 5, 6]), ..base },
        })
    }

    /// The layout from column names (`x`, `y`, `z`, `i`/`intensity`, `r`/`red`, ...). `None`
    /// unless x, y and z are all named.
    fn from_names<S: AsRef<str>>(names: &[S]) -> Option<Layout> {
        let find = |keys: &[&str]| {
            names.iter().position(|n| {
                let n = n.as_ref().trim().trim_start_matches(['/', '#']).trim().to_ascii_lowercase();
                keys.contains(&n.as_str())
            })
        };
        let xyz = [find(&["x"])?, find(&["y"])?, find(&["z"])?];
        let intensity = find(&["i", "intensity", "scalar_intensity", "int"]);
        let rgb = match (find(&["r", "red"]), find(&["g", "green"]), find(&["b", "blue"])) {
            (Some(r), Some(g), Some(b)) => Some([r, g, b]),
            _ => None,
        };
        Some(Layout { xyz, intensity, rgb })
    }

    fn columns(&self) -> usize {
        let mut m = self.xyz.iter().copied().max().unwrap_or(0);
        if let Some(i) = self.intensity {
            m = m.max(i);
        }
        if let Some(c) = self.rgb {
            m = c.iter().copied().fold(m, usize::max);
        }
        m.saturating_add(1)
    }
}

/// Split a line into numeric-looking fields.
fn fields(line: &str) -> Vec<&str> {
    if line.contains(';') {
        line.split(';').map(str::trim).filter(|s| !s.is_empty()).collect()
    } else {
        line.split(|c: char| c.is_ascii_whitespace() || c == ',').filter(|s| !s.is_empty()).collect()
    }
}

fn num(field: &str, semicolons: bool) -> Option<f64> {
    let v = if semicolons && field.contains(',') { field.replace(',', ".").parse::<f64>().ok() } else { field.parse::<f64>().ok() };
    v.filter(|v| v.is_finite())
}

fn is_numeric_line(f: &[&str], semi: bool) -> bool {
    !f.is_empty() && f.iter().all(|t| num(t, semi).is_some())
}

/// Read one line (without its end), capped at [`MAX_LINE`]. `Ok(None)` at end of file.
fn read_line<R: BufRead>(r: &mut R, buf: &mut Vec<u8>) -> Result<Option<()>> {
    buf.clear();
    let n = r.by_ref().take(MAX_LINE.saturating_add(1)).read_until(b'\n', buf)?;
    if n == 0 {
        return Ok(None);
    }
    if buf.last() != Some(&b'\n') && n as u64 > MAX_LINE {
        return Err(Error::Invalid(format!("a line is longer than {MAX_LINE} bytes; is this really a text point cloud?")));
    }
    while matches!(buf.last(), Some(b'\n' | b'\r')) {
        buf.pop();
    }
    Ok(Some(()))
}

pub struct TextSource {
    path: PathBuf,
    pts: bool,
    layout: Layout,
    intensity_range: (f64, f64),
    scans: Vec<ScanInfo>,
}

impl TextSource {
    pub fn open(path: &Path, pts: bool, opts: &ReadOptions) -> Result<TextSource> {
        let mut r = BufReader::new(File::open(path)?);
        let mut buf = Vec::new();
        let mut header: Option<Layout> = None;
        let mut first: Option<usize> = None;
        let mut samples: Vec<f64> = Vec::new();
        let mut declared: u64 = 0;
        let mut layout_from_count: Option<Layout> = None;
        let mut lines = 0usize;
        while read_line(&mut r, &mut buf)?.is_some() {
            lines += 1;
            let line = String::from_utf8_lossy(&buf);
            let semi = line.contains(';');
            let f = fields(&line);
            if f.is_empty() {
                continue;
            }
            if !is_numeric_line(&f, semi) {
                if first.is_none() && header.is_none() {
                    header = Layout::from_names(&f);
                }
                if first.is_none() && lines > SNIFF_LINES {
                    break;
                }
                continue;
            }
            if f.len() < 3 {
                // PTS point-count line.
                if let Some(n) = f.first().and_then(|t| t.parse::<u64>().ok()) {
                    declared = declared.saturating_add(n);
                }
                continue;
            }
            if first.is_none() {
                first = Some(f.len());
                layout_from_count = Layout::from_count(f.len());
            }
            let layout = opts.columns.as_ref().and_then(|c| Layout::from_names(c)).or_else(|| header.clone()).or_else(|| layout_from_count.clone());
            if let Some(v) = layout.and_then(|l| l.intensity).and_then(|i| f.get(i)).and_then(|t| num(t, semi)) {
                samples.push(v);
            }
            if samples.len() >= SNIFF_POINTS || lines > SNIFF_LINES.saturating_mul(20) {
                break;
            }
        }
        let mut notes = Vec::new();
        let layout = if let Some(cols) = &opts.columns {
            notes.push(format!("columns given: {}", cols.join(" ")));
            Layout::from_names(cols).ok_or_else(|| Error::Invalid("--columns must name x, y and z".into()))?
        } else if let Some(h) = header {
            notes.push("columns from the header line".into());
            h
        } else {
            let n = first.ok_or_else(|| Error::Invalid("no point found (expected lines of at least three numbers: x y z ...)".into()))?;
            let l = layout_from_count.ok_or_else(|| Error::Invalid("a point needs at least three numbers".into()))?;
            notes.push(format!(
                "{n} columns read as: x y z{}{}",
                if l.intensity.is_some() { " intensity" } else { "" },
                if l.rgb.is_some() { " red green blue" } else { "" }
            ));
            l
        };
        let intensity_range = match opts.intensity_range {
            Some(r) => r,
            None => detect_range(&samples, pts),
        };
        if layout.intensity.is_some() {
            notes.push(format!("intensity range {} .. {}", intensity_range.0, intensity_range.1));
        }
        if layout.rgb.is_some() {
            notes.push("colours read as 8-bit (0..255)".into());
        }
        let name = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        let scans = vec![ScanInfo {
            name,
            points: if pts && declared > 0 { Some(declared) } else { None },
            attributes: Attributes { intensity: layout.intensity.is_some(), color: layout.rgb.is_some(), classification: false },
            notes,
        }];
        Ok(TextSource { path: path.to_path_buf(), pts, layout, intensity_range, scans })
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
        let mut r = BufReader::with_capacity(1 << 20, File::open(&self.path)?);
        let mut buf = Vec::new();
        let mut invalid = 0u64;
        let need = self.layout.columns();
        let (lo, hi) = self.intensity_range;
        let span = hi - lo;
        while read_line(&mut r, &mut buf)?.is_some() {
            let line = String::from_utf8_lossy(&buf);
            let semi = line.contains(';');
            let f = fields(&line);
            if f.is_empty() || (self.pts && f.len() < 3) {
                continue;
            }
            let get = |i: usize| f.get(i).and_then(|t| num(t, semi));
            let [xi, yi, zi] = self.layout.xyz;
            let (Some(x), Some(y), Some(z)) = (get(xi), get(yi), get(zi)) else {
                // A header or comment line is not a point; a short numeric line is a broken one.
                if is_numeric_line(&f, semi) || f.len() >= need {
                    invalid = invalid.saturating_add(1);
                }
                continue;
            };
            let intensity = self.layout.intensity.and_then(get).map(|v| if span > 0.0 { ((v - lo) / span).clamp(0.0, 1.0) as f32 } else { 0.0 });
            let color = self.layout.rgb.and_then(|[r, g, b]| Some([byte16(get(r)?), byte16(get(g)?), byte16(get(b)?)]));
            sink(Point { x, y, z, intensity, color, classification: None })?;
        }
        Ok(invalid)
    }
}

/// 0..255 to 0..65535.
fn byte16(v: f64) -> u16 {
    (v.clamp(0.0, 255.0).round() as u16).saturating_mul(257)
}

/// Guess the raw intensity range from sample values: Leica's -2048..2047 when negative values
/// appear (or a PTS file stays within it), else the smallest common range that holds them.
fn detect_range(samples: &[f64], pts: bool) -> (f64, f64) {
    let (min, max) = samples.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &v| (a.min(v), b.max(v)));
    if samples.is_empty() {
        return if pts { (-2048.0, 2047.0) } else { (0.0, 255.0) };
    }
    if min < 0.0 {
        if min >= -1.0 && max <= 1.0 { (-1.0, 1.0) } else { (-2048.0, 2047.0) }
    } else if max <= 1.0 {
        (0.0, 1.0)
    } else if max <= 255.0 {
        (0.0, 255.0)
    } else if max <= 4095.0 {
        (0.0, 4095.0)
    } else {
        (0.0, 65535.0)
    }
}

/// Merge every scan into one space-separated text file: `x y z [intensity] [r g b]`, with a
/// header line naming the columns (intensity 0..65535, colours 0..255).
pub fn write_xyz(sources: &mut [Source], out: &Path, attrs: Attributes, opts: &WriteOptions, report: &mut Report) -> Result<()> {
    let decimals = (-opts.precision.log10()).ceil().clamp(0.0, 9.0) as usize;
    let mut w = BufWriter::with_capacity(1 << 20, File::create(out)?);
    write!(w, "//x y z")?;
    if attrs.intensity {
        write!(w, " intensity")?;
    }
    if attrs.color {
        write!(w, " red green blue")?;
    }
    writeln!(w)?;
    let mut filter = Filter::new(opts.every);
    for src in sources.iter_mut() {
        for i in 0..src.scans().len() {
            let invalid = src.read_scan(i, &mut |p: Point| {
                if !filter.keep(report, &p) {
                    return Ok(());
                }
                write!(w, "{:.d$} {:.d$} {:.d$}", p.x, p.y, p.z, d = decimals)?;
                if attrs.intensity {
                    write!(w, " {}", crate::unit_to_u16(f64::from(p.intensity.unwrap_or(0.0))))?;
                }
                if attrs.color {
                    let c = p.color.unwrap_or([0; 3]);
                    write!(w, " {} {} {}", c[0] / 257, c[1] / 257, c[2] / 257)?;
                }
                writeln!(w)?;
                Ok(())
            })?;
            report.points_invalid = report.points_invalid.saturating_add(invalid);
        }
    }
    w.flush()?;
    Ok(())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn layouts() {
        assert_eq!(Layout::from_count(2), None);
        assert_eq!(Layout::from_count(7).and_then(|l| l.rgb), Some([4, 5, 6]));
        let l = Layout::from_names(&["//X", "Y", "Z", "R", "G", "B", "Nx"]).unwrap();
        assert_eq!((l.rgb, l.intensity), (Some([3, 4, 5]), None));
        assert!(Layout::from_names(&["a", "b", "c"]).is_none());
    }

    #[test]
    fn french_csv_fields() {
        let f = fields("12,5;3,25;1");
        assert_eq!(f, ["12,5", "3,25", "1"]);
        assert_eq!(num(f[0], true), Some(12.5));
        assert_eq!(fields("1.5, 2 ,3\t4"), ["1.5", "2", "3", "4"]);
    }

    #[test]
    fn ranges() {
        assert_eq!(detect_range(&[-100.0, 300.0], true), (-2048.0, 2047.0));
        assert_eq!(detect_range(&[0.2, 0.9], false), (0.0, 1.0));
        assert_eq!(detect_range(&[12.0, 200.0], false), (0.0, 255.0));
        assert_eq!(detect_range(&[], true), (-2048.0, 2047.0));
    }
}
