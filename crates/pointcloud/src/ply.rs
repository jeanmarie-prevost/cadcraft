//! PLY (Stanford polygon format): the `vertex` element of ASCII and binary (little/big endian)
//! files. Faces and other elements are ignored; the vertex element must come first (as every
//! point cloud writer does), elements before it must be empty.

use crate::{Attributes, Error, Point, PointSink, ReadOptions, Result, ScanInfo};
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::{Path, PathBuf};

const MAX_HEADER_LINES: usize = 10_000;
const MAX_LINE: u64 = 64 * 1024;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Encoding {
    Ascii,
    Le,
    Be,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Ty {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Ty {
    fn parse(s: &str) -> Option<Ty> {
        Some(match s {
            "char" | "int8" => Ty::I8,
            "uchar" | "uint8" => Ty::U8,
            "short" | "int16" => Ty::I16,
            "ushort" | "uint16" => Ty::U16,
            "int" | "int32" => Ty::I32,
            "uint" | "uint32" => Ty::U32,
            "float" | "float32" => Ty::F32,
            "double" | "float64" => Ty::F64,
            _ => return None,
        })
    }
    fn size(self) -> usize {
        match self {
            Ty::I8 | Ty::U8 => 1,
            Ty::I16 | Ty::U16 => 2,
            Ty::I32 | Ty::U32 | Ty::F32 => 4,
            Ty::F64 => 8,
        }
    }
    /// The largest value of an integer type (for normalising colours/intensity); 1 for floats.
    fn full_scale(self) -> f64 {
        match self {
            Ty::I8 => 127.0,
            Ty::U8 => 255.0,
            Ty::I16 => 32767.0,
            Ty::U16 => 65535.0,
            Ty::I32 => 2147483647.0,
            Ty::U32 => 4294967295.0,
            Ty::F32 | Ty::F64 => 1.0,
        }
    }
    fn decode(self, b: &[u8], enc: Encoding) -> Option<f64> {
        macro_rules! rd {
            ($t:ty, $n:expr) => {{
                let a: [u8; $n] = b.get(..$n)?.try_into().ok()?;
                if enc == Encoding::Be { <$t>::from_be_bytes(a) } else { <$t>::from_le_bytes(a) }
            }};
        }
        Some(match self {
            Ty::I8 => f64::from(rd!(i8, 1)),
            Ty::U8 => f64::from(rd!(u8, 1)),
            Ty::I16 => f64::from(rd!(i16, 2)),
            Ty::U16 => f64::from(rd!(u16, 2)),
            Ty::I32 => f64::from(rd!(i32, 4)),
            Ty::U32 => f64::from(rd!(u32, 4)),
            Ty::F32 => f64::from(rd!(f32, 4)),
            Ty::F64 => rd!(f64, 8),
        })
    }
}

#[derive(Clone, Debug)]
struct Prop {
    name: String,
    ty: Ty,
}

pub struct PlySource {
    path: PathBuf,
    enc: Encoding,
    props: Vec<Prop>,
    count: u64,
    data_start: u64,
    xyz: [usize; 3],
    intensity: Option<usize>,
    rgb: Option<[usize; 3]>,
    intensity_range: Option<(f64, f64)>,
    scans: Vec<ScanInfo>,
}

fn header_line<R: BufRead>(r: &mut R, buf: &mut Vec<u8>, consumed: &mut u64) -> Result<String> {
    buf.clear();
    let n = r.by_ref().take(MAX_LINE).read_until(b'\n', buf)?;
    if n == 0 {
        return Err(Error::Invalid("PLY header ends before `end_header`".into()));
    }
    *consumed = consumed.saturating_add(n as u64);
    Ok(String::from_utf8_lossy(buf).trim().to_string())
}

impl PlySource {
    pub fn open(path: &Path, opts: &ReadOptions) -> Result<PlySource> {
        let mut r = BufReader::new(File::open(path)?);
        let mut buf = Vec::new();
        let mut consumed = 0u64;
        if header_line(&mut r, &mut buf, &mut consumed)? != "ply" {
            return Err(Error::Invalid("not a PLY file (no `ply` magic line)".into()));
        }
        let mut enc = None;
        let mut current: Option<(String, u64)> = None;
        let mut vertex: Option<(u64, Vec<Prop>)> = None;
        let mut list_in_vertex = false;
        for _ in 0..MAX_HEADER_LINES {
            let line = header_line(&mut r, &mut buf, &mut consumed)?;
            let t: Vec<&str> = line.split_ascii_whitespace().collect();
            match t.as_slice() {
                ["format", f, ..] => {
                    enc = Some(match *f {
                        "ascii" => Encoding::Ascii,
                        "binary_little_endian" => Encoding::Le,
                        "binary_big_endian" => Encoding::Be,
                        other => return Err(Error::Unsupported(format!("PLY format `{other}`"))),
                    })
                }
                ["element", name, n] => {
                    let n: u64 = n.parse().map_err(|_| Error::Invalid(format!("bad PLY element count `{n}`")))?;
                    if vertex.is_none() && *name != "vertex" && n > 0 {
                        return Err(Error::Unsupported(format!("PLY element `{name}` before the vertices")));
                    }
                    current = Some((name.to_string(), n));
                    if *name == "vertex" && vertex.is_none() {
                        vertex = Some((n, Vec::new()));
                    }
                }
                ["property", "list", ..] => {
                    if current.as_ref().is_some_and(|c| c.0 == "vertex") {
                        list_in_vertex = true;
                    }
                }
                ["property", ty, name] => {
                    if current.as_ref().is_some_and(|c| c.0 == "vertex") {
                        let ty = Ty::parse(ty).ok_or_else(|| Error::Invalid(format!("unknown PLY type `{ty}`")))?;
                        if let Some((_, props)) = &mut vertex {
                            props.push(Prop { name: name.to_string(), ty });
                        }
                    }
                }
                ["end_header"] => break,
                _ => {}
            }
        }
        if list_in_vertex {
            return Err(Error::Unsupported("PLY vertices with list properties".into()));
        }
        let enc = enc.ok_or_else(|| Error::Invalid("PLY header has no `format` line".into()))?;
        let (count, props) = vertex.ok_or_else(|| Error::Invalid("PLY file has no vertex element".into()))?;
        let find = |keys: &[&str]| props.iter().position(|p| keys.contains(&p.name.to_ascii_lowercase().as_str()));
        let xyz = match (find(&["x"]), find(&["y"]), find(&["z"])) {
            (Some(x), Some(y), Some(z)) => [x, y, z],
            _ => return Err(Error::Invalid("PLY vertices have no x, y, z".into())),
        };
        let intensity = find(&["intensity", "scalar_intensity", "i"]);
        let rgb = match (find(&["red", "diffuse_red", "r"]), find(&["green", "diffuse_green", "g"]), find(&["blue", "diffuse_blue", "b"])) {
            (Some(r), Some(g), Some(b)) => Some([r, g, b]),
            _ => None,
        };
        let mut notes = vec![format!(
            "PLY {}",
            match enc {
                Encoding::Ascii => "ascii",
                Encoding::Le => "binary little endian",
                Encoding::Be => "binary big endian",
            }
        )];
        if let Some(i) = intensity.and_then(|i| props.get(i)) {
            let (lo, hi) = opts.intensity_range.unwrap_or((0.0, i.ty.full_scale()));
            notes.push(format!("intensity `{}` range {lo} .. {hi}", i.name));
        }
        let name = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        let scans = vec![ScanInfo {
            name,
            points: Some(count),
            attributes: Attributes { intensity: intensity.is_some(), color: rgb.is_some(), classification: false },
            notes,
        }];
        Ok(PlySource {
            path: path.to_path_buf(),
            enc,
            props,
            count,
            data_start: consumed,
            xyz,
            intensity,
            rgb,
            intensity_range: opts.intensity_range,
            scans,
        })
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
        std::io::copy(&mut r.by_ref().take(self.data_start), &mut std::io::sink())?;
        let record: usize = self.props.iter().map(|p| p.ty.size()).sum();
        let mut rec = vec![0u8; record];
        let mut vals = vec![0f64; self.props.len()];
        let mut line = Vec::new();
        let mut invalid = 0u64;
        for _ in 0..self.count {
            match self.enc {
                Encoding::Ascii => {
                    line.clear();
                    if r.by_ref().take(MAX_LINE).read_until(b'\n', &mut line)? == 0 {
                        return Err(Error::Invalid("PLY file ends before its last vertex".into()));
                    }
                    let text = String::from_utf8_lossy(&line);
                    let mut toks = text.split_ascii_whitespace();
                    for v in vals.iter_mut() {
                        *v = toks.next().and_then(|t| t.parse().ok()).unwrap_or(f64::NAN);
                    }
                }
                Encoding::Le | Encoding::Be => {
                    r.read_exact(&mut rec).map_err(|_| Error::Invalid("PLY file ends before its last vertex".into()))?;
                    let mut off = 0usize;
                    for (v, p) in vals.iter_mut().zip(&self.props) {
                        *v = rec.get(off..).and_then(|b| p.ty.decode(b, self.enc)).unwrap_or(f64::NAN);
                        off = off.saturating_add(p.ty.size());
                    }
                }
            }
            let get = |i: usize| vals.get(i).copied().filter(|v| v.is_finite());
            let [xi, yi, zi] = self.xyz;
            let (Some(x), Some(y), Some(z)) = (get(xi), get(yi), get(zi)) else {
                invalid = invalid.saturating_add(1);
                continue;
            };
            let scale = |i: usize| self.props.get(i).map_or(1.0, |p| p.ty.full_scale());
            let intensity = self.intensity.and_then(|i| {
                let (lo, hi) = self.intensity_range.unwrap_or((0.0, scale(i)));
                get(i).map(|v| if hi > lo { ((v - lo) / (hi - lo)).clamp(0.0, 1.0) as f32 } else { 0.0 })
            });
            let color = self.rgb.and_then(|c| {
                let ch = |i: usize| get(i).map(|v| crate::unit_to_u16(v / scale(i)));
                Some([ch(c[0])?, ch(c[1])?, ch(c[2])?])
            });
            sink(Point { x, y, z, intensity, color, classification: None })?;
        }
        Ok(invalid)
    }
}
