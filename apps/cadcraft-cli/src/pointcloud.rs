//! `cadcraft-cli pointcloud`: point cloud information and format conversion.

use cadcraft_pointcloud::{Format, LasVersion, ReadOptions, Report, WriteOptions};
use std::path::{Path, PathBuf};

pub const USAGE: &str = "usage:
  cadcraft-cli pointcloud info FILE...
  cadcraft-cli pointcloud convert IN... -o OUT.(las|laz|e57|xyz)       merge the inputs into one file
  cadcraft-cli pointcloud convert IN... --to (las|laz|e57|xyz) [--out-dir DIR]   one output per input

inputs: .e57 .las .laz .pts .xyz .txt .csv .asc .ply, or a folder (its point cloud files).
Autodesk ReCap .rcp/.rcs files cannot be read (undocumented format): in ReCap, export the
project as E57 (or PTS), then convert that file.

options:
  --precision M            coordinate resolution, drawing units (default 0.001: LAS scale, XYZ decimals)
  --las-version 1.2|1.4    LAS written (default 1.2: point format 0/2, read by every tool;
                           1.4: point format 6/7)
  --every N                keep one point in N (subsampling)
  --intensity-range A,B    raw intensity range of text/PLY inputs (default: detected;
                           PTS from Leica scanners is -2048,2047)
  --columns x,y,z,i,r,g,b  column order of text inputs (`_` skips a column; default: the
                           header line, else by column count: xyz, xyzi, xyzrgb, xyzirgb)
  --json                   print the report as JSON";

#[derive(Default)]
struct Args {
    inputs: Vec<PathBuf>,
    output: Option<PathBuf>,
    to: Option<Format>,
    out_dir: Option<PathBuf>,
    json: bool,
    read: ReadOptions,
    write: WriteOptions,
}

fn value<'a>(it: &mut impl Iterator<Item = &'a String>, name: &str) -> Result<&'a String, String> {
    it.next().ok_or_else(|| format!("pointcloud: {name} needs a value\n{USAGE}"))
}

fn parse(args: &[String]) -> Result<Args, String> {
    let mut a = Args::default();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "-o" | "--output" => a.output = Some(PathBuf::from(value(&mut it, arg)?)),
            "--to" => {
                let f = Format::from_extension(value(&mut it, arg)?).map_err(|e| e.to_string())?;
                if !f.writable() {
                    return Err(format!("pointcloud: cannot write .{} (writable: las, laz, e57, xyz)", f.extension()));
                }
                a.to = Some(f);
            }
            "--out-dir" => a.out_dir = Some(PathBuf::from(value(&mut it, arg)?)),
            "--precision" => {
                let v = value(&mut it, arg)?;
                a.write.precision = v
                    .parse::<f64>()
                    .ok()
                    .filter(|p| p.is_finite() && *p > 0.0 && *p <= 1000.0)
                    .ok_or_else(|| format!("--precision: expected a positive number, got `{v}`"))?;
            }
            "--las-version" => {
                a.write.las_version = match value(&mut it, arg)?.as_str() {
                    "1.2" => LasVersion::V1_2,
                    "1.4" => LasVersion::V1_4,
                    v => return Err(format!("--las-version: expected 1.2 or 1.4, got `{v}`")),
                }
            }
            "--every" => {
                let v = value(&mut it, arg)?;
                a.write.every =
                    v.parse::<u64>().ok().filter(|n| *n >= 1).ok_or_else(|| format!("--every: expected a whole number >= 1, got `{v}`"))?;
            }
            "--intensity-range" => {
                let v = value(&mut it, arg)?;
                let r: Vec<f64> = v.split(',').filter_map(|t| t.trim().parse::<f64>().ok()).filter(|x| x.is_finite()).collect();
                match r.as_slice() {
                    [lo, hi] if hi > lo => a.read.intensity_range = Some((*lo, *hi)),
                    _ => return Err(format!("--intensity-range: expected MIN,MAX with MIN < MAX, got `{v}`")),
                }
            }
            "--columns" => {
                let v = value(&mut it, arg)?;
                let cols: Vec<String> = v.split(',').map(|s| s.trim().to_string()).collect();
                if cols.len() > 64 {
                    return Err("--columns: at most 64 columns".into());
                }
                a.read.columns = Some(cols);
            }
            "--json" => a.json = true,
            s if s.starts_with('-') && s.len() > 1 => return Err(format!("pointcloud: unknown option {s}\n{USAGE}")),
            _ => a.inputs.push(PathBuf::from(arg)),
        }
    }
    Ok(a)
}

/// Files and folders to the input files (a folder gives its point cloud files, sorted).
fn expand(inputs: &[PathBuf]) -> Result<Vec<PathBuf>, String> {
    let mut out = Vec::new();
    for p in inputs {
        if p.is_dir() {
            let mut files: Vec<PathBuf> = std::fs::read_dir(p)
                .map_err(|e| format!("{}: {e}", p.display()))?
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|f| f.is_file() && Format::from_path(f).is_ok())
                .collect();
            files.sort();
            if files.is_empty() {
                return Err(format!("{}: no point cloud file in this folder", p.display()));
            }
            out.extend(files);
        } else {
            out.push(p.clone());
        }
    }
    Ok(out)
}

pub fn run(args: &[String]) -> Result<(), String> {
    let (sub, rest) = args.split_first().ok_or_else(|| USAGE.to_string())?;
    match sub.as_str() {
        "info" => info(rest),
        "convert" => convert(rest),
        "--help" | "-h" | "help" => {
            outln!("{USAGE}");
            Ok(())
        }
        other => Err(format!("pointcloud: unknown subcommand `{other}`\n{USAGE}")),
    }
}

fn info(args: &[String]) -> Result<(), String> {
    let a = parse(args)?;
    let files = expand(&a.inputs)?;
    if files.is_empty() {
        return Err(USAGE.into());
    }
    let mut all = Vec::new();
    for f in &files {
        let scans = cadcraft_pointcloud::info(f, &a.read).map_err(|e| e.to_string())?;
        let format = Format::from_path(f).map_err(|e| e.to_string())?;
        all.push(serde_json::json!({ "file": f, "format": format, "scans": scans }));
    }
    outln!("{}", serde_json::to_string_pretty(&all).unwrap_or_default());
    Ok(())
}

fn convert(args: &[String]) -> Result<(), String> {
    let a = parse(args)?;
    let files = expand(&a.inputs)?;
    if files.is_empty() {
        return Err(format!("pointcloud convert: no input\n{USAGE}"));
    }
    match (&a.output, a.to) {
        (Some(out), None) => {
            if a.out_dir.is_some() {
                return Err("pointcloud convert: --out-dir goes with --to, not -o".into());
            }
            let r = cadcraft_pointcloud::convert(&files, out, &a.read, &a.write).map_err(|e| e.to_string())?;
            print_report(&r, a.json);
            Ok(())
        }
        (None, Some(to)) => {
            let mut failed = 0usize;
            for f in &files {
                let dir = a.out_dir.clone().or_else(|| f.parent().map(Path::to_path_buf)).unwrap_or_default();
                let stem = f.file_stem().map(|s| s.to_os_string()).unwrap_or_else(|| "out".into());
                let mut out = dir.join(stem);
                out.set_extension(to.extension());
                match cadcraft_pointcloud::convert(std::slice::from_ref(f), &out, &a.read, &a.write) {
                    Ok(r) => print_report(&r, a.json),
                    Err(e) => {
                        eprintln!("{e}");
                        failed += 1;
                    }
                }
            }
            if failed == 0 { Ok(()) } else { Err(format!("{failed} of {} file(s) failed", files.len())) }
        }
        (Some(_), Some(_)) => Err("pointcloud convert: give either -o OUT or --to FORMAT, not both".into()),
        (None, None) => Err(format!("pointcloud convert: say where to write: -o OUT or --to FORMAT\n{USAGE}")),
    }
}

fn print_report(r: &Report, json: bool) {
    if json {
        outln!("{}", serde_json::to_string(r).unwrap_or_default());
        return;
    }
    let names: Vec<String> = r.inputs.iter().map(|p| p.display().to_string()).collect();
    outln!("{} -> {}", names.join(" + "), r.output.display());
    let mut attrs = vec!["xyz"];
    if r.attributes.intensity {
        attrs.push("intensity");
    }
    if r.attributes.color {
        attrs.push("colour");
    }
    if r.attributes.classification {
        attrs.push("classification");
    }
    outln!(
        "  {} scan(s), {} point(s) read, {} written, {} invalid dropped; attributes: {}",
        r.scans.len(),
        r.points_read,
        r.points_written,
        r.points_invalid,
        attrs.join(", ")
    );
    if let Some(b) = r.bounds {
        outln!("  bounds: x {:.3} .. {:.3}, y {:.3} .. {:.3}, z {:.3} .. {:.3}", b.min[0], b.max[0], b.min[1], b.max[1], b.min[2], b.max[2]);
    }
    for s in &r.scans {
        for n in &s.notes {
            outln!("  {}: {n}", s.name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn parses_options() {
        let a = parse(&args(&["a.pts", "--to", "LAZ", "--every", "10", "--intensity-range", "-2048,2047", "--las-version", "1.4"])).unwrap();
        assert_eq!((a.inputs.len(), a.to, a.write.every), (1, Some(Format::Laz), 10));
        assert_eq!(a.read.intensity_range, Some((-2048.0, 2047.0)));
        assert_eq!(a.write.las_version, LasVersion::V1_4);
    }

    #[test]
    fn hostile_options_are_errors() {
        for bad in [
            &["--every", "0"][..],
            &["--every", "-1"],
            &["--precision", "nan"],
            &["--precision", "0"],
            &["--intensity-range", "5,1"],
            &["--intensity-range", "nan,1"],
            &["--las-version", "2"],
            &["--to", "pts"],
            &["--to", "rcp"],
            &["--bogus"],
            &["-o"],
        ] {
            assert!(parse(&args(bad)).is_err(), "{bad:?}");
        }
    }
}
