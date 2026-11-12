use super::*;
use std::fs;

/// A fresh scratch directory for one test.
fn dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("cadcraft-pointcloud-{name}-{}", std::process::id()));
    let _ = fs::remove_dir_all(&d);
    fs::create_dir_all(&d).unwrap();
    d
}

/// Read every point of a file.
fn points(path: &Path) -> Vec<Point> {
    let mut s = Source::open(path, &ReadOptions::default()).unwrap();
    let mut v = Vec::new();
    for i in 0..s.scans().len() {
        s.read_scan(i, &mut |p| {
            v.push(p);
            Ok(())
        })
        .unwrap();
    }
    v
}

fn conv(inputs: &[PathBuf], out: &Path) -> Report {
    convert(inputs, out, &ReadOptions::default(), &WriteOptions::default()).unwrap()
}

/// A Leica-style PTS file with Lambert-93-sized coordinates.
fn sample_pts(d: &Path) -> PathBuf {
    let p = d.join("scan.pts");
    fs::write(
        &p,
        "3\n652000.123 6862000.456 35.789 -2048 255 0 0\n652001.000 6862001.000 36.000 2047 0 255 0\n652002.5 6862002.25 37.125 0 0 0 255\n",
    )
    .unwrap();
    p
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn pts_to_las_keeps_coordinates_intensity_and_colour() {
    let d = dir("pts-las");
    let src = sample_pts(&d);
    let out = d.join("scan.las");
    let r = conv(std::slice::from_ref(&src), &out);
    assert_eq!((r.points_read, r.points_written, r.points_invalid), (3, 3, 0));
    assert!(r.attributes.color && r.attributes.intensity);
    let p = points(&out);
    assert_eq!(p.len(), 3);
    assert!(close(p[0].x, 652000.123, 5e-4) && close(p[0].y, 6862000.456, 5e-4) && close(p[0].z, 35.789, 5e-4));
    assert_eq!(p[0].color, Some([65535, 0, 0]));
    assert_eq!(p[2].color, Some([0, 0, 65535]));
    assert!(p[0].intensity.unwrap() < 0.001 && p[1].intensity.unwrap() > 0.999);
    let b = r.bounds.unwrap();
    assert!(close(b.min[0], 652000.123, 1e-9) && close(b.max[2], 37.125, 1e-9));
}

#[test]
fn pts_to_e57_to_laz_round_trip() {
    let d = dir("e57");
    let src = sample_pts(&d);
    let e57 = d.join("scan.e57");
    conv(&[src], &e57);
    let scans = info(&e57, &ReadOptions::default()).unwrap();
    assert_eq!(scans.len(), 1);
    assert_eq!(scans[0].points, Some(3));
    assert!(scans[0].attributes.color && scans[0].attributes.intensity);
    let p = points(&e57);
    assert!(close(p[1].x, 652001.0, 1e-9) && close(p[2].z, 37.125, 1e-9));
    assert_eq!(p[1].color, Some([0, 65535, 0]));

    let laz = d.join("scan.laz");
    let r = conv(&[e57], &laz);
    assert_eq!(r.points_written, 3);
    let q = points(&laz);
    assert!(close(q[2].x, 652002.5, 5e-4) && close(q[2].y, 6862002.25, 5e-4));
    assert_eq!(q[1].color, Some([0, 65535, 0]));
}

#[test]
fn merges_inputs_into_separate_e57_scans() {
    let d = dir("merge");
    let a = sample_pts(&d);
    let b = d.join("b.xyz");
    fs::write(&b, "1 2 3\n4 5 6\n").unwrap();
    let out = d.join("all.e57");
    let r = conv(&[a, b], &out);
    assert_eq!(r.points_written, 5);
    let scans = info(&out, &ReadOptions::default()).unwrap();
    assert_eq!(scans.iter().map(|s| s.name.as_str()).collect::<Vec<_>>(), ["scan", "b"]);
    // The xyz scan has no colour: written as black, not dropped.
    assert_eq!(points(&out).len(), 5);
}

#[test]
fn french_csv_with_header_and_subsampling() {
    let d = dir("csv");
    let src = d.join("releve.csv");
    let mut text = String::from("X;Y;Z;Rouge\n");
    for i in 0..10 {
        text.push_str(&format!("{i},5;2,25;0,125;9\n"));
    }
    fs::write(&src, text).unwrap();
    let out = d.join("releve.las");
    let r = convert(&[src], &out, &ReadOptions::default(), &WriteOptions { every: 3, ..Default::default() }).unwrap();
    assert_eq!((r.points_read, r.points_written), (10, 4));
    let p = points(&out);
    assert!(close(p[1].x, 3.5, 5e-4) && close(p[1].y, 2.25, 5e-4) && close(p[1].z, 0.125, 5e-4));
}

#[test]
fn ply_binary_and_ascii() {
    let d = dir("ply");
    let bin = d.join("b.ply");
    let mut bytes = b"ply\nformat binary_little_endian 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face 0\nproperty list uchar int vertex_indices\nend_header\n".to_vec();
    for (x, y, z, c) in [(1.5f32, 2.0f32, 3.25f64, [255u8, 0, 0]), (4.0, 5.0, 6.0, [0, 0, 255])] {
        bytes.extend(x.to_le_bytes());
        bytes.extend(y.to_le_bytes());
        bytes.extend(z.to_le_bytes());
        bytes.extend(c);
    }
    fs::write(&bin, bytes).unwrap();
    let p = points(&bin);
    assert_eq!(p.len(), 2);
    assert_eq!((p[0].x, p[0].z, p[0].color), (1.5, 3.25, Some([65535, 0, 0])));

    let asc = d.join("a.ply");
    fs::write(&asc, "ply\nformat ascii 1.0\nelement vertex 1\nproperty double x\nproperty double y\nproperty double z\nend_header\n7 8 9\n").unwrap();
    let out = d.join("a.xyz");
    conv(&[asc], &out);
    assert_eq!(fs::read_to_string(&out).unwrap(), "//x y z\n7.000 8.000 9.000\n");
}

#[test]
fn rcp_and_rcs_explain_the_recap_export() {
    for ext in ["rcp", "RCS"] {
        let e = Format::from_path(Path::new(&format!("project.{ext}"))).unwrap_err().to_string();
        assert!(e.contains("export it as E57"), "{e}");
    }
}

#[test]
fn refuses_to_overwrite_its_input_and_unwritable_formats() {
    let d = dir("refuse");
    let src = sample_pts(&d);
    assert!(convert(std::slice::from_ref(&src), &d.join("x.pts"), &ReadOptions::default(), &WriteOptions::default()).is_err());
    let xyz = d.join("a.xyz");
    fs::write(&xyz, "1 2 3\n").unwrap();
    assert!(convert(std::slice::from_ref(&xyz), &xyz, &ReadOptions::default(), &WriteOptions::default()).is_err());
    let bad = WriteOptions { precision: f64::NAN, ..Default::default() };
    assert!(convert(&[xyz], &d.join("o.las"), &ReadOptions::default(), &bad).is_err());
}

#[test]
fn hostile_point_cloud_files_do_not_panic() {
    let d = dir("hostile");
    let cases: &[(&str, &[u8])] = &[
        ("a.e57", b"ASTM-E57\0\0\0\0garbage"),
        ("b.las", b"LASF\x01\x02\xff\xff\xff\xff"),
        ("c.laz", b""),
        ("d.ply", b"ply\nformat binary_little_endian 1.0\nelement vertex 99999999999\nproperty float x\nproperty float y\nproperty float z\nend_header\n\x01\x02"),
        ("e.ply", b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nend_header\n"),
        ("f.ply", b"ply\nformat ascii 1.0\nelement vertex 18446744073709551616\nend_header\n"),
        ("g.pts", b"12\nnan inf -inf\n1 2\n1e999 2 3\nx y z\n\xff\xfe\n"),
        ("h.xyz", b""),
        ("i.csv", b";;;\n,,,\n"),
    ];
    for (name, bytes) in cases {
        let p = d.join(name);
        fs::write(&p, bytes).unwrap();
        for out in ["o.las", "o.e57", "o.xyz"] {
            let _ = convert(std::slice::from_ref(&p), &d.join(out), &ReadOptions::default(), &WriteOptions::default());
        }
        let _ = info(&p, &ReadOptions::default());
    }
    // A line far longer than any point is refused rather than buffered without limit.
    let long = d.join("long.xyz");
    fs::write(&long, vec![b'1'; 200_000]).unwrap();
    assert!(info(&long, &ReadOptions::default()).is_err());
}

#[test]
fn invalid_text_points_are_counted_not_written() {
    let d = dir("invalid");
    let src = d.join("p.xyz");
    fs::write(&src, "1 2 3\n4 nan 6\n7 8\n9 10 11\n").unwrap();
    let r = conv(&[src], &d.join("p.las"));
    assert_eq!((r.points_written, r.points_invalid), (2, 2));
}

#[test]
fn guids_are_distinct() {
    let (a, b) = (guid("a"), guid("b"));
    assert_ne!(a, b);
    assert_eq!(a.len(), 38);
}

#[test]
fn las_1_4_output_and_classification_survive_las_to_las() {
    let d = dir("las14");
    let src = sample_pts(&d);
    let a = d.join("a.las");
    let opts = WriteOptions { las_version: LasVersion::V1_4, ..Default::default() };
    convert(&[src], &a, &ReadOptions::default(), &opts).unwrap();
    let scans = info(&a, &ReadOptions::default()).unwrap();
    assert!(scans[0].notes[0].starts_with("LAS 1.4, point format 7"), "{:?}", scans[0].notes);
    let p = points(&a);
    assert_eq!(p[0].classification, Some(0));
    let b = d.join("b.laz");
    conv(&[a], &b);
    let q = points(&b);
    assert_eq!((q.len(), q[2].color), (3, Some([0, 0, 65535])));
}
