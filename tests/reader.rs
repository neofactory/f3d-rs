use f3d::{
    Error, FusionFile, Limits,
    archive::Archive,
    asm::{self, tag},
    math::{Mat4, norm, sub},
};
use std::io::{Cursor, Write};
use zip::{CompressionMethod, write::SimpleFileOptions};

fn text(tag: u8, s: &str, out: &mut Vec<u8>) {
    out.extend([tag, s.len() as u8]);
    out.extend(s.as_bytes());
}
fn real(x: f64, out: &mut Vec<u8>) {
    out.push(tag::DOUBLE);
    out.extend(x.to_le_bytes());
}
fn vector(tag: u8, p: [f64; 3], out: &mut Vec<u8>) {
    out.push(tag);
    for x in p {
        out.extend(x.to_le_bytes());
    }
}
fn word(tag: u8, value: i64, width: usize, out: &mut Vec<u8>) {
    out.push(tag);
    out.extend(&value.to_le_bytes()[..width]);
}
fn header(width: usize) -> Vec<u8> {
    let mut out = format!("ASM BinaryFile{width}").into_bytes();
    for n in [23100u64, 0, 0, 0] {
        out.extend(&n.to_le_bytes()[..width]);
    }
    for s in ["f3d-rs synthetic fixture", "ASM 231.0", "test"] {
        text(tag::STR, s, &mut out);
    }
    for x in [10., 1e-6, 1e-10] {
        real(x, &mut out);
    }
    out
}
fn types(names: &[&str], out: &mut Vec<u8>) {
    for (i, s) in names.iter().enumerate() {
        text(
            if i + 1 == names.len() {
                tag::TYPE
            } else {
                tag::TYPE_EX
            },
            s,
            out,
        );
    }
}
fn finish(out: &mut Vec<u8>) {
    types(&["End", "of", "ASM", "data"], out);
}
fn sample(width: usize) -> Vec<u8> {
    let mut out = header(width);
    types(&["straight", "curve"], &mut out);
    word(tag::POINTER, -1, width, &mut out);
    word(tag::INT, 0, width, &mut out);
    word(tag::POINTER, -1, width, &mut out);
    vector(tag::POSITION, [1., 2., 3.], &mut out);
    vector(tag::DIRECTION, [0., 2., 0.], &mut out);
    out.push(tag::RECORD_END);
    finish(&mut out);
    out
}
fn zip(entries: &[(&str, &[u8])], method: CompressionMethod) -> Vec<u8> {
    let mut zip = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for (name, data) in entries {
        zip.start_file(
            *name,
            SimpleFileOptions::default().compression_method(method),
        )
        .unwrap();
        zip.write_all(data).unwrap();
    }
    zip.finish().unwrap().into_inner()
}

#[test]
fn asm_32_and_64_bit_fields_and_geometry() {
    for width in [4, 8] {
        let model = asm::Model::parse(&sample(width)).unwrap();
        assert_eq!(model.header.word_size, width);
        assert_eq!(model.entities.len(), 1);
        assert!(model.terminated);
        assert_eq!(model.entities[0].pointers().collect::<Vec<_>>(), [-1, -1]);
        let curve = f3d::geometry::Curve::from_entity(&model.entities[0]).unwrap();
        assert_eq!(curve.point_at(2.), [1., 4., 3.]);
        assert_eq!(curve.invert([1., 4., 3.]), 2.);
    }
}
#[test]
fn every_truncated_prefix_fails_without_panicking() {
    let data = sample(8);
    for end in 0..data.len() {
        assert!(asm::Model::parse(&data[..end]).is_err(), "prefix {end}");
    }
}
#[test]
fn unknown_tags_and_malformed_lengths_are_errors() {
    let mut data = header(4);
    data.push(255);
    assert!(matches!(
        asm::Model::parse(&data),
        Err(Error::Invalid { .. })
    ));
    let mut data = header(4);
    data.extend([tag::STR2, 255, 255]);
    assert!(asm::Model::parse(&data).is_err());
    let mut data = header(4);
    data.push(tag::LITERAL);
    data.extend(u32::MAX.to_le_bytes());
    assert!(asm::Model::parse(&data).is_err());
}
#[test]
fn nonfinite_asm_tokens_keep_their_bits_in_diagnostic_json() {
    let nan = f64::from_bits(0x7ff8000000000042);
    let value = asm::Value::Real(nan);
    assert_eq!(
        serde_json::to_string(&value).unwrap(),
        "\"f64:0x7ff8000000000042\""
    );
    let value = asm::Value::Vector([1., f64::INFINITY, f64::NEG_INFINITY]);
    assert_eq!(
        serde_json::to_value(value).unwrap(),
        serde_json::json!([1., "f64:0x7ff0000000000000", "f64:0xfff0000000000000"])
    );
}
#[test]
fn subtype_nesting_and_limits_are_enforced() {
    let mut data = header(4);
    types(&["spline", "surface"], &mut data);
    data.extend([tag::START, tag::START, tag::END, tag::END, tag::RECORD_END]);
    finish(&mut data);
    assert!(asm::Model::parse(&data).is_ok());
    let limits = Limits {
        max_subtype_depth: 1,
        ..Limits::default()
    };
    assert!(matches!(
        asm::Model::parse_with_limits(&data, &limits),
        Err(Error::Limit(_))
    ));
    let mut data = header(4);
    data.push(tag::END);
    assert!(asm::Model::parse(&data).is_err());
    let limits = Limits {
        max_tokens: 1,
        ..Limits::default()
    };
    assert!(matches!(
        asm::Model::parse_with_limits(&sample(4), &limits),
        Err(Error::Limit(_))
    ));
}
#[test]
fn history_does_not_shift_main_pointer_addresses() {
    let mut data = header(4);
    types(&["body"], &mut data);
    word(tag::POINTER, 1, 4, &mut data);
    data.push(tag::RECORD_END);
    types(
        &["Begin", "of", "ASM", "History", "Data", "delta_state"],
        &mut data,
    );
    data.push(tag::RECORD_END);
    types(
        &["End", "of", "ASM", "History", "Section", "point"],
        &mut data,
    );
    vector(tag::POSITION, [2., 3., 4.], &mut data);
    data.push(tag::RECORD_END);
    finish(&mut data);
    let model = asm::Model::parse(&data).unwrap();
    assert!(model.has_history);
    assert_eq!(model.history.len(), 1);
    assert_eq!(model.entities.len(), 2);
    assert_eq!(model.resolve(1).unwrap().name(), "point");
    assert_eq!(model.resolve(1).unwrap().source_record, 2);
}
#[test]
fn unterminated_entities_and_trailing_data_are_not_accepted() {
    let mut data = header(4);
    types(&["point"], &mut data);
    assert!(asm::Model::parse(&data).is_err());
    let mut data = sample(4);
    data.extend([tag::RECORD_END, 255]);
    assert!(asm::Model::parse(&data).is_err());
    let mut data = header(4);
    types(&["point"], &mut data);
    data.push(tag::RECORD_END);
    assert!(asm::Model::parse(&data).is_err());
}
#[test]
fn topology_ring_cycles_terminate_and_tolerant_classes_are_traversable() {
    let mut data = header(4);
    let nodes: [(&[&str], &[i64]); 9] = [
        (&["body"], &[-1, -1, 1]),
        (&["lump"], &[-1, -1, -1, 2]),
        (&["shell"], &[-1, -1, -1, -1, 3]),
        (&["face"], &[-1, -1, 3, 4]),
        (&["loop"], &[-1, -1, 4, 5]),
        (&["tcoedge", "coedge"], &[-1, -1, 5, -1, 5, 6]),
        (&["tedge", "edge"], &[-1, -1, 7, 7, -1, -1]),
        (&["tvertex", "vertex"], &[-1, -1, -1, 8]),
        (&["point"], &[]),
    ];
    for (names, pointers) in nodes {
        types(names, &mut data);
        for &p in pointers {
            word(tag::POINTER, p, 4, &mut data);
        }
        if names == ["point"] {
            vector(tag::POSITION, [1., 2., 3.], &mut data);
        }
        data.push(tag::RECORD_END);
    }
    finish(&mut data);
    let model = asm::Model::parse(&data).unwrap();
    let faces = model.live_faces();
    assert_eq!(faces.len(), 1);
    let edges = model.live_edges();
    assert_eq!(edges.len(), 1);
    assert_eq!(edges[0].start().unwrap().position(), Some([1., 2., 3.]));
}
#[test]
fn zip_stored_deflate_and_zstandard_round_trip() {
    for method in [
        CompressionMethod::Stored,
        CompressionMethod::Deflated,
        CompressionMethod::Zstd,
    ] {
        let raw = zip(&[("folder/[Active]/test", b"payload")], method);
        let archive = Archive::read(Cursor::new(raw), &Limits::default()).unwrap();
        assert_eq!(archive.bytes("folder/[Active]/test").unwrap(), b"payload");
    }
}
#[test]
fn zip_limits_and_unsafe_paths_fail() {
    let raw = zip(&[("test", b"123456789")], CompressionMethod::Deflated);
    let limits = Limits {
        max_entry_bytes: 4,
        ..Limits::default()
    };
    assert!(matches!(
        Archive::read(Cursor::new(raw), &limits),
        Err(Error::Limit(_))
    ));
    let raw = zip(&[("../escape", b"x")], CompressionMethod::Stored);
    assert!(Archive::read(Cursor::new(raw), &Limits::default()).is_err());
}
#[test]
fn zip_crc_is_checked() {
    let mut raw = zip(&[("test", b"UNIQUE_PAYLOAD")], CompressionMethod::Stored);
    let pos = raw
        .windows(14)
        .position(|w| w == b"UNIQUE_PAYLOAD")
        .unwrap();
    raw[pos] ^= 1;
    assert!(Archive::read(Cursor::new(raw), &Limits::default()).is_err());
}
#[test]
fn concatenated_zstd_frames_are_fully_decoded() {
    let payload = b"first frame + second frame";
    let mut compressed = zstd::encode_all(&payload[..14], 1).unwrap();
    compressed.extend(zstd::encode_all(&payload[14..], 1).unwrap());
    let mut raw = zip(&[("frames", &compressed)], CompressionMethod::Stored);
    fn crc32(data: &[u8]) -> u32 {
        let mut c = !0u32;
        for b in data {
            c ^= *b as u32;
            for _ in 0..8 {
                c = (c >> 1) ^ if c & 1 != 0 { 0xedb88320 } else { 0 };
            }
        }
        !c
    }
    let central = raw.windows(4).rposition(|w| w == b"PK\x01\x02").unwrap();
    raw[8..10].copy_from_slice(&93u16.to_le_bytes());
    raw[14..18].copy_from_slice(&crc32(payload).to_le_bytes());
    raw[22..26].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    raw[central + 10..central + 12].copy_from_slice(&93u16.to_le_bytes());
    raw[central + 16..central + 20].copy_from_slice(&crc32(payload).to_le_bytes());
    raw[central + 24..central + 28].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    let archive = Archive::read(Cursor::new(raw), &Limits::default()).unwrap();
    assert_eq!(archive.bytes("frames").unwrap(), payload);
}

fn str8(s: &str, out: &mut Vec<u8>) {
    out.extend((s.len() as u32).to_le_bytes());
    out.extend(s.as_bytes());
}
fn wstr(s: &str, out: &mut Vec<u8>) {
    let v: Vec<_> = s.encode_utf16().collect();
    out.extend((v.len() as u32).to_le_bytes());
    for x in v {
        out.extend(x.to_le_bytes());
    }
}
#[test]
fn manifest_prefix_retains_unknown_tail_instead_of_guessing_its_revision() {
    let mut data = Vec::new();
    str8("3-2-0-0", &mut data);
    str8("FusionDocType", &mut data);
    for s in [
        ".f3d",
        "Fusion Document",
        "description",
        "document-guid",
        "lineage-guid",
    ] {
        wstr(s, &mut data);
    }
    data.extend([0; 16]);
    data.extend(1u32.to_le_bytes());
    str8("Fusion", &mut data);
    data.extend(335u32.to_le_bytes());
    data.extend([255; 7]);
    let m = f3d::streams::Manifest::parse(&data, &Limits::default()).unwrap();
    assert_eq!(m.schema["Fusion"], 335);
    assert_eq!(m.unparsed_tail_bytes, 7);
}
#[test]
fn segment_object_index_is_bounded_and_unknown_payloads_remain_readable() {
    let mut meta = Vec::new();
    str8("Design", &mut meta);
    meta.extend(0u32.to_le_bytes());
    wstr("", &mut meta);
    meta.extend([0; 16]);
    str8("Design", &mut meta);
    str8("Fusion", &mut meta);
    meta.extend([0; 12]);
    meta.extend(0u32.to_le_bytes()); // no types or roots
    meta.extend(1u32.to_le_bytes());
    meta.extend(5u64.to_le_bytes());
    meta.extend(0u64.to_le_bytes());
    meta.extend(0u32.to_le_bytes());
    meta.extend(6u64.to_le_bytes());
    meta.extend(0u32.to_le_bytes());
    let legacy_meta = meta.clone();
    meta.extend(2u32.to_le_bytes());
    for s in ["Application", "Server"] {
        str8(s, &mut meta);
        meta.extend(1u32.to_le_bytes());
    }
    let mut bulk = Vec::new();
    str8("335", &mut bulk);
    bulk.extend(0u64.to_le_bytes());
    bulk.extend(b"native payload");
    let segment =
        f3d::streams::Segment::parse("Asset/Design1", &meta, &bulk, &Limits::default()).unwrap();
    assert_eq!(segment.object(5), Some(b"native payload".as_slice()));
    assert_eq!(segment.next_id, 6);
    assert_eq!(segment.schema["Server"], 1);
    let legacy =
        f3d::streams::Segment::parse("Asset/Design1", &legacy_meta, &bulk, &Limits::default())
            .unwrap();
    assert!(legacy.schema.is_empty());
    assert_eq!(legacy.object(5), Some(b"native payload".as_slice()));
    for n in 0..meta.len() / 2 {
        assert!(
            f3d::streams::Segment::parse("bad", &meta[..n], &bulk, &Limits::default()).is_err()
        );
    }
}
#[test]
fn nested_f3z_documents_remain_separate_and_share_byte_budget() {
    // A malformed manifest is retained with an explicit issue, not mistaken
    // for a geometry parse failure or silently thrown away.
    let inner = zip(
        &[
            ("Manifest.dat", b"unknown revision"),
            ("Asset/Breps.BlobParts/a.smb", &sample(4)),
        ],
        CompressionMethod::Stored,
    );
    let outer = zip(
        &[("root.f3d", &inner), ("child.f3d", &inner)],
        CompressionMethod::Deflated,
    );
    let file = FusionFile::read(Cursor::new(&outer), &Limits::default()).unwrap();
    assert_eq!(file.documents().len(), 2);
    assert_eq!(file.documents()[0].breps.len(), 1);
    assert_eq!(file.documents()[0].issues[0].stage, "manifest");
    let limits = Limits {
        max_total_bytes: (inner.len() * 2) as u64,
        ..Limits::default()
    };
    assert!(matches!(
        FusionFile::read(Cursor::new(outer), &limits),
        Err(Error::Limit(_))
    ));
}
#[test]
fn proper_rigid_inverse_and_negative_axis_rotation() {
    let m = Mat4::rotate_about([-1., 0., 0.], [-284., 0., 50.], 90f64.to_radians()).unwrap();
    let p = m.transform_point([0.; 3]);
    assert!(norm(sub(p, [0., -50., 50.])) < 1e-9);
    assert!(norm(sub(m.inverse_rigid().unwrap().transform_point(p), [0.; 3])) < 1e-9);
    let mut reflection = Mat4::IDENTITY;
    reflection.0[0][0] = -1.;
    assert!(!reflection.is_rigid(1e-9));
}

#[test]
fn embedded_machine_definition_remains_an_opaque_asset() {
    let payload = b"opaque machine payload";
    let bytes = zip(
        &[
            ("Manifest.dat", b"unknown revision"),
            ("Asset/Machine/model.mch", payload),
        ],
        CompressionMethod::Stored,
    );
    let file = FusionFile::read(Cursor::new(bytes), &Limits::default()).unwrap();
    let doc = file.documents()[0];
    assert_eq!(
        doc.archive.bytes("Asset/Machine/model.mch").unwrap(),
        payload
    );
    assert_eq!(doc.issues.len(), 1);
    assert_eq!(doc.issues[0].stage, "manifest");
    let summary = serde_json::to_value(doc.summary()).unwrap();
    assert!(summary.get("machines").is_none());
}
