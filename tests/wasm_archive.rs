//! Exercise the portable decoder natively as well as in the browser build.
use f3d::{Limits, archive::Archive};
use std::io::{Cursor, Write};
use zip::{CompressionMethod, write::SimpleFileOptions};

// A standard Zstandard single-segment frame with a five-byte raw block.
// Kept literal so these tests do not depend on a native Zstandard encoder.
const HELLO: &[u8] = b"\x28\xb5\x2f\xfd\x20\x05\x29\x00\x00hello";

fn archive(compressed: &[u8], decoded: &[u8], declared_size: u32) -> Vec<u8> {
    let mut writer = zip::ZipWriter::new(Cursor::new(Vec::new()));
    writer
        .start_file(
            "asset",
            SimpleFileOptions::default().compression_method(CompressionMethod::Stored),
        )
        .unwrap();
    writer.write_all(compressed).unwrap();
    let mut zip = writer.finish().unwrap().into_inner();
    let central = zip.windows(4).rposition(|x| x == b"PK\x01\x02").unwrap();
    for start in [8, central + 10] {
        zip[start..start + 2].copy_from_slice(&93u16.to_le_bytes());
    }
    for start in [14, central + 16] {
        zip[start..start + 4].copy_from_slice(&crc32fast::hash(decoded).to_le_bytes());
    }
    for start in [22, central + 24] {
        zip[start..start + 4].copy_from_slice(&declared_size.to_le_bytes());
    }
    zip
}

#[test]
fn portable_zstd_decodes_frames_separated_by_skippable_metadata() {
    let mut stream = HELLO.to_vec();
    stream.extend(b"\x50\x2a\x4d\x18\x03\x00\x00\x00tag");
    stream.extend(HELLO);
    let result = Archive::read(
        Cursor::new(archive(&stream, b"hellohello", 10)),
        &Limits::default(),
    )
    .unwrap();
    assert_eq!(result.bytes("asset").unwrap(), b"hellohello");
    assert_eq!(result.entries[0].compression, "Zstd");
}

#[test]
fn portable_zstd_rejects_corruption_truncation_and_false_sizes() {
    let mut corrupt = HELLO.to_vec();
    *corrupt.last_mut().unwrap() ^= 1;
    for (stream, size) in [
        (corrupt.as_slice(), 5),
        (&HELLO[..HELLO.len() - 1], 5),
        (HELLO, 4),
        (HELLO, 6),
        (b"\x50\x2a\x4d\x18\xff\x00\x00\x00short", 0),
    ] {
        assert!(
            Archive::read(
                Cursor::new(archive(stream, b"hello", size)),
                &Limits::default(),
            )
            .is_err()
        );
    }
}

#[test]
fn portable_zstd_obeys_shared_archive_budget() {
    let limits = Limits {
        max_total_bytes: 4,
        ..Limits::default()
    };
    assert!(Archive::read(Cursor::new(archive(HELLO, b"hello", 5)), &limits).is_err());
}
