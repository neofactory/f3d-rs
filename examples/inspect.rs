//! cargo run --example inspect -- machine.f3d [--json | --summary-json]
use std::io::{self, Write};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .ok_or("usage: inspect FILE [--json | --summary-json]")?;
    let file = f3d::FusionFile::open(path)?;
    let mut output = io::BufWriter::new(io::stdout().lock());
    if args.iter().any(|a| a == "--json") {
        serde_json::to_writer_pretty(&mut output, &file)?;
        writeln!(output)?;
    } else if args.iter().any(|a| a == "--summary-json") {
        serde_json::to_writer_pretty(
            &mut output,
            &file
                .documents()
                .into_iter()
                .map(|d| d.summary())
                .collect::<Vec<_>>(),
        )?;
        writeln!(output)?;
    } else {
        for document in file.documents() {
            let s = document.summary();
            writeln!(
                output,
                "{}: {} B-Rep streams, {} body records, {} face records, {} segments, {} component bindings",
                s.name,
                s.brep_streams,
                s.body_records,
                s.face_records,
                s.segments,
                s.component_bindings
            )?;
            for issue in s.issues {
                writeln!(
                    output,
                    "  {} [{}]: {}",
                    issue.path, issue.stage, issue.message
                )?;
            }
        }
    }
    Ok(())
}
