//! cargo run --release --example mesh -- model.f3d [--json]
use std::{collections::BTreeMap, io};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let file = f3d::FusionFile::open(args.first().ok_or("usage: mesh FILE [--json]")?)?;
    let mut meshes = Vec::new();
    for document in file.documents() {
        for brep in &document.breps {
            let report = f3d::mesh::tessellate(&brep.model, &Default::default())?;
            let mut reasons = BTreeMap::<&str, usize>::new();
            for issue in &report.issues {
                *reasons.entry(&issue.reason).or_default() += 1;
            }
            eprintln!(
                "{}: {} triangles; {} faces meshed, {} skipped; {:?}",
                brep.path,
                report.triangles.len(),
                report.faces_meshed,
                report.faces_skipped,
                reasons
            );
            if args.iter().any(|arg| arg == "--json") {
                meshes.push(serde_json::json!({"path": brep.path, "mesh": report}));
            }
        }
    }
    if !meshes.is_empty() {
        serde_json::to_writer(io::stdout().lock(), &meshes)?;
    }
    Ok(())
}
