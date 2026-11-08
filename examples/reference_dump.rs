//! Diagnostic JSON used by the optional Python differential validator.
use f3d::{
    FusionFile,
    geometry::{Curve, Surface},
};
use serde_json::{Value, json};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = FusionFile::open(std::env::args().nth(1).ok_or("provide F3D path")?)?;
    let mut checks = Vec::new();
    for doc in file.documents() {
        for brep in &doc.breps {
            let curves: Vec<Value> = brep
                .model
                .entities
                .iter()
                .filter(|e| e.base() == "curve")
                .filter_map(|e| {
                    Curve::from_entity(e)
                        .ok()
                        .map(|c| json!({"entity":e.index,"point":c.point_at(0.37)}))
                })
                .collect();
            let surfaces: Vec<Value> = brep
                .model
                .entities
                .iter()
                .filter(|e| e.base() == "surface")
                .filter_map(|e| {
                    Surface::from_entity(e)
                        .ok()
                        .map(|s| json!({"entity":e.index,"point":s.point_at(0.37,0.23)}))
                })
                .collect();
            checks.push(json!({"path":brep.path,"counts":brep.model.counts(),"census":brep.model.census(),"bounds":brep.model.vertex_bounds(),"live_faces":brep.model.live_faces().len(),"live_edges":brep.model.live_edges().len(),"curves":curves,"surfaces":surfaces}));
        }
    }
    serde_json::to_writer(
        std::io::BufWriter::new(std::io::stdout().lock()),
        &json!({"file":file,"checks":checks}),
    )?;
    Ok(())
}
