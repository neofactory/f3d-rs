//! cargo run --example machine_pose -- machine.mch X=100 Y=50 Z=-100 A=90 C=1080
use std::collections::BTreeMap;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .ok_or("usage: machine_pose FILE.mch [AXIS=VALUE ...]")?;
    let machine = f3d::machine::MachineDefinition::parse(&std::fs::read(path)?)?;
    let mut values = BTreeMap::new();
    for arg in args {
        let (axis, value) = arg.split_once('=').ok_or("use AXIS=VALUE")?;
        if values
            .insert(axis.to_owned(), value.parse::<f64>()?)
            .is_some()
        {
            return Err("duplicate axis argument".into());
        }
    }
    serde_json::to_writer_pretty(
        std::io::stdout().lock(),
        &serde_json::json!({
            "machine":machine.name(),"coordinates":values,"length_unit":machine.kinematics.units.get("length"),
            "world_transforms":machine.pose(&values)?,"spindle_attachment_in_table":machine.spindle_in_table(&values)?,
        }),
    )?;
    Ok(())
}
