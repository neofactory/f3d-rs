//! Read documents and nested F3Z packages. Nested documents are kept separate;
//! cross-document reference graphs and assembly flattening are not inferred.
use crate::{
    Error, Limits, Result,
    archive::Archive,
    asm,
    machine::MachineDefinition,
    streams::{BodyBinding, Component, Manifest, PlacementCandidate, Segment},
};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{Cursor, Read, Seek},
    path::Path,
};

#[derive(Debug, Serialize)]
pub struct Brep {
    pub path: String,
    pub model: asm::Model,
}
#[derive(Debug, Serialize)]
pub struct Machine {
    pub path: String,
    pub definition: MachineDefinition,
}
#[derive(Debug, Clone, Serialize)]
pub struct Issue {
    pub path: String,
    pub stage: String,
    pub message: String,
}
#[derive(Debug, Serialize)]
pub struct Assembly {
    pub segment: String,
    pub components: Vec<Component>,
    pub body_bindings: Vec<BodyBinding>,
    pub placements: Vec<PlacementCandidate>,
}
#[derive(Debug, Serialize)]
pub struct Document {
    pub name: String,
    pub archive: Archive,
    pub manifest: Option<Manifest>,
    pub segments: Vec<Segment>,
    pub breps: Vec<Brep>,
    pub machines: Vec<Machine>,
    pub assemblies: Vec<Assembly>,
    pub issues: Vec<Issue>,
}
impl Document {
    fn from_archive(name: String, archive: Archive, limits: &Limits) -> Result<Self> {
        let mut doc = Self {
            name,
            archive,
            manifest: None,
            segments: Vec::new(),
            breps: Vec::new(),
            machines: Vec::new(),
            assemblies: Vec::new(),
            issues: Vec::new(),
        };
        let mut segment_dirs = BTreeMap::<String, (bool, bool)>::new();
        for entry in &doc.archive.entries {
            if entry.name == "Manifest.dat" {
                match Manifest::parse(&entry.data, limits) {
                    Ok(m) => doc.manifest = Some(m),
                    Err(e) => doc.issues.push(Issue {
                        path: entry.name.clone(),
                        stage: "manifest".into(),
                        message: e.to_string(),
                    }),
                }
            }
            if entry.name.ends_with(".smb") || entry.name.ends_with(".smbh") {
                let model =
                    asm::Model::parse_with_limits(&entry.data, limits).map_err(|source| {
                        Error::Entry {
                            path: entry.name.clone(),
                            source: Box::new(source),
                        }
                    })?;
                doc.breps.push(Brep {
                    path: entry.name.clone(),
                    model,
                });
            }
            if entry.name.ends_with(".mch") {
                match MachineDefinition::parse(&entry.data) {
                    Ok(definition) => doc.machines.push(Machine {
                        path: entry.name.clone(),
                        definition,
                    }),
                    Err(e) => doc.issues.push(Issue {
                        path: entry.name.clone(),
                        stage: "machine".into(),
                        message: e.to_string(),
                    }),
                }
            }
            if let Some(dir) = entry.name.strip_suffix("/MetaStream.dat") {
                segment_dirs.entry(dir.into()).or_default().0 = true;
            }
            if let Some(dir) = entry.name.strip_suffix("/BulkStream.dat") {
                segment_dirs.entry(dir.into()).or_default().1 = true;
            }
        }
        for (dir, (meta, bulk)) in segment_dirs {
            if !meta || !bulk {
                doc.issues.push(Issue {
                    path: dir,
                    stage: "segment".into(),
                    message: "missing stream pair".into(),
                });
                continue;
            }
            match Segment::parse(
                &dir,
                doc.archive.bytes(&(dir.clone() + "/MetaStream.dat"))?,
                doc.archive.bytes(&(dir.clone() + "/BulkStream.dat"))?,
                limits,
            ) {
                Ok(s) => {
                    if s.is_design() {
                        let components = s.components();
                        let body_bindings = s.body_bindings();
                        let placements = s.placement_candidates();
                        let raw_refs = s.body_blob_names();
                        for name in raw_refs {
                            if !body_bindings.iter().any(|b| b.blob == name) {
                                doc.issues.push(Issue {
                                    path: dir.clone(),
                                    stage: "body_binding".into(),
                                    message: format!("no supported ownership binding for {name}"),
                                });
                            }
                        }
                        if !placements.is_empty() {
                            doc.issues.push(Issue{path:dir.clone(),stage:"placement".into(),message:"placements are uniquely validated rigid-matrix candidates; full occurrence grammar not decoded".into()});
                        }
                        doc.assemblies.push(Assembly {
                            segment: dir,
                            components,
                            body_bindings,
                            placements,
                        });
                    }
                    doc.segments.push(s);
                }
                Err(e) => doc.issues.push(Issue {
                    path: dir,
                    stage: "segment".into(),
                    message: e.to_string(),
                }),
            }
        }
        Ok(doc)
    }
    pub fn summary(&self) -> Summary {
        Summary {
            name: self.name.clone(),
            entries: self.archive.entries.len(),
            brep_streams: self.breps.len(),
            body_records: self.breps.iter().map(|b| b.model.census().bodies()).sum(),
            face_records: self.breps.iter().map(|b| b.model.census().faces()).sum(),
            entities: self.breps.iter().map(|b| b.model.entities.len()).sum(),
            segments: self.segments.len(),
            machines: self.machines.len(),
            component_bindings: self.assemblies.iter().map(|a| a.body_bindings.len()).sum(),
            issues: self.issues.clone(),
        }
    }
}
#[derive(Debug, Serialize)]
pub struct Summary {
    pub name: String,
    pub entries: usize,
    pub brep_streams: usize,
    pub body_records: usize,
    pub face_records: usize,
    pub entities: usize,
    pub segments: usize,
    pub machines: usize,
    pub component_bindings: usize,
    pub issues: Vec<Issue>,
}

#[derive(Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FusionFile {
    Document(Box<Document>),
    Package {
        archive: Archive,
        members: BTreeMap<String, FusionFile>,
    },
}
impl FusionFile {
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        Self::open_with_limits(path, &Limits::default())
    }
    pub fn open_with_limits(path: impl AsRef<Path>, limits: &Limits) -> Result<Self> {
        let path = path.as_ref();
        let mut budget = limits.max_total_bytes;
        Self::read_internal(
            path.file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned(),
            File::open(path)?,
            limits,
            &mut budget,
            0,
        )
    }
    pub fn read<R: Read + Seek>(reader: R, limits: &Limits) -> Result<Self> {
        let mut budget = limits.max_total_bytes;
        Self::read_internal("memory.f3d".into(), reader, limits, &mut budget, 0)
    }
    fn read_internal<R: Read + Seek>(
        name: String,
        reader: R,
        limits: &Limits,
        budget: &mut u64,
        depth: usize,
    ) -> Result<Self> {
        if depth > limits.max_package_depth {
            return Err(Error::Limit("nested package depth".into()));
        }
        let archive = Archive::read_budgeted(reader, limits, budget)?;
        let nested: Vec<_> = archive
            .entries
            .iter()
            .filter(|e| {
                e.name.to_ascii_lowercase().ends_with(".f3d")
                    || e.name.to_ascii_lowercase().ends_with(".f3z")
            })
            .map(|e| e.name.clone())
            .collect();
        if archive.get("Manifest.dat").is_none() && !nested.is_empty() {
            let mut members = BTreeMap::new();
            for key in nested {
                let doc = Self::read_internal(
                    key.clone(),
                    Cursor::new(archive.bytes(&key)?),
                    limits,
                    budget,
                    depth + 1,
                )?;
                members.insert(key, doc);
            }
            Ok(Self::Package { archive, members })
        } else {
            if archive.get("Manifest.dat").is_none() {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: "ZIP has neither a Fusion manifest nor packaged Fusion documents"
                        .into(),
                });
            }
            Ok(Self::Document(Box::new(Document::from_archive(
                name, archive, limits,
            )?)))
        }
    }
    pub fn documents(&self) -> Vec<&Document> {
        match self {
            Self::Document(d) => vec![d],
            Self::Package { members, .. } => members.values().flat_map(Self::documents).collect(),
        }
    }
}
