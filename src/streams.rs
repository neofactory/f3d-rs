//! Indexed Fusion Design/ACT streams. Structural fields are decoded; native
//! object payloads are retained for schemas not interpreted by this crate.
//! The manifest reader deliberately retains its version-dependent tail.
use crate::{Error, Limits, Result, binary::Reader, math::Mat4};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Serialize)]
pub struct Manifest {
    pub format_version: String,
    pub doc_type_key: String,
    pub extension: String,
    pub doc_type: String,
    pub description: String,
    pub document_guid: String,
    pub lineage_guid: String,
    pub reserved: Vec<u32>,
    pub schema: BTreeMap<String, u32>,
    pub tail_offset: usize,
    pub unparsed_tail_bytes: usize,
}
impl Manifest {
    pub fn parse(data: &[u8], limits: &Limits) -> Result<Self> {
        let mut r = Reader::new(data, limits.max_string_bytes);
        let format_version = r.str8()?;
        let doc_type_key = r.str8()?;
        let extension = r.wstr()?;
        let doc_type = r.wstr()?;
        let description = r.wstr()?;
        let document_guid = r.wstr()?;
        let lineage_guid = r.wstr()?;
        let anchor = r.pos;
        for words in [4, 2, 6] {
            r.pos = anchor;
            let candidate = (|| -> Result<(Vec<u32>, BTreeMap<String, u32>)> {
                let reserved = (0..words).map(|_| r.u32()).collect::<Result<Vec<_>>>()?;
                let n = r.count(8)?;
                if n == 0 || n > 64 {
                    return Err(r.invalid("schema table count"));
                }
                let mut schema = BTreeMap::new();
                for _ in 0..n {
                    let name = r.str8()?;
                    let version = r.u32()?;
                    if !identifier(&name) || version >= 1 << 24 {
                        return Err(r.invalid("invalid schema table entry"));
                    }
                    if schema.insert(name, version).is_some() {
                        return Err(r.invalid("duplicate schema entry"));
                    }
                }
                Ok((reserved, schema))
            })();
            if let Ok((reserved, schema)) = candidate {
                return Ok(Self {
                    format_version,
                    doc_type_key,
                    extension,
                    doc_type,
                    description,
                    document_guid,
                    lineage_guid,
                    reserved,
                    schema,
                    tail_offset: r.pos,
                    unparsed_tail_bytes: r.remaining(),
                });
            }
        }
        Err(r.invalid("unrecognized manifest version/schema block"))
    }
}

fn identifier(s: &str) -> bool {
    !s.is_empty()
        && s.bytes()
            .enumerate()
            .all(|(i, b)| b == b'_' || b.is_ascii_alphabetic() || (i > 0 && b.is_ascii_digit()))
}

#[derive(Debug, Clone, Serialize)]
pub struct TypeRecord {
    pub identity: String,
    pub base: String,
    pub version: u32,
    pub owner: String,
    pub ids: Vec<u64>,
}
#[derive(Debug, Clone, Serialize)]
pub struct Object {
    pub id: u64,
    pub offset: usize,
    pub end: usize,
}
#[derive(Debug, Clone, Serialize)]
pub struct Segment {
    pub path: String,
    pub prefix: String,
    pub slot: u32,
    pub guid: String,
    pub reserved: Vec<u32>,
    pub declared_type: String,
    pub owner: String,
    pub records: Vec<TypeRecord>,
    pub roots: Vec<u64>,
    pub schema: BTreeMap<String, u32>,
    pub objects: Vec<Object>,
    pub next_id: u64,
    pub unread_meta_bytes: usize,
    pub bulk_version: String,
    pub bulk_flags: u64,
    #[serde(skip)]
    pub bulk: Vec<u8>,
}

fn footer(data: &[u8], limits: &Limits) -> (usize, BTreeMap<String, u32>) {
    let needle = b"\x0b\0\0\0Application";
    for pos in (4..data.len().saturating_sub(needle.len())).rev() {
        if !data[pos..].starts_with(needle) {
            continue;
        }
        let mut r = Reader::new(data, limits.max_string_bytes);
        r.pos = pos - 4;
        let parsed = (|| -> Result<BTreeMap<String, u32>> {
            if r.u32()? != 2 {
                return Err(r.invalid("footer count"));
            }
            let mut out = BTreeMap::new();
            for expected in ["Application", "Server"] {
                let name = r.str8()?;
                if name != expected {
                    return Err(r.invalid("footer name"));
                }
                out.insert(name, r.u32()?);
            }
            if r.remaining() != 0 {
                return Err(r.invalid("footer trailing data"));
            }
            Ok(out)
        })();
        if let Ok(schema) = parsed {
            return (pos - 4, schema);
        }
    }
    (data.len(), BTreeMap::new())
}

impl Segment {
    pub fn parse(
        path: impl Into<String>,
        meta: &[u8],
        bulk: &[u8],
        limits: &Limits,
    ) -> Result<Self> {
        let mut r = Reader::new(meta, limits.max_string_bytes);
        let prefix = r.str8()?;
        let slot = r.u32()?;
        let guid = r.wstr()?;
        let anchor = r.pos;
        let mut version = None;
        for words in [4, 2, 6] {
            r.pos = anchor;
            let parsed = (|| -> Result<_> {
                let reserved = (0..words).map(|_| r.u32()).collect::<Result<Vec<_>>>()?;
                let kind = r.str8()?;
                let owner = r.str8()?;
                if !identifier(&kind) || (!owner.is_empty() && !identifier(&owner)) {
                    return Err(r.invalid("segment type"));
                }
                Ok((reserved, kind, owner))
            })();
            if let Ok(v) = parsed {
                version = Some(v);
                break;
            }
        }
        let (reserved, declared_type, owner) =
            version.ok_or_else(|| r.invalid("unrecognized segment version"))?;
        let body = r.data.get(r.pos..).ok_or_else(|| r.invalid("meta body"))?;
        let (end, schema) = footer(body, limits);
        // Older (including the Brother) streams end after the index trailer
        // and have no Application/Server footer.
        let mut m = Reader::new(&body[..end], limits.max_string_bytes);
        m.take(8)?;
        let count = m.count(20)?;
        if count > limits.max_records {
            return Err(Error::Limit("meta type records".into()));
        }
        let mut records = Vec::new();
        for _ in 0..count {
            let identity = m.str8()?;
            let base = m.str8()?;
            let version = m.u32()?;
            let owner = m.str8()?;
            let n = m.count(8)?;
            let ids = (0..n).map(|_| m.u64()).collect::<Result<Vec<_>>>()?;
            records.push(TypeRecord {
                identity,
                base,
                version,
                owner,
                ids,
            });
        }
        let n = m.count(8)?;
        let roots = (0..n).map(|_| m.u64()).collect::<Result<Vec<_>>>()?;
        let mut index = BTreeMap::new();
        let mut saw_index = false;
        while m.remaining() >= 4 {
            if saw_index && m.remaining() == 12 {
                break;
            }
            let mut probe = m.clone();
            let n = probe.u32()? as usize;
            if n > probe.remaining() / 16 {
                break;
            }
            for _ in 0..n {
                let id = probe.u64()?;
                let off = probe.u64()?;
                index.insert(
                    id,
                    usize::try_from(off).map_err(|_| probe.invalid("bulk offset overflow"))?,
                );
            }
            saw_index = true;
            m = probe;
            if n == 0 {
                break;
            }
        }
        if !saw_index {
            return Err(m.invalid("missing or truncated object index"));
        }
        let next_id = m.u64()?;
        if m.u32()? != 0 {
            return Err(m.invalid("invalid object-index closing lane"));
        }
        let unread_meta_bytes = m.remaining();
        let mut b = Reader::new(bulk, limits.max_string_bytes);
        let bulk_version = b.str8()?;
        let bulk_flags = b.u64()?;
        let data = b.take(b.remaining())?.to_vec();
        let mut ordered: Vec<_> = index.into_iter().collect();
        ordered.sort_by_key(|(_, off)| *off);
        if ordered.len() > limits.max_records {
            return Err(Error::Limit("bulk object count".into()));
        }
        let mut objects = Vec::new();
        for (i, &(id, offset)) in ordered.iter().enumerate() {
            let mut end = ordered.get(i + 1).map(|x| x.1).unwrap_or(data.len());
            if offset > end || end > data.len() || (i > 0 && offset == ordered[i - 1].1) {
                return Err(Error::Invalid {
                    offset,
                    reason: "invalid bulk object index".into(),
                });
            }
            // Index offsets point after the 15-byte indexed-record header in
            // this grammar. Exclude the next record's header only if it closes
            // with that next indexed object's exact ID.
            if let Some(&(next_id, next_offset)) = ordered.get(i + 1)
                && next_offset >= 15
            {
                let h = &data[next_offset - 15..next_offset];
                if h[..4] == 3u32.to_le_bytes()
                    && h[4..7].iter().all(u8::is_ascii_digit)
                    && u64::from_le_bytes(h[7..15].try_into().unwrap()) == next_id
                    && next_offset - 15 >= offset
                {
                    end -= 15;
                }
            }
            objects.push(Object { id, offset, end });
        }
        objects.sort_by_key(|o| o.id);
        Ok(Self {
            path: path.into(),
            prefix,
            slot,
            guid,
            reserved,
            declared_type,
            owner,
            records,
            roots,
            schema,
            objects,
            next_id,
            unread_meta_bytes,
            bulk_version,
            bulk_flags,
            bulk: data,
        })
    }
    pub fn object(&self, id: u64) -> Option<&[u8]> {
        let i = self.objects.binary_search_by_key(&id, |o| o.id).ok()?;
        let o = &self.objects[i];
        self.bulk.get(o.offset..o.end)
    }
    pub fn type_for(&self, id: u64) -> Option<&TypeRecord> {
        self.records.iter().find(|r| r.ids.contains(&id))
    }
    pub fn is_design(&self) -> bool {
        self.declared_type.contains("Design")
    }
    pub fn body_blob_names(&self) -> BTreeSet<String> {
        scan_strings(&self.bulk)
            .into_iter()
            .filter(|s| {
                s.value.starts_with("BREP.")
                    && (s.value.ends_with(".smb") || s.value.ends_with(".smbh"))
            })
            .map(|s| s.value)
            .collect()
    }
    pub fn components(&self) -> Vec<Component> {
        let mut result = Vec::new();
        let Some(root) = self
            .roots
            .iter()
            .filter_map(|id| self.object(*id))
            .find(|raw| {
                let mut r = Reader::new(raw, 512);
                r.str8().is_ok_and(|s| s == "ComponentsRoot")
            })
        else {
            return result;
        };
        let known: BTreeSet<_> = self.objects.iter().map(|o| o.id).collect();
        let mut seen = BTreeSet::new();
        for id in references(root).filter(|id| known.contains(id) && seen.insert(*id)) {
            let Some(raw) = self.object(id) else {
                continue;
            };
            let strings = scan_strings(raw);
            let Some(name) = strings
                .iter()
                .rev()
                .find(|s| s.wide)
                .map(|s| s.value.clone())
            else {
                continue;
            };
            result.push(Component { id, name });
        }
        result.sort_by_key(|c| c.id);
        result
    }
    pub fn body_bindings(&self) -> Vec<BodyBinding> {
        let components = self.components();
        let mut out = Vec::new();
        for item in &self.objects {
            let Some(raw) = self.object(item.id) else {
                continue;
            };
            let Some(name) = scan_strings(raw)
                .into_iter()
                .find(|s| s.value.starts_with("BREP.") && s.value.ends_with(".smb"))
            else {
                continue;
            };
            let parsed = (|| -> Result<BodyBinding> {
                let mut r = Reader::new(raw, 1024);
                if r.take(6)? != [0; 6] || r.u8()? != 1 {
                    return Err(r.invalid("snapshot envelope"));
                }
                let companion = r.u64()?;
                if companion != item.id + 1 || r.take(2)? != [0; 2] {
                    return Err(r.invalid("snapshot companion"));
                }
                let n = r.count(16)?;
                let mut selectors = Vec::new();
                for _ in 0..n {
                    selectors.push([r.u64()?, r.u64()?]);
                }
                let container_id = local_reference(&mut r)?;
                r.take(1)?;
                if r.wstr()? != name.value || r.remaining() != 0 {
                    return Err(r.invalid("snapshot name or extent"));
                }
                let container = self
                    .object(container_id)
                    .ok_or_else(|| r.invalid("missing body container"))?;
                let tail = container
                    .get(container.len().saturating_sub(11)..)
                    .ok_or_else(|| r.invalid("container owner tail"))?;
                let component_id = local_reference(&mut Reader::new(tail, 1024))?;
                let component = components
                    .iter()
                    .find(|c| c.id == component_id)
                    .ok_or_else(|| r.invalid("unresolved component"))?;
                let component_raw = self
                    .object(component_id)
                    .ok_or_else(|| r.invalid("missing component"))?;
                if !references(component_raw).any(|id| id == container_id) {
                    return Err(r.invalid("missing reciprocal component link"));
                }
                Ok(BodyBinding {
                    map_record: item.id,
                    container_id,
                    component_id,
                    component_name: component.name.clone(),
                    blob: name.value,
                    selectors,
                })
            })();
            if let Ok(binding) = parsed {
                out.push(binding);
            }
        }
        out
    }
    /// Recover only a unique proper rigid matrix from each registered occurrence.
    /// This is explicitly marked a candidate, not a full occurrence-schema decode.
    pub fn placement_candidates(&self) -> Vec<PlacementCandidate> {
        let Some(reg) = self
            .records
            .iter()
            .find(|r| r.identity == "CE2913AA-CFE0-4F04-9102-24424ED3BCFA")
        else {
            return Vec::new();
        };
        let mut out = Vec::new();
        for id in reg.ids.iter().copied().collect::<BTreeSet<_>>() {
            let Some(raw) = self.object(id) else {
                continue;
            };
            let mut r = Reader::new(raw, 1024);
            let target = (|| -> Result<u64> {
                if r.take(4)? != [0; 4] || r.u8()? != 1 || r.u32()? != 1 {
                    return Err(r.invalid("occurrence envelope"));
                }
                local_reference(&mut r)
            })();
            let Ok(component_id) = target else {
                continue;
            };
            let mut candidates = Vec::new();
            for offset in 0..raw.len().saturating_sub(127) {
                let mut r = Reader::new(raw, 1024);
                r.pos = offset;
                let mut m = Mat4::IDENTITY;
                let parsed = (|| -> Result<()> {
                    for row in &mut m.0 {
                        for v in row {
                            *v = r.f64()?;
                        }
                    }
                    Ok(())
                })();
                if parsed.is_ok() && m.is_rigid(1e-10) {
                    candidates.push((offset, m));
                }
            }
            if candidates.len() == 1 {
                let (offset, matrix) = candidates[0];
                out.push(PlacementCandidate{occurrence_id:id,component_id,offset,matrix,units:"cm".into(),evidence:"unique rigid matrix in registered occurrence record; not a complete occurrence grammar".into()});
            }
        }
        out
    }
}

fn local_reference(r: &mut Reader<'_>) -> Result<u64> {
    if r.u8()? != 1 {
        return Err(r.invalid("expected local reference"));
    }
    let id = r.u64()?;
    if r.take(2)? != [0; 2] {
        return Err(r.invalid("reference trailer"));
    }
    Ok(id)
}
fn references(data: &[u8]) -> impl Iterator<Item = u64> + '_ {
    data.windows(11)
        .filter(|w| w[0] == 1 && w[9..] == [0, 0])
        .map(|w| u64::from_le_bytes(w[1..9].try_into().unwrap()))
}

#[derive(Debug, Clone, Serialize)]
pub struct Component {
    pub id: u64,
    pub name: String,
}
#[derive(Debug, Clone, Serialize)]
pub struct BodyBinding {
    pub map_record: u64,
    pub container_id: u64,
    pub component_id: u64,
    pub component_name: String,
    pub blob: String,
    pub selectors: Vec<[u64; 2]>,
}
#[derive(Debug, Clone, Serialize)]
pub struct PlacementCandidate {
    pub occurrence_id: u64,
    pub component_id: u64,
    pub offset: usize,
    pub matrix: Mat4,
    pub units: String,
    pub evidence: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct FoundString {
    pub offset: usize,
    pub wide: bool,
    pub value: String,
}
/// Conservative forensic scan: UTF-16 Latin-1 and printable Latin-1 strings.
/// This is a discovery utility, not a schema decoder.
pub fn scan_strings(data: &[u8]) -> Vec<FoundString> {
    let mut out = Vec::new();
    let mut pos = 0;
    while pos + 4 <= data.len() {
        let n = u32::from_le_bytes(data[pos..pos + 4].try_into().unwrap()) as usize;
        if (3..=512).contains(&n) {
            if let Some(raw) = data.get(pos + 4..pos + 4 + n * 2)
                && raw
                    .chunks_exact(2)
                    .all(|b| b[1] == 0 && b[0] >= 32 && b[0] != 127)
            {
                let value = raw.chunks_exact(2).map(|b| char::from(b[0])).collect();
                out.push(FoundString {
                    offset: pos,
                    wide: true,
                    value,
                });
                pos += 4 + n * 2;
                continue;
            }
            if let Some(raw) = data.get(pos + 4..pos + 4 + n)
                && raw.iter().all(|b| *b >= 32 && *b != 127)
            {
                let value = raw.iter().map(|b| char::from(*b)).collect();
                out.push(FoundString {
                    offset: pos,
                    wide: false,
                    value,
                });
                pos += 4 + n;
                continue;
            }
        }
        pos += 1;
    }
    out
}
