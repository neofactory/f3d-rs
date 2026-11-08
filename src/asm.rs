//! Autodesk ShapeManager / ACIS binary records and cycle-safe B-Rep traversal.
//! Ported in part from ezf3d; see THIRD-PARTY-NOTICES and LICENSE-MIT-ezf3d.

use crate::{Error, Limits, Result, binary::Reader, math::Vec3};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

pub mod tag {
    pub const NONE: u8 = 0;
    pub const BYTE: u8 = 1;
    pub const CHAR: u8 = 2;
    pub const SHORT: u8 = 3;
    pub const INT: u8 = 4;
    pub const FLOAT: u8 = 5;
    pub const DOUBLE: u8 = 6;
    pub const STR: u8 = 7;
    pub const STR2: u8 = 8;
    pub const STR3: u8 = 9;
    pub const FALSE: u8 = 10;
    pub const TRUE: u8 = 11;
    pub const POINTER: u8 = 12;
    pub const TYPE: u8 = 13;
    pub const TYPE_EX: u8 = 14;
    pub const START: u8 = 15;
    pub const END: u8 = 16;
    pub const RECORD_END: u8 = 17;
    pub const LITERAL: u8 = 18;
    pub const POSITION: u8 = 19;
    pub const DIRECTION: u8 = 20;
    pub const ENUM: u8 = 21;
}

#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Integer(i64),
    Real(f64),
    Text(String),
    Vector(Vec3),
    Empty,
}

// JSON has no NaN/Infinity. Preserve those exact IEEE-754 bits instead of
// silently serializing a geometry sentinel as null.
struct JsonReal(f64);
impl Serialize for JsonReal {
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> std::result::Result<S::Ok, S::Error> {
        if self.0.is_finite() {
            serializer.serialize_f64(self.0)
        } else {
            serializer.serialize_str(&format!("f64:0x{:016x}", self.0.to_bits()))
        }
    }
}
impl Serialize for Value {
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> std::result::Result<S::Ok, S::Error> {
        match self {
            Self::Integer(v) => serializer.serialize_i64(*v),
            Self::Real(v) => JsonReal(*v).serialize(serializer),
            Self::Text(v) => serializer.serialize_str(v),
            Self::Vector(v) => serializer.collect_seq(v.iter().copied().map(JsonReal)),
            Self::Empty => serializer.serialize_unit(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Token {
    pub tag: u8,
    pub value: Value,
}
impl Token {
    pub fn integer(&self) -> Option<i64> {
        if let Value::Integer(v) = self.value {
            Some(v)
        } else {
            None
        }
    }
    pub fn real(&self) -> Option<f64> {
        if let Value::Real(v) = self.value {
            Some(v)
        } else {
            None
        }
    }
    pub fn text(&self) -> Option<&str> {
        if let Value::Text(ref v) = self.value {
            Some(v)
        } else {
            None
        }
    }
    pub fn vector(&self) -> Option<Vec3> {
        if let Value::Vector(v) = self.value {
            Some(v)
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Header {
    pub signature: String,
    pub version: u64,
    pub reserved: [u64; 3],
    pub product: String,
    pub kernel_version: String,
    pub written: String,
    /// First header real, retained under ezf3d's name. No implicit coordinate
    /// conversion is performed by this crate.
    pub sizebox: f64,
    pub resabs: f64,
    pub resnor: f64,
    pub body_offset: usize,
    pub word_size: usize,
}
impl Header {
    pub fn parse(data: &[u8], limits: &Limits) -> Result<Self> {
        let mut r = Reader::new(data, limits.max_string_bytes);
        let (signature, word_size) = if data.starts_with(b"ASM BinaryFile") {
            r.take(14)?;
            let width = match r.u8()? {
                b'4' => 4,
                b'8' => 8,
                _ => return Err(r.invalid("unsupported ASM pointer width")),
            };
            (String::from_utf8_lossy(&data[..15]).into_owned(), width)
        } else if data.starts_with(b"ACIS BinaryFile") {
            r.take(15)?;
            ("ACIS BinaryFile".into(), 4)
        } else {
            return Err(r.invalid("not an ASM/ACIS binary stream"));
        };
        let version = r.unsigned_word(word_size)?;
        let reserved = [
            r.unsigned_word(word_size)?,
            r.unsigned_word(word_size)?,
            r.unsigned_word(word_size)?,
        ];
        let mut strings = Vec::new();
        for _ in 0..3 {
            if r.u8()? != tag::STR {
                return Err(r.invalid("expected header string tag"));
            }
            let n = r.u8()? as usize;
            strings.push(r.latin1(n)?);
        }
        let mut doubles = [0.; 3];
        for value in &mut doubles {
            if r.u8()? != tag::DOUBLE {
                return Err(r.invalid("expected header double tag"));
            }
            *value = r.f64()?;
        }
        if !doubles.iter().all(|v| v.is_finite()) {
            return Err(r.invalid("invalid header units/tolerances"));
        }
        Ok(Self {
            signature,
            version,
            reserved,
            product: strings.remove(0),
            kernel_version: strings.remove(0),
            written: strings.remove(0),
            sizebox: doubles[0],
            resabs: doubles[1],
            resnor: doubles[2],
            body_offset: r.pos,
            word_size,
        })
    }
}

/// `index` is the address in the current section. History never shifts model
/// pointer addresses; `source_record` retains the original physical ordinal.
#[derive(Debug, Clone, Serialize)]
pub struct Entity {
    pub index: usize,
    pub source_record: usize,
    pub types: Vec<String>,
    pub tokens: Vec<Token>,
}
impl Entity {
    pub fn name(&self) -> &str {
        self.types.first().map(String::as_str).unwrap_or("")
    }
    pub fn base(&self) -> &str {
        self.types.last().map(String::as_str).unwrap_or("")
    }
    pub fn fields(&self) -> &[Token] {
        &self.tokens[self.types.len()..]
    }
    pub fn pointers(&self) -> impl Iterator<Item = i64> + '_ {
        self.fields()
            .iter()
            .filter(|t| t.tag == tag::POINTER)
            .filter_map(Token::integer)
    }
    pub fn pointer(&self, slot: usize) -> Option<i64> {
        self.pointers().nth(slot)
    }
    pub fn positions(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.fields()
            .iter()
            .filter(|t| t.tag == tag::POSITION)
            .filter_map(Token::vector)
    }
    pub fn directions(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.fields()
            .iter()
            .filter(|t| t.tag == tag::DIRECTION)
            .filter_map(Token::vector)
    }
    pub fn reals(&self) -> impl Iterator<Item = f64> + '_ {
        self.fields()
            .iter()
            .filter(|t| t.tag == tag::DOUBLE)
            .filter_map(Token::real)
    }
    pub fn sense(&self) -> bool {
        self.fields()
            .iter()
            .find(|t| t.tag == tag::TRUE || t.tag == tag::FALSE)
            .is_none_or(|t| t.tag == tag::TRUE)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Model {
    pub header: Header,
    pub entities: Vec<Entity>,
    pub history: Vec<Entity>,
    pub has_history: bool,
    pub terminated: bool,
}

impl Model {
    pub fn parse(data: &[u8]) -> Result<Self> {
        Self::parse_with_limits(data, &Limits::default())
    }
    pub fn parse_with_limits(data: &[u8], limits: &Limits) -> Result<Self> {
        let header = Header::parse(data, limits)?;
        let mut r = Reader::new(data, limits.max_string_bytes);
        r.pos = header.body_offset;
        let mut model = Self {
            header,
            entities: Vec::new(),
            history: Vec::new(),
            has_history: false,
            terminated: false,
        };
        let mut record = Vec::new();
        let mut depth = 0usize;
        let mut tokens = 0usize;
        let mut ordinal = 0usize;
        let mut in_history = false;
        while r.remaining() > 0 || !record.is_empty() {
            let at = r.pos;
            let at_eof = r.remaining() == 0;
            let code = if at_eof { tag::RECORD_END } else { r.u8()? };
            if model.terminated {
                return Err(r.invalid("data after ASM end marker"));
            }
            if code == tag::RECORD_END {
                if depth != 0 {
                    return Err(r.invalid("unclosed subtype at record boundary"));
                }
                if ordinal >= limits.max_records {
                    return Err(Error::Limit("ASM record count".into()));
                }
                let mut types: Vec<String> = record
                    .iter()
                    .take_while(|t: &&Token| t.tag == tag::TYPE || t.tag == tag::TYPE_EX)
                    .filter_map(|t| t.text().map(str::to_owned))
                    .collect();
                loop {
                    let begin = ["Begin", "of", "ASM", "History", "Data"];
                    let end = ["End", "of", "ASM", "History", "Section"];
                    if types.iter().map(String::as_str).take(begin.len()).eq(begin) {
                        in_history = true;
                        model.has_history = true;
                        types.drain(..begin.len());
                        record.drain(..begin.len());
                    } else if types.iter().map(String::as_str).take(end.len()).eq(end) {
                        in_history = false;
                        model.has_history = true;
                        types.drain(..end.len());
                        record.drain(..end.len());
                    } else {
                        break;
                    }
                }
                let is_end =
                    types == ["End", "of", "ASM", "data"] || types == ["End", "of", "ACIS", "data"];
                if at_eof && !is_end {
                    return Err(r.invalid("unterminated ASM record"));
                }
                if is_end {
                    model.terminated = true;
                    record.clear();
                } else if !record.is_empty() {
                    let target = if in_history {
                        &mut model.history
                    } else {
                        &mut model.entities
                    };
                    target.push(Entity {
                        index: target.len(),
                        source_record: ordinal,
                        types,
                        tokens: std::mem::take(&mut record),
                    });
                }
                ordinal += 1;
                continue;
            }
            tokens += 1;
            if tokens > limits.max_tokens {
                return Err(Error::Limit("ASM token count".into()));
            }
            let value = match code {
                tag::POINTER | tag::INT | tag::ENUM => {
                    Value::Integer(r.signed_word(model.header.word_size)?)
                }
                tag::BYTE => Value::Integer(r.u8()? as i64),
                tag::CHAR => Value::Integer(r.u8()? as i8 as i64),
                tag::SHORT => Value::Integer(r.u16()? as i16 as i64),
                tag::FLOAT => Value::Real(r.f32()? as f64),
                tag::DOUBLE => Value::Real(r.f64()?),
                tag::POSITION | tag::DIRECTION => Value::Vector(r.vec3()?),
                tag::STR | tag::TYPE | tag::TYPE_EX => {
                    let n = r.u8()? as usize;
                    Value::Text(r.latin1(n)?)
                }
                tag::STR2 => {
                    let n = r.u16()? as i16;
                    if n < 0 {
                        return Err(r.invalid("negative string length"));
                    }
                    Value::Text(r.latin1(n as usize)?)
                }
                tag::STR3 | tag::LITERAL => {
                    let n = r.u32()? as usize;
                    Value::Text(r.latin1(n)?)
                }
                tag::START => {
                    depth += 1;
                    if depth > limits.max_subtype_depth {
                        return Err(Error::Limit("ASM subtype nesting".into()));
                    }
                    Value::Empty
                }
                tag::END => {
                    depth = depth
                        .checked_sub(1)
                        .ok_or_else(|| r.invalid("unmatched subtype close"))?;
                    Value::Empty
                }
                tag::NONE | tag::FALSE | tag::TRUE => Value::Empty,
                _ => {
                    return Err(Error::Invalid {
                        offset: at,
                        reason: format!("unknown SAB tag 0x{code:02x}"),
                    });
                }
            };
            record.push(Token { tag: code, value });
        }
        if !record.is_empty() {
            return Err(r.invalid("unterminated ASM record"));
        }
        if !model.terminated {
            return Err(r.invalid("missing ASM end marker"));
        }
        Ok(model)
    }
    pub fn resolve(&self, pointer: i64) -> Option<&Entity> {
        usize::try_from(pointer)
            .ok()
            .and_then(|i| self.entities.get(i))
    }
    pub fn counts(&self) -> BTreeMap<String, usize> {
        let mut map = BTreeMap::new();
        for e in &self.entities {
            *map.entry(e.name().into()).or_default() += 1;
        }
        map
    }
    pub fn census(&self) -> Census {
        let mut c = Census::default();
        for e in &self.entities {
            let map = if [
                "body", "lump", "shell", "subshell", "face", "loop", "coedge", "edge", "vertex",
                "wire",
            ]
            .contains(&e.base())
            {
                &mut c.topology
            } else if e.base() == "surface" {
                &mut c.surfaces
            } else if e.base() == "curve" {
                &mut c.curves
            } else {
                continue;
            };
            *map.entry(e.name().into()).or_default() += 1;
        }
        c
    }
    pub fn bodies(&self) -> impl Iterator<Item = Body<'_>> {
        self.entities
            .iter()
            .filter(|e| e.base() == "body")
            .map(|e| {
                Body(Node {
                    model: self,
                    index: e.index,
                })
            })
    }
    pub fn live_faces(&self) -> Vec<Face<'_>> {
        let mut seen = BTreeSet::new();
        self.bodies()
            .flat_map(|b| b.faces())
            .filter(|f| seen.insert(f.index()))
            .collect()
    }
    pub fn live_edges(&self) -> Vec<Edge<'_>> {
        let mut seen = BTreeSet::new();
        self.live_faces()
            .into_iter()
            .flat_map(|f| f.edges())
            .filter(|e| seen.insert(e.index()))
            .collect()
    }
    pub fn vertex_bounds(&self) -> Option<[Vec3; 2]> {
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        let mut found = false;
        for e in self.entities.iter().filter(|e| e.name() == "point") {
            for p in e.positions() {
                if !p.iter().all(|v| v.is_finite()) {
                    continue;
                }
                found = true;
                for (i, x) in p.into_iter().enumerate() {
                    bounds[0][i] = bounds[0][i].min(x);
                    bounds[1][i] = bounds[1][i].max(x);
                }
            }
        }
        found.then_some(bounds)
    }
}

#[derive(Debug, Default, Clone, Serialize, PartialEq)]
pub struct Census {
    pub topology: BTreeMap<String, usize>,
    pub surfaces: BTreeMap<String, usize>,
    pub curves: BTreeMap<String, usize>,
}
impl Census {
    pub fn faces(&self) -> usize {
        *self.topology.get("face").unwrap_or(&0)
    }
    pub fn bodies(&self) -> usize {
        *self.topology.get("body").unwrap_or(&0)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Node<'a> {
    model: &'a Model,
    index: usize,
}
impl<'a> Node<'a> {
    pub fn entity(self) -> &'a Entity {
        &self.model.entities[self.index]
    }
    pub fn index(self) -> usize {
        self.index
    }
    fn at(self, slot: usize) -> Option<Node<'a>> {
        self.entity()
            .pointer(slot)
            .and_then(|p| self.model.resolve(p))
            .map(|e| Node {
                model: self.model,
                index: e.index,
            })
    }
    fn chain(self, slot: usize, base: &str) -> Vec<Node<'a>> {
        let mut out = Vec::new();
        let mut seen = BTreeSet::new();
        let mut next = self.at(slot);
        while let Some(node) = next {
            if !seen.insert(node.index) || node.entity().base() != base {
                break;
            }
            out.push(node);
            next = node.at(2);
        }
        out
    }
}
macro_rules! wrapper {($($name:ident),*)=>{$(#[derive(Clone,Copy,Debug)]pub struct $name<'a>(Node<'a>);impl<'a>$name<'a>{pub fn index(self)->usize{self.0.index()}pub fn entity(self)->&'a Entity{self.0.entity()}})*};}
wrapper!(Body, Lump, Shell, Face, Loop, Coedge, Edge, Vertex);
impl<'a> Body<'a> {
    pub fn lumps(self) -> Vec<Lump<'a>> {
        self.0.chain(2, "lump").into_iter().map(Lump).collect()
    }
    pub fn faces(self) -> Vec<Face<'a>> {
        self.lumps()
            .into_iter()
            .flat_map(|l| l.shells())
            .flat_map(|s| s.faces())
            .collect()
    }
    pub fn transform_entity(self) -> Option<&'a Entity> {
        self.0.at(4).map(Node::entity)
    }
}
impl<'a> Lump<'a> {
    pub fn shells(self) -> Vec<Shell<'a>> {
        self.0.chain(3, "shell").into_iter().map(Shell).collect()
    }
}
impl<'a> Shell<'a> {
    pub fn faces(self) -> Vec<Face<'a>> {
        self.0.chain(4, "face").into_iter().map(Face).collect()
    }
}
impl<'a> Face<'a> {
    pub fn loops(self) -> Vec<Loop<'a>> {
        self.0.chain(3, "loop").into_iter().map(Loop).collect()
    }
    pub fn surface_entity(self) -> Option<&'a Entity> {
        self.0.at(6).map(Node::entity)
    }
    pub fn edges(self) -> Vec<Edge<'a>> {
        self.loops()
            .into_iter()
            .flat_map(|l| l.coedges())
            .filter_map(|c| c.edge())
            .collect()
    }
}
impl<'a> Loop<'a> {
    pub fn coedges(self) -> Vec<Coedge<'a>> {
        self.0.chain(3, "coedge").into_iter().map(Coedge).collect()
    }
}
impl<'a> Coedge<'a> {
    pub fn edge(self) -> Option<Edge<'a>> {
        self.0
            .at(5)
            .filter(|n| n.entity().base() == "edge")
            .map(Edge)
    }
    pub fn partner(self) -> Option<Self> {
        self.0
            .at(4)
            .filter(|n| n.entity().base() == "coedge")
            .map(Self)
    }
}
impl<'a> Edge<'a> {
    pub fn start(self) -> Option<Vertex<'a>> {
        self.0
            .at(2)
            .filter(|n| n.entity().base() == "vertex")
            .map(Vertex)
    }
    pub fn end(self) -> Option<Vertex<'a>> {
        self.0
            .at(3)
            .filter(|n| n.entity().base() == "vertex")
            .map(Vertex)
    }
    pub fn curve_entity(self) -> Option<&'a Entity> {
        self.0.at(5).map(Node::entity)
    }
    pub fn range(self) -> Option<[f64; 2]> {
        let mut r = self.entity().reals();
        Some([r.next()?, r.next()?])
    }
}
impl Vertex<'_> {
    pub fn position(self) -> Option<Vec3> {
        self.0.at(3)?.entity().positions().next()
    }
}
