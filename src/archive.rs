//! ZIP inventory and payloads, including Deflate and Zstandard (method 93).

use crate::{Error, Limits, Result};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{Read, Seek},
    path::Path,
};

#[derive(Debug, Clone, Serialize)]
pub struct Entry {
    pub name: String,
    pub size: u64,
    pub compressed_size: u64,
    pub compression: String,
    pub crc32: u32,
    #[serde(skip)]
    pub data: Vec<u8>,
}

#[derive(Debug, Serialize)]
pub struct Archive {
    pub entries: Vec<Entry>,
    #[serde(skip)]
    by_name: BTreeMap<String, usize>,
}

impl Archive {
    pub fn open(path: impl AsRef<Path>, limits: &Limits) -> Result<Self> {
        Self::read(File::open(path)?, limits)
    }
    pub fn read<R: Read + Seek>(reader: R, limits: &Limits) -> Result<Self> {
        let mut budget = limits.max_total_bytes;
        Self::read_budgeted(reader, limits, &mut budget)
    }
    pub(crate) fn read_budgeted<R: Read + Seek>(
        reader: R,
        limits: &Limits,
        budget: &mut u64,
    ) -> Result<Self> {
        let mut zip = zip::ZipArchive::new(reader)?;
        if zip.len() > limits.max_entries {
            return Err(Error::Limit("archive entry count".into()));
        }
        let mut entries = Vec::new();
        let mut by_name = BTreeMap::new();
        for i in 0..zip.len() {
            let mut file = zip.by_index(i)?;
            if file.is_dir() {
                continue;
            }
            let name = file.name()?.into_owned();
            if name.len() > limits.max_string_bytes {
                return Err(Error::Limit("entry name length".into()));
            }
            if file.enclosed_name().is_none() || name.contains('\\') || name.as_bytes().contains(&0)
            {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("unsafe ZIP entry name {name:?}"),
                });
            }
            if by_name.contains_key(&name) {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("duplicate ZIP entry {name:?}"),
                });
            }
            let size = file.size();
            if size > limits.max_entry_bytes || size > *budget {
                return Err(Error::Limit(format!(
                    "entry {name:?} exceeds decompression budget"
                )));
            }
            let mut entry = Entry {
                name: name.clone(),
                size,
                compressed_size: file.compressed_size(),
                compression: format!("{:?}", file.compression()),
                crc32: file.crc32(),
                data: Vec::new(),
            };
            // Read to EOF to make the ZIP reader verify the CRC. The extra byte
            // also rejects a stream that expands past its advertised size.
            file.by_ref()
                .take(size.saturating_add(1))
                .read_to_end(&mut entry.data)?;
            if entry.data.len() as u64 != size {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("size mismatch in {name:?}"),
                });
            }
            *budget -= size;
            by_name.insert(name, entries.len());
            entries.push(entry);
        }
        Ok(Self { entries, by_name })
    }
    pub fn get(&self, name: &str) -> Option<&Entry> {
        self.by_name.get(name).map(|&i| &self.entries[i])
    }
    pub fn bytes(&self, name: &str) -> Result<&[u8]> {
        self.get(name)
            .map(|e| e.data.as_slice())
            .ok_or_else(|| Error::MissingEntry(name.into()))
    }
    pub fn total_bytes(&self) -> u64 {
        self.entries.iter().map(|e| e.size).sum()
    }
}
