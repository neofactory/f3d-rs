use crate::{Error, Result};

#[derive(Clone)]
pub(crate) struct Reader<'a> {
    pub data: &'a [u8],
    pub pos: usize,
    pub max_string: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8], max_string: usize) -> Self {
        Self {
            data,
            pos: 0,
            max_string,
        }
    }
    pub fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }
    pub fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        if n > self.remaining() {
            return Err(Error::Truncated {
                offset: self.pos,
                needed: n,
                remaining: self.remaining(),
            });
        }
        let start = self.pos;
        self.pos += n;
        Ok(&self.data[start..self.pos])
    }
    pub fn invalid(&self, reason: impl Into<String>) -> Error {
        Error::Invalid {
            offset: self.pos,
            reason: reason.into(),
        }
    }
    pub fn count(&mut self, width: usize) -> Result<usize> {
        let n = self.u32()? as usize;
        if width != 0 && n > self.remaining() / width {
            return Err(self.invalid(format!("count {n} exceeds remaining payload")));
        }
        Ok(n)
    }
    pub fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }
    pub fn u16(&mut self) -> Result<u16> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    pub fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    pub fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    pub fn f32(&mut self) -> Result<f32> {
        Ok(f32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    pub fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    pub fn signed_word(&mut self, width: usize) -> Result<i64> {
        match width {
            4 => Ok(self.u32()? as i32 as i64),
            8 => Ok(self.u64()? as i64),
            _ => Err(self.invalid("invalid word width")),
        }
    }
    pub fn unsigned_word(&mut self, width: usize) -> Result<u64> {
        match width {
            4 => Ok(self.u32()? as u64),
            8 => self.u64(),
            _ => Err(self.invalid("invalid word width")),
        }
    }
    pub fn vec3(&mut self) -> Result<[f64; 3]> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }
    pub fn latin1(&mut self, n: usize) -> Result<String> {
        if n > self.max_string {
            return Err(Error::Limit(format!("string has {n} bytes")));
        }
        Ok(self.take(n)?.iter().map(|&b| char::from(b)).collect())
    }
    pub fn str8(&mut self) -> Result<String> {
        let n = self.u32()? as usize;
        if n > self.max_string {
            return Err(Error::Limit(format!("string has {n} bytes")));
        }
        let data = self.take(n)?;
        std::str::from_utf8(data)
            .map(str::to_owned)
            .map_err(|e| self.invalid(e.to_string()))
    }
    pub fn wstr(&mut self) -> Result<String> {
        let n = self.u32()? as usize;
        let bytes = n
            .checked_mul(2)
            .ok_or_else(|| self.invalid("UTF-16 length overflow"))?;
        if bytes > self.max_string {
            return Err(Error::Limit(format!("UTF-16 string has {bytes} bytes")));
        }
        let words: Vec<_> = self
            .take(bytes)?
            .chunks_exact(2)
            .map(|b| u16::from_le_bytes([b[0], b[1]]))
            .collect();
        String::from_utf16(&words).map_err(|e| self.invalid(e.to_string()))
    }
}
