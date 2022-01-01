use std::collections::HashMap;

#[derive(Debug, Default)]
pub struct KindTracker {
    pub i2k: HashMap<String, u16>,
    pub k2i: HashMap<u16, String>,
}

impl KindTracker {
    pub fn new() -> KindTracker {
        Self::default()
    }

    pub fn get_kind(&mut self, s: &str) -> Kind {
        match self.i2k.get(s) {
            Some(k) => Kind(*k),
            None => {
                let k = self.i2k.len() as u16;
                self.i2k.insert(s.into(), k);
                self.k2i.insert(k, s.into());
                Kind(k)
            }
        }
    }

    pub fn resolve_kind(&self, k: Kind) -> &str {
        self.k2i.get(&k.0).expect("missing kind")
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Kind(u16);
