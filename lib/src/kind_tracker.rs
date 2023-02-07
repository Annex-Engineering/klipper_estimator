use std::collections::HashMap;

#[derive(Debug, Default)]
pub struct KindTracker {
    pub i2k: HashMap<String, u16>,
    pub k2i: HashMap<u16, String>,
    pub current_kind: Option<Kind>,
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

    pub fn kind_from_comment(&mut self, comment: &Option<String>) -> Option<Kind> {
        comment
            .as_ref()
            .map(|s| s.trim())
            .map(|s| {
                if s.starts_with("move to next layer ") {
                    "move to next layer"
                } else {
                    s
                }
            })
            .map(|s| self.get_kind(s))
            .or(self.current_kind)
    }

    pub fn set_current(&mut self, kind: Option<Kind>) {
        self.current_kind = kind;
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Kind(u16);
