use regex::Regex;

#[derive(Debug, Clone)]
pub enum SlicerPreset {
    PrusaSlicer { version: String },
    SuperSlicer { version: String },
    OrcaSlicer { version: String },
    IdeaMaker { version: String },
    Cura { version: Option<String> },
    Simplify3D { version: String },
}

impl std::fmt::Display for SlicerPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SlicerPreset::PrusaSlicer { version } => write!(f, "PrusaSlicer {}", version),
            SlicerPreset::SuperSlicer { version } => write!(f, "SuperSlicer {}", version),
            SlicerPreset::OrcaSlicer { version } => write!(f, "OrcaSlicer {}", version),
            SlicerPreset::IdeaMaker { version } => write!(f, "ideaMaker {}", version),
            SlicerPreset::Cura { version: None } => write!(f, "Cura"),
            SlicerPreset::Cura {
                version: Some(version),
            } => write!(f, "Cura {}", version),
            SlicerPreset::Simplify3D { version } => write!(f, "Simplify3D {}", version),
        }
    }
}

impl SlicerPreset {
    pub fn determine(comment: &str) -> Option<SlicerPreset> {
        None.or_else(|| Self::try_slic3r(comment))
            .or_else(|| Self::try_ideamaker(comment))
            .or_else(|| Self::try_cura_old(comment))
            .or_else(|| Self::try_cura_new(comment))
            .or_else(|| Self::try_simplify3d(comment))
    }

    #[allow(clippy::manual_map)]
    fn try_slic3r(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE_PRUSA: Regex = Regex::new(r"PrusaSlicer\s(.*)\son").unwrap();
            static ref RE_SUPER: Regex = Regex::new(r"SuperSlicer\s(.*)\son").unwrap();
            static ref RE_ORCA: Regex = Regex::new(r"OrcaSlicer\s(.*)\son").unwrap();
        }
        if let Some(m) = RE_PRUSA.captures(comment) {
            Some(SlicerPreset::PrusaSlicer {
                version: m.get(1).unwrap().as_str().into(),
            })
        } else if let Some(m) = RE_SUPER.captures(comment) {
            Some(SlicerPreset::SuperSlicer {
                version: m.get(1).unwrap().as_str().into(),
            })
        } else if let Some(m) = RE_ORCA.captures(comment) {
            Some(SlicerPreset::OrcaSlicer {
                version: m.get(1).unwrap().as_str().into(),
            })
        } else {
            None
        }
    }

    fn try_ideamaker(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE: Regex = Regex::new(r"Sliced by ideaMaker\s(.*),").unwrap();
        }
        RE.captures(comment).map(|c| SlicerPreset::IdeaMaker {
            version: c.get(1).unwrap().as_str().into(),
        })
    }

    fn try_cura_old(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE: Regex = Regex::new(r"Generated with Cura_SteamEngine\s(.*)").unwrap();
        }
        RE.captures(comment).map(|c| SlicerPreset::Cura {
            version: Some(c.get(1).unwrap().as_str().into()),
        })
    }

    fn try_cura_new(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE: Regex = Regex::new(r"GENERATOR.NAME:Cura_SteamEngine").unwrap();
        }
        RE.captures(comment)
            .map(|_| SlicerPreset::Cura { version: None })
    }

    fn try_simplify3d(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE: Regex = Regex::new(r"Simplify3D\(R\)\sVersion\s(.*)").unwrap();
        }
        RE.captures(comment).map(|c| SlicerPreset::Simplify3D {
            version: c.get(1).unwrap().as_str().into(),
        })
    }
}
