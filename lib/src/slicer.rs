use regex::Regex;

#[derive(Debug, Clone)]
pub enum SlicerPreset {
    PrusaSlicer { version: String },
    SuperSlicer { version: String },
    IdeaMaker { version: String },
    Cura { version: String },
}

impl SlicerPreset {
    pub fn determine(comment: &str) -> Option<SlicerPreset> {
        None.or_else(|| Self::try_slic3r(comment))
            .or_else(|| Self::try_ideamaker(comment))
            .or_else(|| Self::try_cura(comment))
    }

    fn try_slic3r(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE_PRUSA: Regex = Regex::new(r"PrusaSlicer\s(.*)\son").unwrap();
            static ref RE_SUPER: Regex = Regex::new(r"SuperSlicer\s(.*)\son").unwrap();
        }
        if let Some(m) = RE_PRUSA.captures(comment) {
            Some(SlicerPreset::PrusaSlicer {
                version: m.get(1).unwrap().as_str().into(),
            })
        } else if let Some(m) = RE_SUPER.captures(comment) {
            Some(SlicerPreset::SuperSlicer {
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

    fn try_cura(comment: &str) -> Option<SlicerPreset> {
        lazy_static! {
            static ref RE: Regex = Regex::new(r"Generated with Cura_SteamEngine\s(.*)").unwrap();
        }
        RE.captures(comment).map(|c| SlicerPreset::Cura {
            version: c.get(1).unwrap().as_str().into(),
        })
    }
}
