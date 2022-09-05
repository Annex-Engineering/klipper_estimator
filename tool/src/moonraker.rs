use std::collections::HashMap;

use lib_klipper::glam::DVec3;
use lib_klipper::planner::{
    FirmwareRetractionOptions, GCodeMacro, MacroConfiguration, MoveChecker, PrinterConfiguration,
    PrinterLimits,
};

use serde::Deserialize;
use thiserror::Error;
use url::Url;

#[derive(Debug, Deserialize)]
struct MoonrakerResultRoot {
    result: MoonrakerResult,
}

#[derive(Debug, Deserialize)]
struct MoonrakerResult {
    status: MoonrakerResultStatus,
}

#[derive(Debug, Deserialize)]
struct MoonrakerResultStatus {
    configfile: MoonrakerConfigFile,
}

#[derive(Debug, Deserialize)]
struct MoonrakerConfigFile {
    // settings: MoonrakerConfig,
    settings: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Deserialize)]
struct MoonrakerConfig {
    printer: PrinterConfig,
    extruder: ExtruderConfig,
    firmware_retraction: Option<FirmwareRetractionConfig>,
}

#[derive(Debug, Deserialize)]
struct PrinterConfig {
    max_velocity: f64,
    max_accel: f64,
    max_accel_to_decel: f64,
    square_corner_velocity: f64,

    max_x_velocity: Option<f64>,
    max_x_accel: Option<f64>,
    max_y_velocity: Option<f64>,
    max_y_accel: Option<f64>,
    max_z_velocity: Option<f64>,
    max_z_accel: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct ExtruderConfig {
    max_extrude_only_velocity: f64,
    max_extrude_only_accel: f64,
    instantaneous_corner_velocity: f64,
}

#[derive(Debug, Deserialize)]
struct FirmwareRetractionConfig {
    retract_length: f64,
    unretract_extra_length: f64,
    unretract_speed: f64,
    retract_speed: f64,
    #[serde(default)]
    lift_z: f64,
}

#[derive(Error, Debug)]
pub enum MoonrakerConfigError {
    #[error("given URL cannot be a base URL")]
    URLCannotBeBase,
    #[error("invalid URL: {}", .0)]
    URLParseError(#[from] url::ParseError),
    #[error("request failed: {}", .0)]
    RequestError(#[from] reqwest::Error),
}

fn query_moonraker(
    source_url: &str,
    api_key: Option<&str>,
) -> Result<MoonrakerConfigFile, MoonrakerConfigError> {
    let mut url = Url::parse(source_url)?;
    url.query_pairs_mut().append_pair("configfile", "settings");
    {
        let mut path = url
            .path_segments_mut()
            .map_err(|_| MoonrakerConfigError::URLCannotBeBase)?;
        path.extend(&["printer", "objects", "query"]);
    }

    let client = reqwest::blocking::Client::new();
    let mut req = client.get(url);

    if let Some(api_key) = api_key {
        req = req.header("X-Api-Key", api_key);
    }

    Ok(req
        .send()?
        .json::<MoonrakerResultRoot>()?
        .result
        .status
        .configfile)
}

pub fn moonraker_config(
    source_url: &str,
    api_key: Option<&str>,
    target: &mut PrinterLimits,
) -> Result<PrinterConfiguration, MoonrakerConfigError> {
    let cfg_json = query_moonraker(source_url, api_key)?.settings;

    let cfg = {
        let cfg_printer: PrinterConfig = {
            let cfg_section = cfg_json.get("printer").unwrap().to_owned();
            serde_json::from_value(cfg_section).unwrap()
        };
        let cfg_extruder: ExtruderConfig = {
            let cfg_section = cfg_json.get("extruder").unwrap().to_owned();
            serde_json::from_value(cfg_section).unwrap()
        };
        let cfg_firmware_retraction: Option<FirmwareRetractionConfig> = {
            if let Some(cfg_section) = cfg_json.get("firmware_retraction") {
                serde_json::from_value(cfg_section.to_owned()).unwrap()
            } else {
                None
            }
        };
        MoonrakerConfig {
            printer: cfg_printer,
            extruder: cfg_extruder,
            firmware_retraction: cfg_firmware_retraction,
        }
    };

    fn deserialize_macro<'a>(key: &'a str, val: &'a serde_json::Value) -> Result<GCodeMacro, ()> {
        let name = key
            .strip_prefix("gcode_macro ")
            .map(|x| x.to_owned())
            .ok_or(())?;
        let description = val["description"].as_str().map(|x| x.to_owned());
        let gcode = val["gcode"].as_str().map(|x| x.to_owned()).ok_or(())?;

        fn fixup_value(val: serde_json::Value) -> serde_json::Value {
            let strval = val.as_str().unwrap().to_owned().to_lowercase();
            let new_val = serde_json::from_str::<serde_json::Value>(&strval);
            match new_val {
                Ok(new_val) => new_val,
                Err(_) => val,
            }
        }

        let val_obj = val.as_object().ok_or(())?;
        let variables: HashMap<String, serde_json::Value> = val_obj
            .iter()
            .filter(|(k, _)| k.starts_with("variable_"))
            .map(|(k, v)| {
                (
                    k.strip_prefix("variable_").unwrap().to_owned(),
                    fixup_value(v.to_owned()),
                )
            })
            .collect();

        Ok(GCodeMacro {
            name,
            description,
            variables,
            gcode,
        })
    }

    let macros = cfg_json
        .iter()
        .filter(|(k, _)| k.starts_with("gcode_macro "))
        .map(|(k, v)| deserialize_macro(k, v))
        .collect::<Result<Vec<_>, ()>>()
        .unwrap();

    target.set_max_velocity(cfg.printer.max_velocity);
    target.set_max_acceleration(cfg.printer.max_accel);
    target.set_max_accel_to_decel(cfg.printer.max_accel_to_decel);
    target.set_square_corner_velocity(cfg.printer.square_corner_velocity);
    target.set_instant_corner_velocity(cfg.extruder.instantaneous_corner_velocity);

    target.firmware_retraction = cfg.firmware_retraction.map(|fr| FirmwareRetractionOptions {
        retract_length: fr.retract_length,
        unretract_extra_length: fr.unretract_extra_length,
        unretract_speed: fr.unretract_speed,
        retract_speed: fr.retract_speed,
        lift_z: fr.lift_z,
    });

    let limits = [
        (
            DVec3::X,
            cfg.printer.max_x_velocity,
            cfg.printer.max_x_accel,
        ),
        (
            DVec3::Y,
            cfg.printer.max_y_velocity,
            cfg.printer.max_y_accel,
        ),
        (
            DVec3::Z,
            cfg.printer.max_z_velocity,
            cfg.printer.max_z_accel,
        ),
    ];

    for (axis, m, a) in limits {
        if let (Some(max_velocity), Some(max_accel)) = (m, a) {
            target.move_checkers.push(MoveChecker::AxisLimiter {
                axis,
                max_velocity,
                max_accel,
            });
        }
    }

    target.move_checkers.push(MoveChecker::ExtruderLimiter {
        max_velocity: cfg.extruder.max_extrude_only_velocity,
        max_accel: cfg.extruder.max_extrude_only_accel,
    });
    Ok(PrinterConfiguration {
        limits: target.clone(),
        macro_config: MacroConfiguration {
            macros,
            config: cfg_json,
        },
    })
}
