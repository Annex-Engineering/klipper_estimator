use std::error::Error;

use lib_klipper::glam::DVec3;
use lib_klipper::planner::{FirmwareRetractionOptions, MoveChecker, Planner, PrinterLimits};

use clap::Parser;
use once_cell::sync::OnceCell;
use serde::Deserialize;
#[macro_use]
extern crate lazy_static;

mod cmd;

#[derive(Parser, Debug)]
#[clap(version = env!("VERGEN_GIT_SEMVER_LIGHTWEIGHT"), author = "Lasse Dalegaard <dalegaard@gmail.com>")]
pub struct Opts {
    #[clap(long = "config_moonraker_url")]
    config_moonraker: Option<String>,

    #[clap(long = "config_file")]
    config_filename: Option<String>,

    #[clap(subcommand)]
    cmd: SubCommand,

    #[clap(skip)]
    config: OnceCell<PrinterLimits>,
}

impl Opts {
    fn printer_limits(&self) -> &PrinterLimits {
        match self.config.get() {
            Some(limits) => limits,
            None => match self.load_config() {
                Ok(limits) => {
                    let _ = self.config.set(limits);
                    self.config.get().unwrap()
                }
                Err(e) => {
                    eprintln!("Failed to load printer configuration: {:?}", e);
                    std::process::exit(1);
                }
            },
        }
    }

    fn load_config(&self) -> Result<PrinterLimits, Box<dyn Error>> {
        // Load config file
        let mut limits = if let Some(filename) = &self.config_filename {
            let src = std::fs::read_to_string(filename)?;
            let mut limits: PrinterLimits = deser_hjson::from_str(&src)?;

            // Do any fix-ups
            limits.set_square_corner_velocity(limits.square_corner_velocity);

            limits
        } else {
            PrinterLimits::default()
        };

        // Was moonraker config requested? If so, try to grab that first.
        if let Some(url) = &self.config_moonraker {
            moonraker_config(url, &mut limits)?;
        }

        Ok(limits)
    }

    fn make_planner(&self) -> Planner {
        Planner::from_limits(self.printer_limits().clone())
    }
}

fn moonraker_config(source_url: &str, target: &mut PrinterLimits) -> Result<(), Box<dyn Error>> {
    let mut url = source_url.to_string();
    url.push_str("/printer/objects/query?configfile=settings");

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
        settings: MoonrakerConfig,
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

    let cfg = reqwest::blocking::get(url)?
        .json::<MoonrakerResultRoot>()?
        .result
        .status
        .configfile
        .settings;

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
    Ok(())
}

#[derive(Parser, Debug)]
enum SubCommand {
    Estimate(cmd::estimate::EstimateCmd),
    DumpMoves(cmd::estimate::DumpMovesCmd),
    PostProcess(cmd::post_process::PostProcessCmd),
    DumpConfig(cmd::dump_config::DumpConfigCmd),
}

impl SubCommand {
    fn run(&self, opts: &Opts) {
        match self {
            Self::Estimate(i) => i.run(opts),
            Self::DumpMoves(i) => i.run(opts),
            Self::PostProcess(i) => i.run(opts),
            Self::DumpConfig(i) => i.run(opts),
        }
    }
}

fn main() {
    let opts = Opts::parse();
    opts.cmd.run(&opts);
}
