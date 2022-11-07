use lib_klipper::glam::DVec3;
use lib_klipper::planner::{FirmwareRetractionOptions, MoveChecker, Planner, PrinterLimits};

use clap::Parser;
use once_cell::sync::OnceCell;
use serde::Deserialize;
use thiserror::Error;
use url::Url;
#[macro_use]
extern crate lazy_static;

mod cmd;

#[derive(Parser, Debug)]
#[clap(version = env!("TOOL_VERSION"), author = "Lasse Dalegaard <dalegaard@gmail.com>")]
pub struct Opts {
    #[clap(long = "config_moonraker_url")]
    config_moonraker: Option<String>,
    #[clap(long = "config_moonraker_api_key")]
    config_moonraker_api_key: Option<String>,
    #[clap(long = "config_moonraker_ignore_error")]
    config_moonraker_ignore_error: bool,
    #[clap(long = "config_moonraker_cache_file")]
    config_moonraker_cache_file: Option<String>,

    #[clap(long = "config_file")]
    config_filename: Option<String>,

    #[clap(short = 'c')]
    config_override: Vec<String>,

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
                    eprintln!("Failed to load printer configuration: {}", e);
                    std::process::exit(1);
                }
            },
        }
    }

    fn opt_parse<'a>(s: &'a str) -> anyhow::Result<(&'a str, &'a str)> {
        let eqat = match s.find('=') {
            None => anyhow::bail!("invalid config override, format key=value"),
            Some(idx) => idx,
        };
        let key = &s[..eqat];
        let value = &s[eqat + 1..];
        Ok((key, value))
    }

    fn load_config(&self) -> anyhow::Result<PrinterLimits> {
        use config::Config;

        let builder = Config::builder();

        let builder = if let Some(url) = &self.config_moonraker {
            builder.add_source(MoonrakerSource::new(
                url,
                self.config_moonraker_api_key.as_deref(),
                self.config_moonraker_ignore_error,
                self.config_moonraker_cache_file.as_deref(),
            ))
        } else {
            builder
        };

        let builder = if let Some(filename) = &self.config_filename {
            builder.add_source(config::File::new(filename, config::FileFormat::Json5))
        } else {
            builder
        };

        let builder = self
            .config_override
            .iter()
            .try_fold(builder, |builder, opt| {
                let (k, v) = Self::opt_parse(opt)?;
                Ok::<_, anyhow::Error>(builder.set_override(k, v)?)
            })?;

        let limits = builder.build()?.try_deserialize::<PrinterLimits>()?;
        Ok(limits)
    }

    fn make_planner(&self) -> Planner {
        Planner::from_limits(self.printer_limits().clone())
    }
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

#[derive(Debug, Clone)]
struct MoonrakerSource {
    url: String,
    api_key: Option<String>,
    ignore_error: bool,
    cache_file: Option<String>,
}

impl MoonrakerSource {
    fn new(
        url: &str,
        api_key: Option<&str>,
        ignore_error: bool,
        cache_file: Option<&str>,
    ) -> MoonrakerSource {
        MoonrakerSource {
            url: url.into(),
            api_key: api_key.map(str::to_string),
            ignore_error,
            cache_file: cache_file.map(str::to_string),
        }
    }
}

impl config::Source for MoonrakerSource {
    fn clone_into_box(&self) -> Box<dyn config::Source + Send + Sync> {
        Box::new(self.clone())
    }

    fn collect(&self) -> Result<config::Map<String, config::Value>, config::ConfigError> {
        let mut limits = PrinterLimits::default();

        let res = moonraker_config(&self.url, self.api_key.as_deref(), &mut limits);
        let cfg = if let Err(e) = res {
            if self.ignore_error {
                eprintln!("Could not get config from Moonraker, ignoring. Error was:\n{e}");

                if let Some(cache_file) = self.cache_file.as_deref() {
                    eprintln!("Using cached Moonraker config");
                    match std::fs::read(cache_file) {
                        Err(e) => {
                            eprintln!("Could not read Moonraker cached config: {e}");
                            return Ok(Default::default());
                        }
                        Ok(cfg) => std::str::from_utf8(&cfg).unwrap_or("").into(),
                    }
                } else {
                    return Ok(Default::default());
                }
            } else {
                return Err(config::ConfigError::Foreign(Box::new(e)));
            }
        } else {
            let cfg = serde_json::to_string(&limits).unwrap();
            if let Some(cache_file) = self.cache_file.as_deref() {
                if let Err(e) = std::fs::write(cache_file, &cfg) {
                    eprintln!("Could not write Moonraker cached config: {e}");
                }
            }
            cfg
        };
        let file = config::File::from_str(&cfg, config::FileFormat::Json);
        file.collect()
    }
}

fn moonraker_config(
    source_url: &str,
    api_key: Option<&str>,
    target: &mut PrinterLimits,
) -> Result<(), MoonrakerConfigError> {
    let mut url = Url::parse(source_url)?;
    url.query_pairs_mut().append_pair("configfile", "settings");
    {
        let mut path = url
            .path_segments_mut()
            .map_err(|_| MoonrakerConfigError::URLCannotBeBase)?;
        path.extend(&["printer", "objects", "query"]);
    }

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

    let client = reqwest::blocking::Client::new();
    let mut req = client.get(url);

    if let Some(api_key) = api_key {
        req = req.header("X-Api-Key", api_key);
    }

    let cfg = req
        .send()?
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
