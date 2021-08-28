mod gcode;
mod planner;

use std::collections::BTreeMap;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;

use gcode::{GCodeCommand, GCodeOperation, GCodeReader};
use planner::{PrinterLimits, Vec3};
use std::f64::EPSILON;

use clap::{AppSettings, Clap};
use glam::Vec4Swizzles;
use once_cell::sync::OnceCell;
use serde::Deserialize;

#[derive(Clap, Debug)]
#[clap(version = "0.0", author = "Lasse Dalegaard <dalegaard@gmail.com>")]
#[clap(setting = AppSettings::ColoredHelp)]
struct Opts {
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
            moonraker_config(&url, &mut limits)?;
        }

        Ok(limits)
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
    }

    #[derive(Debug, Deserialize)]
    struct PrinterConfig {
        kinematics: String,
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

    let limits = [
        (Vec3::X, cfg.printer.max_x_velocity, cfg.printer.max_x_accel),
        (Vec3::Y, cfg.printer.max_y_velocity, cfg.printer.max_y_accel),
        (Vec3::Z, cfg.printer.max_z_velocity, cfg.printer.max_z_accel),
    ];

    for (axis, m, a) in limits {
        if let (Some(max_velocity), Some(max_accel)) = (m, a) {
            target
                .move_checkers
                .push(planner::MoveChecker::AxisLimiter {
                    axis,
                    max_velocity,
                    max_accel,
                });
        }
    }

    target
        .move_checkers
        .push(planner::MoveChecker::ExtruderLimiter {
            max_velocity: cfg.extruder.max_extrude_only_velocity,
            max_accel: cfg.extruder.max_extrude_only_accel,
        });
    Ok(())
}

#[derive(Clap, Debug)]
enum SubCommand {
    Estimate(EstimateCmd),
    PostProcess(PostProcessCmd),
    DumpConfig(DumpConfigCmd),
}

impl SubCommand {
    fn run(&self, opts: &Opts) {
        match self {
            Self::Estimate(i) => i.run(opts),
            Self::PostProcess(i) => i.run(opts),
            Self::DumpConfig(i) => i.run(opts),
        }
    }
}

#[derive(Clap, Debug)]
struct DumpConfigCmd;

impl DumpConfigCmd {
    fn run(&self, opts: &Opts) {
        let _ = serde_json::to_writer_pretty(std::io::stdout(), &opts.printer_limits());
    }
}

#[derive(Clap, Debug)]
struct EstimateCmd {
    input: String,
}

impl EstimateCmd {
    fn run(&self, opts: &Opts) {
        let src: Box<dyn std::io::Read> = match self.input.as_str() {
            "-" => Box::new(std::io::stdin()),
            filename => Box::new(File::open(filename).expect("opening gcode file failed")),
        };
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = planner::Planner::default();
        planner.toolhead_state.limits = opts.printer_limits().clone();

        for cmd in rdr {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(cmd);
        }

        planner.finalize();

        let move_kinds: BTreeMap<_, _> = planner.move_kinds.iter().map(|(k, v)| (v, k)).collect();

        println!("Sequences:");
        for (i, c) in planner.move_sequences.iter_mut().enumerate() {
            let moves: Vec<_> = c.iter().collect();

            println!(" Run {}:", i);
            println!("  Total moves: {}", c.total_moves());
            println!(
                "  Total distance: {}",
                moves.iter().map(|m| m.distance).sum::<f64>()
            );
            println!(
                "  Total extrude distance: {}",
                moves.iter().map(|m| m.end.w - m.start.w).sum::<f64>()
            );
            let min_time = 0.25 + moves.iter().map(|m| m.total_time()).sum::<f64>();
            let phase_times = [
                moves.iter().map(|m| m.accel_time()).sum::<f64>(),
                moves.iter().map(|m| m.cruise_time()).sum::<f64>(),
                moves.iter().map(|m| m.decel_time()).sum::<f64>(),
            ];

            println!("  Minimal time: {} ({})", format_time(min_time), min_time);
            println!("  Phases:");
            println!("    Acceleration: {}", format_time(phase_times[0]));
            println!("    Cruise:       {}", format_time(phase_times[1]));
            println!("    Deceleration: {}", format_time(phase_times[2]));

            println!("  Moves:");
            let width = (moves.len() as f64).log10().ceil() as usize;
            let mut layer_times = BTreeMap::new();
            let mut ctime = 0.25;
            let mut ztime = 0.0;
            for (i, m) in moves.iter().enumerate() {
                let mut kind = String::new();
                if m.is_extrude_move() {
                    kind.push('E');
                }
                if m.is_kinematic_move() {
                    kind.push('K');
                }
                println!(
                    "   {:width$}[{}] @ {:.8} => {:.8} / z{:.8}:",
                    i,
                    kind,
                    ctime,
                    ctime + m.total_time(),
                    ztime,
                    width = width
                );
                println!(
                    "    Path:       {:?} => {:?} [{:.3}∠{:.2}]",
                    m.start,
                    m.end,
                    m.distance,
                    m.rate.xy().angle_between(glam::DVec2::new(1.0, 0.0)) * 180.0
                        / std::f64::consts::PI,
                );
                println!("    Axes {:?}", m.rate);
                println!("    Line width: {:?}", m.line_width(1.75 / 2.0, 0.25),);
                println!("    Flow rate: {:?}", m.flow_rate(1.75 / 2.0));
                println!(
                    "    Kind: {:?}",
                    m.kind.as_ref().and_then(|v| move_kinds.get(v))
                );
                println!("    Acceleration {:?}", m.acceleration);
                println!("    Max dv2: {}", m.max_dv2);
                println!("    Max start_v2: {}", m.max_start_v2);
                println!("    Max smoothed_v2: {}", m.max_smoothed_v2);
                println!(
                    "    Velocity:   {} / {} / {}",
                    m.start_v, m.cruise_v, m.end_v
                );
                println!(
                    "    Time:       {:4}+{:4}+{:4}  = {:4}",
                    m.accel_time(),
                    m.cruise_time(),
                    m.decel_time(),
                    m.total_time(),
                );
                ctime += m.total_time();

                println!(
                    "    Distances:  {:.3}+{:.3}+{:.3} = {:.3}",
                    m.accel_distance(),
                    m.cruise_distance(),
                    m.decel_distance(),
                    m.distance
                );

                println!();

                if (m.start.z - m.end.z).abs() < EPSILON {
                    *layer_times
                        .entry((m.start.z * 1000.0) as usize)
                        .or_insert(0.0) += m.total_time();
                } else {
                    ztime += m.total_time();
                }
            }

            println!("  Layer times:");
            for (z, t) in layer_times.iter() {
                println!("   {:7} => {}", (*z as f64) / 1000.0, format_time(*t));
            }
        }
    }
}

#[derive(Clap, Debug)]
struct PostProcessCmd {
    filename: String,
}

impl PostProcessCmd {
    fn run(&self, opts: &Opts) {
        let src = File::open(&self.filename).expect("opening gcode file failed");
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = planner::Planner::default();
        planner.toolhead_state.limits = opts.printer_limits().clone();

        for cmd in rdr {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(cmd);
        }

        planner.finalize();

        let mut total_time = 0.0;
        for (i, c) in planner.move_sequences.iter_mut().enumerate() {
            total_time += 0.25 + c.iter().map(|m| m.total_time()).sum::<f64>();
        }
        println!("Total print time: {}", total_time);
    }
}

fn main() {
    let opts = Opts::parse();
    opts.cmd.run(&opts);
}

fn format_time(mut seconds: f64) -> String {
    let mut parts = Vec::new();

    if seconds > 86400.0 {
        parts.push(format!("{}d", (seconds / 86400.0).floor()));
        seconds %= 86400.0;
    }
    if seconds > 3600.0 {
        parts.push(format!("{}h", (seconds / 3600.0).floor()));
        seconds %= 3600.0;
    }
    if seconds > 60.0 {
        parts.push(format!("{}m", (seconds / 60.0).floor()));
        seconds %= 60.0;
    }
    if seconds > 0.0 {
        parts.push(format!("{:.3}s", seconds));
    }

    if parts.is_empty() {
        return "0s".into();
    }

    parts.join("")
}
