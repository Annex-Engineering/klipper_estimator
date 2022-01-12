use std::collections::{BTreeMap, VecDeque};
use std::error::Error;
use std::f64::EPSILON;
use std::ffi::OsString;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use klipper_estimator::gcode::{
    parse_gcode, GCodeCommand, GCodeOperation, GCodeReader, GCodeTraditionalParams,
};
use klipper_estimator::glam::{DVec2, DVec3, Vec4Swizzles};
use klipper_estimator::planner::{
    FirmwareRetractionOptions, MoveChecker, Planner, PlanningOperation, PrinterLimits,
};

use clap::Parser;
use once_cell::sync::OnceCell;
use regex::Regex;
use serde::Deserialize;
#[macro_use]
extern crate lazy_static;

#[derive(Parser, Debug)]
#[clap(version = env!("VERGEN_GIT_SEMVER_LIGHTWEIGHT"), author = "Lasse Dalegaard <dalegaard@gmail.com>")]
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

#[derive(Parser, Debug)]
struct DumpConfigCmd;

impl DumpConfigCmd {
    fn run(&self, opts: &Opts) {
        let _ = serde_json::to_writer_pretty(std::io::stdout(), &opts.printer_limits());
    }
}

#[derive(Parser, Debug)]
struct EstimateCmd {
    input: String,
    #[clap(long = "dump_moves")]
    dump_moves: bool,
    #[clap(long = "dump_summary")]
    dump_summary: bool,
}

impl EstimateCmd {
    fn run(&self, opts: &Opts) {
        let src: Box<dyn std::io::Read> = match self.input.as_str() {
            "-" => Box::new(std::io::stdin()),
            filename => Box::new(File::open(filename).expect("opening gcode file failed")),
        };
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = opts.make_planner();

        for cmd in rdr {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(&cmd);
        }

        planner.finalize();

        println!("Sequences:");

        let mut layer_times = BTreeMap::new();
        let mut kind_times = BTreeMap::new();

        let ops: Vec<_> = planner.iter().collect();
        let cross_section = std::f64::consts::PI * (1.75f64 / 2.0).powf(2.0);
        let mut move_idx = 0;
        for (i, moves) in ops.split(|o| !o.is_move()).enumerate() {
            let moves: Vec<_> = moves.iter().flat_map(|o| o.get_move()).collect();

            println!(" Run {}:", i);
            println!("  Total moves: {}", moves.len());
            println!(
                "  Total distance: {}",
                moves.iter().map(|m| m.distance).sum::<f64>()
            );
            let extrude_distance = moves.iter().map(|m| m.end.w - m.start.w).sum::<f64>();
            println!("  Total extrude distance: {}", extrude_distance);
            let min_time = 0.25 + moves.iter().map(|m| m.total_time()).sum::<f64>();
            let phase_times = [
                moves.iter().map(|m| m.accel_time()).sum::<f64>(),
                moves.iter().map(|m| m.cruise_time()).sum::<f64>(),
                moves.iter().map(|m| m.decel_time()).sum::<f64>(),
            ];

            println!("  Minimal time: {} ({})", format_time(min_time), min_time);
            println!(
                "  Average flow: {} mm3/s",
                extrude_distance * cross_section / min_time
            );
            println!("  Phases:");
            println!("    Acceleration: {}", format_time(phase_times[0]));
            println!("    Cruise:       {}", format_time(phase_times[1]));
            println!("    Deceleration: {}", format_time(phase_times[2]));

            println!("  Moves:");
            let width = (moves.len() as f64).log10().ceil() as usize;
            let mut ctime = 0.25;
            let mut ztime = 0.0;
            for m in moves.iter() {
                let i = move_idx;
                move_idx += 1;
                if self.dump_summary {
                    println!(
                        "SUM {:9}[] {:.3} / {:.3} / {:.3}",
                        i, m.start_v, m.cruise_v, m.end_v
                    );
                }
                if self.dump_moves {
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
                        m.rate.xy().angle_between(DVec2::new(1.0, 0.0)) * 180.0
                            / std::f64::consts::PI,
                    );
                    println!("    Axes {:?}", m.rate);
                    println!("    Line width: {:?}", m.line_width(1.75 / 2.0, 0.25),);
                    println!("    Flow rate: {:?}", m.flow_rate(1.75 / 2.0));
                    println!("    Kind: {:?}", planner.move_kind(m));
                    println!("    Acceleration {:?}", m.acceleration);
                    println!("    Max dv2: {}", m.max_dv2);
                    println!("    Max start_v2: {}", m.max_start_v2);
                    println!("    Max cruise_v2: {}", m.max_cruise_v2);
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
                }

                if (m.start.z - m.end.z).abs() < EPSILON {
                    *layer_times
                        .entry((m.start.z * 1000.0).round() as usize)
                        .or_insert(0.0) += m.total_time();
                } else {
                    ztime += m.total_time();
                }

                let move_kind = planner.move_kind(m).unwrap_or("Other");
                if let Some(t) = kind_times.get_mut(move_kind) {
                    *t += m.total_time();
                } else {
                    kind_times.insert(move_kind.to_string(), m.total_time());
                }
            }
        }
        println!("  Layer times:");
        for (z, t) in layer_times.iter() {
            println!("   {:7} => {}", (*z as f64) / 1000.0, format_time(*t));
        }

        println!("  Kind times:");
        for (k, t) in kind_times.iter() {
            println!("   {:20} => {}", format_time(*t), k);
        }
    }
}

#[derive(Parser, Debug)]
struct PostProcessCmd {
    #[clap(parse(try_from_str))]
    filename: PathBuf,
}

#[derive(Debug, PartialOrd, PartialEq, Clone)]
pub enum Metadata {}

trait MetadataReceiver {
    fn metadata_tag(&mut self, tag: Metadata);
}

trait GCodeInterceptor: std::fmt::Debug {
    fn need_sync(&self, _command: &GCodeCommand, _result: &PostProcessEstimationResult) -> bool {
        false
    }

    fn post_command(&mut self, _command: &GCodeCommand, _result: &mut PostProcessEstimationResult) {
    }

    fn output_process(
        &mut self,
        _command: &GCodeCommand,
        _result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        None
    }
}

#[derive(Debug, Default)]
struct NoopGCodeInterceptor {}

impl GCodeInterceptor for NoopGCodeInterceptor {}

#[derive(Debug, Default)]
struct M73GcodeInterceptor {
    time_buffer: VecDeque<f64>,
}

impl GCodeInterceptor for M73GcodeInterceptor {
    fn need_sync(&self, command: &GCodeCommand, _result: &PostProcessEstimationResult) -> bool {
        matches!(
            command.op,
            GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                ..
            }
        )
    }

    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        if matches!(
            command.op,
            GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                ..
            }
        ) {
            self.time_buffer.push_back(result.total_time);
        }
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if !matches!(
            command.op,
            GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                ..
            }
        ) {
            return None;
        }
        let next = self.time_buffer.pop_front()?;
        let params = vec![
            (
                'P',
                format!("{}", (next / result.total_time * 100.0).round()),
            ),
            (
                'R',
                format!("{}", ((result.total_time - next) / 60.0).round()),
            ),
        ];
        Some(GCodeCommand {
            op: GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                params: GCodeTraditionalParams::from_vec(params),
            },
            comment: None,
        })
    }
}

#[derive(Debug, Default)]
struct PSSSGCodeInterceptor {
    m73_interceptor: M73GcodeInterceptor,
}

impl PSSSGCodeInterceptor {
    fn format_dhms(mut time: f64) -> String {
        use std::fmt::Write;
        let mut out = String::new();
        time = time.ceil();
        let d = (time / 86400.0).floor();
        if d > 0.0 {
            write!(out, " {:.0}d", d).unwrap();
        }
        time %= 86400.0;
        let h = (time / 3600.0).floor();
        if h > 0.0 {
            write!(out, " {:.0}h", h).unwrap();
        }
        time %= 3600.0;
        let m = (time / 60.0).floor();
        if m > 0.0 {
            write!(out, " {:.0}m", m).unwrap();
        }
        time %= 60.0;
        let s = time;
        write!(out, " {:.0}s", s).unwrap();
        out
    }
}

impl GCodeInterceptor for PSSSGCodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        self.m73_interceptor.post_command(command, result);
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        lazy_static! {
            static ref RE_EST_TIME: Regex =
                Regex::new(r"^ estimated printing time \(.*?\) =").unwrap();
        }

        if let Some(cmd) = self.m73_interceptor.output_process(command, result) {
            return Some(cmd);
        }

        if let Some(com) = &command.comment {
            if let Some(c) = RE_EST_TIME.captures(&com) {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!(
                        "{}{}",
                        c.get(0).unwrap().as_str(),
                        Self::format_dhms(result.total_time)
                    )),
                });
            }
        }

        None
    }
}

#[derive(Debug, Default)]
struct IdeaMakerGCodeInterceptor {
    time_buffer: VecDeque<f64>,
}

impl GCodeInterceptor for IdeaMakerGCodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        if let Some(com) = &command.comment {
            if com.starts_with("PRINTING_TIME: ") {
                self.time_buffer.push_back(result.total_time);
            }
        }
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if let Some(com) = &command.comment {
            if com.starts_with("Print Time: ") {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("Print Time: {:.0}", result.total_time.ceil())),
                });
            } else if com.starts_with("PRINTING_TIME: ") {
                if let Some(next) = self.time_buffer.front() {
                    return Some(GCodeCommand {
                        op: GCodeOperation::Nop,
                        comment: Some(format!("PRINTING_TIME: {:.0}", next.ceil())),
                    });
                }
            } else if com.starts_with("REMAINING_TIME: ") {
                if let Some(next) = self.time_buffer.pop_front() {
                    return Some(GCodeCommand {
                        op: GCodeOperation::Nop,
                        comment: Some(format!(
                            "REMAINING_TIME: {:.0}",
                            (result.total_time - next).ceil()
                        )),
                    });
                }
            }
        }
        None
    }
}

#[derive(Debug, Clone)]
enum SlicerPreset {
    PrusaSlicer { version: String },
    SuperSlicer { version: String },
    IdeaMaker { version: String },
}

impl SlicerPreset {
    fn determine(comment: &str) -> Option<SlicerPreset> {
        None.or_else(|| Self::try_slic3r(comment))
            .or_else(|| Self::try_ideamaker(comment))
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

    fn metadata_processor(&self) -> Box<dyn GCodeInterceptor> {
        match self {
            SlicerPreset::PrusaSlicer { .. } => Box::new(PSSSGCodeInterceptor::default()),
            SlicerPreset::SuperSlicer { .. } => Box::new(PSSSGCodeInterceptor::default()),
            SlicerPreset::IdeaMaker { .. } => Box::new(IdeaMakerGCodeInterceptor::default()),
        }
    }
}

#[derive(Debug)]
struct PostProcessEstimationResult {
    total_time: f64,
    slicer: Option<SlicerPreset>,
}

impl std::default::Default for PostProcessEstimationResult {
    fn default() -> Self {
        PostProcessEstimationResult {
            total_time: 0.25,
            slicer: None,
        }
    }
}

#[derive(Debug)]
struct PostProcessState {
    result: PostProcessEstimationResult,
    gcode_interceptor: Box<dyn GCodeInterceptor>,
}

impl std::default::Default for PostProcessState {
    fn default() -> Self {
        PostProcessState {
            result: PostProcessEstimationResult::default(),
            gcode_interceptor: Box::new(NoopGCodeInterceptor::default()),
        }
    }
}

impl MetadataReceiver for PostProcessEstimationResult {
    fn metadata_tag(&mut self, tag: Metadata) {}
}

impl PostProcessCmd {
    fn estimate(&self, opts: &Opts) -> PostProcessState {
        let mut state = PostProcessState::default();

        let src = File::open(&self.filename).expect("opening gcode file failed");
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = opts.make_planner();

        for (n, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");

            if cmd.op.is_nop() && cmd.comment.is_some() && state.result.slicer.is_none() {
                state.result.slicer = SlicerPreset::determine(cmd.comment.as_ref().unwrap());
                if let Some(preset) = state.result.slicer.as_ref() {
                    state.gcode_interceptor = preset.metadata_processor();
                }
            }

            planner.process_cmd(&cmd);

            let flush = state.gcode_interceptor.need_sync(&cmd, &state.result);

            if flush || n % 1000 == 0 {
                for c in planner.iter() {
                    match c {
                        PlanningOperation::Dwell(t) => state.result.total_time += t,
                        PlanningOperation::Move(m) => state.result.total_time += m.total_time(),
                    }
                }
            }

            state
                .gcode_interceptor
                .post_command(&cmd, &mut state.result);
        }

        planner.finalize();

        for c in planner.iter() {
            match c {
                PlanningOperation::Dwell(t) => state.result.total_time += t,
                PlanningOperation::Move(m) => state.result.total_time += m.total_time(),
            }
        }

        state
    }

    fn apply_changes(&self, mut state: PostProcessState) {
        let src = File::open(&self.filename).expect("opening gcode file failed");
        let rdr = BufReader::new(src);

        let mut dst_name = Into::<OsString>::into(".estimate.");
        dst_name.push(self.filename.file_name().expect("invalid file name"));
        let dst_path = self
            .filename
            .parent()
            .unwrap_or(Path::new("/"))
            .join(dst_name);
        let dst = File::create(&dst_path).expect("creating target gcode file failed");
        let mut wr = BufWriter::new(dst);

        for line in rdr.lines() {
            let line = line.expect("IO error");
            if let Ok(cmd) = parse_gcode(&line) {
                if let Some(cmd) = state.gcode_interceptor.output_process(&cmd, &state.result) {
                    write!(wr, "{}\n", cmd).expect("IO error");
                } else {
                    write!(wr, "{}\n", line).expect("IO error");
                }
            } else {
                write!(wr, "{}\n", line).expect("IO error");
            }
        }

        std::fs::rename(&dst_path, &self.filename).expect("rename failed");
    }

    fn run(&self, opts: &Opts) {
        let state = self.estimate(opts);
        println!("{:?}", state);
        self.apply_changes(state);
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
