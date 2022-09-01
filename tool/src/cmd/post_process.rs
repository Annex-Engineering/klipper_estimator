use std::collections::VecDeque;
use std::ffi::OsString;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use clap::Parser;
use regex::Regex;

use lib_klipper::gcode::{
    parse_gcode, GCodeCommand, GCodeOperation, GCodeReader, GCodeTraditionalParams,
};
use lib_klipper::planner::{Planner, PlanningOperation};
use lib_klipper::slicer::SlicerPreset;

use crate::Opts;

#[derive(Parser, Debug)]
pub struct PostProcessCmd {
    #[clap(parse(try_from_str))]
    filename: PathBuf,
}

trait GCodeInterceptor: std::fmt::Debug {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        let _ = command;
        let _ = result;
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        let _ = command;
        let _ = result;
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
            ('P', format!("{:.3}", (next / result.total_time * 100.0))),
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
            if let Some(c) = RE_EST_TIME.captures(com) {
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

#[derive(Debug, Default)]
struct CuraGCodeInterceptor {
    time_buffer: VecDeque<f64>,
}

impl GCodeInterceptor for CuraGCodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        if let Some(com) = &command.comment {
            if com.starts_with("TIME_ELAPSED:") {
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
            if com.starts_with("TIME:") {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("TIME:{:.0}", result.total_time.ceil())),
                });
            } else if com.starts_with("PRINT.TIME:") {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("PRINT.TIME:{:.0}", result.total_time.ceil())),
                });
            } else if com.starts_with("TIME_ELAPSED:") {
                if let Some(next) = self.time_buffer.pop_front() {
                    return Some(GCodeCommand {
                        op: GCodeOperation::Nop,
                        comment: Some(format!("TIME_ELAPSED:{:.0}", (next).ceil())),
                    });
                }
            }
        }
        None
    }
}

fn metadata_processor(preset: &SlicerPreset) -> Box<dyn GCodeInterceptor> {
    match preset {
        SlicerPreset::PrusaSlicer { .. } => Box::new(PSSSGCodeInterceptor::default()),
        SlicerPreset::SuperSlicer { .. } => Box::new(PSSSGCodeInterceptor::default()),
        SlicerPreset::IdeaMaker { .. } => Box::new(IdeaMakerGCodeInterceptor::default()),
        SlicerPreset::Cura { .. } => Box::new(CuraGCodeInterceptor::default()),
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
            total_time: 0.0,
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

#[derive(Debug)]
struct EstimateRunner {
    state: PostProcessState,
    planner: Planner,
    // We use this buffer to synchronize planned moves with input moves
    buffer: VecDeque<(usize, GCodeCommand)>,
}

impl EstimateRunner {
    fn run<T: BufRead>(&mut self, rdr: &mut GCodeReader<T>) {
        for (n, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");

            // If we don't have a slicer figured out yet, and this is a comment, try
            if cmd.op.is_nop() && cmd.comment.is_some() && self.state.result.slicer.is_none() {
                self.state.result.slicer = SlicerPreset::determine(cmd.comment.as_ref().unwrap());
                if let Some(preset) = self.state.result.slicer.as_ref() {
                    self.state.gcode_interceptor = metadata_processor(preset);
                }
            }

            let x = self.planner.process_cmd(&cmd);
            self.buffer.push_back((x, cmd));

            if n % 1000 == 0 {
                self.flush();
            }
        }

        self.planner.finalize();
        self.flush();
    }

    fn flush(&mut self) {
        for c in self.planner.iter() {
            let (n, cmd) = self.buffer.front_mut().unwrap();
            match c {
                PlanningOperation::Delay(d) => {
                    self.state.result.total_time += d.duration().as_secs_f64()
                }
                PlanningOperation::Move(m) => self.state.result.total_time += m.total_time(),
                PlanningOperation::Fill => {}
            }
            self.state
                .gcode_interceptor
                .post_command(cmd, &mut self.state.result);
            if *n <= 1 {
                let _ = self.buffer.pop_front();
            } else {
                *n -= 1;
            }
        }
    }
}

impl PostProcessCmd {
    fn estimate(&self, opts: &Opts) -> PostProcessState {
        let src = File::open(&self.filename).expect("opening gcode file failed");
        let mut rdr = GCodeReader::new(BufReader::new(src));

        let mut runner = EstimateRunner {
            state: PostProcessState::default(),
            planner: opts.make_planner(),
            buffer: VecDeque::new(),
        };
        runner.run(&mut rdr);
        runner.state
    }

    fn apply_changes(&self, mut state: PostProcessState) {
        let src = File::open(&self.filename).expect("opening gcode file failed");
        let rdr = BufReader::new(src);

        let mut dst_name = Into::<OsString>::into(".estimate.");
        dst_name.push(self.filename.file_name().expect("invalid file name"));
        let dst_path = self
            .filename
            .parent()
            .unwrap_or_else(|| Path::new("/"))
            .join(dst_name);
        let dst = File::create(&dst_path).expect("creating target gcode file failed");
        let mut wr = BufWriter::new(dst);

        for line in rdr.lines() {
            let line = line.expect("IO error");
            if let Ok(cmd) = parse_gcode(&line) {
                if let Some(cmd) = state.gcode_interceptor.output_process(&cmd, &state.result) {
                    writeln!(wr, "{}", cmd).expect("IO error");
                } else {
                    writeln!(wr, "{}", line).expect("IO error");
                }
            } else {
                writeln!(wr, "{}", line).expect("IO error");
            }
        }

        write!(
            wr,
            "; Processed by klipper_estimator {}, {}",
            env!("TOOL_VERSION"),
            if let Some(slicer) = state.result.slicer {
                format!("detected slicer {}", slicer)
            } else {
                "no slicer detected".into()
            }
        )
        .expect("IO error");

        // Flush output file before renaming
        wr.flush().expect("IO error");
        std::fs::rename(&dst_path, &self.filename).expect("rename failed");
    }

    pub fn run(&self, opts: &Opts) {
        let state = self.estimate(opts);
        self.apply_changes(state);
    }
}
