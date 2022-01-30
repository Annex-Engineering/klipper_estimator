use std::collections::BTreeMap;
use std::f64::EPSILON;
use std::fs::File;
use std::io::BufReader;

use lib_klipper::gcode::GCodeReader;
use lib_klipper::glam::{DVec2, Vec4Swizzles};
use lib_klipper::planner::{Planner, PlanningMove, PlanningOperation};

use clap::Parser;
use ordered_float::NotNan;
use serde::{ser::SerializeSeq, Serialize, Serializer};

use crate::Opts;

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

#[derive(clap::ArgEnum, Debug, Clone, Copy, Eq, PartialEq)]
pub enum OutputFormat {
    Human,
    JSON,
}

#[derive(Parser, Debug)]
pub struct EstimateCmd {
    input: String,
    #[clap(arg_enum, long, short, default_value_t = OutputFormat::Human)]
    format: OutputFormat,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationState {
    sequences: Vec<EstimationSequence>,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationSequence {
    total_time: f64,
    total_distance: f64,
    total_extrude_distance: f64,
    num_moves: usize,
    total_z_time: f64,
    total_output_time: f64,
    total_travel_time: f64,
    total_extrude_only_time: f64,
    phase_times: EstimationPhaseTimes,
    kind_times: BTreeMap<String, f64>,
    #[serde(serialize_with = "serialize_layer_times")]
    layer_times: BTreeMap<NotNan<f64>, f64>,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationPhaseTimes {
    acceleration: f64,
    cruise: f64,
    deceleration: f64,
}

fn serialize_layer_times<S: Serializer>(
    lts: &BTreeMap<NotNan<f64>, f64>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    let mut seq = serializer.serialize_seq(Some(lts.len()))?;

    for (z, t) in lts {
        seq.serialize_element(&[z, t])?;
    }

    seq.end()
}

impl EstimationState {
    fn add(&mut self, planner: &Planner, op: &PlanningOperation) {
        match op {
            PlanningOperation::Move(m) => self.add_move(planner, m),
            PlanningOperation::Dwell(t) => {
                // If current sequence has moves or there is no sequence, make a new one
                if self
                    .sequences
                    .last()
                    .map(|s| s.num_moves != 0)
                    .unwrap_or(true)
                {
                    self.sequences.push(EstimationSequence::default());
                }
                self.sequences.last_mut().unwrap().total_time += t;
            }
            _ => {}
        }
    }

    fn get_cur_seq(&mut self) -> &mut EstimationSequence {
        if self.sequences.is_empty() {
            self.sequences.push(EstimationSequence::default());
        }
        self.sequences.last_mut().unwrap()
    }

    fn add_move(&mut self, planner: &Planner, m: &PlanningMove) {
        let seq = self.get_cur_seq();
        if seq.total_time == 0.0 && seq.num_moves == 0 {
            seq.total_time += 0.25;
        }

        seq.total_time += m.total_time();
        seq.total_distance += m.distance;
        seq.total_extrude_distance += m.end.w - m.start.w;
        seq.num_moves += 1;

        match (m.is_extrude_move(), m.is_kinematic_move()) {
            (true, true) => seq.total_output_time += m.total_time(),
            (true, false) => seq.total_extrude_only_time += m.total_time(),
            (false, true) => seq.total_travel_time += m.total_time(),
            _ => {}
        }

        {
            let pt = &mut seq.phase_times;
            pt.acceleration += m.accel_time();
            pt.cruise += m.cruise_time();
            pt.deceleration += m.decel_time();
        }

        let kind = planner.move_kind(m).unwrap_or("Other");
        if let Some(t) = seq.kind_times.get_mut(kind) {
            *t += m.total_time();
        } else {
            seq.kind_times.insert(kind.to_string(), m.total_time());
        }

        if (m.start.z - m.end.z).abs() < EPSILON {
            *seq.layer_times
                .entry(NotNan::new((m.start.z * 1000.0).round() / 1000.0).unwrap())
                .or_insert(0.0) += m.total_time();
        } else {
            seq.total_z_time += m.total_time();
        }
    }
}

impl EstimateCmd {
    pub fn run(&self, opts: &Opts) {
        let src: Box<dyn std::io::Read> = match self.input.as_str() {
            "-" => Box::new(std::io::stdin()),
            filename => Box::new(File::open(filename).expect("opening gcode file failed")),
        };
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = opts.make_planner();
        let mut state = EstimationState::default();

        for (i, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(&cmd);

            if i % 1000 == 0 {
                for o in planner.iter().collect::<Vec<_>>() {
                    state.add(&planner, &o);
                }
            }
        }

        planner.finalize();
        for o in planner.iter().collect::<Vec<_>>() {
            state.add(&planner, &o);
        }

        match self.format {
            OutputFormat::Human => {
                println!("Sequences:");

                let cross_section = std::f64::consts::PI * (1.75f64 / 2.0).powf(2.0);
                for (i, seq) in state.sequences.iter().enumerate() {
                    if i > 0 {
                        println!("");
                    }
                    println!(" Run {}:", i);
                    println!("  Total moves:                 {}", seq.num_moves);
                    println!("  Total distance:              {:.3}mm", seq.total_distance);
                    println!(
                        "  Total extrude distance:      {:.3}mm",
                        seq.total_extrude_distance
                    );
                    println!(
                        "  Minimal time:                {} ({:.3}s)",
                        format_time(seq.total_time),
                        seq.total_time
                    );
                    println!(
                        "  Total print move time:       {} ({:.3}s)",
                        format_time(seq.total_output_time),
                        seq.total_output_time
                    );
                    println!(
                        "  Total extrude-only time:     {} ({:.3}s)",
                        format_time(seq.total_extrude_only_time),
                        seq.total_extrude_only_time
                    );
                    println!(
                        "  Total travel time:           {} ({:.3}s)",
                        format_time(seq.total_travel_time),
                        seq.total_travel_time
                    );
                    println!(
                        "  Average flow:                {:.3} mm³/s",
                        seq.total_extrude_distance * cross_section / seq.total_time
                    );
                    println!(
                        "  Average flow (output only):  {:.3} mm³/s",
                        seq.total_extrude_distance * cross_section / seq.total_output_time
                    );
                    println!("  Phases:");
                    println!(
                        "   Acceleration:               {}",
                        format_time(seq.phase_times.acceleration)
                    );
                    println!(
                        "   Cruise:                     {}",
                        format_time(seq.phase_times.cruise)
                    );
                    println!(
                        "   Deceleration:               {}",
                        format_time(seq.phase_times.deceleration)
                    );

                    let mut kind_times = seq.kind_times.iter().collect::<Vec<_>>();
                    if !kind_times.is_empty() {
                        println!("  Move kind distribution:");
                        kind_times.sort_by_key(|(_, t)| NotNan::new(**t).unwrap());
                        let kind_length = kind_times
                            .iter()
                            .map(|(_, t)| format_time(**t).len())
                            .max()
                            .unwrap_or(0);
                        for (k, t) in kind_times.iter().rev() {
                            println!("   {:kind_length$}     {}", format_time(**t), k);
                        }
                    }

                    let layer_times = seq
                        .layer_times
                        .iter()
                        .map(|(l, t)| (format!("{l:.3}"), format_time(*t)))
                        .collect::<Vec<_>>();
                    if !layer_times.is_empty() {
                        println!("  Layer time distribution:");
                        let longest_z = layer_times.iter().map(|(z, _)| z.len()).max().unwrap_or(0);
                        let longest_t = layer_times.iter().map(|(_, t)| t.len()).max().unwrap_or(0);
                        let colon = ": ";
                        let column = longest_z + longest_t + colon.len();
                        let offset = " ".repeat(3);
                        let spacing = " ".repeat(4);

                        let term_width = term_size::dimensions().map(|(w, _)| w).unwrap_or(0);
                        let available_width = (term_width - offset.len()).max(0);

                        let num_columns =
                            ((available_width - column) / (column + spacing.len()) + 1).max(1);
                        let chunk_size = layer_times.len() / num_columns
                            + usize::from(layer_times.len() % num_columns != 0);
                        let columnized = layer_times.chunks(chunk_size).collect::<Vec<_>>();
                        for line in 0.. {
                            if columnized
                                .iter()
                                .map(|c| c.len().saturating_sub(line))
                                .max()
                                .unwrap_or(0)
                                == 0
                            {
                                break;
                            }

                            print!("{offset}");
                            for i in 0..num_columns {
                                if let Some((t, l)) =
                                    columnized.get(i).and_then(|col| col.get(line))
                                {
                                    if i > 0 {
                                        print!("{spacing}");
                                    }
                                    print!("{t:>longest_z$}{colon}{l:>longest_t$}");
                                }
                            }
                            println!("");
                        }
                    }
                }
            }
            OutputFormat::JSON => {
                serde_json::to_writer_pretty(std::io::stdout(), &state)
                    .expect("Serialization error");
            }
        }
    }
}

#[derive(Parser, Debug)]
pub struct DumpMovesCmd {
    input: String,
}

#[derive(Debug)]
struct DumpMovesState {
    move_idx: usize,
    ctime: f64,
    ztime: f64,
}

impl DumpMovesState {
    fn flush(&mut self, planner: &mut Planner) {
        for o in planner.iter().collect::<Vec<_>>() {
            let m = match o.get_move() {
                Some(m) => m,
                None => continue,
            };
            self.move_idx += 1;

            let mut kind = String::new();
            if m.is_extrude_move() {
                kind.push('E');
            }
            if m.is_kinematic_move() {
                kind.push('K');
            }
            println!(
                "N{}[{}] @ {:.8} => {:.8} / z{:.8}:",
                self.move_idx,
                kind,
                self.ctime,
                self.ctime + m.total_time(),
                self.ztime,
            );
            println!(
                "    Path:       {} => {} [{:.3}∠{:.2}]",
                (m.start * 1000.0).round() / 1000.0,
                (m.end * 1000.0).round() / 1000.0,
                m.distance,
                m.rate.xy().angle_between(DVec2::new(1.0, 0.0)) * 180.0 / std::f64::consts::PI,
            );
            println!("    Axes {}", (m.rate * 1000.0).round() / 1000.0);
            println!("    Line width: {:?}", m.line_width(1.75 / 2.0, 0.25),);
            println!("    Flow rate: {:?}", m.flow_rate(1.75 / 2.0));
            println!("    Kind: {}", planner.move_kind(&m).unwrap_or("Other"));
            println!("    Acceleration {:.4}", m.acceleration);
            println!("    Max dv2: {:.4}", m.max_dv2);
            println!("    Max start_v2: {:.4}", m.max_start_v2);
            println!("    Max cruise_v2: {:.4}", m.max_cruise_v2);
            println!("    Max smoothed_v2: {:.4}", m.max_smoothed_v2);
            println!(
                "    Velocity:   {:.3} => {:.3} => {:.3}",
                m.start_v, m.cruise_v, m.end_v
            );
            println!(
                "    Time:       {:.4}+{:.4}+{:.4} = {:.4}",
                m.accel_time(),
                m.cruise_time(),
                m.decel_time(),
                m.total_time(),
            );
            self.ctime += m.total_time();

            println!(
                "    Distances:  {:.3}+{:.3}+{:.3} = {:.3}",
                m.accel_distance(),
                m.cruise_distance(),
                m.decel_distance(),
                m.distance
            );

            println!();

            self.ztime += m.total_time();
        }
    }
}

impl DumpMovesCmd {
    pub fn run(&self, opts: &Opts) {
        let src: Box<dyn std::io::Read> = match self.input.as_str() {
            "-" => Box::new(std::io::stdin()),
            filename => Box::new(File::open(filename).expect("opening gcode file failed")),
        };
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = opts.make_planner();
        let mut state = DumpMovesState {
            move_idx: 0,
            ctime: 0.25,
            ztime: 0.0,
        };

        for (i, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(&cmd);

            if i % 1000 == 0 {
                state.flush(&mut planner);
            }
        }
        planner.finalize();
        state.flush(&mut planner);
    }
}
