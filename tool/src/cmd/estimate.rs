use std::collections::BTreeMap;
use std::f64::EPSILON;
use std::fs::File;
use std::io::BufReader;

use klipper_estimator::gcode::GCodeReader;
use klipper_estimator::glam::{DVec2, Vec4Swizzles};

use clap::Parser;

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
#[derive(Parser, Debug)]
pub struct EstimateCmd {
    input: String,
    #[clap(long = "dump_moves")]
    dump_moves: bool,
    #[clap(long = "dump_summary")]
    dump_summary: bool,
}

impl EstimateCmd {
    pub fn run(&self, opts: &Opts) {
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

        let ops: Vec<_> = planner.iter().filter(|op| !op.is_fill()).collect();
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
