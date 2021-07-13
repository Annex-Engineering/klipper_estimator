mod gcode;
mod planner;

use std::fs::File;
use std::io::BufReader;

use gcode::{GCodeCommand, GCodeOperation, GCodeReader};

use structopt::StructOpt;
#[derive(Debug, StructOpt)]
#[structopt(
    name = "klipper_estimater",
    about = "Calculates various print statistics using Klipper kinematics"
)]
struct Opt {
    #[structopt(name = "INPUT")]
    file_name: String,

    #[structopt(subcommand)]
    cmd: Command,
}

#[derive(Debug, StructOpt)]
enum Command {
    PrintTime,
}

fn main() {
    let opt = Opt::from_args();
    println!("{:?}", opt);

    let src: Box<dyn std::io::Read> = match opt.file_name.as_str() {
        "-" => Box::new(std::io::stdin()),
        filename => Box::new(File::open(filename).expect("opening gcode file failed")),
    };
    let rdr = GCodeReader::new(BufReader::new(src));

    let mut planner = planner::Planner::default();
    planner.toolhead_state.limits.set_max_velocity(800.0);
    planner.toolhead_state.limits.set_max_acceleration(7000.0);
    planner.toolhead_state.limits.set_max_accel_to_decel(7000.0);
    planner
        .toolhead_state
        .limits
        .set_square_corner_velocity(5.0);

    planner
        .toolhead_state
        .move_checkers
        .push(Box::new(planner::KinematicCartesian {
            max_z_velocity: 20.0,
            max_z_accel: 50.0,
        }));

    planner
        .toolhead_state
        .move_checkers
        .push(Box::new(planner::KinematicExtruder {
            max_velocity: 30.0,
            max_accel: 3000.0,
        }));

    for cmd in rdr {
        let cmd = cmd.expect("gcode read");
        planner.process_cmd(cmd);
    }

    planner.finalize();

    println!("Sequences:");
    for (i, c) in planner.move_sequences.iter_mut().enumerate() {
        println!(" Run {}:", i);
        println!("  Total moves: {}", c.moves.len());
        println!(
            "  Total distance: {}",
            c.moves.iter().map(|m| m.distance).sum::<f64>()
        );
        println!(
            "  Total extrude distance: {}",
            c.moves.iter().map(|m| m.end.w - m.start.w).sum::<f64>()
        );
        let min_time = 0.25 + c.moves.iter().map(|m| m.total_time()).sum::<f64>();
        println!("  Minimal time: {} ({})", format_time(min_time), min_time);
    }

    //     println!("  Moves:");
    //     let width = (c.moves.len() as f64).log10().ceil() as usize;
    //     let mut layer_times = BTreeMap::new();
    //     let mut ctime = 0.25;
    //     for (i, m) in c.moves.iter().enumerate() {
    //         let mut kind = String::new();
    //         if m.is_extrude_move() {
    //             kind.push('E');
    //         }
    //         if m.is_kinematic_move() {
    //             kind.push('K');
    //         }
    //         println!(
    //             "   {:width$}[{}] @ {:.8} => {:.8}:",
    //             i,
    //             kind,
    //             ctime,
    //             ctime + m.total_time(),
    //             width = width
    //         );
    //         println!(
    //             "    Path:       {:?} => {:?} [{:.3}∠{:.2}]",
    //             m.start,
    //             m.end,
    //             m.distance,
    //             m.rate.xy().angle_between(glam::DVec2::new(1.0, 0.0)) * 180.0
    //                 / std::f64::consts::PI,
    //         );
    //         println!("    Axes {:?}", m.rate);
    //         println!("    Line widht: {:?}", m.line_width(1.75 / 2.0, 0.25));
    //         println!("    Acceleration {:?}", m.acceleration);
    //         println!("    Max dv2: {}", m.max_dv2);
    //         println!("    Max start_v2: {}", m.max_start_v2);
    //         println!("    Max smoothed_v2: {}", m.max_smoothed_v2);
    //         println!(
    //             "    Velocity:   {} / {} / {}",
    //             m.start_v, m.cruise_v, m.end_v
    //         );
    //         println!(
    //             "    Time:       {:4}+{:4}+{:4}  = {:4}",
    //             m.accel_time(),
    //             m.cruise_time(),
    //             m.decel_time(),
    //             m.total_time(),
    //         );
    //         // println!("    CT {}", ctime.to_bits());
    //         ctime += m.total_time();

    //         println!(
    //             "    Distances:  {:.3}+{:.3}+{:.3} = {:.3}",
    //             m.accel_distance(),
    //             m.cruise_distance(),
    //             m.decel_distance(),
    //             m.distance
    //         );

    //         println!();

    //         if (m.start.z - m.end.z).abs() < EPSILON {
    //             *layer_times
    //                 .entry((m.start.z * 1000.0) as usize)
    //                 .or_insert(0.0) += m.total_time();
    //         }

    //         // println!(
    //         //     "{:width$} BT {}",
    //         //     i,
    //         //     m.total_time().to_bits(),
    //         //     width = width
    //         // );
    //     }

    //     println!("  Layer times:");
    //     for (z, t) in layer_times.iter() {
    //         println!("   {:7} => {}", (*z as f64) / 1000.0, format_time(*t));
    //     }
    // }
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
