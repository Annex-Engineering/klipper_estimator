mod gcode;

use std::collections::{BTreeMap, HashMap};
use std::f64::EPSILON;
use std::fs::File;
use std::io::BufReader;

use gcode::{GCodeCommand, GCodeOperation, GCodeReader};

use glam::{DVec4 as Vec4, Vec4Swizzles};

#[derive(Debug)]
struct PlanningMove {
    start: Vec4,
    end: Vec4,
    distance: f64,
    rate: Vec4,
    requested_velocity: f64,
    acceleration: f64,
    max_start_v2: f64,
    max_cruise_v2: f64,
    max_dv2: f64,
    max_smoothed_v2: f64,
    smoothed_dv2: f64,

    kind: Option<u16>,

    start_v: f64,
    cruise_v: f64,
    end_v: f64,
}

impl PlanningMove {
    /// Create a new `PlanningMove` that travels between the two points `start`
    /// and `end`.
    pub fn new(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
        if start.xyz() == end.xyz() {
            Self::new_extrude_move(start, end, toolhead_state)
        } else {
            Self::new_kinematic_move(start, end, toolhead_state)
        }
    }

    fn new_extrude_move(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
        let dirs = Vec4::new(0.0, 0.0, 0.0, end.w - start.w);
        let move_d = dirs.w.abs();
        let inv_move_d = if move_d > 0.0 { 1.0 / move_d } else { 0.0 };
        PlanningMove {
            start,
            end,
            distance: (start.w - end.w).abs(),
            rate: dirs * inv_move_d,
            requested_velocity: toolhead_state.velocity,
            acceleration: f64::MAX,
            max_start_v2: 0.0,
            max_cruise_v2: toolhead_state.velocity * toolhead_state.velocity,
            max_dv2: f64::MAX,
            max_smoothed_v2: 0.0,
            smoothed_dv2: f64::MAX,
            kind: None,

            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
        }
    }

    fn new_kinematic_move(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
        let distance = start.xyz().distance(end.xyz()); // Can't be zero
        let velocity = toolhead_state
            .velocity
            .min(toolhead_state.limits.max_velocity);

        PlanningMove {
            start,
            end,
            distance,
            rate: (end - start) / distance,
            requested_velocity: velocity,
            acceleration: toolhead_state.limits.max_acceleration,
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            max_dv2: 2.0 * distance * toolhead_state.limits.max_acceleration,
            max_smoothed_v2: 0.0,
            smoothed_dv2: 2.0 * distance * toolhead_state.limits.max_accel_to_decel,
            kind: None,

            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
        }
    }

    fn apply_junction(&mut self, previous_move: &PlanningMove, toolhead_state: &ToolheadState) {
        if !self.is_kinematic_move() || !previous_move.is_kinematic_move() {
            return;
        }

        let mut junction_cos_theta = -self.rate.xyz().dot(previous_move.rate.xyz());
        if junction_cos_theta > 0.99999 {
            // Move was not at an angle, skip all this
            return;
        }
        junction_cos_theta = junction_cos_theta.max(-0.999999);
        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).sqrt();
        let r = toolhead_state.limits.junction_deviation * sin_theta_d2 / (1.0 - sin_theta_d2);
        let tan_theta_d2 = sin_theta_d2 / (0.5 * (1.0 + junction_cos_theta)).sqrt();
        let move_centripetal_v2 = 0.5 * self.distance * tan_theta_d2 * self.acceleration;
        let prev_move_centripetal_v2 =
            0.5 * previous_move.distance * tan_theta_d2 * previous_move.acceleration;

        let extruder_v2 = toolhead_state.extruder_junction_speed_v2(self, previous_move);

        self.max_start_v2 = extruder_v2
            .min(r * self.acceleration)
            .min(r * previous_move.acceleration)
            .min(move_centripetal_v2)
            .min(prev_move_centripetal_v2)
            .min(self.max_cruise_v2)
            .min(previous_move.max_cruise_v2)
            .min(previous_move.max_start_v2 + previous_move.max_dv2);
        self.max_smoothed_v2 = self
            .max_start_v2
            .min(previous_move.max_smoothed_v2 + previous_move.smoothed_dv2);
    }

    fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        self.start_v = start_v2.sqrt();
        self.cruise_v = cruise_v2.sqrt();
        self.end_v = end_v2.sqrt();
    }

    fn is_kinematic_move(&self) -> bool {
        self.start.xyz() != self.end.xyz()
    }

    fn is_extrude_move(&self) -> bool {
        (self.end.w - self.start.w).abs() >= EPSILON
    }

    fn is_extrude_only_move(&self) -> bool {
        !self.is_kinematic_move() && self.is_extrude_move()
    }

    fn line_width(&self, nozzle_radius: f64, layer_height: f64) -> Option<f64> {
        // Only moves that are both extruding and moving have a line width
        if !self.is_kinematic_move() || !self.is_extrude_move() {
            return None;
        }
        Some(self.rate.w * nozzle_radius * nozzle_radius * std::f64::consts::PI / layer_height)
    }

    fn limit_speed(&mut self, velocity: f64, acceleration: f64) {
        let v2 = velocity * velocity;
        if v2 < self.max_cruise_v2 {
            self.max_cruise_v2 = v2;
        }
        self.acceleration = self.acceleration.min(acceleration);
        self.max_dv2 = 2.0 * self.distance * self.acceleration;
        self.smoothed_dv2 = self.smoothed_dv2.min(self.max_dv2);
    }

    fn accel_distance(&self) -> f64 {
        (self.cruise_v * self.cruise_v - self.start_v * self.start_v) * 0.5 / self.acceleration
    }

    fn accel_time(&self) -> f64 {
        self.accel_distance() / ((self.start_v + self.cruise_v) * 0.5)
    }

    fn cruise_distance(&self) -> f64 {
        (self.distance - self.accel_distance() - self.decel_distance()).max(0.0)
    }

    fn cruise_time(&self) -> f64 {
        self.cruise_distance() / self.cruise_v
    }

    fn decel_distance(&self) -> f64 {
        (self.cruise_v * self.cruise_v - self.end_v * self.end_v) * 0.5 / self.acceleration
    }

    fn decel_time(&self) -> f64 {
        self.decel_distance() / ((self.end_v + self.cruise_v) * 0.5)
    }

    #[inline(never)]
    fn total_time(&self) -> f64 {
        self.accel_time() + self.cruise_time() + self.decel_time()
    }
}

#[derive(Debug, Default)]
struct MoveSequence {
    moves: Vec<PlanningMove>,
}

impl MoveSequence {
    fn add_move(&mut self, mut move_cmd: PlanningMove, toolhead_state: &ToolheadState) {
        if move_cmd.distance == 0.0 {
            return;
        }
        if let Some(prev_move) = self.moves.last() {
            move_cmd.apply_junction(prev_move, toolhead_state);
        }
        self.moves.push(move_cmd);
    }

    fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    fn process(&mut self) {
        let mut delayed: Vec<(&mut PlanningMove, f64, f64)> = Vec::new();

        let mut next_end_v2 = 0.0;
        let mut next_smoothed_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;

        for m in self.moves.iter_mut().rev() {
            let reachable_start_v2 = next_end_v2 + m.max_dv2;
            let start_v2 = m.max_start_v2.min(reachable_start_v2);
            let reachable_smoothed_v2 = next_smoothed_v2 + m.smoothed_dv2;
            let smoothed_v2 = m.max_smoothed_v2.min(reachable_smoothed_v2);
            if smoothed_v2 < reachable_smoothed_v2 {
                if (smoothed_v2 + m.smoothed_dv2 > next_smoothed_v2) || !delayed.is_empty() {
                    peak_cruise_v2 = m
                        .max_cruise_v2
                        .min((smoothed_v2 + reachable_smoothed_v2) * 0.5);

                    if !delayed.is_empty() {
                        let mut mc_v2 = peak_cruise_v2;
                        for (m, ms_v2, me_v2) in delayed.into_iter().rev() {
                            mc_v2 = mc_v2.min(ms_v2);
                            m.set_junction(ms_v2.min(mc_v2), mc_v2, me_v2.min(mc_v2));
                        }
                        delayed = Vec::new();
                    }
                }

                let cruise_v2 = ((start_v2 + reachable_start_v2) * 0.5)
                    .min(m.max_cruise_v2)
                    .min(peak_cruise_v2);
                m.set_junction(
                    start_v2.min(cruise_v2),
                    cruise_v2,
                    next_end_v2.min(cruise_v2),
                );
            } else {
                delayed.push((m, start_v2, next_end_v2));
            }
            next_end_v2 = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
enum PositionMode {
    Absolute,
    Relative,
}

impl Default for PositionMode {
    fn default() -> Self {
        PositionMode::Absolute
    }
}

trait MoveChecker: std::fmt::Debug {
    fn check(&self, move_cmd: &mut PlanningMove);
}

#[derive(Debug)]
struct PrintLimits {
    max_velocity: f64,
    max_acceleration: f64,
    max_accel_to_decel: f64,
    square_corner_velocity: f64,
    junction_deviation: f64,
    instant_corner_velocity: f64,
}

impl Default for PrintLimits {
    fn default() -> Self {
        PrintLimits {
            max_velocity: 100.0,
            max_acceleration: 100.0,
            max_accel_to_decel: 50.0,
            square_corner_velocity: 5.0,
            junction_deviation: Self::scv_to_jd(5.0, 100000.0),
            instant_corner_velocity: 1.0,
        }
    }
}

impl PrintLimits {
    pub fn set_max_velocity(&mut self, v: f64) {
        self.max_velocity = v;
    }

    pub fn set_max_acceleration(&mut self, v: f64) {
        self.max_acceleration = v;
        self.junction_deviation =
            Self::scv_to_jd(self.square_corner_velocity, self.max_acceleration);
    }

    pub fn set_max_accel_to_decel(&mut self, v: f64) {
        self.max_accel_to_decel = v;
    }

    fn set_square_corner_velocity(&mut self, scv: f64) {
        self.square_corner_velocity = scv;
        self.junction_deviation =
            Self::scv_to_jd(self.square_corner_velocity, self.max_acceleration);
    }

    fn scv_to_jd(scv: f64, acceleration: f64) -> f64 {
        let scv2 = scv * scv;
        scv2 * (2.0f64.sqrt() - 1.0) / acceleration
    }
}

#[derive(Debug)]
struct ToolheadState {
    position: Vec4,
    position_modes: [PositionMode; 4],
    limits: PrintLimits,
    move_checkers: Vec<Box<dyn MoveChecker>>,

    velocity: f64,
}

impl ToolheadState {
    fn new() -> Self {
        let limits = PrintLimits::default();
        ToolheadState {
            position: Vec4::ZERO,
            position_modes: [
                PositionMode::Absolute,
                PositionMode::Absolute,
                PositionMode::Absolute,
                PositionMode::Relative,
            ],
            velocity: limits.max_velocity,
            limits,
            move_checkers: vec![],
        }
    }

    pub fn perform_move(&mut self, axes: [Option<f64>; 4]) -> PlanningMove {
        let mut new_pos = self.position;

        for (axis, v) in axes.iter().enumerate() {
            if let Some(v) = v {
                new_pos.as_mut()[axis] =
                    Self::new_element(*v, new_pos.as_mut()[axis], self.position_modes[axis]);
            }
        }

        let mut pm = PlanningMove::new(self.position, new_pos, self);

        for c in self.move_checkers.iter() {
            c.check(&mut pm);
        }

        self.position = new_pos;
        pm
    }

    fn new_element(v: f64, old: f64, mode: PositionMode) -> f64 {
        match mode {
            PositionMode::Relative => old + v,
            PositionMode::Absolute => v,
        }
    }

    pub fn set_speed(&mut self, v: f64) {
        self.velocity = v
    }

    fn extruder_junction_speed_v2(&self, cur_move: &PlanningMove, prev_move: &PlanningMove) -> f64 {
        let diff_r = (cur_move.rate.w - prev_move.rate.w).abs();
        if diff_r > 0.0 {
            let v = self.limits.instant_corner_velocity / diff_r;
            v * v
        } else {
            cur_move.max_cruise_v2
        }
    }
}

#[derive(Debug)]
struct KinematicCartesian {
    max_z_velocity: f64,
    max_z_accel: f64,
}

impl MoveChecker for KinematicCartesian {
    fn check(&self, move_cmd: &mut PlanningMove) {
        let z_ratio = move_cmd.distance / (move_cmd.end.z - move_cmd.start.z).abs();
        move_cmd.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
    }
}

#[derive(Debug)]
struct KinematicExtruder {
    max_velocity: f64,
    max_accel: f64,
}

impl MoveChecker for KinematicExtruder {
    fn check(&self, move_cmd: &mut PlanningMove) {
        if !move_cmd.is_extrude_only_move() {
            return;
        }
        let e_rate = move_cmd.rate.w;
        if move_cmd.rate.xy() == glam::DVec2::ZERO || e_rate < 0.0 {
            let inv_extrude_r = 1.0 / e_rate.abs();
            move_cmd.limit_speed(
                self.max_velocity * inv_extrude_r,
                self.max_accel * inv_extrude_r,
            );
        }
    }
}

fn is_dwell(_cmd: &GCodeCommand) -> bool {
    false
}

fn main() {
    let file = File::open(std::env::args().nth(1).expect("fn")).expect("open file");
    let rdr = BufReader::new(file);
    let rdr = GCodeReader::new(rdr);

    let mut move_sequences: Vec<MoveSequence> = Vec::new();

    let mut move_kinds: HashMap<String, u16> = HashMap::new();

    let mut toolhead_state = ToolheadState::new();

    toolhead_state.limits.set_max_velocity(600.0);
    toolhead_state.limits.set_max_acceleration(25000.0);
    toolhead_state.limits.set_max_accel_to_decel(25000.0);
    toolhead_state.limits.set_square_corner_velocity(50.0);

    toolhead_state
        .move_checkers
        .push(Box::new(KinematicCartesian {
            max_z_velocity: 35.0,
            max_z_accel: 1000.0,
        }));

    toolhead_state
        .move_checkers
        .push(Box::new(KinematicExtruder {
            max_velocity: 75.0,
            max_accel: 1500.0,
        }));

    let mut cur_sequence = MoveSequence::default();
    for cmd in rdr {
        let cmd = cmd.expect("gcode read");

        if is_dwell(&cmd) && !move_sequences.is_empty() {
            move_sequences.push(cur_sequence);
            cur_sequence = MoveSequence::default();
        } else if let GCodeOperation::Move { x, y, z, e, f } = &cmd.op {
            // println!("OP {:?}", cmd.op);
            // println!("KIND {:?}", cmd.comment);
            // println!("THS {:?}", toolhead_state);
            if let Some(v) = f {
                toolhead_state.set_speed(v / 60.0);
            }

            let num_kinds = move_kinds.len() as u16;
            let move_kind = cmd
                .comment
                .map(|comment| *move_kinds.entry(comment).or_insert_with(|| num_kinds));

            if x.is_some() || y.is_some() || z.is_some() || e.is_some() {
                let mut m = toolhead_state.perform_move([*x, *y, *z, *e]);
                m.kind = move_kind;
                cur_sequence.add_move(m, &toolhead_state);
            }
        } else if let GCodeOperation::Traditional {
            letter,
            code,
            params,
        } = &cmd.op
        {
            match (letter, code) {
                ('G', 92) => {
                    if let Some(v) = params.get_number::<f64>('X') {
                        toolhead_state.position.x = v;
                    }
                    if let Some(v) = params.get_number::<f64>('Y') {
                        toolhead_state.position.y = v;
                    }
                    if let Some(v) = params.get_number::<f64>('Z') {
                        toolhead_state.position.z = v;
                    }
                    if let Some(v) = params.get_number::<f64>('E') {
                        toolhead_state.position.w = v;
                    }
                }
                ('M', 82) => toolhead_state.position_modes[3] = PositionMode::Absolute,
                ('M', 83) => toolhead_state.position_modes[3] = PositionMode::Relative,
                ('M', 204) => {
                    let s = params.get_number::<f64>('S');
                    let p = params.get_number::<f64>('P');
                    let t = params.get_number::<f64>('T');
                    match (s, p, t) {
                        (Some(s), _, _) => toolhead_state.limits.set_max_acceleration(s),
                        (_, Some(p), Some(t)) => {
                            toolhead_state.limits.set_max_acceleration(p.min(t))
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        } else if let GCodeOperation::Extended { cmd, params } = &cmd.op {
            if cmd.as_str() == "set_velocity_limit" {
                if let Some(v) = params.get_number::<f64>("velocity") {
                    toolhead_state.limits.set_max_velocity(v);
                }
                if let Some(v) = params.get_number::<f64>("accel") {
                    toolhead_state.limits.set_max_acceleration(v);
                }
                if let Some(v) = params.get_number::<f64>("accel_to_decel") {
                    toolhead_state.limits.set_max_accel_to_decel(v);
                }
                if let Some(v) = params.get_number::<f64>("square_corner_velocity") {
                    toolhead_state.limits.set_square_corner_velocity(v);
                }
            }
        }
    }

    if !cur_sequence.is_empty() {
        move_sequences.push(cur_sequence);
    }

    println!("Sequences:");
    for (i, c) in move_sequences.iter_mut().enumerate() {
        c.process();

        println!(" Run {}:", i);
        println!("  Total moves: {}", c.moves.len());
        println!(
            "  Total distance: {}",
            c.moves.iter().map(|m| m.distance).sum::<f64>()
        );
        println!(
            "  Total extrude distance: {}",
            c.moves
                .iter()
                .map(|m| m.end.w)
                .fold(0.0f64, |a, b| a.max(b))
        );
        let min_time = 0.25 + c.moves.iter().map(|m| m.total_time()).sum::<f64>();
        println!("  Minimal time: {} ({})", format_time(min_time), min_time);

        println!("  Moves:");
        let width = (c.moves.len() as f64).log10().ceil() as usize;
        let mut layer_times = BTreeMap::new();
        let mut ctime = 0.25;
        for (i, m) in c.moves.iter().enumerate() {
            let mut kind = String::new();
            if m.is_extrude_move() {
                kind.push('E');
            }
            if m.is_kinematic_move() {
                kind.push('K');
            }
            println!(
                "   {:width$}[{}] @ {:.8} => {:.8}:",
                i,
                kind,
                ctime,
                ctime + m.total_time(),
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
            println!("    Line widht: {:?}", m.line_width(1.75 / 2.0, 0.25));
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
            // println!("    CT {}", ctime.to_bits());
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
            }

            // println!(
            //     "{:width$} BT {}",
            //     i,
            //     m.total_time().to_bits(),
            //     width = width
            // );
        }

        println!("  Layer times:");
        for (z, t) in layer_times.iter() {
            println!("   {:7} => {}", (*z as f64) / 1000.0, format_time(*t));
        }
    }
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
