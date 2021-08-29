use std::collections::{HashMap, VecDeque};
use std::f64::EPSILON;

use crate::{GCodeCommand, GCodeOperation};

use glam::Vec4Swizzles;
pub use glam::{DVec3 as Vec3, DVec4 as Vec4};
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub struct Planner {
    sequences: VecDeque<MoveSequence>,
    pub toolhead_state: ToolheadState,
    pub current_kind: Option<String>,
    pub kind_to_index: HashMap<String, u16>,
    pub index_to_kind: HashMap<u16, String>,
}

impl Default for Planner {
    fn default() -> Planner {
        Planner {
            sequences: [MoveSequence::default()].into(),
            toolhead_state: ToolheadState::default(),
            current_kind: None,
            kind_to_index: HashMap::new(),
            index_to_kind: HashMap::new(),
        }
    }
}

impl Planner {
    /// Processes a gcode command through the planning engine and appends it to the currently
    /// open move sequence.
    pub fn process_cmd(&mut self, cmd: GCodeCommand) {
        if Self::is_dwell(&cmd) {
            if let Some(seq) = self.sequences.back_mut() {
                if !seq.is_empty() {
                    seq.flush();
                    self.sequences.push_back(MoveSequence::default());
                }
            }
        } else if let GCodeOperation::Move { x, y, z, e, f } = &cmd.op {
            if let Some(v) = f {
                self.toolhead_state.set_speed(v / 60.0);
            }

            let num_kinds = self.kind_to_index.len() as u16;
            let move_kind = cmd.comment.as_ref().or(self.current_kind.as_ref());

            let kind_idx = match move_kind {
                None => None,
                Some(s) => {
                    if let Some(idx) = self.kind_to_index.get(s) {
                        Some(*idx)
                    } else {
                        self.kind_to_index.insert(s.into(), num_kinds);
                        self.index_to_kind.insert(num_kinds, s.into());
                        Some(num_kinds)
                    }
                }
            };

            if x.is_some() || y.is_some() || z.is_some() || e.is_some() {
                let mut m = self.toolhead_state.perform_move([*x, *y, *z, *e]);
                m.kind = kind_idx;
                self.sequences
                    .back_mut()
                    .unwrap()
                    .add_move(m, &self.toolhead_state);
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
                        self.toolhead_state.position.x = v;
                    }
                    if let Some(v) = params.get_number::<f64>('Y') {
                        self.toolhead_state.position.y = v;
                    }
                    if let Some(v) = params.get_number::<f64>('Z') {
                        self.toolhead_state.position.z = v;
                    }
                    if let Some(v) = params.get_number::<f64>('E') {
                        self.toolhead_state.position.w = v;
                    }
                }
                ('M', 82) => self.toolhead_state.position_modes[3] = PositionMode::Absolute,
                ('M', 83) => self.toolhead_state.position_modes[3] = PositionMode::Relative,
                ('M', 204) => {
                    let s = params.get_number::<f64>('S');
                    let p = params.get_number::<f64>('P');
                    let t = params.get_number::<f64>('T');
                    match (s, p, t) {
                        (Some(s), _, _) => self.toolhead_state.limits.set_max_acceleration(s),
                        (_, Some(p), Some(t)) => {
                            self.toolhead_state.limits.set_max_acceleration(p.min(t))
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        } else if let GCodeOperation::Extended { cmd, params } = &cmd.op {
            if cmd.as_str() == "set_velocity_limit" {
                if let Some(v) = params.get_number::<f64>("velocity") {
                    self.toolhead_state.limits.set_max_velocity(v);
                }
                if let Some(v) = params.get_number::<f64>("accel") {
                    self.toolhead_state.limits.set_max_acceleration(v);
                }
                if let Some(v) = params.get_number::<f64>("accel_to_decel") {
                    self.toolhead_state.limits.set_max_accel_to_decel(v);
                }
                if let Some(v) = params.get_number::<f64>("square_corner_velocity") {
                    self.toolhead_state.limits.set_square_corner_velocity(v);
                }
            }
        } else if cmd.op.is_nop() && cmd.comment.is_some() {
            let comment = cmd.comment.unwrap(); // Same, we checked for is_some

            if comment.starts_with("TYPE:") {
                // IdeaMaker only gives us `TYPE:`s
                self.current_kind = Some(comment[5..].into());
            }
        }
    }

    /// Performs final processing on the final sequence, if one is active.
    pub fn finalize(&mut self) {
        self.sequences.back_mut().map(|seq| seq.flush());
    }

    fn is_dwell(cmd: &GCodeCommand) -> bool {
        match &cmd.op {
            GCodeOperation::Traditional {
                letter: 'M',
                code: 109,
                ..
            } => true,
            GCodeOperation::Traditional {
                letter: 'M',
                code: 190,
                ..
            } => true,
            GCodeOperation::Extended { cmd, .. } if cmd == "temperature_wait" => true,
            _ => false,
        }
    }

    pub fn next_operation(&mut self) -> Option<PlanningOperation> {
        let fseq = self.sequences.front_mut().unwrap();

        match fseq.next_move() {
            Some(m) => return Some(PlanningOperation::Move(m)),
            None => {
                // There's no next move, if this isn't the active sequence then remove it
                if self.sequences.len() > 1 {
                    self.sequences.pop_front();
                    Some(PlanningOperation::Dwell)
                } else {
                    None
                }
            }
        }
    }

    pub fn iter(&mut self) -> PlanningOperationIter {
        PlanningOperationIter { planner: self }
    }

    pub fn move_kind<'a>(&'a mut self, m: &PlanningMove) -> Option<&'a str> {
        let kind = m.kind?;
        let kind_str = self.index_to_kind.get(&kind)?;
        Some(kind_str.as_str())
    }
}

#[derive(Debug)]
pub enum PlanningOperation {
    Dwell,
    Move(PlanningMove),
}

impl PlanningOperation {
    pub fn is_move(&self) -> bool {
        match self {
            Self::Move(_) => true,
            _ => false,
        }
    }

    pub fn get_move(&self) -> Option<PlanningMove> {
        match self {
            Self::Move(m) => Some(*m),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub struct PlanningOperationIter<'a> {
    planner: &'a mut Planner,
}

impl<'a> Iterator for PlanningOperationIter<'a> {
    type Item = PlanningOperation;

    fn next(&mut self) -> Option<Self::Item> {
        self.planner.next_operation()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlanningMove {
    pub start: Vec4,
    pub end: Vec4,
    pub distance: f64,
    pub rate: Vec4,
    pub requested_velocity: f64,
    pub acceleration: f64,
    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub max_dv2: f64,
    pub max_smoothed_v2: f64,
    pub smoothed_dv2: f64,

    pub kind: Option<u16>,

    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
}

impl PlanningMove {
    /// Create a new `PlanningMove` that travels between the two points `start`
    /// and `end`.
    fn new(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
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

    pub fn is_kinematic_move(&self) -> bool {
        self.start.xyz() != self.end.xyz()
    }

    pub fn is_extrude_move(&self) -> bool {
        (self.end.w - self.start.w).abs() >= EPSILON
    }

    pub fn is_extrude_only_move(&self) -> bool {
        !self.is_kinematic_move() && self.is_extrude_move()
    }

    pub fn is_zero_distance(&self) -> bool {
        self.distance.abs() < EPSILON
    }

    pub fn line_width(&self, filament_radius: f64, layer_height: f64) -> Option<f64> {
        // Only moves that are both extruding and moving have a line width
        if !self.is_kinematic_move() || !self.is_extrude_move() {
            return None;
        }
        Some(self.rate.w * filament_radius * filament_radius * std::f64::consts::PI / layer_height)
    }

    pub fn flow_rate(&self, filament_radius: f64) -> Option<f64> {
        if !self.is_extrude_move() {
            return None;
        }
        Some(
            self.delta().w * filament_radius * filament_radius * std::f64::consts::PI
                / self.total_time(),
        )
    }

    pub fn limit_speed(&mut self, velocity: f64, acceleration: f64) {
        let v2 = velocity * velocity;
        if v2 < self.max_cruise_v2 {
            self.max_cruise_v2 = v2;
        }
        self.acceleration = self.acceleration.min(acceleration);
        self.max_dv2 = 2.0 * self.distance * self.acceleration;
        self.smoothed_dv2 = self.smoothed_dv2.min(self.max_dv2);
    }

    pub fn delta(&self) -> Vec4 {
        self.end - self.start
    }

    pub fn accel_distance(&self) -> f64 {
        (self.cruise_v * self.cruise_v - self.start_v * self.start_v) * 0.5 / self.acceleration
    }

    pub fn accel_time(&self) -> f64 {
        self.accel_distance() / ((self.start_v + self.cruise_v) * 0.5)
    }

    pub fn cruise_distance(&self) -> f64 {
        (self.distance - self.accel_distance() - self.decel_distance()).max(0.0)
    }

    pub fn cruise_time(&self) -> f64 {
        self.cruise_distance() / self.cruise_v
    }

    pub fn decel_distance(&self) -> f64 {
        (self.cruise_v * self.cruise_v - self.end_v * self.end_v) * 0.5 / self.acceleration
    }

    pub fn decel_time(&self) -> f64 {
        self.decel_distance() / ((self.end_v + self.cruise_v) * 0.5)
    }

    pub fn total_time(&self) -> f64 {
        self.accel_time() + self.cruise_time() + self.decel_time()
    }
}

#[derive(Debug, Default)]
pub struct MoveSequence {
    moves: VecDeque<PlanningMove>,
    flush_count: usize,
}

impl MoveSequence {
    fn add_move(&mut self, mut move_cmd: PlanningMove, toolhead_state: &ToolheadState) {
        if move_cmd.distance == 0.0 {
            return;
        }
        if let Some(prev_move) = self.moves.back() {
            move_cmd.apply_junction(prev_move, toolhead_state);
        }
        self.moves.push_back(move_cmd);
    }

    fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    fn process(&mut self, partial: bool) {
        if self.flush_count == self.moves.len() {
            // If there's nothing to flush, bail quickly
            return;
        }

        let mut delayed: Vec<(&mut PlanningMove, f64, f64)> = Vec::new();

        let mut next_end_v2 = 0.0;
        let mut next_smoothed_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;

        let mut update_flush_count = partial;

        for (idx, m) in self
            .moves
            .iter_mut()
            .enumerate()
            .skip(self.flush_count)
            .rev()
        {
            let reachable_start_v2 = next_end_v2 + m.max_dv2;
            let start_v2 = m.max_start_v2.min(reachable_start_v2);
            let reachable_smoothed_v2 = next_smoothed_v2 + m.smoothed_dv2;
            let smoothed_v2 = m.max_smoothed_v2.min(reachable_smoothed_v2);
            if smoothed_v2 < reachable_smoothed_v2 {
                if (smoothed_v2 + m.smoothed_dv2 > next_smoothed_v2) || !delayed.is_empty() {
                    if update_flush_count && peak_cruise_v2 != 0.0 {
                        self.flush_count = idx + 1;
                        update_flush_count = false;
                    }

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

        // We flushed to the end
        if !partial {
            self.flush_count = self.moves.len();
        }
    }

    pub fn flush(&mut self) {
        self.process(false);
    }

    pub fn next_move(&mut self) -> Option<PlanningMove> {
        self.process(true);
        if self.flush_count == 0 {
            return None;
        }
        match self.moves.pop_front() {
            None => None,
            v @ Some(_) => {
                self.flush_count -= 1;
                v
            }
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct PrinterLimits {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_accel_to_decel: f64,
    pub square_corner_velocity: f64,
    #[serde(skip)]
    pub junction_deviation: f64,
    pub instant_corner_velocity: f64,
    pub move_checkers: Vec<MoveChecker>,
}

impl Default for PrinterLimits {
    fn default() -> Self {
        PrinterLimits {
            max_velocity: 100.0,
            max_acceleration: 100.0,
            max_accel_to_decel: 50.0,
            square_corner_velocity: 5.0,
            junction_deviation: Self::scv_to_jd(5.0, 100000.0),
            instant_corner_velocity: 1.0,
            move_checkers: vec![],
        }
    }
}

impl PrinterLimits {
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

    pub fn set_square_corner_velocity(&mut self, scv: f64) {
        self.square_corner_velocity = scv;
        self.junction_deviation =
            Self::scv_to_jd(self.square_corner_velocity, self.max_acceleration);
    }

    pub fn set_instant_corner_velocity(&mut self, icv: f64) {
        self.instant_corner_velocity = icv;
    }

    fn scv_to_jd(scv: f64, acceleration: f64) -> f64 {
        let scv2 = scv * scv;
        scv2 * (2.0f64.sqrt() - 1.0) / acceleration
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum PositionMode {
    Absolute,
    Relative,
}

impl Default for PositionMode {
    fn default() -> Self {
        PositionMode::Absolute
    }
}

#[derive(Debug)]
pub struct ToolheadState {
    pub position: Vec4,
    pub position_modes: [PositionMode; 4],
    pub limits: PrinterLimits,

    pub velocity: f64,
}

impl Default for ToolheadState {
    fn default() -> Self {
        let limits = PrinterLimits::default();
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
        }
    }
}

impl ToolheadState {
    pub fn perform_move(&mut self, axes: [Option<f64>; 4]) -> PlanningMove {
        let mut new_pos = self.position;

        for (axis, v) in axes.iter().enumerate() {
            if let Some(v) = v {
                new_pos.as_mut()[axis] =
                    Self::new_element(*v, new_pos.as_mut()[axis], self.position_modes[axis]);
            }
        }

        let mut pm = PlanningMove::new(self.position, new_pos, self);

        for c in self.limits.move_checkers.iter() {
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

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MoveChecker {
    AxisLimiter {
        axis: Vec3,
        max_velocity: f64,
        max_accel: f64,
    },
    ExtruderLimiter {
        max_velocity: f64,
        max_accel: f64,
    },
}

impl MoveChecker {
    pub fn check(&self, move_cmd: &mut PlanningMove) {
        match self {
            Self::AxisLimiter {
                axis,
                max_velocity,
                max_accel,
            } => Self::check_axis(move_cmd, *axis, *max_velocity, *max_accel),
            Self::ExtruderLimiter {
                max_velocity,
                max_accel,
            } => Self::check_extruder(move_cmd, *max_velocity, *max_accel),
        }
    }

    fn check_axis(move_cmd: &mut PlanningMove, axis: Vec3, max_velocity: f64, max_accel: f64) {
        if move_cmd.is_zero_distance() {
            return;
        }
        let ratio = move_cmd.distance / (move_cmd.delta().xyz().dot(axis)).abs();
        move_cmd.limit_speed(max_velocity * ratio, max_accel * ratio);
    }

    fn check_extruder(move_cmd: &mut PlanningMove, max_velocity: f64, max_accel: f64) {
        if !move_cmd.is_extrude_only_move() {
            return;
        }
        let e_rate = move_cmd.rate.w;
        if move_cmd.rate.xy() == glam::DVec2::ZERO || e_rate < 0.0 {
            let inv_extrude_r = 1.0 / e_rate.abs();
            move_cmd.limit_speed(max_velocity * inv_extrude_r, max_accel * inv_extrude_r);
        }
    }
}
