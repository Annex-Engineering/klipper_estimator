use std::collections::VecDeque;
use std::f64::EPSILON;
use std::time::Duration;

use crate::arcs::ArcState;
pub use crate::firmware_retraction::FirmwareRetractionOptions;
use crate::firmware_retraction::FirmwareRetractionState;
use crate::gcode::{GCodeCommand, GCodeOperation};

use crate::kind_tracker::{Kind, KindTracker};
use glam::Vec4Swizzles;
use glam::{DVec3 as Vec3, DVec4 as Vec4};
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub struct Planner {
    operations: OperationSequence,
    pub toolhead_state: ToolheadState,
    pub kind_tracker: KindTracker,
    pub firmware_retraction: Option<FirmwareRetractionState>,
    pub arc_state: ArcState,
}

impl Planner {
    pub fn from_limits(limits: PrinterLimits) -> Planner {
        let firmware_retraction = limits
            .firmware_retraction
            .as_ref()
            .map(|_| FirmwareRetractionState::default());
        Planner {
            operations: OperationSequence::default(),
            toolhead_state: ToolheadState::from_limits(limits),
            kind_tracker: KindTracker::new(),
            firmware_retraction,
            arc_state: ArcState::default(),
        }
    }

    /// Processes a gcode command through the planning engine and appends it to the currently
    /// open move sequence.
    /// Returns the number of planning operations the command resulted in
    pub fn process_cmd(&mut self, cmd: &GCodeCommand) -> usize {
        if let Some(m) = Self::is_dwell(cmd, &mut self.kind_tracker) {
            self.operations.add_delay(m);
        } else if let GCodeOperation::Move { x, y, z, e, f } = &cmd.op {
            if let Some(v) = f {
                self.toolhead_state.set_speed(v / 60.0);
            }

            let move_kind = self.kind_tracker.kind_from_comment(&cmd.comment);

            if x.is_some() || y.is_some() || z.is_some() || e.is_some() {
                let mut m = self.toolhead_state.perform_move([*x, *y, *z, *e]);
                m.kind = move_kind;
                self.operations.add_move(m, &self.toolhead_state);
            } else {
                self.operations.add_fill();
            }
        } else if let GCodeOperation::Traditional {
            letter,
            code,
            params,
        } = &cmd.op
        {
            match (letter, code) {
                ('G', 10) => {
                    let kt = &mut self.kind_tracker;
                    let m = &mut self.toolhead_state;
                    let seq = &mut self.operations;
                    if let Some(fr) = self.firmware_retraction.as_mut() {
                        return fr.retract(kt, m, seq);
                    }
                }
                ('G', 11) => {
                    let kt = &mut self.kind_tracker;
                    let m = &mut self.toolhead_state;
                    let seq = &mut self.operations;
                    if let Some(fr) = self.firmware_retraction.as_mut() {
                        return fr.unretract(kt, m, seq);
                    }
                }
                ('G', v @ 2 | v @ 3) => {
                    let move_kind = self.kind_tracker.kind_from_comment(&cmd.comment);
                    let m = &mut self.toolhead_state;
                    let seq = &mut self.operations;
                    return self.arc_state.generate_arc(
                        m,
                        seq,
                        move_kind,
                        params,
                        match v {
                            2 => crate::arcs::ArcDirection::Clockwise,
                            3 => crate::arcs::ArcDirection::CounterClockwise,
                            _ => unreachable!("v can only be 2 or 3"),
                        },
                    );
                }
                ('G', 17) => {
                    self.arc_state.set_plane(crate::arcs::Plane::XY);
                }
                ('G', 18) => {
                    self.arc_state.set_plane(crate::arcs::Plane::XZ);
                }
                ('G', 19) => {
                    self.arc_state.set_plane(crate::arcs::Plane::YZ);
                }
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
            self.operations.add_fill();
        } else if let GCodeOperation::Extended { command, params } = &cmd.op {
            match command.as_str() {
                "set_velocity_limit" => {
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
                "set_retraction" => {
                    let m = &mut self.toolhead_state;
                    if let Some(fr) = self.firmware_retraction.as_ref() {
                        fr.set_options(m, params);
                    }
                }
                _ => {}
            }
            self.operations.add_fill();
        } else if cmd.op.is_nop() && cmd.comment.is_some() {
            let comment = cmd.comment.as_ref().unwrap(); // Same, we checked for is_some

            if let Some(comment) = comment.strip_prefix("TYPE:") {
                // IdeaMaker only gives us `TYPE:`s
                let kind = self.kind_tracker.get_kind(comment);
                self.kind_tracker.set_current(Some(kind));
                self.operations.add_fill();
            } else if let Some(cmd) = comment.trim_start().strip_prefix("ESTIMATOR_ADD_TIME ") {
                if let Some((duration, kind)) = Self::parse_buffer_cmd(&mut self.kind_tracker, cmd)
                {
                    self.operations.add_delay(Delay::Indeterminate(
                        Duration::from_secs_f64(duration),
                        kind,
                    ));
                } else {
                    self.operations.add_fill();
                }
            } else {
                self.operations.add_fill();
            }
        } else {
            self.operations.add_fill();
        }
        1 // Most commands result in a single planning op
    }

    /// Performs final processing on the final sequence, if one is active.
    pub fn finalize(&mut self) {
        self.operations.flush();
    }

    fn is_dwell(cmd: &GCodeCommand, kind_tracker: &mut KindTracker) -> Option<Delay> {
        let indef = Duration::from_secs_f64(0.1);
        match &cmd.op {
            GCodeOperation::Traditional {
                letter: 'G',
                code: 4,
                params,
            } => Some(Delay::Pause(Duration::from_secs_f64(
                params.get_number('P').map_or(0.25, |v: f64| v / 1000.0),
            ))),
            GCodeOperation::Traditional {
                letter: 'G',
                code: 28,
                ..
            } => Some(Delay::Indeterminate(
                indef,
                Some(kind_tracker.get_kind("Indeterminate time")),
            )),
            GCodeOperation::Traditional {
                letter: 'M',
                code: 109 | 190,
                ..
            } => Some(Delay::Indeterminate(
                indef,
                Some(kind_tracker.get_kind("Indeterminate time")),
            )),
            GCodeOperation::Extended { command: cmd, .. } if cmd == "temperature_wait" => Some(
                Delay::Indeterminate(indef, Some(kind_tracker.get_kind("Indeterminate time"))),
            ),
            GCodeOperation::Traditional {
                letter: 'M',
                code: 600,
                ..
            } => Some(Delay::Indeterminate(
                indef,
                Some(kind_tracker.get_kind("Indeterminate time")),
            )),
            _ => None,
        }
    }

    fn parse_buffer_cmd(kind_tracker: &mut KindTracker, cmd: &str) -> Option<(f64, Option<Kind>)> {
        let (a, b) = cmd
            .split_once(' ')
            .map_or((cmd, None), |(l, r)| (l, Some(r)));
        let duration = a.parse().ok()?;
        let kind = b.map(|s| kind_tracker.get_kind(s));
        Some((duration, kind))
    }

    pub fn next_operation(&mut self) -> Option<PlanningOperation> {
        self.operations.next_operation()
    }

    pub fn iter(&mut self) -> PlanningOperationIter {
        PlanningOperationIter { planner: self }
    }

    pub fn move_kind_str<'a>(&'a self, m: &PlanningMove) -> Option<&'a str> {
        m.kind.map(|k| self.kind_tracker.resolve_kind(k))
    }

    pub fn kind_str<'a>(&'a self, kind: &Option<Kind>) -> Option<&'a str> {
        kind.map(|k| self.kind_tracker.resolve_kind(k))
    }
}

#[derive(Debug)]
pub enum Delay {
    Indeterminate(Duration, Option<Kind>),
    Pause(Duration),
}

impl Delay {
    pub fn duration(&self) -> Duration {
        match self {
            Delay::Indeterminate(d, _) => *d,
            Delay::Pause(d) => *d,
        }
    }
}

#[derive(Debug)]
pub enum PlanningOperation {
    Delay(Delay),
    Move(PlanningMove),
    Fill,
}

impl PlanningOperation {
    pub fn is_fill(&self) -> bool {
        matches!(self, Self::Fill)
    }

    pub fn is_move(&self) -> bool {
        matches!(self, Self::Move(_))
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
    pub junction_deviation: f64,
    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub max_dv2: f64,
    pub max_smoothed_v2: f64,
    pub smoothed_dv2: f64,

    pub kind: Option<Kind>,

    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
}

impl PlanningMove {
    /// Create a new `PlanningMove` that travels between the two points `start`
    /// and `end`.
    pub(crate) fn new(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
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
            junction_deviation: toolhead_state.limits.junction_deviation,
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
            junction_deviation: toolhead_state.limits.junction_deviation,
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
        if junction_cos_theta > 0.999999 {
            // Move was not at an angle, skip all this
            return;
        }
        junction_cos_theta = junction_cos_theta.max(-0.999999);
        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).sqrt();
        let r = sin_theta_d2 / (1.0 - sin_theta_d2);
        let tan_theta_d2 = sin_theta_d2 / (0.5 * (1.0 + junction_cos_theta)).sqrt();
        let move_centripetal_v2 = 0.5 * self.distance * tan_theta_d2 * self.acceleration;
        let prev_move_centripetal_v2 =
            0.5 * previous_move.distance * tan_theta_d2 * previous_move.acceleration;

        let extruder_v2 = toolhead_state.extruder_junction_speed_v2(self, previous_move);

        self.max_start_v2 = extruder_v2
            .min(r * self.junction_deviation * self.acceleration)
            .min(r * previous_move.junction_deviation * previous_move.acceleration)
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

#[derive(Debug)]
enum OperationSequenceOperation {
    Delay(Delay),
    MoveSequence(MoveSequence),
    Fill,
}

impl From<OperationSequenceOperation> for PlanningOperation {
    fn from(oso: OperationSequenceOperation) -> Self {
        match oso {
            OperationSequenceOperation::Delay(d) => PlanningOperation::Delay(d),
            OperationSequenceOperation::Fill => PlanningOperation::Fill,
            OperationSequenceOperation::MoveSequence(_) => {
                panic!("Invalid conversion of move sequence to planning op")
            }
        }
    }
}

#[derive(Debug, Default)]
pub struct OperationSequence {
    ops: VecDeque<OperationSequenceOperation>,
}

impl OperationSequence {
    pub(crate) fn add_delay(&mut self, delay: Delay) {
        self.ops.push_back(OperationSequenceOperation::Delay(delay));
    }

    pub(crate) fn add_move(&mut self, move_cmd: PlanningMove, toolhead_state: &ToolheadState) {
        if let Some(OperationSequenceOperation::MoveSequence(ms)) = self.ops.back_mut() {
            ms.add_move(move_cmd, toolhead_state);
        } else {
            let mut ms = MoveSequence::default();
            ms.add_move(move_cmd, toolhead_state);
            self.ops
                .push_back(OperationSequenceOperation::MoveSequence(ms));
        }
    }

    pub(crate) fn add_fill(&mut self) {
        if let Some(OperationSequenceOperation::MoveSequence(ms)) = self.ops.back_mut() {
            ms.add_fill();
        } else {
            self.ops.push_back(OperationSequenceOperation::Fill);
        }
    }

    pub(crate) fn flush(&mut self) {
        for o in self.ops.iter_mut() {
            if let OperationSequenceOperation::MoveSequence(ms) = o {
                ms.flush();
            }
        }
    }

    fn next_operation(&mut self) -> Option<PlanningOperation> {
        if let Some(OperationSequenceOperation::MoveSequence(ms)) = self.ops.front_mut() {
            let m = ms.next_move();
            if ms.is_empty() {
                self.ops.pop_front();
            }
            m
        } else {
            self.ops.pop_front().map(|o| o.into())
        }
    }
}

#[derive(Debug)]
enum MoveSequenceOperation {
    Move(PlanningMove),
    Fill,
}

impl MoveSequenceOperation {
    fn is_fill(&self) -> bool {
        matches!(self, MoveSequenceOperation::Fill)
    }
}

impl From<MoveSequenceOperation> for PlanningOperation {
    fn from(mso: MoveSequenceOperation) -> Self {
        match mso {
            MoveSequenceOperation::Move(m) => PlanningOperation::Move(m),
            MoveSequenceOperation::Fill => PlanningOperation::Fill,
        }
    }
}

#[derive(Debug, Default)]
pub struct MoveSequence {
    moves: VecDeque<MoveSequenceOperation>,
    flush_count: usize,
}

impl MoveSequence {
    pub(crate) fn add_fill(&mut self) {
        self.moves.push_back(MoveSequenceOperation::Fill);
    }

    pub(crate) fn add_move(&mut self, mut move_cmd: PlanningMove, toolhead_state: &ToolheadState) {
        if move_cmd.distance == 0.0 {
            self.add_fill();
            return;
        }
        if let Some(prev_move) = self.last_move() {
            move_cmd.apply_junction(prev_move, toolhead_state);
        }
        self.moves.push_back(MoveSequenceOperation::Move(move_cmd));
    }

    fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    fn last_move(&self) -> Option<&PlanningMove> {
        self.moves.iter().rev().find_map(|o| match o {
            MoveSequenceOperation::Move(m) => Some(m),
            _ => None,
        })
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
        let skip = if partial { self.flush_count } else { 0 };
        if !partial {
            self.flush_count = self.moves.len();
        }

        for (idx, m) in self.moves.iter_mut().enumerate().skip(skip).rev() {
            if let MoveSequenceOperation::Move(m) = m {
                let reachable_start_v2 = next_end_v2 + m.max_dv2;
                let start_v2 = m.max_start_v2.min(reachable_start_v2);
                let reachable_smoothed_v2 = next_smoothed_v2 + m.smoothed_dv2;
                let smoothed_v2 = m.max_smoothed_v2.min(reachable_smoothed_v2);
                if smoothed_v2 < reachable_smoothed_v2 {
                    if (smoothed_v2 + m.smoothed_dv2 > next_smoothed_v2) || !delayed.is_empty() {
                        if update_flush_count && peak_cruise_v2 != 0.0 {
                            self.flush_count = idx;
                            update_flush_count = false;
                        }

                        peak_cruise_v2 = m
                            .max_cruise_v2
                            .min((smoothed_v2 + reachable_smoothed_v2) * 0.5);

                        if !delayed.is_empty() {
                            if !update_flush_count && idx < self.flush_count {
                                let mut mc_v2 = peak_cruise_v2;
                                for (m, ms_v2, me_v2) in delayed.iter_mut().rev() {
                                    mc_v2 = mc_v2.min(*ms_v2);
                                    m.set_junction(ms_v2.min(mc_v2), mc_v2, me_v2.min(mc_v2));
                                }
                            }
                            delayed.clear();
                        }
                    }

                    if !update_flush_count && idx < self.flush_count {
                        let cruise_v2 = ((start_v2 + reachable_start_v2) * 0.5)
                            .min(m.max_cruise_v2)
                            .min(peak_cruise_v2);
                        m.set_junction(
                            start_v2.min(cruise_v2),
                            cruise_v2,
                            next_end_v2.min(cruise_v2),
                        );
                    }
                } else {
                    delayed.push((m, start_v2, next_end_v2));
                }
                next_end_v2 = start_v2;
                next_smoothed_v2 = smoothed_v2;
            }
        }

        if update_flush_count {
            self.flush_count = 0;
        }

        // Advance while the next operation is a fill
        while self.flush_count < self.moves.len() && self.moves[self.flush_count].is_fill() {
            self.flush_count += 1;
        }
    }

    fn flush(&mut self) {
        self.process(false);
    }

    fn next_move(&mut self) -> Option<PlanningOperation> {
        self.process(true);
        if self.flush_count == 0 {
            return None;
        }
        match self.moves.pop_front() {
            None => None,
            Some(v) => {
                self.flush_count -= 1;
                Some(v.into())
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmware_retraction: Option<FirmwareRetractionOptions>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mm_per_arc_segment: Option<f64>,
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
            firmware_retraction: None,
            mm_per_arc_segment: None,
        }
    }
}

impl PrinterLimits {
    pub fn update_junction_deviation(&mut self) {
        self.junction_deviation =
            Self::scv_to_jd(self.square_corner_velocity, self.max_acceleration);
    }

    pub fn set_max_velocity(&mut self, v: f64) {
        self.max_velocity = v;
    }

    pub fn set_max_acceleration(&mut self, v: f64) {
        self.max_acceleration = v;
        self.update_junction_deviation();
    }

    pub fn set_max_accel_to_decel(&mut self, v: f64) {
        self.max_accel_to_decel = v;
    }

    pub fn set_square_corner_velocity(&mut self, scv: f64) {
        self.square_corner_velocity = scv;
        self.update_junction_deviation();
    }

    pub fn set_instant_corner_velocity(&mut self, icv: f64) {
        self.instant_corner_velocity = icv;
    }

    fn scv_to_jd(scv: f64, acceleration: f64) -> f64 {
        let scv2 = scv * scv;
        scv2 * (2.0f64.sqrt() - 1.0) / acceleration
    }
}

#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub enum PositionMode {
    #[default]
    Absolute,
    Relative,
}

#[derive(Debug)]
pub struct ToolheadState {
    pub position: Vec4,
    pub position_modes: [PositionMode; 4],
    pub limits: PrinterLimits,

    pub velocity: f64,
}

impl ToolheadState {
    fn from_limits(limits: PrinterLimits) -> Self {
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

    pub fn perform_relative_move(
        &mut self,
        axes: [Option<f64>; 4],
        kind: Option<Kind>,
    ) -> PlanningMove {
        let cur_pos_mode = self.position_modes;
        self.position_modes = [PositionMode::Relative; 4];
        let mut pm = self.perform_move(axes);
        pm.kind = kind;
        self.position_modes = cur_pos_mode;
        pm
    }

    pub(crate) fn new_element(v: f64, old: f64, mode: PositionMode) -> f64 {
        match mode {
            PositionMode::Relative => old + v,
            PositionMode::Absolute => v,
        }
    }

    pub fn set_speed(&mut self, v: f64) {
        if v <= 0.0 {
            panic!("Requested toolhead velocity {} <= 0", v);
        }
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
