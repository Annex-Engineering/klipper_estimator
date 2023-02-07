use glam::{DVec3 as Vec3, Vec4Swizzles};

use crate::gcode::GCodeTraditionalParams;
use crate::kind_tracker::Kind;
use crate::planner::{OperationSequence, PositionMode, ToolheadState};

#[derive(Debug, Default)]
pub struct ArcState {
    plane: Plane,
}

impl ArcState {
    pub fn set_plane(&mut self, plane: Plane) {
        self.plane = plane;
    }

    pub fn generate_arc(
        &self,
        toolhead_state: &mut ToolheadState,
        op_sequence: &mut OperationSequence,
        move_kind: Option<Kind>,
        params: &GCodeTraditionalParams,
        direction: ArcDirection,
    ) -> usize {
        let args = match self.get_args(toolhead_state, params) {
            None => return 0,
            Some(args) => args,
        };

        let (segments, arc) = args.plan_arc(
            toolhead_state.position.xyz(),
            direction,
            args.mm_per_arc_segment,
        );
        let mut e_base = toolhead_state.position.w;
        let e_per_move = args.e.map_or(0.0, |e| (e - e_base) / (segments as f64));

        toolhead_state.set_speed(args.velocity);

        let old_pos_mode = toolhead_state.position_modes;
        toolhead_state.position_modes = [PositionMode::Absolute; 4];
        for segment in arc {
            e_base += e_per_move;
            let coord = [
                Some(segment.x),
                Some(segment.y),
                Some(segment.z),
                Some(e_base),
            ];
            let mut pm = toolhead_state.perform_move(coord);
            pm.kind = move_kind;
            op_sequence.add_move(pm, toolhead_state);
        }
        toolhead_state.position_modes = old_pos_mode;

        segments
    }

    fn get_args(
        &self,
        toolhead_state: &mut ToolheadState,
        params: &GCodeTraditionalParams,
    ) -> Option<ArcArgs> {
        let mm_per_arc_segment = toolhead_state.limits.mm_per_arc_segment?;

        let map_coord = |c: f64, axis: usize| {
            ToolheadState::new_element(
                c,
                toolhead_state.position.as_ref()[axis],
                toolhead_state.position_modes[axis],
            )
        };

        let (axes, offset) = match self.plane {
            Plane::XY => (
                (0, 1, 2),
                (
                    params.get_number::<f64>('I').unwrap_or(0.0),
                    params.get_number::<f64>('J').unwrap_or(0.0),
                ),
            ),
            Plane::XZ => (
                (0, 2, 1),
                (
                    params.get_number::<f64>('I').unwrap_or(0.0),
                    params.get_number::<f64>('K').unwrap_or(0.0),
                ),
            ),
            Plane::YZ => (
                (1, 2, 0),
                (
                    params.get_number::<f64>('J').unwrap_or(0.0),
                    params.get_number::<f64>('K').unwrap_or(0.0),
                ),
            ),
        };

        if offset.0 == 0.0 && offset.1 == 0.0 {
            return None; // We need at least one coordinate to work with
        }

        Some(ArcArgs {
            target: Vec3::new(
                params
                    .get_number::<f64>('X')
                    .map_or(toolhead_state.position.x, |c| map_coord(c, 0)),
                params
                    .get_number::<f64>('Y')
                    .map_or(toolhead_state.position.y, |c| map_coord(c, 1)),
                params
                    .get_number::<f64>('Z')
                    .map_or(toolhead_state.position.z, |c| map_coord(c, 2)),
            ),
            e: params.get_number::<f64>('E').map(|c| map_coord(c, 3)),
            velocity: params
                .get_number::<f64>('F')
                .map_or(toolhead_state.velocity, |v| v / 60.0),
            axes,
            offset,
            mm_per_arc_segment,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
struct ArcArgs {
    target: Vec3,
    e: Option<f64>,
    velocity: f64,
    axes: (usize, usize, usize),
    offset: (f64, f64),
    mm_per_arc_segment: f64,
}

impl ArcArgs {
    // Ported from klipper, originally from Marlins plan-arc() function.
    fn plan_arc(
        &self,
        start_position: Vec3,
        direction: ArcDirection,
        mm_per_arc_segment: f64,
    ) -> (usize, impl Iterator<Item = Vec3> + '_) {
        let current_position = start_position.as_ref();
        let target_position = self.target.as_ref();
        let (alpha_axis, beta_axis, helical_axis) = self.axes;

        let r_p = -self.offset.0;
        let r_q = -self.offset.1;

        let center_p = current_position[alpha_axis] - r_p;
        let center_q = current_position[beta_axis] - r_q;
        let rt_alpha = target_position[alpha_axis] - center_p;
        let rt_beta = target_position[beta_axis] - center_q;
        let mut angular_travel =
            (r_p * rt_beta - r_q * rt_alpha).atan2(r_p * rt_alpha + r_q * rt_beta);
        if angular_travel < 0.0 {
            angular_travel += 2.0 * std::f64::consts::PI;
        }
        if direction == ArcDirection::Clockwise {
            angular_travel -= 2.0 * std::f64::consts::PI;
        }

        if angular_travel == 0.0
            && current_position[alpha_axis] == target_position[alpha_axis]
            && current_position[beta_axis] == target_position[beta_axis]
        {
            angular_travel = 2.0 * std::f64::consts::PI
        }

        let linear_travel = target_position[helical_axis] - current_position[helical_axis];
        let radius = r_p.hypot(r_q);
        let flat_mm = radius * angular_travel;
        let mm_of_travel = if linear_travel != 0.0 {
            flat_mm.hypot(linear_travel)
        } else {
            flat_mm.abs()
        };

        let segments = ((mm_of_travel / mm_per_arc_segment).floor() as usize).max(1);

        let theta_per_segment = angular_travel / (segments as f64);
        let linear_per_segment = linear_travel / (segments as f64);
        (
            segments,
            (1..segments)
                .map(move |i| {
                    let i = i as f64;
                    let dist_helical = i * linear_per_segment;
                    let cos_ti = (i * theta_per_segment).cos();
                    let sin_ti = (i * theta_per_segment).sin();
                    let r_p = -self.offset.0 * cos_ti + self.offset.1 * sin_ti;
                    let r_q = -self.offset.0 * sin_ti - self.offset.1 * cos_ti;
                    let mut coord = [0.0f64; 3];
                    coord[alpha_axis] = center_p + r_p;
                    coord[beta_axis] = center_q + r_q;
                    coord[helical_axis] = start_position.as_ref()[helical_axis] + dist_helical;
                    coord.into()
                })
                .chain(std::iter::once(self.target)),
        )
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum ArcDirection {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Plane {
    XY,
    XZ,
    YZ,
}

impl Default for Plane {
    fn default() -> Self {
        Self::XY
    }
}
