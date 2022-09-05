use crate::gcode::GCodeExtendedParams;
use crate::kind_tracker::KindTracker;
use crate::planner::{OperationSequence, ToolheadState};
use serde::{Deserialize, Serialize};

#[allow(clippy::trivially_copy_pass_by_ref)]
fn is_zero(num: &f64) -> bool {
    *num < f64::EPSILON
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FirmwareRetractionOptions {
    pub retract_length: f64,
    pub unretract_extra_length: f64,
    pub unretract_speed: f64,
    pub retract_speed: f64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub lift_z: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum FirmwareRetractionState {
    Unretracted,
    Retracted {
        lifted_z: f64,
        unretract_length: f64,
    },
}

impl Default for FirmwareRetractionState {
    fn default() -> Self {
        FirmwareRetractionState::Unretracted
    }
}

impl FirmwareRetractionState {
    pub fn set_options(&self, toolhead_state: &mut ToolheadState, params: &GCodeExtendedParams) {
        let settings = &mut toolhead_state.limits.firmware_retraction.as_mut().unwrap();
        if let Some(v) = params.get_number::<f64>("retract_length") {
            settings.retract_length = v.max(0.0);
        }
        if let Some(v) = params.get_number::<f64>("retract_speed") {
            settings.retract_speed = v.max(0.0);
        }
        if let Some(v) = params.get_number::<f64>("unretract_extra_length") {
            settings.unretract_extra_length = v.max(0.0);
        }
        if let Some(v) = params.get_number::<f64>("unretract_speed") {
            settings.unretract_speed = v.max(0.0);
        }
        if let Some(v) = params.get_number::<f64>("lift_z") {
            settings.lift_z = v.max(0.0);
        }
    }

    pub fn retract(
        &mut self,
        kind_tracker: &mut KindTracker,
        toolhead_state: &mut ToolheadState,
        op_sequence: &mut OperationSequence,
    ) -> usize {
        let mut n = 0;
        if let FirmwareRetractionState::Unretracted = self {
            let settings = &toolhead_state.limits.firmware_retraction.as_mut().unwrap();
            let lifted_z = settings.lift_z;
            let retract_length = settings.retract_length;
            let unretract_extra_length = settings.unretract_extra_length;

            if retract_length > 0.0 {
                let v = toolhead_state.velocity;
                toolhead_state.velocity = settings.retract_speed;
                let m = toolhead_state.perform_relative_move(
                    [None, None, None, Some(retract_length)],
                    Some(kind_tracker.get_kind("Firmware retract")),
                );
                op_sequence.add_move(m, toolhead_state);
                toolhead_state.velocity = v;
                n += 1;
            }

            if lifted_z > 0.0 {
                op_sequence.add_move(
                    toolhead_state.perform_relative_move(
                        [None, None, Some(lifted_z), None],
                        Some(kind_tracker.get_kind("Firmware retract Z hop")),
                    ),
                    toolhead_state,
                );
                n += 1;
            }

            *self = FirmwareRetractionState::Retracted {
                lifted_z,
                unretract_length: retract_length + unretract_extra_length,
            };
        }
        n
    }

    pub fn unretract(
        &mut self,
        kind_tracker: &mut KindTracker,
        toolhead_state: &mut ToolheadState,
        op_sequence: &mut OperationSequence,
    ) -> usize {
        let mut n = 0;
        if let FirmwareRetractionState::Retracted {
            lifted_z,
            unretract_length: retracted_length,
        } = self
        {
            let settings = &toolhead_state.limits.firmware_retraction.as_mut().unwrap();
            if *retracted_length > 0.0 {
                let v = toolhead_state.velocity;
                toolhead_state.velocity = settings.unretract_speed;
                let m = toolhead_state.perform_relative_move(
                    [None, None, None, Some(-*retracted_length)],
                    Some(kind_tracker.get_kind("Firmware unretract")),
                );
                op_sequence.add_move(m, toolhead_state);
                toolhead_state.velocity = v;
                n += 1;
            }

            if *lifted_z > 0.0 {
                op_sequence.add_move(
                    toolhead_state.perform_relative_move(
                        [None, None, Some(-*lifted_z), None],
                        Some(kind_tracker.get_kind("Firmware unretract Z hop")),
                    ),
                    toolhead_state,
                );
                n += 1;
            }

            *self = FirmwareRetractionState::Unretracted;
        }
        n
    }
}
