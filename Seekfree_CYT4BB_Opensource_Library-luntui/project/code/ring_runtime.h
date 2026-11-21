#ifndef CODE_RING_RUNTIME_H_
#define CODE_RING_RUNTIME_H_

#include <stdint.h>

typedef struct {
    uint32_t now_ms, control_dt_ms, command_age_ms, detected, exited;
    int32_t wheel_mrev;
    int16_t phase, turn_deg, steer_error, island_near;
    int16_t cap, requested_boost, applied_boost;
    int16_t original_target10, final_target10, original_turn10, final_turn10;
    int16_t route, raw_state;
    uint8_t candidate, cross_veto, committed, path_valid, stop, fault;
    uint8_t active, output_state;
    /* Optional telemetry never owns control or refreshes its watchdog. */
    uint32_t command_sequence, command_epoch, diagnostic_command_sequence, diagnostic_epoch;
    uint32_t frame_sequence, diagnostic_age_ms;
    uint16_t visual_dt_ms, reject;
    int16_t near_x, look_x;
    int8_t entry_side;
    uint8_t candidate_detected, candidate_allowed, score, confirm_frames;
    uint8_t branch_ok, path_source, classification, diagnostic_valid;
    /* Independent entry telemetry; never read by the controller/watchdog. */
    uint32_t entry_diagnostic_command_sequence, entry_diagnostic_epoch;
    uint32_t entry_frame_sequence, entry_diagnostic_age_ms;
    uint16_t entry_flags, hold_reject, path_age_ms;
    int16_t hold_turn10, entry_turn10;
    uint8_t decision, lost_frames, entry_diagnostic_valid;
    /* Independent branch telemetry, separate from command/D/E validity. */
    uint32_t geometry_command_sequence, geometry_epoch;
    uint32_t geometry_frame_sequence, geometry_age_ms;
    uint16_t strong_age_ms;
    int16_t guide_x;
    uint8_t branch_mode, inner_rows, associated_rows, target_support, guide_y;
    uint8_t geometry_valid;
    uint32_t topology_command_sequence, topology_epoch, topology_frame, topology_age_ms;
    int32_t lap_origin_mdeg, lap_mdeg;
    int16_t near_row, far_row;
    int8_t ring_side;
    uint8_t approach_stage, left_entry, lap_valid, topology_valid, mouth_source;
    uint32_t mouth_command_sequence,mouth_epoch,mouth_frame,mouth_age_ms;
    int16_t mouth_near_x,mouth_near_y,mouth_far_x,mouth_far_y,mouth_first_y,mouth_progress;
    uint16_t mouth_gate;
    uint8_t mouth_ref,mouth_near_count,mouth_far_count,mouth_gap,mouth_valid;
    uint8_t mouth_near_missed,mouth_pass_reason;
    /* Optional exit evidence, isolated from control freshness and ownership. */
    uint32_t exit_diagnostic_command_sequence,exit_diagnostic_epoch;
    uint32_t exit_frame_sequence,exit_diagnostic_age_ms;
    uint16_t exit_reject;
    uint8_t exit_candidate,exit_latched,exit_confirmed;
    uint8_t exit_candidate_frames,exit_confirm_frames,exit_diagnostic_valid;
    int16_t exit_lap10;
    int32_t exit_forward_mrev;
    uint8_t control_version,vision_version,pair_ok;
    uint8_t force_active,force_reason;
    int8_t force_side;
    int16_t force_target10,force_rate10;
    uint8_t release_reason,run_requested,standing;
    /* Measured final <=70deg finish, separate from the entry maneuver. */
    uint8_t exit_force_stage,exit_force_reason;
    int8_t exit_force_side;
    int16_t exit_force_start10,exit_force_turn10,exit_force_remaining10;
    uint16_t exit_force_elapsed_ms;
    uint8_t maneuver_stage;
    int16_t maneuver_turn10,maneuver_remaining10;
    uint16_t maneuver_elapsed_ms;
    /* Diagnostic mirrors. Control uses separate local motion samples. */
    int16_t held_target10, speed_goal10, measured_forward10, pitch100, speed_tilt100;
    int32_t left_pwm, right_pwm;
    uint32_t motion_age_ms;
    int16_t yaw_raw,yaw_bias_raw,entry_turn_cap10;
    uint8_t yaw_calibrated,yaw_window_state;
    uint32_t yaw_calibration_ms,yaw_calibration_count;
} ring_runtime_diagnostics_t;

void ring_runtime_init(float gyro_dps_per_count);
void ring_runtime_set_operator(uint8_t run_requested, uint8_t binary_display);
void ring_runtime_observe_motion(float measured_rpm,float pitch_deg,
    float speed_tilt_deg,int left_pwm,int right_pwm);
/* Called AFTER original IMU/quaternion work at 1ms. -1=no buzzer change,
 * 0=turn event beep off, 1=turn event beep on. Parked preview events may
 * beep without enabling balance/motion. Requested startup owns the buzzer
 * until standing. No waits or sensor reads. */
int8_t ring_runtime_tick(int16_t gyro_z_raw, uint8_t standing);
uint8_t ring_runtime_watchdog_stop(void);
void ring_runtime_poll_timeout(float *forward_target,float *turn_out);
/* Called AFTER the unchanged original controller. Ordinary steering is
 * preserved; production forward targets use a continuous speed profile. */
void ring_runtime_apply(float *forward_target, float *turn_out, float wheel_revs,
                        int vision_state);
const ring_runtime_diagnostics_t *ring_runtime_diagnostics(void);

#endif
