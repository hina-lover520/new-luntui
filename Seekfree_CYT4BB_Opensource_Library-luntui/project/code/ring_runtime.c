#include "ring_runtime.h"
#include "ring_link.h"
#include "ring_control.h"
#include "ring_force.h"
#include "ring_maneuver.h"
#include "ring_exit_turn.h"
#include "ring_event_beep.h"
#include "ring_entry_speed.h"
#include "ring_forward_profile.h"
#include "ring_yaw_bias.h"
#include "ring.h"
#include <string.h>

static volatile uint32_t runtime_ms, last_command_ms;
static volatile float heading_deg;
static volatile float heading_rate;
static ring_force_t force_control;
static ring_maneuver_t maneuver;
static ring_exit_turn_t exit_turn;
static ring_event_beep_t event_beep;
static ring_entry_speed_t entry_speed;
static ring_forward_profile_t forward_profile;
static float measured_forward;
static uint8_t measured_forward_valid;
static float preceding_target;
static uint32_t preceding_target_ms,motion_sample_ms;
static uint8_t preceding_target_valid,motion_sample_valid;
static uint32_t maneuver_event_seen;
static int force_requested,force_side;
static volatile int speed_cap;
static volatile uint8_t runtime_ready;
static float gyro_scale;
static ring_yaw_bias_t yaw_bias;
static uint32_t epoch, feedback_sequence, seen_sequence;
static uint32_t detected_seen, exited_seen;
static int ring_fault;
static volatile uint8_t ring_active;
static volatile uint8_t abort_pending;
static volatile float fallback_forward,fallback_turn;
static int requested_boost;
static float applied_boost;
static ring_runtime_diagnostics_t diagnostics;
static uint32_t diagnostic_sequence, last_diagnostic_ms;
static uint32_t entry_diagnostic_sequence, last_entry_diagnostic_ms;
static uint32_t geometry_diagnostic_sequence, last_geometry_diagnostic_ms;
static uint32_t topology_sequence, last_topology_ms;
static uint32_t mouth_sequence,last_mouth_ms;
static uint32_t exit_diagnostic_sequence,last_exit_diagnostic_ms;
static uint32_t pair_last_ms;
static uint8_t pair_packet_valid;
static volatile uint32_t operator_flags; /* main writes; 1ms buzzer tick reads */

/* The first EXIT must independently prove the lap. Later EXIT/RECOVER
 * packets belong to this same finite departure, rather than re-running
 * the initial 15-degree gate while the car follows the outgoing road. */
static struct {
    volatile uint8_t valid,blocked;
    uint8_t force_requested;
    uint32_t epoch,detected,started_ms;
    int side;
    float heading,wheel_high;
} exit_latch;

static void reset_exit_latch(void)
{
    memset(&exit_latch,0,sizeof(exit_latch));
    memset(&exit_turn,0,sizeof(exit_turn));
}

static void abort_exit_latch(unsigned reason)
{
    unsigned forced=exit_latch.force_requested;
    reset_exit_latch();exit_latch.blocked=1;
    if(forced){exit_turn.stage=3u;exit_turn.reason=reason; }
    abort_pending=(uint8_t)reason;ring_active=0;speed_cap=-1;
    ring_fault=0;requested_boost=0;applied_boost=0;force_requested=0;
    memset(&force_control,0,sizeof(force_control));
    diagnostics.release_reason=(uint8_t)reason;
    if(maneuver.stage && maneuver.stage<4u) {
        maneuver.stage=4;
        maneuver.reason=6;maneuver.target_rate=0;
    }
}

static int exit_motion_valid(uint32_t now,float wheel_revs,float forward)
{
    return runtime_ready && yaw_bias.ready && (operator_flags&RING_OPERATOR_RUN) &&
        !abort_pending && isfinite(forward) && forward>0.f && isfinite(wheel_revs) &&
        isfinite(heading_deg) && measured_forward_valid && motion_sample_valid &&
        now-motion_sample_ms<=150u && measured_forward>0.f;
}

static int exit_continuation_valid(uint32_t now,float wheel_revs,float forward)
{
    /* 2mrev permits the feedback's 1mrev quantization. Comparing with the
     * high-water mark prevents a sequence of small reversals accumulating. */
    float progress=maneuver.side*(heading_deg-maneuver.turn_yaw);
    int heading_ok=maneuver.net_exit_reached && progress>=RING_EXIT_MIN_DEG &&
        fabsf(heading_deg-exit_latch.heading)<=RING_EXIT_CONTINUE_YAW_DEG &&
        (RING_EXIT_NET_MIN_DEG<=0.f ||
         fabsf(progress-RING_EXIT_MIN_DEG)<=RING_EXIT_FORCE_ALIGN_DEG);
    if(exit_latch.force_requested) {
        /* A measured exit starts only after net400; its bounded reverse
         * correction may then finish at the original360-degree heading.
         * The net400 history stays latched, rather than cancelling the
         * accepted departure as soon as progress falls below400. */
        int correcting=exit_turn.side==-maneuver.side;
        heading_ok=maneuver.net_exit_reached && exit_turn.stage!=3u &&
            progress>=(correcting?RING_EXIT_MIN_DEG-(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_FORCE_BACKTRACK_DEG):
                exit_turn.start_lap-RING_EXIT_FORCE_BACKTRACK_DEG) &&
            progress<=(correcting?exit_turn.start_lap+RING_EXIT_FORCE_BACKTRACK_DEG:
                RING_EXIT_SEARCH_END_DEG) &&
            (exit_turn.stage!=2u || (maneuver.lap_reached &&
                fabsf(progress-RING_EXIT_MIN_DEG)<=(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_FORCE_BACKTRACK_DEG)));
    }
    return exit_latch.valid && !exit_latch.blocked && ring_active &&
        maneuver.stage==3u && maneuver_event_seen==exit_latch.detected &&
        epoch==exit_latch.epoch && maneuver.side==exit_latch.side &&
        exit_motion_valid(now,wheel_revs,forward) && heading_ok &&
        now-exit_latch.started_ms<=RING_EXIT_TIMEOUT_MS &&
        wheel_revs>=exit_latch.wheel_high-.002f;
}

const ring_runtime_diagnostics_t *ring_runtime_diagnostics(void)
{
    return &diagnostics;
}

void ring_runtime_init(float gyro_dps_per_count)
{
    ring_packet_t previous;
    epoch = 1;
    if (ring_link_read(ring_motion_mailbox, &previous)) {
        epoch = (uint32_t)previous.word[RL_EPOCH] + 1u;
        if (!epoch) epoch = 1;
    }
    runtime_ms = last_command_ms = pair_last_ms = 0;
    pair_packet_valid=0;
    heading_deg = 0.0f;
    heading_rate=0;force_requested=force_side=0;
    memset(&force_control,0,sizeof(force_control));memset(&maneuver,0,sizeof(maneuver));
    reset_exit_latch();
    maneuver_event_seen=0;
    memset(&entry_speed,0,sizeof(entry_speed));
    memset(&forward_profile,0,sizeof(forward_profile));
    measured_forward=0;measured_forward_valid=0;
    preceding_target=0;preceding_target_ms=motion_sample_ms=0;
    preceding_target_valid=motion_sample_valid=0;
    gyro_scale = gyro_dps_per_count;
    ring_yaw_bias_init(&yaw_bias,gyro_scale);
    speed_cap = -1;
    runtime_ready = 0;
    operator_flags = RING_OPERATOR_VALID;
    feedback_sequence = seen_sequence = 0;
    detected_seen = exited_seen = 0;
    ring_event_beep_init(&event_beep);
    ring_fault = 0;
    ring_active = 0;
    abort_pending=0;fallback_forward=fallback_turn=0;
    requested_boost = 0;
    applied_boost = 0.0f;
    memset(&diagnostics, 0, sizeof(diagnostics));
    diagnostics.cap = -1;
    diagnostics.motion_age_ms=UINT32_MAX;
    diagnostics.command_epoch = epoch;
    diagnostics.control_version=RING_PAIR_VERSION;
    diagnostics.island_near = -1;
    diagnostics.near_x = diagnostics.look_x = -1;
    diagnostics.visual_dt_ms = 65535u;
    diagnostic_sequence = last_diagnostic_ms = 0;
    entry_diagnostic_sequence = last_entry_diagnostic_ms = 0;
    geometry_diagnostic_sequence = last_geometry_diagnostic_ms = 0;
    diagnostics.guide_x = -1;
    diagnostics.guide_y = 255u;
    topology_sequence = last_topology_ms = mouth_sequence = last_mouth_ms = 0;
    exit_diagnostic_sequence=last_exit_diagnostic_ms=0;
    diagnostics.mouth_near_x=diagnostics.mouth_near_y=-1;
    diagnostics.mouth_far_x=diagnostics.mouth_far_y=diagnostics.mouth_first_y=-1;
    diagnostics.near_row = diagnostics.far_row = -1;
}

void ring_runtime_set_operator(uint8_t run_requested, uint8_t binary_display)
{
    if(!run_requested){
        exit_latch.valid=0;exit_latch.blocked=1;
        ring_event_beep_cancel_pairs(&event_beep);
    }
    operator_flags = RING_OPERATOR_VALID |
        (run_requested ? RING_OPERATOR_RUN : 0u) |
        (binary_display ? RING_OPERATOR_BINARY : 0u);
}

void ring_runtime_observe_motion(float measured_rpm,float pitch_deg,
    float speed_tilt_deg,int left_pwm,int right_pwm)
{
    measured_forward_valid=(uint8_t)isfinite(measured_rpm);
    measured_forward=measured_forward_valid?measured_rpm:0.f;
    diagnostics.measured_forward10=(int16_t)fmaxf(-32768.f,fminf(32767.f,measured_rpm*10.f));
    diagnostics.pitch100=(int16_t)fmaxf(-32768.f,fminf(32767.f,pitch_deg*100.f));
    diagnostics.speed_tilt100=(int16_t)fmaxf(-32768.f,fminf(32767.f,speed_tilt_deg*100.f));
    diagnostics.left_pwm=left_pwm;diagnostics.right_pwm=right_pwm;
    motion_sample_ms=runtime_ms;motion_sample_valid=1;
}

int8_t ring_runtime_tick(int16_t gyro_z_raw, uint8_t standing)
{
    uint32_t now = ++runtime_ms;
    if (!RING_ENABLE) return -1;
    runtime_ready = standing;
    if(!standing && maneuver.stage){exit_latch.valid=0;exit_latch.blocked=1;}
    /* Ring yaw has its own stable standby calibration. RUN freezes a
     * valid zero before servos move; quaternion/balance offsets stay intact.
     * No fixed one-second boot sample is averaged for the whole session. */
    ring_yaw_bias_update(&yaw_bias,gyro_z_raw,now,standing,
        (operator_flags&RING_OPERATOR_RUN)!=0u);
    if (standing && now >= 2000u && yaw_bias.ready) {
        float rate=ring_yaw_bias_rate(&yaw_bias,gyro_z_raw);
        heading_rate+=.02f*(rate-heading_rate);
        heading_deg += rate * 0.001f;
    } else heading_rate=0;

    return ring_event_beep_tick(&event_beep,now,
        !standing && (operator_flags & RING_OPERATOR_RUN));
}

uint8_t ring_runtime_watchdog_stop(void)
{
    return (uint8_t)(!RING_TRACK_MODE && RING_ENABLE && runtime_ready && ring_active && speed_cap >= 0 &&
                     runtime_ms - last_command_ms > RING_LINK_TIMEOUT_MS);
}

/* Existing 1ms call site: restore the last output calculated BEFORE ring
 * overrides once if its producer stops. No waiting, stop command, forced
 * steering or stale cap is kept alive by a stalled main loop. A later
 * normal vision frame supplies the current ordinary error as usual. */
void ring_runtime_poll_timeout(float *forward,float *turn)
{
    if(!RING_ENABLE)return;
    if(!RING_TRACK_MODE) {
        if(ring_runtime_watchdog_stop()){*forward=0;*turn=0;}
        return;
    }
    if(runtime_ready && ring_active && runtime_ms-last_command_ms>RING_LINK_TIMEOUT_MS) {
        reset_exit_latch();exit_latch.blocked=1;
        abort_pending=4;ring_active=0;speed_cap=-1;
        *forward=(operator_flags&RING_OPERATOR_RUN)?fallback_forward:0;
        *turn=(operator_flags&RING_OPERATOR_RUN)?fallback_turn:0;
    }
}

/* Retry separately: the producer publishes command before diagnostics.
 * Telemetry validity cannot change control validity, freshness or outputs. */
static void read_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RD_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence != 0u &&
        (uint32_t)packet.word[RL_SEQUENCE] != diagnostic_sequence) {
        uint32_t flags = (uint32_t)packet.word[RD_FLAGS];
        int source = ring_diagnostic_source(packet.word[RD_FLAGS]);
        int near_x = ring_diagnostic_near_x(packet.word[RD_POINTS]);
        int look_x = ring_diagnostic_look_x(packet.word[RD_POINTS]);
        int valid = (flags & 0xfc000000u) == 0u &&
            ((flags >> 8) & 3u) != 3u &&
            (source == 0 || source == 1 || source == 2 || source == 4 || source == 5 || source == 6 || source == 7) &&
            near_x >= -1 && near_x < RING_W && look_x >= -1 && look_x < RING_W;
        if (valid) {
            diagnostic_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            last_diagnostic_ms = now;
            diagnostics.diagnostic_epoch = epoch;
            diagnostics.diagnostic_command_sequence = (uint32_t)packet.word[RD_COMMAND_SEQUENCE];
            diagnostics.frame_sequence = (uint32_t)packet.word[RD_FRAME];
            diagnostics.visual_dt_ms = (uint16_t)(uint32_t)packet.word[RD_TIMING_REJECT];
            diagnostics.reject = (uint16_t)((uint32_t)packet.word[RD_TIMING_REJECT] >> 16);
            diagnostics.candidate_detected = (uint8_t)ring_diagnostic_detected(packet.word[RD_FLAGS]);
            diagnostics.candidate_allowed = (uint8_t)ring_diagnostic_allowed(packet.word[RD_FLAGS]);
            diagnostics.score = (uint8_t)ring_diagnostic_score(packet.word[RD_FLAGS]);
            diagnostics.confirm_frames = (uint8_t)ring_diagnostic_confirm_frames(packet.word[RD_FLAGS]);
            diagnostics.entry_side = (int8_t)ring_diagnostic_side(packet.word[RD_FLAGS]);
            diagnostics.branch_ok = (uint8_t)ring_diagnostic_branch(packet.word[RD_FLAGS]);
            diagnostics.path_source = (uint8_t)source;
            diagnostics.classification = (uint8_t)ring_diagnostic_classification(packet.word[RD_FLAGS]);
            diagnostics.near_x = (int16_t)near_x;
            diagnostics.look_x = (int16_t)look_x;
        }
    }
    diagnostics.diagnostic_age_ms = now - last_diagnostic_ms;
    diagnostics.diagnostic_valid = (uint8_t)(diagnostic_sequence != 0u &&
        diagnostics.diagnostic_epoch == epoch &&
        diagnostics.diagnostic_command_sequence == diagnostics.command_sequence &&
        diagnostics.diagnostic_age_ms <= RING_LINK_TIMEOUT_MS);
}

static void read_entry_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_entry_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RE_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence != 0u &&
        (uint32_t)packet.word[RL_SEQUENCE] != entry_diagnostic_sequence) {
        uint32_t flags = (uint32_t)packet.word[RE_FLAGS];
        uint32_t age_lost = (uint32_t)packet.word[RE_AGE_LOST];
        unsigned entry_flags = flags & 65535u;
        unsigned reject = (flags >> 16) & 255u;
        unsigned decision = flags >> 24;
        /* Unknown bits/decisions are rejected independently of commands/D. */
        if ((entry_flags & ~8191u) == 0u && (reject & ~63u) == 0u &&
            decision <= (RING_TRACK_MODE?16u:14u) && (age_lost & 0xff000000u) == 0u) {
            entry_diagnostic_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            last_entry_diagnostic_ms = now;
            diagnostics.entry_diagnostic_epoch = epoch;
            diagnostics.entry_diagnostic_command_sequence = (uint32_t)packet.word[RE_COMMAND_SEQUENCE];
            diagnostics.entry_frame_sequence = (uint32_t)packet.word[RE_FRAME];
            diagnostics.entry_flags = (uint16_t)entry_flags;
            diagnostics.hold_reject = (uint16_t)reject;
            diagnostics.decision = (uint8_t)decision;
            diagnostics.path_age_ms = (uint16_t)age_lost;
            diagnostics.lost_frames = (uint8_t)(age_lost >> 16);
            diagnostics.hold_turn10 = (int16_t)ring_entry_diagnostic_hold_turn(packet.word[RE_ANGLES]);
            diagnostics.entry_turn10 = (int16_t)ring_entry_diagnostic_entry_turn(packet.word[RE_ANGLES]);
        }
    }
    diagnostics.entry_diagnostic_age_ms = now - last_entry_diagnostic_ms;
    diagnostics.entry_diagnostic_valid = (uint8_t)(entry_diagnostic_sequence != 0u &&
        diagnostics.entry_diagnostic_epoch == epoch &&
        diagnostics.entry_diagnostic_command_sequence == diagnostics.command_sequence &&
        diagnostics.entry_diagnostic_age_ms <= RING_LINK_TIMEOUT_MS);
}

static void read_geometry_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_geometry_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RG_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence != 0u &&
        (uint32_t)packet.word[RL_SEQUENCE] != geometry_diagnostic_sequence) {
        uint32_t geometry = (uint32_t)packet.word[RG_GEOMETRY];
        unsigned mode = geometry & 255u;
        unsigned rows = (geometry >> 8) & 255u;
        unsigned associated = (geometry >> 16) & 255u;
        unsigned support = geometry >> 24;
        int x = ring_geometry_diagnostic_x(packet.word[RG_GUIDE]);
        unsigned y = ring_geometry_diagnostic_y(packet.word[RG_GUIDE]);
        int valid_point = (x == -1 && y == 255u) ||
            (x >= 0 && x < RING_W && y < RING_H);
        if (mode <= 6u && rows <= RING_H && associated <= RING_H &&
            (support & ~7u) == 0u &&
            ((uint32_t)packet.word[RG_STRONG_AGE] & 0xffff0000u) == 0u &&
            valid_point) {
            geometry_diagnostic_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            last_geometry_diagnostic_ms = now;
            diagnostics.geometry_epoch = epoch;
            diagnostics.geometry_command_sequence = (uint32_t)packet.word[RG_COMMAND_SEQUENCE];
            diagnostics.geometry_frame_sequence = (uint32_t)packet.word[RG_FRAME];
            diagnostics.branch_mode = (uint8_t)mode;
            diagnostics.inner_rows = (uint8_t)rows;
            diagnostics.associated_rows = (uint8_t)associated;
            diagnostics.target_support = (uint8_t)support;
            diagnostics.strong_age_ms = (uint16_t)(uint32_t)packet.word[RG_STRONG_AGE];
            diagnostics.guide_x = (int16_t)x;
            diagnostics.guide_y = (uint8_t)y;
        }
    }
    diagnostics.geometry_age_ms = now - last_geometry_diagnostic_ms;
    diagnostics.geometry_valid = (uint8_t)(geometry_diagnostic_sequence != 0u &&
        diagnostics.geometry_epoch == epoch &&
        diagnostics.geometry_command_sequence == diagnostics.command_sequence &&
        diagnostics.geometry_age_ms <= RING_LINK_TIMEOUT_MS);
}

/* H is optional evidence. Missing/corrupt/stale H cannot acquire control,
 * refresh last_command_ms, stop motion, or modify ordinary outputs. */
static void read_topology_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_topology_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RH_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence &&
        (uint32_t)packet.word[RL_SEQUENCE] != topology_sequence) {
        uint32_t flags=(uint32_t)packet.word[RH_FLAGS];
        unsigned stage=(flags>>2)&7u;
        int near_row=ring_topology_near_row(packet.word[RH_FLAGS]);
        int far_row=ring_topology_far_row(packet.word[RH_FLAGS]);
        if ((flags&0xfe000000u)==0u && ((flags>>23)&3u)<=2u && (flags&3u)!=3u &&
            stage<=RING_TANGENT_EXIT && near_row<RING_H && far_row<RING_H) {
            topology_sequence=(uint32_t)packet.word[RL_SEQUENCE];
            last_topology_ms=now;
            diagnostics.topology_epoch=epoch;
            diagnostics.topology_command_sequence=(uint32_t)packet.word[RH_COMMAND_SEQUENCE];
            diagnostics.topology_frame=(uint32_t)packet.word[RH_FRAME];
            diagnostics.ring_side=(int8_t)ring_topology_side(packet.word[RH_FLAGS]);
            diagnostics.approach_stage=(uint8_t)stage;
            diagnostics.left_entry=(uint8_t)((flags>>5)&1u);
            diagnostics.lap_valid=(uint8_t)((flags>>6)&1u);
            diagnostics.mouth_source=(uint8_t)((flags>>23)&3u);
            diagnostics.near_row=(int16_t)near_row;
            diagnostics.far_row=(int16_t)far_row;
            diagnostics.lap_origin_mdeg=packet.word[RH_ORIGIN_MDEG];
            diagnostics.lap_mdeg=packet.word[RH_LAP_MDEG];
        }
    }
    diagnostics.topology_age_ms=now-last_topology_ms;
    diagnostics.topology_valid=(uint8_t)(topology_sequence &&
        diagnostics.topology_epoch==epoch &&
        diagnostics.topology_command_sequence==diagnostics.command_sequence &&
        diagnostics.topology_age_ms<=RING_LINK_TIMEOUT_MS);
}

static void read_mouth_diagnostics(uint32_t now)
{
    ring_packet_t p;
    if(ring_link_read_mouth_diagnostic(&p) && (uint32_t)p.word[RL_EPOCH]==epoch &&
        (uint32_t)p.word[RJ_COMMAND_SEQUENCE]==diagnostics.command_sequence &&
        diagnostics.command_sequence && (uint32_t)p.word[RL_SEQUENCE]!=mouth_sequence) {
        uint32_t gates=(uint32_t)p.word[RJ_GATES],pair=(uint32_t)p.word[RJ_PROGRESS_PAIR];
        int nx=ring_mouth_coordinate(p.word[RJ_POINTS],0);
        int ny=ring_mouth_coordinate(p.word[RJ_POINTS],8);
        int fx=ring_mouth_coordinate(p.word[RJ_POINTS],16);
        int fy=ring_mouth_coordinate(p.word[RJ_POINTS],24);
        int first=ring_mouth_coordinate(p.word[RJ_PROGRESS_PAIR],0);
        int nvalid=(nx== -1 && ny== -1)||(nx>=0 && nx<RING_W && ny>=0 && ny<RING_H);
        int fvalid=(fx== -1 && fy== -1)||(fx>=0 && fx<RING_W && fy>=0 && fy<RING_H);
        if((gates&0x80000000u)==0u && nvalid && fvalid && first<RING_H &&
            ((pair>>16)&255u)==RING_PAIR_VERSION && (pair>>24)==RING_PAIR_VERSION) {
            mouth_sequence=(uint32_t)p.word[RL_SEQUENCE];last_mouth_ms=now;
            diagnostics.mouth_epoch=epoch;diagnostics.mouth_command_sequence=(uint32_t)p.word[RJ_COMMAND_SEQUENCE];
            diagnostics.mouth_frame=(uint32_t)p.word[RJ_FRAME];
            diagnostics.mouth_near_x=(int16_t)nx;diagnostics.mouth_near_y=(int16_t)ny;
            diagnostics.mouth_far_x=(int16_t)fx;diagnostics.mouth_far_y=(int16_t)fy;
            diagnostics.mouth_first_y=(int16_t)first;
            diagnostics.mouth_progress=(int16_t)ring_mouth_progress(p.word[RJ_PROGRESS_PAIR]);
            diagnostics.mouth_gate=(uint16_t)ring_mouth_gate_value((int32_t)gates);diagnostics.mouth_ref=(uint8_t)((gates>>13)&1u);
            diagnostics.mouth_near_count=(uint8_t)((gates>>14)&15u);
            diagnostics.mouth_far_count=(uint8_t)((gates>>18)&15u);
            diagnostics.mouth_gap=(uint8_t)((gates>>22)&3u);
            diagnostics.mouth_near_missed=(uint8_t)((gates>>26)&7u);
            diagnostics.mouth_pass_reason=(uint8_t)((gates>>29)&3u);
        }
    }
    diagnostics.mouth_age_ms=now-last_mouth_ms;
    diagnostics.mouth_valid=(uint8_t)(mouth_sequence && diagnostics.mouth_epoch==epoch &&
        diagnostics.mouth_command_sequence==diagnostics.command_sequence &&
        diagnostics.mouth_age_ms<=RING_LINK_TIMEOUT_MS);
}

/* L reports exit evidence only. Its validity never participates in control
 * ownership, command freshness, event beeps, or steering. */
static void read_exit_diagnostics(uint32_t now)
{
    ring_packet_t p;
    if(ring_link_read_exit_diagnostic(&p) && (uint32_t)p.word[RL_EPOCH]==epoch &&
        (uint32_t)p.word[RX_COMMAND_SEQUENCE]==diagnostics.command_sequence &&
        diagnostics.command_sequence && (uint32_t)p.word[RL_SEQUENCE]!=exit_diagnostic_sequence) {
        uint32_t status=(uint32_t)p.word[RX_STATUS];
        uint32_t counts=(uint32_t)p.word[RX_COUNTS_LAP];
        if((status&~0x0007007fu)==0u) {
            exit_diagnostic_sequence=(uint32_t)p.word[RL_SEQUENCE];last_exit_diagnostic_ms=now;
            diagnostics.exit_diagnostic_epoch=epoch;
            diagnostics.exit_diagnostic_command_sequence=(uint32_t)p.word[RX_COMMAND_SEQUENCE];
            diagnostics.exit_frame_sequence=(uint32_t)p.word[RX_FRAME];
            diagnostics.exit_reject=(uint16_t)(status&127u);
            diagnostics.exit_candidate=(uint8_t)((status>>16)&1u);
            diagnostics.exit_latched=(uint8_t)((status>>17)&1u);
            diagnostics.exit_confirmed=(uint8_t)((status>>18)&1u);
            diagnostics.exit_candidate_frames=(uint8_t)counts;
            diagnostics.exit_confirm_frames=(uint8_t)(counts>>8);
            diagnostics.exit_lap10=(int16_t)ring_exit_diagnostic_lap10(p.word[RX_COUNTS_LAP]);
            diagnostics.exit_forward_mrev=p.word[RX_FORWARD_MREV];
        }
    }
    diagnostics.exit_diagnostic_age_ms=now-last_exit_diagnostic_ms;
    diagnostics.exit_diagnostic_valid=(uint8_t)(exit_diagnostic_sequence &&
        diagnostics.exit_diagnostic_epoch==epoch &&
        diagnostics.exit_diagnostic_command_sequence==diagnostics.command_sequence &&
        diagnostics.exit_diagnostic_age_ms<=RING_LINK_TIMEOUT_MS);
}

void ring_runtime_apply(float *forward_target, float *turn_out, float wheel_revs,
                        int vision_state)
{
    ring_packet_t packet;
    uint32_t now = runtime_ms;
    float original_forward = *forward_target;
    float original_turn = *turn_out;
    int command_read;
    if (!RING_ENABLE) return;
    fallback_forward=original_forward;fallback_turn=original_turn;
    diagnostics.entry_turn_cap10=0;
    /* Integrator-owned signed yaw is the only source of this episode
     * latch; a vision packet's advertised progress cannot arm it. */
    if((maneuver.stage==2u || maneuver.stage==3u) &&
       isfinite(heading_deg) &&
       maneuver.side*(heading_deg-maneuver.turn_yaw)>=RING_EXIT_NET_MIN_DEG)
        maneuver.net_exit_reached=1u;
    if(RING_TRACK_MODE && RING_TIMED_ENTRY && ring_active && maneuver.stage &&
       (exit_latch.blocked || (exit_latch.valid &&
        !exit_continuation_valid(now,wheel_revs,original_forward))))
        abort_exit_latch(exit_latch.valid && now-exit_latch.started_ms>RING_EXIT_TIMEOUT_MS?3u:1u);
    if(exit_latch.valid && wheel_revs>exit_latch.wheel_high)exit_latch.wheel_high=wheel_revs;
    if(abort_pending) {
        requested_boost=0;applied_boost=0;ring_fault=0;
        memset(&force_control,0,sizeof(force_control));
        diagnostics.release_reason=abort_pending;
    }
    command_read=ring_link_read(ring_command_mailbox,&packet);
    if(command_read && (uint32_t)packet.word[RL_EPOCH]!=epoch && exit_latch.valid)
        abort_exit_latch(1u);
    if (command_read &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RL_SEQUENCE] != seen_sequence) {
        uint32_t detected = (uint32_t)packet.word[RC_DETECTED];
        uint32_t exited = (uint32_t)packet.word[RC_EXITED];
        int cap = ring_command_cap_value(packet.word[RC_CAP]);
        int turn_deg = ring_command_turn_deg(packet.word[RC_CAP]);
        int phase = (int)((uint32_t)packet.word[RC_PHASE] & 255u);
        int boost = ring_command_boost(packet.word[RC_PHASE]);
        unsigned version=ring_command_version(packet.word[RC_PHASE]);
        uint32_t diag_word = (uint32_t)packet.word[RC_FAULT];
        int fault = ring_command_diag_fault(packet.word[RC_FAULT]);
        int raw_state = ring_command_diag_raw_state(packet.word[RC_FAULT]);
        int near = ring_command_diag_near(packet.word[RC_FAULT]);
        int route_code = (int)((diag_word >> 25) & 3u);
        int steer_error = ring_command_diag_error(packet.word[RC_FAULT]);
        int exit_requested=(diag_word&RING_COMMAND_EXIT_TURN)!=0u &&
            (phase==RING_EXIT || phase==RING_RECOVER);
        int timed=(diag_word&RING_COMMAND_TIMED_ENTRY)!=0u && !exit_requested;
        int owned = (timed && phase==RING_APPROACH) || (phase >= (RING_TRACK_MODE ? RING_ENTER : RING_APPROACH) &&
            phase <= RING_RECOVER);
        int was_active = ring_active;
        int force=(diag_word&RING_COMMAND_FORCE_ENTRY)!=0u;
        int route=ring_command_diag_route(packet.word[RC_FAULT]);
        unsigned released=(diag_word>>RING_COMMAND_RELEASE_SHIFT)&7u;
        /* A negative cap can retain ONLY a current, already-acquired timed
         * lap. It requests ordinary steering/speed while vision observes
         * the same event for its exit; it cannot acquire or revive a ring. */
        int observing_circle=RING_TRACK_MODE && RING_TIMED_ENTRY &&
            phase==RING_CIRCLE && cap==-1 && was_active && maneuver.stage==3u &&
            now-last_command_ms<=RING_LINK_TIMEOUT_MS && !abort_pending &&
            detected==maneuver_event_seen && route==-maneuver.side &&
            ring_command_diag_committed(packet.word[RC_FAULT]) &&
            !ring_command_diag_path_valid(packet.word[RC_FAULT]) &&
            !ring_command_diag_stop(packet.word[RC_FAULT]) && !force && !timed &&
            !exit_requested && !fault && !boost && !released && runtime_ready &&
            yaw_bias.ready && (operator_flags&RING_OPERATOR_RUN);
        int valid = phase <= RING_FAULT &&
                    (version==RING_PAIR_VERSION || (!RING_TRACK_MODE && version==0u)) &&
                    (!timed || (RING_TRACK_MODE && RING_TIMED_ENTRY &&
                        version==RING_PAIR_VERSION && phase>=RING_APPROACH && phase<=RING_RECOVER &&
                        route && (!force || phase==RING_APPROACH) && !fault && cap>0 && !boost &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]))) && fault <= 4 &&
                    (!released || (RING_TRACK_MODE && released<=4u &&
                        phase==RING_COOLDOWN && cap==-1 && !boost && !force && !timed && !fault &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]))) &&
                    (!force || (RING_TRACK_MODE && version==RING_PAIR_VERSION &&
                        ((timed && phase==RING_APPROACH) || (!timed &&
                        (phase==RING_ENTER || phase==RING_CIRCLE))) && !fault && cap>0 && route &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]) &&
                        !ring_command_diag_cross_veto(packet.word[RC_FAULT]))) &&
                    (!exit_requested || (RING_TRACK_MODE && RING_TIMED_ENTRY &&
                        version==RING_PAIR_VERSION && route && !force && !fault && cap>0 && !boost &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]) &&
                        ring_command_diag_committed(packet.word[RC_FAULT]))) &&
                    raw_state <= 2 && route_code <= 2 && near < RING_H &&
                    steer_error >= -90 && steer_error <= 90 &&
                    boost >= -RING_ENTRY_BOOST_RPM && boost <= RING_ENTRY_BOOST_RPM &&
                    cap >= -1 && cap <= RING_APPROACH_RPM &&
                    (!owned || cap >= 0 || observing_circle) &&
                    (phase != RING_FAULT || cap == 0) &&
                    (!RING_TRACK_MODE || phase != RING_APPROACH || ((timed?cap>0:cap==-1) && boost == 0)) &&
                    ((phase != RING_NORMAL && phase != RING_COOLDOWN) ||
                     (cap == -1 && boost == 0));
        if(valid && RING_TRACK_MODE && RING_TIMED_ENTRY && (maneuver.stage ||
           phase==RING_EXIT || phase==RING_RECOVER || (was_active && exited>exited_seen))) {
            int success=phase==RING_EXIT || phase==RING_RECOVER || exited>exited_seen;
            if(exit_latch.valid && owned && phase!=RING_EXIT && phase!=RING_RECOVER) {
                abort_exit_latch(1u);valid=0;
            }
            if(exit_latch.valid && owned && exit_latch.force_requested!=exit_requested) {
                abort_exit_latch(1u);valid=0;
            }
            if(exit_latch.valid && (owned || success) &&
               (detected!=exit_latch.detected || route!=-exit_latch.side)) {
                abort_exit_latch(1u);valid=0;
            }
            if(valid && was_active && owned && !timed && maneuver.stage<3u)valid=0;
            if(valid && success) {
                float progress=maneuver.side*(heading_deg-maneuver.turn_yaw);
                int matching=detected==maneuver_event_seen && route==-maneuver.side &&
                    version==RING_PAIR_VERSION && !fault &&
                    !ring_command_diag_stop(packet.word[RC_FAULT]) &&
                    (ring_command_diag_path_valid(packet.word[RC_FAULT]) ||
                     /* The paired explicit request is a latched control
                      * certificate of the vision core's measured corridor.
                      * A46ms control poll may miss its first path1 frame. */
                     (exit_requested && phase==RING_EXIT &&
                      ((!exit_latch.valid && ring_command_diag_committed(packet.word[RC_FAULT])) ||
                       (exit_latch.force_requested && exit_turn.stage==1u))));
                if(exit_latch.valid) {
                    if(!matching || (phase!=RING_EXIT && phase!=RING_RECOVER && phase!=RING_COOLDOWN) ||
                       !exit_continuation_valid(now,wheel_revs,original_forward))valid=0;
                    if(valid && exit_latch.force_requested &&
                       (phase==RING_RECOVER || exited>exited_seen) &&
                       (exit_turn.stage!=2u || !maneuver.net_exit_reached || !maneuver.lap_reached ||
                        fabsf(progress-RING_EXIT_MIN_DEG)>(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_FORCE_ALIGN_DEG:RING_EXIT_FORCE_BACKTRACK_DEG)))valid=0;
                } else {
                    /* A legacy first COOLDOWN+exit event is accepted only
                     * through this same independent initial gate. RECOVER
                     * cannot start a departure without an observed EXIT. */
                    if(!matching || !was_active || exit_latch.blocked || maneuver.stage!=3u ||
                       (!exit_requested && !maneuver.lap_reached) ||
                       !exit_motion_valid(now,wheel_revs,original_forward) ||
                       (phase!=RING_EXIT && !(phase==RING_COOLDOWN && exited>exited_seen)) ||
                       !maneuver.net_exit_reached ||
                       (RING_EXIT_NET_MIN_DEG>0.f && !exit_requested &&
                        fabsf(progress-RING_EXIT_MIN_DEG)>RING_EXIT_FORCE_ALIGN_DEG) ||
                       progress<((exit_requested && RING_EXIT_NET_MIN_DEG<=0.f)?
                           RING_EXIT_FORCE_START_DEG:RING_EXIT_MIN_DEG) ||
                       progress>((exit_requested || RING_EXIT_NET_MIN_DEG>0.f)?RING_EXIT_SEARCH_END_DEG:
                           RING_EXIT_MIN_DEG+RING_LAP_EXIT_HEADING_TOL) ||
                       wheel_revs-maneuver.turn_revs<RING_EXIT_MIN_REVS)valid=0;
                    else if(phase==RING_EXIT) {
                        maneuver.net_exit_reached=1u;
                        exit_latch.valid=1;exit_latch.epoch=epoch;exit_latch.detected=detected;
                        exit_latch.side=maneuver.side;exit_latch.started_ms=now;
                        exit_latch.heading=heading_deg;exit_latch.wheel_high=wheel_revs;
                        exit_latch.force_requested=(uint8_t)exit_requested;
                        if(exit_requested) {
                            if(!ring_exit_turn_begin(&exit_turn,now,maneuver.side,heading_deg,
                                maneuver.turn_yaw+maneuver.side*RING_EXIT_MIN_DEG,
                                progress,wheel_revs,*turn_out)) {
                                abort_exit_latch(1u);valid=0;
                            }
                        }
                        if(valid)ring_event_beep_triple(&event_beep);
                    }
                }
            }
        }
        seen_sequence = (uint32_t)packet.word[RL_SEQUENCE];
        diagnostics.vision_version=(uint8_t)version;
        pair_packet_valid=(uint8_t)(valid && version==RING_PAIR_VERSION);
        if(pair_packet_valid)pair_last_ms=now;
        /* Ring caps only belong to an actual ring steering handover. A
         * candidate packet, NORMAL with a bogus cap, or unacquired FAULT
         * must not limit/stop the original controller. */
        if (valid && (phase == RING_NORMAL || phase == RING_COOLDOWN ||
            (RING_TRACK_MODE && phase == RING_APPROACH && !timed))) {
            ring_active = 0;
            speed_cap = -1;
            ring_fault = 0;
            requested_boost = 0;
            applied_boost = 0.0f;
            force_requested=0;memset(&force_control,0,sizeof(force_control));
            memset(&maneuver,0,sizeof(maneuver));
            reset_exit_latch();
            abort_pending=0;diagnostics.release_reason=(uint8_t)released;
            if(detected<maneuver_event_seen)maneuver_event_seen=detected;
        } else if (valid && ((owned && detected && (vision_state == 3 || timed)) ||
                            (ring_active && (owned || phase == RING_FAULT))) && !abort_pending &&
                            (!timed || (runtime_ready && yaw_bias.ready && (operator_flags&RING_OPERATOR_RUN)))) {
            ring_active = 1;
            speed_cap = cap;
            ring_fault = fault;
            requested_boost = owned ? boost : 0;
            if(timed && !was_active && phase==RING_APPROACH && detected>maneuver_event_seen &&
               runtime_ready && yaw_bias.ready && (operator_flags&RING_OPERATOR_RUN) &&
               !ring_command_diag_cross_veto(packet.word[RC_FAULT])) {
                ring_maneuver_begin(&maneuver,now,-route,heading_deg,wheel_revs);
                reset_exit_latch();
                ring_entry_speed_begin(&entry_speed,now,
                    preceding_target_valid && now-preceding_target_ms<=RING_LINK_TIMEOUT_MS?
                    preceding_target:original_forward);
                maneuver_event_seen=detected;
            }
            if(timed && maneuver.stage && maneuver.stage<4u && maneuver.side!=-route){
                maneuver.stage=4;maneuver.reason=7;
                reset_exit_latch();exit_latch.blocked=1;
            }
            if(!timed && phase==RING_ENTER && !was_active)
                ring_force_begin(&force_control,now,-route,heading_deg,wheel_revs);
            force_requested=force;force_side=-route;
        }
        /* A restarted vision core can reset counters. Count each observed
         * forward event once; repeated snapshots never extend the beep. */
        if (valid) {
            int standby_event = !runtime_ready &&
                !(operator_flags & RING_OPERATOR_RUN) &&
                (phase == RING_NORMAL || phase == RING_COOLDOWN);
            int running_detection = runtime_ready &&
                (RING_TRACK_MODE ? ((operator_flags & RING_OPERATOR_RUN) != 0u) : ring_active);
            last_command_ms = now;
            diagnostics.command_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            diagnostics.phase = (int16_t)phase;
            diagnostics.turn_deg = (int16_t)turn_deg;
            diagnostics.cap = (int16_t)cap;
            diagnostics.requested_boost = (int16_t)boost;
            diagnostics.fault = (uint8_t)fault;
            diagnostics.candidate = (uint8_t)ring_command_diag_candidate(packet.word[RC_FAULT]);
            diagnostics.cross_veto = (uint8_t)ring_command_diag_cross_veto(packet.word[RC_FAULT]);
            diagnostics.committed = (uint8_t)ring_command_diag_committed(packet.word[RC_FAULT]);
            diagnostics.path_valid = (uint8_t)ring_command_diag_path_valid(packet.word[RC_FAULT]);
            diagnostics.stop = (uint8_t)ring_command_diag_stop(packet.word[RC_FAULT]);
            diagnostics.raw_state = (int16_t)raw_state;
            diagnostics.island_near = (int16_t)near;
            diagnostics.steer_error = (int16_t)steer_error;
            diagnostics.route = (int16_t)ring_command_diag_route(packet.word[RC_FAULT]);
            diagnostics.detected = detected;
            diagnostics.exited = exited;
            if (detected > detected_seen) {
                if(running_detection)ring_event_beep_fast_single(&event_beep);
                else if(standby_event)ring_event_beep_single(&event_beep);
            }
            if (runtime_ready && exited > exited_seen && was_active && phase == RING_COOLDOWN)
                ring_event_beep_pair(&event_beep);
            /* A normal preview event can be consumed while parked. During
             * running, wait for the actual steering handover so its first
             * confirmation snapshot cannot swallow the detection beep. */
            if (ring_active || running_detection || standby_event || !detected) detected_seen = detected;
            exited_seen = exited;
        }
    }
    if(RING_TRACK_MODE && ring_active &&
       (runtime_ms-last_command_ms>RING_LINK_TIMEOUT_MS || speed_cap==0 || ring_fault || diagnostics.stop)) {
        /* Production ring failures relinquish ownership. They never
         * change the caller's original speed/turn to zero. */
        diagnostics.release_reason=(uint8_t)(runtime_ms-last_command_ms>RING_LINK_TIMEOUT_MS?4:1);
        abort_pending=diagnostics.release_reason;ring_active=0;speed_cap=-1;ring_fault=0;
        reset_exit_latch();exit_latch.blocked=1;
        requested_boost=0;applied_boost=0;force_requested=0;
        memset(&force_control,0,sizeof(force_control));
    }
    if (runtime_ready && ring_active && (speed_cap >= 0 ||
        (RING_TRACK_MODE && RING_TIMED_ENTRY && speed_cap==-1 &&
         diagnostics.phase==RING_CIRCLE && maneuver.stage==3u))) {
        int fresh = now-last_command_ms<=RING_LINK_TIMEOUT_MS;
        if (ring_control_stop(speed_cap, 3, ring_fault, fresh)) {
            *turn_out = 0.0f;
            applied_boost = 0.0f;
        } else if (speed_cap >= 0 && vision_state == 3 && original_forward > 0.0f) {
            float step = requested_boost - applied_boost;
            if (step > RING_BOOST_STEP_RPM) step = RING_BOOST_STEP_RPM;
            if (step < -RING_BOOST_STEP_RPM) step = -RING_BOOST_STEP_RPM;
            applied_boost += step;
            *turn_out = ring_control_turn(*turn_out, applied_boost, 1);
        } else applied_boost = 0.0f;
        /* Optional D/E/G/H/J packets never arm or renew this control.
         * Only the matching fresh command can request the locked F turn. */
        if(maneuver.stage) {
            int eligible=fresh && runtime_ready && original_forward>0 && yaw_bias.ready &&
                (operator_flags&RING_OPERATOR_RUN);
            if(maneuver.stage==1u) {
                int motion_ok=measured_forward_valid && motion_sample_valid &&
                    now-motion_sample_ms<=150u && measured_forward>=RING_ENTRY_MIN_FORWARD_RPM &&
                    measured_forward<=RING_ENTRY_READY_RPM && entry_speed.valid &&
                    entry_speed.output<=RING_APPROACH_RPM+5.f;
                /* A confirmed scheduled entry survives temporarily cropped
                 * N/F pixels. Fresh paired command + speed + RUN remain
                 * mandatory; the current visual path only guides approach. */
                maneuver.armed=(unsigned)(force_requested && diagnostics.phase==RING_APPROACH && motion_ok);
                maneuver.reason=(unsigned)(force_requested?(motion_ok?1:2):0);
            }
            *turn_out=ring_maneuver_apply(&maneuver,*turn_out,now,heading_deg,wheel_revs,heading_rate,
                eligible,diagnostics.path_valid && !diagnostics.stop &&
                diagnostics.phase>=RING_ENTER && diagnostics.phase<=RING_RECOVER);
            if(exit_latch.valid && exit_latch.force_requested) {
                *turn_out=ring_exit_turn_apply(&exit_turn,*turn_out,now,heading_deg,wheel_revs,
                    heading_rate,eligible);
                if(exit_turn.stage==3u) {
                    unsigned reason=exit_turn.reason;
                    abort_exit_latch(reason==3u?3u:1u);
                    exit_turn.stage=3u;exit_turn.reason=reason;
                    *turn_out=original_turn;
                }
            }
            if(maneuver.stage==2u || (maneuver.stage==3u &&
               diagnostics.phase>=RING_ENTER && diagnostics.phase<=RING_RECOVER &&
               (diagnostics.path_valid || (exit_latch.valid && exit_latch.force_requested &&
                exit_turn.stage>0u && exit_turn.stage<3u)))) {
                /* Preserve the full existing turn at normal ring speed.
                 * A wheel-speed collapse must not leave full differential
                 * spinning the car. Forward target and balance PID remain
                 * responsible for recovering positive forward motion.
                 * An observed episode without a current ring path leaves
                 * the ordinary controller's turn untouched. The same bound
                 * still covers a finite gyro departure during path loss. */
                float limit=ring_maneuver_forward_limit(measured_forward,
                    measured_forward_valid && motion_sample_valid && now-motion_sample_ms<=150u);
                diagnostics.entry_turn_cap10=(int16_t)(limit*10.f);
                *turn_out=ring_maneuver_clamp(*turn_out,-limit,limit);
                maneuver.output=*turn_out;
                if(exit_latch.valid && exit_latch.force_requested)exit_turn.output=*turn_out;
                if(maneuver.stage==2u && limit<RING_FORCE_MAX_RPM) maneuver.reason=8u;
            }
        } else *turn_out=ring_force_apply(&force_control,*turn_out,now,heading_deg,wheel_revs,
            heading_rate,force_side,force_requested,
            fresh && !ring_control_stop(speed_cap,3,ring_fault,fresh) &&
            vision_state==3 && original_forward>0 && yaw_bias.ready &&
            (operator_flags&RING_OPERATOR_RUN),diagnostics.path_valid &&
            (diagnostics.phase==RING_ENTER || diagnostics.phase==RING_CIRCLE));
        /* Approach already decelerates. Entry/circle caps continue from
         * the delivered target without a one-frame100/90RPM step. */
        if(RING_TRACK_MODE && RING_TIMED_ENTRY && maneuver.stage) {
            /* Uncapped observation invalidates the old entry hold. A fresh
             * measured ring/exit reacquisition needs its own cap profile,
             * starting at the actually delivered continuous speed target. */
            if(!entry_speed.valid && speed_cap>0 && maneuver.stage==3u &&
               fresh && !ring_fault && !diagnostics.stop && runtime_ready &&
               (operator_flags&RING_OPERATOR_RUN))
                ring_entry_speed_begin(&entry_speed,now,
                    forward_profile.valid?forward_profile.output:original_forward);
            *forward_target=ring_entry_speed_apply(&entry_speed,original_forward,
                speed_cap,maneuver.stage,now,fresh && !ring_fault && !diagnostics.stop &&
                runtime_ready && (operator_flags&RING_OPERATOR_RUN));
        } else *forward_target=ring_control_speed(original_forward,speed_cap,3,ring_fault,fresh);
    }
    if(maneuver.stage && maneuver.stage<4u && (abort_pending || !runtime_ready ||
       !(operator_flags&RING_OPERATOR_RUN) || original_forward<=0.f)) {
        maneuver.stage=4;maneuver.reason=6;maneuver.target_rate=0;
    }
    if(!ring_active || !runtime_ready || !(operator_flags&RING_OPERATOR_RUN) || original_forward<=0.f)
        memset(&entry_speed,0,sizeof(entry_speed));
    if(RING_TRACK_MODE && RING_TIMED_ENTRY) {
        int run=runtime_ready && (operator_flags&RING_OPERATOR_RUN) && original_forward>0.f;
        *forward_target=ring_forward_profile_apply(&forward_profile,*forward_target,now,run,
            vision_state==0,measured_forward,
            measured_forward_valid && motion_sample_valid && now-motion_sample_ms<=150u);
        /* Timeout release must not bypass the last delivered speed. */
        fallback_forward=*forward_target;
    }
    if(runtime_ready && (operator_flags&RING_OPERATOR_RUN) && !ring_active && original_forward>0.f){
        preceding_target=*forward_target;preceding_target_ms=now;preceding_target_valid=1;
    }else if(!runtime_ready || !(operator_flags&RING_OPERATOR_RUN) || original_forward<=0.f)
        preceding_target_valid=0;
    diagnostics.held_target10=entry_speed.valid?(int16_t)(entry_speed.held*10.f):0;
    diagnostics.speed_goal10=(RING_TRACK_MODE && RING_TIMED_ENTRY)?
        (int16_t)(forward_profile.goal*10.f):(entry_speed.valid?(int16_t)(entry_speed.goal*10.f):0);
    diagnostics.motion_age_ms=motion_sample_valid?now-motion_sample_ms:UINT32_MAX;
    diagnostics.yaw_raw=yaw_bias.raw;
    diagnostics.yaw_bias_raw=(int16_t)lroundf(yaw_bias.bias);
    diagnostics.yaw_calibrated=(uint8_t)yaw_bias.ready;
    diagnostics.yaw_calibration_ms=yaw_bias.updated_ms;
    diagnostics.yaw_calibration_count=yaw_bias.accepted;
    diagnostics.yaw_window_state=(uint8_t)yaw_bias.window_state;
    diagnostics.maneuver_stage=(uint8_t)maneuver.stage;
    diagnostics.maneuver_turn10=(int16_t)fmaxf(-32768.f,fminf(32767.f,maneuver.progress*10.f));
    diagnostics.maneuver_remaining10=(int16_t)fmaxf(-32768.f,fminf(32767.f,
        maneuver.stage?(RING_EXIT_MIN_DEG-maneuver.progress)*10.f:0.f));
    diagnostics.maneuver_elapsed_ms=(uint16_t)fminf(65535.f,
        maneuver.stage?(float)(now-(maneuver.stage==1 || !maneuver.turn_ms?maneuver.started_ms:maneuver.turn_ms)):0.f);
    diagnostics.force_active=(uint8_t)force_control.active;
    diagnostics.force_reason=(uint8_t)force_control.reason;
    diagnostics.force_side=(int8_t)(force_control.begun?force_control.side:0);
    diagnostics.force_target10=force_control.active?(int16_t)(force_control.side*RING_FORCE_RATE_DPS*10):0;
    diagnostics.force_rate10=(int16_t)fmaxf(-32768.f,fminf(32767.f,heading_rate*10));
    if(maneuver.stage){
        diagnostics.force_active=(uint8_t)(ring_active && maneuver.stage==2u);
        diagnostics.force_reason=(uint8_t)maneuver.reason;diagnostics.force_side=(int8_t)maneuver.side;
        diagnostics.force_target10=(int16_t)(maneuver.target_rate*10.f);
    }
    if(exit_latch.valid && exit_latch.force_requested) {
        diagnostics.force_active=(uint8_t)(ring_active && exit_turn.stage==1u);
        diagnostics.force_reason=(uint8_t)exit_turn.reason;
        diagnostics.force_side=(int8_t)exit_turn.side;
        diagnostics.force_target10=(int16_t)(exit_turn.target_rate*10.f);
    }
    diagnostics.exit_force_stage=(uint8_t)exit_turn.stage;
    diagnostics.exit_force_reason=(uint8_t)exit_turn.reason;
    diagnostics.exit_force_side=(int8_t)exit_turn.side;
    diagnostics.exit_force_start10=(int16_t)fmaxf(-32768.f,fminf(32767.f,exit_turn.start_lap*10.f));
    diagnostics.exit_force_turn10=(int16_t)fmaxf(-32768.f,fminf(32767.f,exit_turn.progress*10.f));
    diagnostics.exit_force_remaining10=(int16_t)fmaxf(-32768.f,fminf(32767.f,exit_turn.remaining*10.f));
    diagnostics.exit_force_elapsed_ms=(uint16_t)fminf(65535.f,
        exit_turn.stage?(float)(now-exit_turn.started_ms):0.f);
    diagnostics.pair_ok=(uint8_t)(pair_packet_valid && now-pair_last_ms<=RING_LINK_TIMEOUT_MS);
    read_diagnostics(now);
    read_entry_diagnostics(now);
    read_geometry_diagnostics(now);
    read_topology_diagnostics(now);
    read_mouth_diagnostics(now);
    read_exit_diagnostics(now);
    diagnostics.control_dt_ms = now - diagnostics.now_ms;
    diagnostics.now_ms = now;
    diagnostics.command_age_ms = now - last_command_ms;
    diagnostics.wheel_mrev = (int32_t)(wheel_revs * 1000.0f);
    diagnostics.active = ring_active;
    diagnostics.run_requested=(uint8_t)((operator_flags&RING_OPERATOR_RUN)!=0u);
    diagnostics.standing=runtime_ready;
    diagnostics.output_state = (uint8_t)vision_state;
    diagnostics.applied_boost = (int16_t)applied_boost;
    diagnostics.original_target10 = (int16_t)(original_forward * 10.0f);
    diagnostics.final_target10 = (int16_t)(*forward_target * 10.0f);
    diagnostics.original_turn10 = (int16_t)(original_turn * 10.0f);
    diagnostics.final_turn10 = (int16_t)(*turn_out * 10.0f);

    packet = (ring_packet_t){{0}};
    packet.word[RL_MAGIC] = RING_LINK_MAGIC;
    packet.word[RL_EPOCH] = (int32_t)epoch;
    packet.word[RM_MS] = (int32_t)now;
    packet.word[RM_YAW_MDEG] = (int32_t)(heading_deg * 1000.0f);
    packet.word[RM_WHEEL_MREV] = (int32_t)(wheel_revs * 1000.0f);
    packet.word[RM_READY] = ring_motion_status_exit(runtime_ready | ((now >= 2000u && yaw_bias.ready) ? 2u : 0u),
        maneuver.stage,diagnostics.maneuver_turn10,exit_turn.stage);
    packet.word[RM_RESERVED] = ring_motion_operator_version(operator_flags|
        (abort_pending?RING_OPERATOR_ABORT_RING:0u),RING_PAIR_VERSION);
    ring_link_write(ring_motion_mailbox, &packet, &feedback_sequence);
}
