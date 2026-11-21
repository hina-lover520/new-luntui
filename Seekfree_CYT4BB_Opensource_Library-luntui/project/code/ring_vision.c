#include "ring_vision.h"
#include "ring_link.h"
#include <string.h>
#include <stdlib.h>
#ifndef RING_HOST_TEST
#include "zf_common_headfile.h"
#endif

static ring_motion_t motion;
static uint32_t command_sequence, diagnostic_sequence, epoch;
static uint32_t entry_diagnostic_sequence;
static uint32_t geometry_diagnostic_sequence, topology_diagnostic_sequence,mouth_diagnostic_sequence;
static uint32_t exit_diagnostic_sequence;
static uint8_t control_version;
static uint8_t updated, vision_handover;
static const uint8_t (*current_image)[RING_W];
static int original_track_state;
static uint32_t frame_clock_previous, frame_clock_hz, frame_coarse_previous;
static uint8_t frame_clock_valid;
static uint8_t binary_display, run_requested, preview_available;
static ring_result_t preview_result;
static uint32_t preview_events, preview_last_ms, preview_missing_since;
static uint8_t preview_confirm, preview_absent, preview_latched;
static int16_t preview_near, preview_look;
static int8_t preview_route;

/* Frozen exact binary inputs. Full slots are never overwritten. */
#define RING_CAPTURE_SLOTS 8u
typedef struct { int32_t meta[RING_IMAGE_HEADER_WORDS]; uint8_t bits[RING_IMAGE_BYTES]; uint8_t preview; } ring_capture_t;
static ring_capture_t captures[RING_CAPTURE_SLOTS];
static unsigned capture_head,capture_count,capture_offset,capture_pending_length,capture_preview_count;
static uint32_t capture_id,capture_dropped,capture_sequence,capture_pending_sequence,capture_detected;
static uint32_t capture_exit_window_ms;
static uint8_t capture_exit_window_seen;
static uint8_t capture_previous_stage,capture_previous_phase,capture_previous_branch,capture_previous_timed;
static uint16_t capture_event_mask,capture_previous_gate;
static void capture_reset(void)
{
    capture_head=capture_count=capture_offset=capture_pending_length=capture_preview_count=0;
    capture_id=capture_dropped=capture_sequence=capture_pending_sequence=capture_detected=0;
    capture_exit_window_ms=0;capture_exit_window_seen=0;
    capture_previous_stage=capture_previous_phase=capture_previous_branch=capture_previous_timed=0;
    capture_event_mask=capture_previous_gate=0;
}
static int capture_image(const uint8_t image[RING_H][RING_W],const ring_result_t *r,unsigned reason)
{
    ring_capture_t *slot;
    unsigned x,y,index=0; uint32_t flags;
    if(reason!=RING_IMAGE_EXIT_WINDOW && (capture_event_mask&(1u<<reason))) return 0;
    if(capture_count==RING_CAPTURE_SLOTS || (!motion.ready && capture_preview_count>=2u) ||
        (reason>=RING_IMAGE_FAR_CANDIDATE && reason<=RING_IMAGE_GATE_CHANGED &&
         capture_count>=RING_CAPTURE_SLOTS-2u)) { ++capture_dropped;return 0; }
    /* Only mark a snapshot delivered to the queue, not a dropped attempt. */
    capture_event_mask=(uint16_t)(capture_event_mask|(1u<<reason));
    slot=&captures[(capture_head+capture_count)%RING_CAPTURE_SLOTS];
    memset(slot,0,sizeof(*slot));
    for(y=0;y<RING_H;++y) for(x=0;x<RING_W;++x,++index)
        if(image[y][x]) slot->bits[index>>3]|=(uint8_t)(128u>>(index&7u));
    flags=(reason&15u)|(((unsigned)r->phase&15u)<<4)|
        (((unsigned)r->approach_stage&15u)<<8)|(((unsigned)r->path_source&15u)<<12)|
        ((unsigned)(uint8_t)r->route<<16)|((unsigned)(r->left_entry!=0)<<24)|
        ((unsigned)(r->lap_valid!=0)<<25)|((unsigned)(r->branch_ok!=0)<<26);
    slot->meta[RL_MAGIC]=RING_IMAGE_MAGIC;slot->meta[RL_EPOCH]=(int32_t)epoch;
    slot->meta[RI_ID]=(int32_t)++capture_id;slot->meta[RI_FRAME]=(int32_t)r->frame_sequence;
    slot->meta[RI_COMMAND]=(int32_t)(command_sequence+2u);
    slot->meta[RI_MS]=(int32_t)motion.now_ms;slot->meta[RI_FLAGS]=(int32_t)flags;
    slot->meta[RI_YAW10]=(int32_t)(motion.yaw_deg*10.0f);
    slot->meta[RI_LAP10]=(int32_t)(r->lap_deg*10.0f);
    slot->meta[RI_WHEEL_MREV]=(int32_t)(motion.wheel_revs*1000.0f);
    slot->meta[RI_CRC32]=(int32_t)ring_image_crc32(slot->bits,RING_IMAGE_BYTES);
    slot->meta[RI_DROPPED]=(int32_t)capture_dropped;
    slot->meta[RI_ROWS]=ring_diagnostic_points(r->near_row,r->far_row);
    slot->meta[RI_RESERVED]=(int32_t)((unsigned)control_version|(RING_PAIR_VERSION<<8));
    slot->preview=(uint8_t)!motion.ready;
    capture_preview_count+=slot->preview;
    ++capture_count;
    return 1;
}
static void capture_observe(const uint8_t image[RING_H][RING_W],const ring_result_t *r,uint32_t detected)
{
    if(detected!=capture_detected) {
        capture_event_mask=0;capture_image(image,r,RING_IMAGE_CONFIRMED);capture_detected=detected;
    }
    if(r->approach_stage>=RING_TANGENT_NEAR_PASSED && capture_previous_stage<RING_TANGENT_NEAR_PASSED)
        capture_image(image,r,RING_IMAGE_NEAR_PASSED);
    if(r->phase==RING_ENTER && capture_previous_phase!=RING_ENTER)
        capture_image(image,r,RING_IMAGE_ENTERED);
    /* The first ENTER image is the START of the75deg turn. Capture the
     * actual visual handover separately; old start/lost snapshots cannot
     * reconstruct which boundary the controller saw at its end. */
    if(motion.ready && capture_previous_timed && !r->timed_entry &&
       r->phase>=RING_ENTER && r->phase<=RING_CIRCLE)
        capture_image(image,r,RING_IMAGE_HANDOVER);
    if(motion.ready && r->phase==RING_EXIT && capture_previous_phase!=RING_EXIT)
        capture_image(image,r,RING_IMAGE_EXIT);
    if(motion.ready && r->release_reason && r->phase==RING_COOLDOWN &&
       capture_previous_phase>=RING_ENTER && capture_previous_phase<=RING_RECOVER)
        capture_image(image,r,RING_IMAGE_RELEASE);
    if(r->phase>=RING_ENTER && r->phase<=RING_CIRCLE && !r->branch_ok && capture_previous_branch)
        capture_image(image,r,RING_IMAGE_BRANCH_LOST);
    /* Scheduled entry no longer waits on F. Preserve queue capacity for
     * the turn/first lost branch rather than obsolete F-gate snapshots. */
    if(motion.ready && !r->timed_entry && r->phase==RING_APPROACH && r->mouth_far_x>=0 && r->mouth_far_y>=0) {
        unsigned had_far=capture_event_mask&(1u<<RING_IMAGE_FAR_CANDIDATE);
        capture_image(image,r,RING_IMAGE_FAR_CANDIDATE);
        if(r->mouth_progress>=4 && r->mouth_far_count)
            capture_image(image,r,RING_IMAGE_FAR_PROGRESS);
        if(had_far &&
            r->mouth_gate!=capture_previous_gate)
            capture_image(image,r,RING_IMAGE_GATE_CHANGED);
    }
    if(motion.ready && r->phase==RING_COOLDOWN && capture_previous_phase==RING_APPROACH)
        capture_image(image,r,RING_IMAGE_APPROACH_CANCEL);
    /* Observe rejected exits too: candidate-only snapshots hide the exact
     * geometry that prevented the exit. This low-priority capture keeps
     * two slots for EXIT/release and never changes a frozen/pending image.
     * Upload remains idle-only through the existing bounded mechanism. */
    if(motion.ready && r->phase==RING_CIRCLE && r->committed && r->lap_valid &&
       !r->timed_entry && r->visual_dt_ms!=65535u &&
       r->lap_deg>=385.f && r->lap_deg<=435.f &&
       (!capture_exit_window_seen || motion.now_ms-capture_exit_window_ms>=1500u) &&
       capture_count<RING_CAPTURE_SLOTS-2u &&
       capture_image(image,r,RING_IMAGE_EXIT_WINDOW)) {
        capture_exit_window_ms=motion.now_ms;capture_exit_window_seen=1;
    }
    capture_previous_gate=r->mouth_gate;
    capture_previous_stage=r->approach_stage;capture_previous_phase=(uint8_t)r->phase;
    capture_previous_branch=r->branch_ok;capture_previous_timed=r->timed_entry;
}
static void capture_service(void)
{
    ring_image_packet_t chunk;ring_packet_t ack;ring_capture_t *slot;unsigned length;
    int idle=!motion.ready || ring_result.phase==RING_NORMAL ||
        ring_result.phase==RING_COOLDOWN || ring_result.phase==RING_FAULT;
    if(!capture_count) return;
    slot=&captures[capture_head];
    if(capture_pending_sequence) {
        if(!ring_link_read_image_ack(&ack) || ack.word[RL_EPOCH]!=slot->meta[RL_EPOCH] ||
            (uint32_t)ack.word[RA_IMAGE_SEQUENCE]!=capture_pending_sequence ||
            ack.word[RA_ID]!=slot->meta[RI_ID] ||
            (uint32_t)ack.word[RA_OFFSET_LENGTH]!=
                ((uint32_t)capture_offset|((uint32_t)capture_pending_length<<16))) return;
        capture_offset+=capture_pending_length;capture_pending_sequence=0;
        if(capture_offset==RING_IMAGE_BYTES) {
            capture_preview_count-=slot->preview;
            capture_head=(capture_head+1u)%RING_CAPTURE_SLOTS;--capture_count;capture_offset=0;
            if(!capture_count) return;
            slot=&captures[capture_head];
        }
    }
    if(!idle) return;
    memset(&chunk,0,sizeof(chunk));memcpy(chunk.word,slot->meta,sizeof(chunk.word));
    chunk.word[RI_DROPPED]=(int32_t)capture_dropped;
    length=RING_IMAGE_BYTES-capture_offset;
    if(length>RING_IMAGE_CHUNK_BYTES) length=RING_IMAGE_CHUNK_BYTES;
    chunk.word[RI_OFFSET_LENGTH]=(int32_t)((uint32_t)capture_offset|((uint32_t)length<<16));
    memcpy(chunk.data,slot->bits+capture_offset,length);
    ring_link_write_image(&chunk,&capture_sequence);capture_pending_sequence=capture_sequence;
    capture_pending_length=length;
}

/* Recognition feedback is independent of vehicle motion. Only repeated
 * fresh, associated, measured branches can trigger a standby notification.
 * A short detection dropout cannot re-arm the same island's beep. */
static void preview_observe(void)
{
    int detected = preview_result.branch_ok && preview_result.path_source == 1 &&
        preview_result.candidate && preview_result.classification == 2 && !preview_result.cross_veto;
    if (detected) {
        int continuous = preview_confirm && preview_result.visual_dt_ms != 65535u &&
            motion.now_ms - preview_last_ms <= RING_OBSERVE_MS &&
            preview_route == preview_result.route &&
            abs(preview_result.near_x - preview_near) <= RING_CORRIDOR_TOL &&
            abs(preview_result.look_x - preview_look) <= RING_CORRIDOR_TOL;
        preview_confirm = (uint8_t)(continuous ? preview_confirm + 1u : 1u);
        if (preview_confirm > RING_CONFIRM_FRAMES) preview_confirm = RING_CONFIRM_FRAMES;
        preview_absent = 0;
        preview_last_ms = motion.now_ms;
        preview_near = preview_result.near_x;
        preview_look = preview_result.look_x;
        preview_route = preview_result.route;
        if (preview_confirm >= RING_CONFIRM_FRAMES && !preview_latched) {
            ++preview_events;
            preview_latched = 1;
        }
    } else {
        preview_confirm = 0;
        if (!preview_absent) preview_missing_since = motion.now_ms;
        if (preview_absent < 255u) ++preview_absent;
        if (preview_absent >= RING_OBSERVE_FRAMES &&
            motion.now_ms - preview_missing_since >= RING_PATH_TIMEOUT_MS)
            preview_latched = 0;
    }
    preview_result.confirm_frames = preview_confirm;
}

uint8_t ring_vision_binary_display(void) { return binary_display; }
uint8_t ring_vision_run_requested(void) { return run_requested; }
uint8_t ring_vision_preview(void) { return preview_available; }
const ring_result_t *ring_vision_display_result(void)
{ return preview_available ? &preview_result : &ring_result; }

/* CM7_0-only timing. Never reset CYCCNT, take a timer, or add an ISR.
 * 65535 means unavailable; control-loop time is not visual-frame time. */
static void frame_clock_init(void)
{
    frame_clock_valid = 0;
    frame_clock_previous = frame_clock_hz = frame_coarse_previous = 0;
#ifndef RING_HOST_TEST
    if (!(DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        __DSB();
        __ISB();
    }
#endif
}

static uint16_t frame_interval(void)
{
    uint32_t now, hz, delta, coarse_delta;
    uint16_t result = 65535u;
#ifdef RING_HOST_TEST
    now = motion.now_ms;
    hz = 1000u;
#else
    hz = SystemCoreClock;
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) || hz < 1000u) {
        frame_clock_valid = 0;
        return result;
    }
    now = DWT->CYCCNT;
#endif
    delta = now - frame_clock_previous;
    coarse_delta = motion.now_ms - frame_coarse_previous;
    if (frame_clock_valid && hz == frame_clock_hz) {
#ifdef RING_HOST_TEST
        if (delta < 65535u) result = (uint16_t)delta;
#else
        /* The coarse clock is only a multi-wrap guard, never the vdt source. */
        if (delta != 0u && coarse_delta < UINT32_MAX / (hz / 1000u)) {
            uint32_t milliseconds = delta / (hz / 1000u);
            if (milliseconds < 65535u) result = (uint16_t)milliseconds;
        }
#endif
    }
#ifdef RING_HOST_TEST
    (void)coarse_delta;
#endif
    frame_clock_previous = now;
    frame_clock_hz = hz;
    frame_coarse_previous = motion.now_ms;
    frame_clock_valid = 1;
    return result;
}

static void publish(void)
{
    ring_packet_t packet = {{0}};
    float degrees = ring_result.turn_deg;
    if (degrees > 32767.0f) degrees = 32767.0f;
    if (degrees < -32768.0f) degrees = -32768.0f;
    packet.word[RL_MAGIC] = RING_LINK_MAGIC;
    packet.word[RL_EPOCH] = (int32_t)epoch;
    packet.word[RC_CAP] = ring_command_cap_turn(ring_speed_limit(),
        (int)degrees);
    packet.word[RC_PHASE] = ring_command_versioned_phase(ring_result.phase,ring_turn_boost(),
        RING_TRACK_MODE?RING_PAIR_VERSION:0u);
    packet.word[RC_DETECTED] = (int32_t)(preview_events + ring_result.detected_events);
    packet.word[RC_EXITED] = (int32_t)ring_result.exited_events;
    packet.word[RC_FAULT] = ring_command_diagnostics(ring_result.fault,
        ring_result.candidate, ring_result.cross_veto, ring_result.committed,
        ring_result.path_valid, ring_result.stop, original_track_state,
        ring_result.steer_error, ring_result.island_near, ring_result.route);
    if(RING_TRACK_MODE && (ring_result.timed_entry || (ring_result.exit_turn_request &&
       (ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER))))
        packet.word[RC_FAULT]=(int32_t)((uint32_t)packet.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
    if(RING_TRACK_MODE && ring_result.force_entry)
        packet.word[RC_FAULT]=(int32_t)((uint32_t)packet.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
    if(RING_TRACK_MODE && ring_result.release_reason)
        packet.word[RC_FAULT]=(int32_t)((uint32_t)packet.word[RC_FAULT]|
            ((uint32_t)ring_result.release_reason<<RING_COMMAND_RELEASE_SHIFT));
    ring_link_write(ring_command_mailbox, &packet, &command_sequence);
    packet = (ring_packet_t){{0}};
    packet.word[RL_MAGIC] = RING_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH] = (int32_t)epoch;
    packet.word[RD_COMMAND_SEQUENCE] = (int32_t)command_sequence;
    packet.word[RD_FRAME] = (int32_t)ring_result.frame_sequence;
    packet.word[RD_TIMING_REJECT] = ring_diagnostic_timing(ring_result.visual_dt_ms, ring_result.reject);
    packet.word[RD_FLAGS] = ring_diagnostic_flags(ring_result.candidate_detected,
        ring_result.candidate, ring_result.score, ring_result.confirm_frames,
        ring_result.entry_side, ring_result.path_source, ring_result.branch_ok,
        ring_result.classification);
    packet.word[RD_POINTS] = ring_diagnostic_points(ring_result.near_x, ring_result.look_x);
    ring_link_write(ring_diagnostic_mailbox, &packet, &diagnostic_sequence);
    packet = (ring_packet_t){{0}};
    packet.word[RL_MAGIC] = RING_ENTRY_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH] = (int32_t)epoch;
    packet.word[RE_COMMAND_SEQUENCE] = (int32_t)command_sequence;
    packet.word[RE_FRAME] = (int32_t)ring_result.frame_sequence;
    packet.word[RE_FLAGS] = ring_entry_diagnostic_flags(ring_result.entry_flags,
        ring_result.hold_reject, ring_result.decision);
    packet.word[RE_AGE_LOST] = ring_entry_diagnostic_age(ring_result.path_age_ms,
        ring_result.lost_frames);
    packet.word[RE_ANGLES] = ring_entry_diagnostic_angles(ring_result.hold_turn10,
        ring_result.entry_turn10);
    ring_link_write(ring_entry_diagnostic_mailbox, &packet, &entry_diagnostic_sequence);
    packet = (ring_packet_t){{0}};
    packet.word[RL_MAGIC] = RING_GEOMETRY_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH] = (int32_t)epoch;
    packet.word[RG_COMMAND_SEQUENCE] = (int32_t)command_sequence;
    packet.word[RG_FRAME] = (int32_t)ring_result.frame_sequence;
    packet.word[RG_GEOMETRY] = ring_geometry_diagnostic_flags(ring_result.branch_mode,
        ring_result.inner_rows, ring_result.associated_rows, ring_result.target_support);
    packet.word[RG_STRONG_AGE] = (int32_t)ring_result.strong_age_ms;
    packet.word[RG_GUIDE] = ring_geometry_diagnostic_guide(ring_result.guide_x,
        ring_result.guide_y);
    ring_link_write(ring_geometry_diagnostic_mailbox, &packet, &geometry_diagnostic_sequence);
    packet=(ring_packet_t){{0}};
    packet.word[RL_MAGIC]=RING_TOPOLOGY_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH]=(int32_t)epoch;
    packet.word[RH_COMMAND_SEQUENCE]=(int32_t)command_sequence;
    packet.word[RH_FRAME]=(int32_t)ring_result.frame_sequence;
    packet.word[RH_FLAGS]=ring_topology_flags(ring_result.ring_side,
        ring_result.approach_stage,ring_result.left_entry,ring_result.lap_valid,
        ring_result.near_row,ring_result.far_row);
    packet.word[RH_FLAGS]=(int32_t)((uint32_t)packet.word[RH_FLAGS]|((uint32_t)ring_result.mouth_source<<23));
    packet.word[RH_ORIGIN_MDEG]=(int32_t)(ring_result.lap_origin_yaw*1000.0f);
    packet.word[RH_LAP_MDEG]=(int32_t)(ring_result.lap_deg*1000.0f);
    ring_link_write(ring_topology_diagnostic_mailbox,&packet,&topology_diagnostic_sequence);
    packet=(ring_packet_t){{0}};
    packet.word[RL_MAGIC]=RING_MOUTH_DIAGNOSTIC_MAGIC;packet.word[RL_EPOCH]=(int32_t)epoch;
    packet.word[RJ_COMMAND_SEQUENCE]=(int32_t)command_sequence;
    packet.word[RJ_FRAME]=(int32_t)ring_result.frame_sequence;
    packet.word[RJ_POINTS]=ring_mouth_points(ring_result.mouth_near_x,ring_result.mouth_near_y,
        ring_result.mouth_far_x,ring_result.mouth_far_y);
    packet.word[RJ_GATES]=ring_mouth_gates(ring_result.mouth_gate,ring_result.mouth_ref,
        ring_result.mouth_near_count,ring_result.mouth_far_count,ring_result.mouth_gap);
    packet.word[RJ_GATES]=ring_mouth_tracking(packet.word[RJ_GATES],
        ring_result.mouth_near_missed,ring_result.mouth_pass_reason);
    packet.word[RJ_PROGRESS_PAIR]=ring_mouth_progress_pair(ring_result.mouth_first_y,
        ring_result.mouth_progress,control_version,RING_PAIR_VERSION);
    ring_link_write(ring_mouth_diagnostic_mailbox,&packet,&mouth_diagnostic_sequence);
    /* L publishes only scalar results already computed by ring_update. No
     * image scan or control decision is performed in the telemetry path. */
    packet=(ring_packet_t){{0}};
    packet.word[RL_MAGIC]=RING_EXIT_DIAGNOSTIC_MAGIC;packet.word[RL_EPOCH]=(int32_t)epoch;
    packet.word[RX_COMMAND_SEQUENCE]=(int32_t)command_sequence;
    packet.word[RX_FRAME]=(int32_t)ring_result.frame_sequence;
    packet.word[RX_STATUS]=ring_exit_diagnostic_status(ring_result.exit_reject,
        ring_result.exit_candidate,ring_result.exit_latched,ring_result.exit_confirmed);
    {
        float lap10=ring_result.lap_deg*10.0f;
        if(lap10>32767.0f) lap10=32767.0f;
        if(lap10< -32768.0f) lap10= -32768.0f;
        packet.word[RX_COUNTS_LAP]=ring_exit_diagnostic_counts(
            ring_result.exit_candidate_frames,ring_result.exit_confirm_frames,(int)lap10);
    }
    packet.word[RX_FORWARD_MREV]=ring_result.exit_forward_mrev;
    ring_link_write(ring_exit_diagnostic_mailbox,&packet,&exit_diagnostic_sequence);
}

void ring_vision_init(void)
{
    ring_init();
    frame_clock_init();
    binary_display = run_requested = preview_available = control_version = 0;
    preview_events = preview_last_ms = preview_missing_since = 0;
    preview_confirm = preview_absent = preview_latched = 0;
    preview_near = preview_look = -1;
    preview_route = 0;
    memset(&preview_result, 0, sizeof(preview_result));
    memset(&motion, 0, sizeof(motion));
    command_sequence = diagnostic_sequence = epoch = 0;
    entry_diagnostic_sequence = 0;
    geometry_diagnostic_sequence = topology_diagnostic_sequence = mouth_diagnostic_sequence = 0;
    exit_diagnostic_sequence = 0;
    capture_reset();
    updated = vision_handover = 0;
    current_image = 0;
    original_track_state = 1;
    publish();
}

static void update(const uint8_t image[RING_H][RING_W])
{
    ring_packet_t packet;
    if (ring_link_read(ring_motion_mailbox, &packet)) {
        uint32_t incoming_epoch = (uint32_t)packet.word[RL_EPOCH];
        if (incoming_epoch != epoch) {
            ring_init();
            vision_handover = 0;
            frame_clock_valid = 0;
            epoch = incoming_epoch;
            preview_events = preview_last_ms = preview_missing_since = 0;
            preview_confirm = preview_absent = preview_latched = 0;
            command_sequence = diagnostic_sequence = 0;
            entry_diagnostic_sequence = 0;
            geometry_diagnostic_sequence = topology_diagnostic_sequence = mouth_diagnostic_sequence = 0;
            capture_reset();
        }
        motion.now_ms = (uint32_t)packet.word[RM_MS];
        motion.yaw_deg = packet.word[RM_YAW_MDEG] * 0.001f;
        motion.wheel_revs = packet.word[RM_WHEEL_MREV] * 0.001f;
        control_version=(uint8_t)ring_motion_version(packet.word[RM_RESERVED]);
        motion.ready = (uint8_t)(epoch != 0 && (((uint32_t)packet.word[RM_READY]&3u)==3u && ring_motion_status_valid(packet.word[RM_READY])) &&
            (control_version==RING_PAIR_VERSION || (!RING_TRACK_MODE && control_version==0u)));
        if(motion.ready) {
            ring_observe_maneuver_feedback(ring_motion_stage(packet.word[RM_READY]),
                (int16_t)ring_motion_progress(packet.word[RM_READY]));
            ring_observe_exit_turn_feedback(ring_motion_exit_stage(packet.word[RM_READY]));
        }
        if (((uint32_t)packet.word[RM_RESERVED] & 0xffff0000u) == RING_OPERATOR_VALID) {
            binary_display = (uint8_t)(((uint32_t)packet.word[RM_RESERVED] & RING_OPERATOR_BINARY) != 0u);
            run_requested = (uint8_t)(((uint32_t)packet.word[RM_RESERVED] & RING_OPERATOR_RUN) != 0u);
        }
    }
    if (!motion.ready) {
        /* Stationary inspection: one independent observation per image.
         * It can expose a tentative path and separately confirm a notification,
         * but cannot publish a handover or advance the driving state machine. */
        ring_motion_t preview_motion = motion;
        preview_motion.ready = 1;
        ring_reset_episode();
        ring_observe_track(original_track_state);
        ring_observe_frame_interval(frame_interval());
        ring_update(image, &preview_motion);
        preview_result = ring_result;
        preview_observe();
        capture_observe(image,&preview_result,preview_events);
        preview_available = 1;
        vision_handover = 0;
        ring_reset_episode();
        publish();
        capture_service();
        return;
    }
    if (preview_available) {
        ring_reset_episode();
        capture_event_mask=capture_previous_gate=0;
        capture_previous_phase=capture_previous_stage=capture_previous_branch=0;
        preview_available = 0;
    }
    /* A control-side stale command releases this same episode. It cannot
     * resume from a frozen ENTER packet when communication returns. */
    if(RING_TRACK_MODE && ring_link_read(ring_motion_mailbox,&packet) &&
       (uint32_t)packet.word[RL_EPOCH]==epoch &&
       ((uint32_t)packet.word[RM_RESERVED]&0xffff0000u)==RING_OPERATOR_VALID &&
       ((uint32_t)packet.word[RM_READY]&3u)==3u &&
       ring_motion_version(packet.word[RM_RESERVED])==RING_PAIR_VERSION &&
       ((uint32_t)packet.word[RM_RESERVED]&RING_OPERATOR_ABORT_RING) &&
       ring_result.phase>=RING_APPROACH && ring_result.phase<=RING_FAULT)
        ring_release_control(4,&motion);
    ring_observe_track(original_track_state);
    ring_observe_frame_interval(frame_interval());
    ring_update(image, &motion);
    capture_observe(image,&ring_result,preview_events+ring_result.detected_events);
    publish();
    capture_service();
}

void ring_vision_before_frame(const uint8_t image[RING_H][RING_W])
{
    current_image = image;
    updated = 0;
}

void ring_vision_select(int *steer_error, int *track_state)
{
    original_track_state = *track_state;
    ring_observe_track(original_track_state);
    /* Existing main hooks remain untouched. Process once here so a newly
     * confirmed approach and its error use the SAME original cross frame.
     * Unconfirmed ordinary/cross outputs still pass through unchanged. */
    if (RING_ENABLE && current_image && (RING_TRACK_MODE || ring_blocks_cross())) {
        update(current_image);
        updated = 1;
    }
    if (ring_result.phase == RING_NORMAL || ring_result.phase == RING_COOLDOWN)
        vision_handover = 0;
    if (ring_owns_path()) {
        vision_handover = 1;
        *steer_error = ring_blend_error((int16_t)*steer_error);
        *track_state = ring_result.stop ? 0 : 3;
    } else if (ring_result.phase == RING_FAULT && (!RING_TRACK_MODE || vision_handover)) {
        *steer_error = 0;
        *track_state = 0;
    }
}

void ring_vision_after_frame(const uint8_t image[RING_H][RING_W])
{
    if (RING_ENABLE && !updated) update(image);
    current_image = 0;
}
