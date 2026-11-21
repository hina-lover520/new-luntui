#ifndef CODE_RING_H_
#define CODE_RING_H_

#include <stdint.h>

/* Existing main supplies binarize(ipm_image), white != 0. These are the
 * displayed IPM pixel coordinates, not raw-camera pixels or metres. */
#define RING_W 188
#define RING_H 120
#ifndef RING_ENABLE
#define RING_ENABLE 1
#endif
/* Production track: a side ring sharing a straight lane. Mode 0 retains
 * the historical head-on-island fixtures for independent regression. */
#ifndef RING_TRACK_MODE
#define RING_TRACK_MODE 1
#endif
/* 0 = automatic; -1 = pass LEFT of island; +1 = pass RIGHT of island. */
#ifndef RING_ROUTE
#define RING_ROUTE 0
#endif
#define RING_CONFIRM_FRAMES       4
#define RING_SPLIT_ROWS           8
#define RING_BRANCH_ROWS          20
#define RING_MIN_RUN              7
#define RING_MIN_ISLAND_WIDTH     10
#define RING_MAX_ISLAND_WIDTH     115
#define RING_ENTRY_ROW            74
#define RING_DEFAULT_LANE_WIDTH   60
#define RING_MIN_LANE_WIDTH       18
#define RING_MAX_LANE_WIDTH       110
#define RING_LOOK_ROW             78
#define RING_NEAR_ROW             108
#define RING_MAX_ROW_STEP         18
#define RING_EXIT_MIN_DEG         360.0f
#define RING_EXIT_MIN_REVS        0.6f
#define RING_EXIT_CONFIRM_FRAMES  4
#define RING_RECOVER_FRAMES       8
#define RING_COOLDOWN_MS          1200u
#define RING_COOLDOWN_REVS        0.5f
/* Minimum interval between confirmed mode entries, measured from the
 * FIRST entry, not exit. Does not interrupt an already active ring. */
#define RING_REENTRY_BLOCK_MS     15000u
#define RING_APPROACH_TIMEOUT_MS  6000u
#define RING_TOTAL_TIMEOUT_MS     30000u
#define RING_EXIT_TIMEOUT_MS      4000u
#define RING_PATH_HOLD_MS         250u
#define RING_PATH_TIMEOUT_MS      500u
/* All bounds run on NEW vision observations, never control-side polls. */
#define RING_OBSERVE_MS           300u
#define RING_OBSERVE_FRAMES       8u
#define RING_HOLD_FRAMES          5u
#define RING_HOLD_REVS            0.4f
#define RING_HOLD_YAW_DEG         12.0f
#define RING_BRANCH_CONFIRM       5u
#define RING_CONNECT_GAP          2
#define RING_CORRIDOR_TOL         25
/* Ring-only settings. No original track/balance/servo parameter is changed. */
#ifndef RING_YAW_SIGN
/* Raw +Z follows positive wheel differential/image-right turning. A car
 * passing RIGHT of an island (route+1, island on its left) laps with -Z.
 * This maps only ring progress; the raw IMU/balance integration is intact. */
#define RING_YAW_SIGN             -1.0f
#endif
#define RING_COMMIT_DEG           20.0f
#define RING_CANCEL_MAX_DEG       8.0f
#define RING_CANCEL_FRAMES        3
#define RING_ENTRY_BOOST_RPM      45
#define RING_CIRCLE_BOOST_RPM     24
#define RING_BOOST_STEP_RPM       8.0f
#define RING_TURN_LIMIT_RPM       120.0f
#define RING_APPROACH_RPM         100
#define RING_CIRCLE_RPM           90
#define RING_EXIT_RPM             90
/* Confirmed ENTER only. These do not relax ordinary branch-hold bounds. */
#define RING_ASSIST_MIN_ERROR     26
#define RING_ASSIST_TARGET_ERROR  12
#define RING_ASSIST_CONFIRM       2
#define RING_ASSIST_TOTAL_MS      600u
#define RING_ASSIST_ENTRY_YAW_DEG 45.0f
#define RING_ASSIST_ENTRY_REVS    0.8f
#define RING_ASSIST_HOLD_MS       200u
#define RING_ASSIST_HOLD_YAW_DEG  35.0f
#define RING_ASSIST_HOLD_REVS     0.45f
/* Recognition -> corridor/deceleration -> scheduled75deg -> VISUAL ring follow.
 * No waits and no ordinary tracking/controller parameter changes. */
#ifndef RING_TIMED_ENTRY
#define RING_TIMED_ENTRY          1
#endif
#define RING_MANEUVER_DELAY_MS    800u         //直行延时
#define RING_MANEUVER_TURN_DEG    75.0f
#define RING_MANEUVER_TURN_MS     2000u
#define RING_MANEUVER_TURN_REVS   3.5f
/*360deg is only a measured EXIT gate, never a steering setpoint.
 * After a visually confirmed lap, aligned ordinary road may independently
 * confirm the exit when the pixel N/F tracker has missed a shoulder. */
#define RING_LAP_EXIT_HEADING_TOL  15.0f
#define RING_VISUAL_EXIT_FRAMES    12u
/* Passive observation only below360; first ownership remains360..375. */
#define RING_EXIT_OBSERVE_DEG      340.0f
/* Net progress since the start of the entry turn, not absolute yaw sum.
 * Once reached, the exit may correct back to the incoming straight. */
#ifndef RING_EXIT_NET_MIN_DEG
#define RING_EXIT_NET_MIN_DEG      400.0f
#endif
#define RING_EXIT_SEARCH_END_DEG   430.0f
#define RING_EXIT_CANDIDATE_FRAMES 3u
#define RING_EXIT_CANDIDATE_MS     90u
#define RING_EXIT_CONTINUE_YAW_DEG 30.0f
#define RING_EXIT_CLEAR_REVS       0.15f
/* Finish the final measured arc toward the ORIGINAL outgoing heading.
 * Never append70 degrees after a complete lap. Ordinary follow still
 * requires measured360 and a currently visible outgoing road. */
#define RING_EXIT_FORCE_DEG          70.0f
#define RING_EXIT_FORCE_START_DEG    (RING_EXIT_MIN_DEG-RING_EXIT_FORCE_DEG)
#define RING_EXIT_FORCE_ALIGN_DEG    8.0f
#define RING_EXIT_FORCE_TIMEOUT_MS   2000u
#define RING_EXIT_FORCE_REVS         3.5f
#define RING_EXIT_FORCE_BACKTRACK_DEG 2.0f
/* Confirmed entry: delay and measured speed must both be ready before turning. */
#define RING_ENTRY_SPEED_SLEW_RPM_S 40.0f
#define RING_FORWARD_RISE_RPM_S    35.0f
#define RING_FORWARD_FALL_RPM_S    80.0f
#define RING_FORWARD_OVERSPEED_RPM 12.0f
#define RING_FORWARD_TRIM_MAX_RPM  25.0f
#define RING_ENTRY_READY_RPM       (RING_APPROACH_RPM + 15.0f)
#define RING_ENTRY_MIN_FORWARD_RPM (RING_CIRCLE_RPM * 0.5f)
#define RING_ENTRY_HANDOVER_MS     300u
#define RING_TANGENT_CONFIRM      3
#define RING_TANGENT_FAR_MIN_ROW  74
#define RING_TANGENT_FAR_MAX_ROW  100
/* Confirmed timed entry or a current F portal may request finite yaw-rate control. */
#define RING_FORCE_RATE_DPS       100.0f
#define RING_FORCE_FEED_RPM       65.0f              //强转基础差速
#define RING_FORCE_RATE_KP        0.35f
#define RING_FORCE_MAX_RPM        90.0f
#define RING_FORCE_SLEW_RPM_S     600.0f
#define RING_FORCE_RELEASE_MS     120u
/* Independent F entry budget; do not extend stale-path/legacy assists. */
#define RING_FORCE_TOTAL_MS       1500u
#define RING_FORCE_ENTRY_YAW_DEG  100.0f
#define RING_FORCE_ENTRY_REVS     2.5f

typedef enum {
    RING_NORMAL = 0, RING_APPROACH, RING_ENTER, RING_CIRCLE,
    RING_EXIT, RING_RECOVER, RING_COOLDOWN, RING_FAULT
} ring_phase_t;

typedef struct {
    uint32_t now_ms;
    float yaw_deg;             /* continuous yaw, integrated with actual 1ms period */
    float wheel_revs;          /* signed forward wheel displacement */
    uint8_t ready;             /* standing up completed */
} ring_motion_t;

typedef struct {
    ring_phase_t phase;
    int8_t route;
    uint8_t candidate, path_valid, stop, fault;
    uint8_t committed, cross_veto;
    int16_t steer_error, lane_width, island_near;
    int16_t mid[RING_H];       /* -1 = no reliable point; never edits input pixels */
    float turn_deg;
    uint32_t detected_events, exited_events;
    /* Telemetry only: 0 unknown, 1 through-cross, 2 ring, 3 conflicting. */
    uint32_t frame_sequence;
    uint16_t visual_dt_ms, reject;
    uint8_t candidate_detected, score, confirm_frames;
    uint8_t branch_ok, path_source, classification;
    int8_t entry_side;
    int16_t near_x, look_x;
    /* Entry decisions use measured branch evolution, not yaw alone. */
    uint16_t entry_flags, hold_reject;
    uint8_t decision, lost_frames;
    uint16_t path_age_ms;
    int16_t hold_turn10, entry_turn10, entry_travel_mrev;
    /* Current path evidence; partial pixels cannot renew a strong anchor. */
    uint8_t branch_mode, inner_rows, associated_rows, target_support, guide_y;
    int16_t guide_x;
    uint16_t strong_age_ms;
    /* Side-ring topology. The lap origin is the real far-mouth entry,
     * never recognition or the later CIRCLE confirmation. */
    int8_t ring_side;
    uint8_t approach_stage, left_entry, lap_valid;
    int16_t near_row, far_row;
    float lap_origin_yaw, lap_deg;
    /* v13 current measurements, not retained H historical maxima. */
    int16_t mouth_near_x, mouth_near_y, mouth_far_x, mouth_far_y;
    int16_t mouth_first_y, mouth_progress;
    uint16_t mouth_gate;
    uint8_t mouth_ref, mouth_near_count, mouth_far_count, mouth_gap;
    uint8_t mouth_near_missed, mouth_pass_reason, mouth_source;
    uint8_t timed_entry;       /* current recognition-driven motion request */
    uint8_t force_entry;       /* current control request; never lap/branch evidence */
    uint8_t release_reason;    /* 1 lost branch,2 total deadline,3 exit deadline,4 stale command */
    /* Independent exit diagnostics; no ordinary tracking input. */
    uint16_t exit_reject;
    uint8_t exit_candidate,exit_latched,exit_confirmed;
    uint8_t exit_turn_request; /* explicit command, not optional telemetry */
    uint8_t circle_observe_only; /* confirmed lap retained; ordinary steering owns road */
    uint8_t exit_candidate_frames,exit_confirm_frames;
    int32_t exit_forward_mrev;
} ring_result_t;

enum {
    RING_EXIT_REJ_LAP=1u,RING_EXIT_REJ_COMMIT=2u,
    RING_EXIT_REJ_GEOMETRY=4u,RING_EXIT_REJ_CANDIDATE=8u,
    RING_EXIT_REJ_MOTION=16u,RING_EXIT_REJ_LOCK_LOST=32u,
    RING_EXIT_REJ_HEADING=64u
};

enum {
    RING_MOUTH_REF=1u, RING_MOUTH_NEAR=2u, RING_MOUTH_NEAR_PROGRESS=4u,
    RING_MOUTH_NEAR_PASSED=8u, RING_MOUTH_FAR=16u,
    RING_MOUTH_ASSOC=32u, RING_MOUTH_VISUAL_PROGRESS=64u,
    RING_MOUTH_FAR_WINDOW=128u, RING_MOUTH_CLOCK=256u,
    RING_MOUTH_MOTION=512u, RING_MOUTH_CROSS=1024u,
    RING_MOUTH_DISTINCT=2048u, RING_MOUTH_CORRIDOR=4096u,
    RING_MOUTH_READY=8192u, RING_MOUTH_ENTRY_PATH=16384u
};

enum {
    RING_TANGENT_NONE=0, RING_TANGENT_NEAR_SEEN=1,
    RING_TANGENT_NEAR_PASSED=2, RING_TANGENT_FAR_READY=3,
    RING_TANGENT_LEFT=4, RING_TANGENT_SHARED=5, RING_TANGENT_EXIT=6
};

enum {
    RING_BRANCH_NONE=0, RING_BRANCH_STRONG=1, RING_BRANCH_PARTIAL=2,
    RING_BRANCH_HOLD=3, RING_BRANCH_EXIT=4, RING_BRANCH_ASSIST=5,
    RING_BRANCH_JUNCTION=6
};

enum {
    RING_ENTRY_MEASURED=1u, RING_ENTRY_HISTORY=2u,
    RING_ENTRY_PROGRESSED=4u, RING_ENTRY_NEAR=8u,
    RING_ENTRY_TURN_OK=16u, RING_ENTRY_TRAVEL_OK=32u,
    RING_ENTRY_HISTORY_FRESH=64u, RING_ENTRY_CROSS=128u,
    RING_ENTRY_ORDINARY=256u, RING_ENTRY_COMMITTED=512u,
    RING_ENTRY_ASSIST=1024u, RING_ENTRY_ASSIST_LEFT=2048u,
    RING_ENTRY_ASSIST_SPENT=4096u
};
enum {
    RING_HOLD_TIME=1u, RING_HOLD_FRAMES_REJECT=2u,
    RING_HOLD_TRAVEL=4u, RING_HOLD_YAW=8u,
    RING_HOLD_UNKNOWN_DT=16u, RING_HOLD_NO_HISTORY=32u
};
enum {
    RING_DECISION_NONE=0, RING_DECISION_MEASURED=1,
    RING_DECISION_HOLD=2, RING_DECISION_COMMIT=3,
    RING_DECISION_CANCEL_CROSS=4, RING_DECISION_CANCEL_ORDINARY=5,
    RING_DECISION_CANCEL_HOLD=6, RING_DECISION_CANCEL_APPROACH=7,
    RING_DECISION_STOP_LOST=8, RING_DECISION_FAULT=9,
    RING_DECISION_EXIT=10, RING_DECISION_ASSIST=11,
    RING_DECISION_ASSIST_RELEASE=12, RING_DECISION_ASSIST_EXPIRED=13,
    RING_DECISION_ASSIST_BLOCKED=14, RING_DECISION_RELEASE=15, RING_DECISION_OBSERVE=16
};

enum {
    RING_REJ_NO_SPLIT=1u, RING_REJ_DISCONNECTED=2u,
    RING_REJ_NO_RETURN=4u, RING_REJ_CROSS=8u,
    RING_REJ_CORRIDOR=16u, RING_REJ_PATH=32u,
    RING_REJ_EXPIRED=64u, RING_REJ_CONFLICT=128u,
    RING_REJ_BRANCH=256u, RING_REJ_STALE=512u,
    RING_REJ_SHORT=1024u, RING_REJ_ASSOC=2048u,
    RING_REJ_RESIDUAL=4096u, RING_REJ_LOOK=8192u,
    RING_REJ_CURVATURE=16384u
};

extern ring_result_t ring_result;
void ring_init(void);
/* Temporary standby/resume reset; retains the last real entry timestamp. */
void ring_reset_episode(void);
void ring_update(const uint8_t image[RING_H][RING_W], const ring_motion_t *motion);
/* Same-frame ORIGINAL track state, before any ring output is selected. */
void ring_observe_track(int track_state);
/* Actual visual clock interval supplied by the vision core; 65535=unknown. */
void ring_observe_frame_interval(uint16_t dt_ms);
/* Paired control feedback: stages0..4, signed turn progress in tenths. */
void ring_observe_maneuver_feedback(unsigned stage,int16_t turn10);
void ring_observe_exit_turn_feedback(unsigned stage);
uint8_t ring_owns_path(void);
uint8_t ring_blocks_cross(void);
int ring_speed_limit(void);    /* RPM; -1 = no ring cap, 0 = zero-speed balance */
int16_t ring_blend_error(int16_t normal_error);
int ring_turn_boost(void);
/* Relinquish ring control without stopping or reporting a successful exit. */
void ring_release_control(uint8_t reason,const ring_motion_t *motion);
#ifdef RING_HOST_TEST
/* Observes one real image without inventing N/F identity or task history. */
void ring_test_observe_mouth(const uint8_t image[RING_H][RING_W], int side, int lane);
#endif

#endif
