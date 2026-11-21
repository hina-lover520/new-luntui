#ifndef CODE_RING_LINK_H_
#define CODE_RING_LINK_H_

#include <stdint.h>

/* These two NEW cache lines never overlap the original 16-word track/debug
 * array at 0x28001000. Original communication remains untouched. */
#define RING_LINK_WORDS 8
#define RING_LINK_MAGIC 0x524E4704
#define RING_PAIR_VERSION 36u
/* v30 bit27: confirmed scheduled entry in timed APPROACH; otherwise the
 * existing non-timed ENTER force request. Never optional telemetry. */
#define RING_COMMAND_FORCE_ENTRY (1u << 27)
/* bit31: confirmed approach / bounded entry maneuver; never a release bit. */
#define RING_COMMAND_TIMED_ENTRY (1u << 31)
/* Same bit has a separate, phase-scoped meaning at EXIT/RECOVER. */
#define RING_COMMAND_EXIT_TURN (1u << 31)
/* Three command bits describe a release; zero ownership/cap required. */
#define RING_COMMAND_RELEASE_SHIFT 28u
#define RING_COMMAND_ADDRESS 0x28001040
#define RING_MOTION_ADDRESS  0x28001060
/* Diagnostic v5 has its own cache line. Control protocol/layout stays v4. */
#define RING_DIAGNOSTIC_ADDRESS 0x28001080
#define RING_DIAGNOSTIC_MAGIC 0x52444705
/* v8 entry decisions use a separate cache line; v4/v5 layouts stay fixed. */
#define RING_ENTRY_DIAGNOSTIC_ADDRESS 0x280010a0
#define RING_ENTRY_DIAGNOSTIC_MAGIC 0x52454708
/* v9 branch geometry is optional telemetry in its own cache line. */
#define RING_GEOMETRY_DIAGNOSTIC_ADDRESS 0x280010c0
#define RING_GEOMETRY_DIAGNOSTIC_MAGIC 0x52474709
/* v12 topology and optional frozen camera input never share control lines. */
#define RING_TOPOLOGY_DIAGNOSTIC_ADDRESS 0x280010e0
#define RING_TOPOLOGY_DIAGNOSTIC_MAGIC 0x5248470c
#define RING_IMAGE_ADDRESS 0x28001100
#define RING_IMAGE_ACK_ADDRESS 0x28001180
#define RING_IMAGE_MAGIC 0x5249470c
#define RING_IMAGE_ACK_MAGIC 0x5241470c
#define RING_MOUTH_DIAGNOSTIC_ADDRESS 0x280011a0
#define RING_MOUTH_DIAGNOSTIC_MAGIC 0x524a470d
/* v33 L is optional exit evidence in a new line, never a control input. */
#define RING_EXIT_DIAGNOSTIC_ADDRESS 0x280011c0
#define RING_EXIT_DIAGNOSTIC_MAGIC 0x524c4721
#define RING_IMAGE_BYTES 2820u /* continuous row-major bits, white=1, MSB first */
#define RING_IMAGE_CHUNK_BYTES 64u
#define RING_IMAGE_HEADER_WORDS 16u
#define RING_LINK_TIMEOUT_MS 300u
#define RING_EVENT_BEEP_MS   120u
/* RM_RESERVED: operator UI only, never changes command validation. */
#define RING_OPERATOR_VALID 0x55490000u
#define RING_OPERATOR_RUN   1u
#define RING_OPERATOR_BINARY 2u
#define RING_OPERATOR_ABORT_RING 4u

enum { RL_MAGIC, RL_SEQUENCE, RL_EPOCH };
enum { RC_CAP = 3, RC_PHASE, RC_DETECTED, RC_EXITED, RC_FAULT };
enum { RM_MS = 3, RM_YAW_MDEG, RM_WHEEL_MREV, RM_READY, RM_RESERVED };
enum { RD_COMMAND_SEQUENCE = 3, RD_FRAME, RD_TIMING_REJECT, RD_FLAGS, RD_POINTS };
enum { RE_COMMAND_SEQUENCE = 3, RE_FRAME, RE_FLAGS, RE_AGE_LOST, RE_ANGLES };
enum { RG_COMMAND_SEQUENCE = 3, RG_FRAME, RG_GEOMETRY, RG_STRONG_AGE, RG_GUIDE };
enum { RH_COMMAND_SEQUENCE = 3, RH_FRAME, RH_FLAGS, RH_ORIGIN_MDEG, RH_LAP_MDEG };
enum { RI_ID = 3, RI_FRAME, RI_COMMAND, RI_MS, RI_FLAGS, RI_OFFSET_LENGTH,
       RI_YAW10, RI_LAP10, RI_WHEEL_MREV, RI_CRC32, RI_DROPPED, RI_ROWS, RI_RESERVED };
enum { RA_IMAGE_SEQUENCE = 3, RA_ID, RA_OFFSET_LENGTH, RA_RESERVED1, RA_RESERVED2 };
enum { RJ_COMMAND_SEQUENCE=3, RJ_FRAME, RJ_POINTS, RJ_GATES, RJ_PROGRESS_PAIR };
enum { RX_COMMAND_SEQUENCE=3, RX_FRAME, RX_STATUS, RX_COUNTS_LAP, RX_FORWARD_MREV };
enum { RING_IMAGE_CONFIRMED=1u, RING_IMAGE_NEAR_PASSED=2u,
       RING_IMAGE_ENTERED=3u, RING_IMAGE_BRANCH_LOST=4u,
       RING_IMAGE_FAR_CANDIDATE=5u, RING_IMAGE_FAR_PROGRESS=6u,
       RING_IMAGE_GATE_CHANGED=7u, RING_IMAGE_APPROACH_CANCEL=8u,
       RING_IMAGE_HANDOVER=9u, RING_IMAGE_EXIT=10u, RING_IMAGE_RELEASE=11u,
       RING_IMAGE_EXIT_WINDOW=12u };

/* Version 4 keeps both 32-byte cache lines fixed. */
/* Phase low8, production pair version8, signed boost high16. Legacy
 * MODE0 fixtures explicitly keep the original zero version byte. */
static inline int32_t ring_command_phase(int phase, int boost)
{
    return (int32_t)((uint32_t)(phase & 255) | ((uint32_t)(uint16_t)boost << 16));
}

/* RC_CAP: signed cap in low halfword; signed net ring degrees in high half. */
static inline int32_t ring_command_versioned_phase(int phase,int boost,unsigned version)
{ return (int32_t)((uint32_t)ring_command_phase(phase,boost)|((version&255u)<<8)); }
static inline unsigned ring_command_version(int32_t v)
{ return ((uint32_t)v>>8)&255u; }
static inline int32_t ring_motion_operator_version(uint32_t flags,unsigned version)
{ return (int32_t)((flags&0xffff00ffu)|((version&255u)<<8)); }
static inline unsigned ring_motion_version(int32_t v)
{ return ((uint32_t)v>>8)&255u; }

/* v23 RM_READY: ready2, maneuver-stage3, reserved3, signed progress16.
 * The last byte is reserved; old 0..3 ready words remain compatible. */
static inline int32_t ring_motion_status(unsigned ready,unsigned stage,int progress10)
{return (int32_t)((ready&3u)|((stage&7u)<<2)|((uint32_t)(uint16_t)progress10<<8));}
static inline unsigned ring_motion_stage(int32_t value)
{return ((uint32_t)value>>2)&7u;}
static inline int ring_motion_progress(int32_t value)
{unsigned v=((uint32_t)value>>8)&65535u;return v>=32768u?(int)v-65536:(int)v;}
static inline int32_t ring_motion_status_exit(unsigned ready,unsigned stage,int progress10,unsigned exit_stage)
{return (int32_t)((uint32_t)ring_motion_status(ready,stage,progress10)|((exit_stage&3u)<<5));}
static inline unsigned ring_motion_exit_stage(int32_t value)
{return ((uint32_t)value>>5)&3u;}
static inline int ring_motion_status_valid(int32_t value)
{return !((uint32_t)value&0xff000080u) && ring_motion_stage(value)<=4u;}

/* RC_CAP: signed cap in low halfword; signed net ring degrees in high half. */
static inline int32_t ring_command_cap_turn(int cap, int turn_deg)
{
    return (int32_t)((uint32_t)(uint16_t)cap |
                     ((uint32_t)(uint16_t)(int16_t)turn_deg << 16));
}

static inline int ring_command_cap_value(int32_t encoded)
{
    int value = (int)((uint32_t)encoded & 65535u);
    return value >= 32768 ? value - 65536 : value;
}

static inline int ring_command_turn_deg(int32_t encoded)
{
    int value = (int)((uint32_t)encoded >> 16);
    return value >= 32768 ? value - 65536 : value;
}

/* RC_FAULT low bits retain the fault code. Upper fields carry raw vision
 * diagnostics for the existing CM7_1-to-UART path; no cache-line growth. */
static inline int32_t ring_command_diagnostics(int fault, int candidate,
    int cross_veto, int committed, int path_valid, int stop, int raw_state,
    int steer_error, int island_near, int route)
{
    unsigned near_code = island_near < 0 ? 0u : (unsigned)(island_near + 1);
    unsigned route_code = route > 0 ? 1u : (route < 0 ? 2u : 0u);
    unsigned error_code = (unsigned)(uint8_t)(int8_t)steer_error;
    return (int32_t)(((unsigned)fault & 7u) |
        ((unsigned)(candidate != 0) << 3) | ((unsigned)(cross_veto != 0) << 4) |
        ((unsigned)(committed != 0) << 5) | ((unsigned)(path_valid != 0) << 6) |
        ((unsigned)(stop != 0) << 7) | (((unsigned)raw_state & 3u) << 8) |
        ((near_code & 127u) << 10) | (error_code << 17) | (route_code << 25));
}

static inline int ring_command_diag_fault(int32_t encoded)
{ return (int)((uint32_t)encoded & 7u); }
static inline int ring_command_diag_candidate(int32_t encoded)
{ return (int)(((uint32_t)encoded >> 3) & 1u); }
static inline int ring_command_diag_cross_veto(int32_t encoded)
{ return (int)(((uint32_t)encoded >> 4) & 1u); }
static inline int ring_command_diag_committed(int32_t encoded)
{ return (int)(((uint32_t)encoded >> 5) & 1u); }
static inline int ring_command_diag_path_valid(int32_t encoded)
{ return (int)(((uint32_t)encoded >> 6) & 1u); }
static inline int ring_command_diag_stop(int32_t encoded)
{ return (int)(((uint32_t)encoded >> 7) & 1u); }
static inline int ring_command_diag_raw_state(int32_t encoded)
{ return (int)(((uint32_t)encoded >> 8) & 3u); }
static inline int ring_command_diag_near(int32_t encoded)
{
    int value = (int)(((uint32_t)encoded >> 10) & 127u);
    return value ? value - 1 : -1;
}
static inline int ring_command_diag_error(int32_t encoded)
{
    int value = (int)(((uint32_t)encoded >> 17) & 255u);
    return value >= 128 ? value - 256 : value;
}
static inline int ring_command_diag_route(int32_t encoded)
{
    int value = (int)(((uint32_t)encoded >> 25) & 3u);
    return value == 1 ? 1 : (value == 2 ? -1 : 0);
}

static inline int ring_command_boost(int32_t encoded)
{
    int value = (int)((uint32_t)encoded >> 16);
    return value >= 32768 ? value - 65536 : value;
}

/* Each diagnostic snapshot is tied to one command sequence and epoch. */
static inline int32_t ring_diagnostic_flags(int detected, int allowed, int score,
    int confirm_frames, int side, int source, int branch, int classification)
{
    unsigned side_code = side > 0 ? 1u : (side < 0 ? 2u : 0u);
    return (int32_t)((unsigned)(detected != 0) | ((unsigned)(allowed != 0) << 1) |
        ((unsigned)(branch != 0) << 2) | (((unsigned)classification & 3u) << 3) |
        (((unsigned)source & 7u) << 5) | (side_code << 8) |
        (((unsigned)score & 255u) << 10) | (((unsigned)confirm_frames & 255u) << 18));
}
static inline int ring_diagnostic_detected(int32_t v) { return (int)((uint32_t)v & 1u); }
static inline int ring_diagnostic_allowed(int32_t v) { return (int)(((uint32_t)v >> 1) & 1u); }
static inline int ring_diagnostic_branch(int32_t v) { return (int)(((uint32_t)v >> 2) & 1u); }
static inline int ring_diagnostic_classification(int32_t v) { return (int)(((uint32_t)v >> 3) & 3u); }
static inline int ring_diagnostic_source(int32_t v) { return (int)(((uint32_t)v >> 5) & 7u); }
static inline int ring_diagnostic_side(int32_t v)
{
    unsigned code = ((uint32_t)v >> 8) & 3u;
    return code == 1u ? 1 : (code == 2u ? -1 : 0);
}
static inline int ring_diagnostic_score(int32_t v) { return (int)(((uint32_t)v >> 10) & 255u); }
static inline int ring_diagnostic_confirm_frames(int32_t v) { return (int)(((uint32_t)v >> 18) & 255u); }
static inline int32_t ring_diagnostic_timing(uint16_t dt, uint16_t reject)
{ return (int32_t)((uint32_t)dt | ((uint32_t)reject << 16)); }
static inline int32_t ring_diagnostic_points(int near_x, int look_x)
{ return (int32_t)((uint32_t)(uint16_t)near_x | ((uint32_t)(uint16_t)look_x << 16)); }
static inline int ring_diagnostic_near_x(int32_t v)
{
    int result = (int)((uint32_t)v & 65535u);
    return result >= 32768 ? result - 65536 : result;
}
static inline int ring_diagnostic_look_x(int32_t v)
{
    int result = (int)((uint32_t)v >> 16);
    return result >= 32768 ? result - 65536 : result;
}

/* Entry diagnostics: flags16/reject8/decision8, age16/lost8/reserved8,
 * signed hold-yaw10 / signed entry-yaw10. No word is a control input. */
static inline int32_t ring_entry_diagnostic_flags(unsigned flags, unsigned reject,
    unsigned decision)
{ return (int32_t)((flags & 65535u) | ((reject & 255u) << 16) | ((decision & 255u) << 24)); }
static inline int32_t ring_entry_diagnostic_age(unsigned age, unsigned lost)
{ return (int32_t)((age & 65535u) | ((lost & 255u) << 16)); }
static inline int32_t ring_entry_diagnostic_angles(int hold10, int entry10)
{ return (int32_t)((uint32_t)(uint16_t)hold10 | ((uint32_t)(uint16_t)entry10 << 16)); }
static inline int ring_entry_diagnostic_hold_turn(int32_t v)
{
    int result = (int)((uint32_t)v & 65535u);
    return result >= 32768 ? result - 65536 : result;
}
static inline int ring_entry_diagnostic_entry_turn(int32_t v)
{
    int result = (int)((uint32_t)v >> 16);
    return result >= 32768 ? result - 65536 : result;
}

/* mode8/rows8/associated8/support8. Guide: signed x16 and unsigned y16,
 * with x=-1/y=255 for an unavailable guide. High strong-age bits reserved. */
static inline int32_t ring_geometry_diagnostic_flags(unsigned mode, unsigned rows,
    unsigned associated, unsigned support)
{ return (int32_t)((mode & 255u) | ((rows & 255u) << 8) |
    ((associated & 255u) << 16) | ((support & 255u) << 24)); }
static inline int32_t ring_geometry_diagnostic_guide(int x, unsigned y)
{ return (int32_t)((uint32_t)(uint16_t)x | ((y & 65535u) << 16)); }
static inline int ring_geometry_diagnostic_x(int32_t v)
{ return ring_diagnostic_near_x(v); }
static inline unsigned ring_geometry_diagnostic_y(int32_t v)
{ return (unsigned)((uint32_t)v >> 16); }

/* Side2/stage3/left1/lap1/near8/far8/source2, high bits reserved.
 * Row code0=-1. Source0=none,1=width neck,2=returning shoulder (v17). */
static inline int32_t ring_topology_flags(int side, unsigned stage, unsigned left,
    unsigned lap_valid, int near_row, int far_row)
{
    unsigned code = side > 0 ? 1u : (side < 0 ? 2u : 0u);
    unsigned near_code = near_row < 0 ? 0u : (unsigned)near_row + 1u;
    unsigned far_code = far_row < 0 ? 0u : (unsigned)far_row + 1u;
    return (int32_t)(code | ((stage & 7u) << 2) | ((left & 1u) << 5) |
        ((lap_valid & 1u) << 6) | ((near_code & 255u) << 7) | ((far_code & 255u) << 15));
}
static inline int ring_topology_side(int32_t v)
{ unsigned code=(uint32_t)v & 3u; return code==1u?1:(code==2u?-1:0); }
static inline int ring_topology_near_row(int32_t v)
{ return (int)(((uint32_t)v >> 7) & 255u)-1; }
static inline int ring_topology_far_row(int32_t v)
{ return (int)(((uint32_t)v >> 15) & 255u)-1; }

/* Current measurements only: missing coordinates use 255, never a retained
 * H near/far row. Counts saturate at15; progress saturates signed8. */
static inline int32_t ring_mouth_points(int nx,int ny,int fx,int fy)
{
    return (int32_t)((uint32_t)(uint8_t)nx|((uint32_t)(uint8_t)ny<<8)|
        ((uint32_t)(uint8_t)fx<<16)|((uint32_t)(uint8_t)fy<<24));
}
static inline int ring_mouth_coordinate(int32_t value,unsigned shift)
{ unsigned v=((uint32_t)value>>shift)&255u;return v==255u?-1:(int)v; }
static inline int32_t ring_mouth_gates(unsigned gate,unsigned ref,unsigned nc,unsigned fc,unsigned gap)
{
    if(nc>15u)nc=15u;
    if(fc>15u)fc=15u;
    if(gap>3u)gap=3u;
    return (int32_t)((gate&8191u)|((ref&1u)<<13)|(nc<<14)|(fc<<18)|(gap<<22)|
        ((gate&24576u)<<11));
}
/* v16 adds independent ready/path rejects in bits24..25. Low gate
 * bits, all control words and all shared addresses retain their meaning. */
static inline unsigned ring_mouth_gate_value(int32_t packed)
{ return ((uint32_t)packed&8191u)|(((uint32_t)packed>>11)&24576u); }
static inline int32_t ring_mouth_progress_pair(int first,int progress,unsigned control,unsigned vision)
{
    if(progress>127)progress=127;
    if(progress< -127)progress=-127;
    return (int32_t)((uint32_t)(uint8_t)first|((uint32_t)(uint8_t)(int8_t)progress<<8)|
        ((control&255u)<<16)|((vision&255u)<<24));
}
/* v14 reuses previously reserved diagnostic bits. No control words or
 * shared addresses change. Missed7 means seven or more visual frames. */
static inline int32_t ring_mouth_tracking(int32_t gates,unsigned missed,unsigned reason)
{
    if(missed>7u) missed=7u;
    return (int32_t)(((uint32_t)gates&0x03ffffffu)|(missed<<26)|((reason&3u)<<29));
}
static inline int ring_mouth_progress(int32_t v)
{ int value=(int)(((uint32_t)v>>8)&255u);return value>=128?value-256:value; }

/* Independent L evidence: reject7/candidate1/latched1/confirmed1 and
 * candidate-count8/confirm-count8/signed lap10 high16. Counts saturate;
 * current signed forward millirevolutions retain the full final word. */
static inline int32_t ring_exit_diagnostic_status(unsigned reject,unsigned candidate,
    unsigned latched,unsigned confirmed)
{ return (int32_t)((reject&127u)|((candidate&1u)<<16)|
    ((latched&1u)<<17)|((confirmed&1u)<<18)); }
static inline int32_t ring_exit_diagnostic_counts(unsigned candidate_frames,
    unsigned confirm_frames,int lap10)
{
    if(candidate_frames>255u) candidate_frames=255u;
    if(confirm_frames>255u) confirm_frames=255u;
    if(lap10>32767) lap10=32767;
    if(lap10< -32768) lap10= -32768;
    return (int32_t)(candidate_frames|(confirm_frames<<8)|
        ((uint32_t)(uint16_t)(int16_t)lap10<<16));
}
static inline int ring_exit_diagnostic_lap10(int32_t value)
{ int v=(int)((uint32_t)value>>16);return v>=32768?v-65536:v; }

typedef struct { int32_t word[RING_IMAGE_HEADER_WORDS];
    uint8_t data[RING_IMAGE_CHUNK_BYTES]; } ring_image_packet_t;
typedef struct { int32_t word[RING_LINK_WORDS]; } ring_packet_t;
extern int32_t ring_command_mailbox[RING_LINK_WORDS]; /* CM7_0 writes */
extern int32_t ring_motion_mailbox[RING_LINK_WORDS];  /* CM7_1 writes */
extern int32_t ring_diagnostic_mailbox[RING_LINK_WORDS]; /* CM7_0 writes */
extern int32_t ring_entry_diagnostic_mailbox[RING_LINK_WORDS]; /* CM7_0 writes */
extern int32_t ring_geometry_diagnostic_mailbox[RING_LINK_WORDS]; /* CM7_0 writes */

extern int32_t ring_mouth_diagnostic_mailbox[RING_LINK_WORDS];
extern int32_t ring_topology_diagnostic_mailbox[RING_LINK_WORDS];
extern int32_t ring_exit_diagnostic_mailbox[RING_LINK_WORDS];
extern ring_image_packet_t ring_image_mailbox;
extern int32_t ring_image_ack_mailbox[RING_LINK_WORDS];

void ring_link_write_image(const ring_image_packet_t *packet, uint32_t *sequence);
int ring_link_read_image(ring_image_packet_t *packet);
void ring_link_ack_image(const ring_image_packet_t *packet, uint32_t *sequence);
int ring_link_read_image_ack(ring_packet_t *packet);
int ring_link_read_topology_diagnostic(ring_packet_t *packet);
int ring_link_read_mouth_diagnostic(ring_packet_t *packet);
int ring_link_read_exit_diagnostic(ring_packet_t *packet);
uint32_t ring_image_crc32(const uint8_t *data, unsigned length);

void ring_link_write(int32_t *destination, const ring_packet_t *packet,
                     uint32_t *sequence);
int ring_link_read(int32_t *source, ring_packet_t *packet);
int ring_link_read_diagnostic(ring_packet_t *packet);
int ring_link_read_entry_diagnostic(ring_packet_t *packet);
int ring_link_read_geometry_diagnostic(ring_packet_t *packet);

#endif
