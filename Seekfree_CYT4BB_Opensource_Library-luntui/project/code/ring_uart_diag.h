#ifndef CODE_RING_UART_DIAG_H_
#define CODE_RING_UART_DIAG_H_

/* CM7_1 only. UART1 remains 115200, P04_1 TX/P04_0 RX. FIFO service is
 * bounded to 64 writes per call and twice per poll, including idle polls.
 * R33/D19/E14/G14 retain their layouts; v11 exposes bounded entry assist. */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "zf_driver_uart.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring.h"

#define RING_DIAG_STRINGIFY_IMPL(value) #value
#define RING_DIAG_STRINGIFY(value) RING_DIAG_STRINGIFY_IMPL(value)

#define RING_DIAG_LINE_SIZE 256u
#define RING_DIAG_QUEUE_LINES 8u
#define RING_DIAG_PERIOD_MS 1000u
#define RING_DIAG_ENTRY_PERIOD_MS 200u
#define RING_DIAG_GEOMETRY_PERIOD_MS 400u
#define RING_DIAG_SERVICE_BYTES 64u

typedef struct {
    char text[RING_DIAG_LINE_SIZE];
    unsigned length, sent;
} ring_diag_line_t;

static ring_diag_line_t ring_diag_queue[RING_DIAG_QUEUE_LINES];
static unsigned ring_diag_head, ring_diag_count;
static uint32_t ring_diag_last_ms, ring_diag_last_key, ring_diag_seq;
static uint32_t ring_diag_dropped;
static unsigned ring_diag_header_line;
static int ring_diag_have_last;
static uint32_t ring_diag_detail_seq, ring_diag_detail_frame, ring_diag_detail_epoch;
static uint32_t ring_diag_detail_key, ring_diag_detail_last_ms;
static int ring_diag_have_detail;
static uint32_t ring_diag_entry_seq, ring_diag_entry_frame, ring_diag_entry_epoch;
static uint32_t ring_diag_entry_key, ring_diag_entry_last_ms;
static int ring_diag_have_entry;
static uint32_t ring_diag_geometry_seq, ring_diag_geometry_frame, ring_diag_geometry_epoch;
static uint32_t ring_diag_geometry_last_ms;
static int ring_diag_have_geometry;
static uint32_t ring_diag_topology_seq,ring_diag_topology_frame,ring_diag_topology_epoch;
static uint32_t ring_diag_topology_last_ms,ring_diag_topology_key;
static int ring_diag_have_topology;
static uint32_t ring_diag_mouth_seq,ring_diag_mouth_frame,ring_diag_mouth_epoch,ring_diag_mouth_last_ms;
static uint16_t ring_diag_mouth_gate;
static int ring_diag_have_mouth;
static uint32_t ring_diag_exit_seq,ring_diag_exit_frame,ring_diag_exit_epoch;
static uint32_t ring_diag_exit_last_ms,ring_diag_exit_key;
static int ring_diag_have_exit;
static uint32_t ring_diag_exit_force_seq,ring_diag_exit_force_epoch,ring_diag_exit_force_detected;
static uint32_t ring_diag_exit_force_last_ms,ring_diag_exit_force_key;
static int ring_diag_have_exit_force;
static unsigned ring_diag_exit_force_header_line;
static uint32_t ring_diag_pair_last_key;
static int ring_diag_have_pair;
static uint32_t ring_diag_force_seq,ring_diag_force_last_ms,ring_diag_force_key;
static int ring_diag_have_force;
static uint32_t ring_diag_bias_seq,ring_diag_bias_last_ms;
static int ring_diag_have_bias;
static uint32_t ring_diag_image_sequence,ring_diag_image_epoch,ring_diag_ack_sequence;

enum { RING_DIAG_M, RING_DIAG_K, RING_DIAG_D, RING_DIAG_L,
       RING_DIAG_J, RING_DIAG_E, RING_DIAG_H, RING_DIAG_G, RING_DIAG_DOC,
       RING_DIAG_NONE };
static unsigned ring_diag_optional_cursor;

/* Admission is shared, not a priority chain: changing E decisions must not
 * hide branch sources in D or exit evidence in L. M/K transitions remain
 * immediate; their routine samples have two turns per round. A current D/L
 * older than330ms precedes routine samples, keeping them below500ms with
 * the existing46ms poll. All records retain their original field layouts. */
static unsigned ring_diag_optional_select(const ring_runtime_diagnostics_t *d,
    uint32_t detail_key)
{
    static const unsigned order[]={RING_DIAG_M,RING_DIAG_K,RING_DIAG_D,
        RING_DIAG_M,RING_DIAG_K,RING_DIAG_L,RING_DIAG_J,RING_DIAG_E,
        RING_DIAG_H,RING_DIAG_G,RING_DIAG_DOC};
    unsigned due=0u,urgent=0u,i,index;
    uint32_t now=d->now_ms;
    if(RING_TRACK_MODE) {
        uint32_t mk=(uint32_t)d->exit_force_stage|((uint32_t)d->exit_force_reason<<8)|
            ((uint32_t)(uint8_t)d->exit_force_side<<16);
        int ne=!ring_diag_have_exit_force || d->command_epoch!=ring_diag_exit_force_epoch;
        int event=(!ring_diag_have_exit_force && d->exit_force_stage) ||
            (ring_diag_have_exit_force && (mk!=ring_diag_exit_force_key ||
             (d->exit_force_stage && (ne || d->detected!=ring_diag_exit_force_detected))));
        if(event || (d->exit_force_stage && (ne ||
           (d->phase>=RING_APPROACH && d->phase<=RING_RECOVER &&
            now-ring_diag_exit_force_last_ms>=200u)))) {
            due|=1u<<RING_DIAG_M;if(event)urgent|=1u<<RING_DIAG_M;
        }
        {
            uint32_t fk=(uint32_t)d->force_active|((uint32_t)d->force_reason<<8)|
                ((uint32_t)d->release_reason<<16)|((uint32_t)d->run_requested<<24)|
                ((uint32_t)d->standing<<25)|((uint32_t)d->maneuver_stage<<26);
            int event=!ring_diag_have_force || fk!=ring_diag_force_key;
            int interesting=(d->maneuver_stage>0u && d->maneuver_stage<3u) ||
                (d->active && d->maneuver_stage==3u) || d->phase==RING_ENTER ||
                d->force_active || (ring_diag_have_force && event) ||
                (RING_TIMED_ENTRY && d->run_requested && d->standing &&
                 now-ring_diag_force_last_ms>=1000u);
            if(interesting && (event || now-ring_diag_force_last_ms>=200u)) {
                due|=1u<<RING_DIAG_K;if(event)urgent|=1u<<RING_DIAG_K;
            }
        }
        if(ring_diag_have_exit_force && d->exit_force_stage &&
           ring_diag_exit_force_header_line<2u && !ring_diag_count)due|=1u<<RING_DIAG_DOC;
    }
    if(d->diagnostic_valid) {
        int ne=!ring_diag_have_detail || d->diagnostic_epoch!=ring_diag_detail_epoch;
        int interesting=d->candidate_detected || d->candidate_allowed || d->active ||
            d->classification || (d->phase>=1 && d->phase<=5);
        if((ne || d->frame_sequence!=ring_diag_detail_frame) &&
           (ne || detail_key!=ring_diag_detail_key ||
            (interesting && (uint32_t)(d->frame_sequence-ring_diag_detail_frame)>=2u) ||
            (!interesting && now-ring_diag_detail_last_ms>=RING_DIAG_PERIOD_MS)))due|=1u<<RING_DIAG_D;
    }
    if(d->exit_diagnostic_valid) {
        uint32_t ek=(uint32_t)d->exit_candidate|((uint32_t)d->exit_latched<<1)|
            ((uint32_t)d->exit_confirmed<<2);
        int ne=!ring_diag_have_exit || d->exit_diagnostic_epoch!=ring_diag_exit_epoch;
        int event=ring_diag_have_exit && (ne || ek!=ring_diag_exit_key);
        int interesting=(d->phase>=RING_APPROACH && d->phase<=RING_RECOVER) ||
            (!ring_diag_have_exit && ek) || event;
        if((ne || d->exit_frame_sequence!=ring_diag_exit_frame) && interesting &&
           (ne || event || now-ring_diag_exit_last_ms>=200u))due|=1u<<RING_DIAG_L;
    }
    if(d->mouth_valid) {
        int ne=!ring_diag_have_mouth || d->mouth_epoch!=ring_diag_mouth_epoch;
        int interesting=d->phase>=RING_APPROACH && d->phase<=RING_RECOVER;
        if((ne || d->mouth_frame!=ring_diag_mouth_frame) &&
           (ne || now-ring_diag_mouth_last_ms>=(interesting?200u:RING_DIAG_PERIOD_MS)))due|=1u<<RING_DIAG_J;
    }
    if(d->entry_diagnostic_valid) {
        uint32_t ek=((uint32_t)d->phase<<8)|d->decision|((uint32_t)(d->entry_flags&7168u)<<6);
        unsigned previous=ring_diag_entry_key&255u;
        int ne=!ring_diag_have_entry || d->entry_diagnostic_epoch!=ring_diag_entry_epoch;
        int event=ne || ((ek^ring_diag_entry_key)&0x7ff00u)!=0u ||
            (d->decision!=previous && (d->decision>=3u || previous>=3u));
        int interesting=d->active || d->decision || d->entry_flags || (d->phase>=1 && d->phase<=5);
        if((ne || d->entry_frame_sequence!=ring_diag_entry_frame) &&
           (event || (interesting && now-ring_diag_entry_last_ms>=RING_DIAG_ENTRY_PERIOD_MS)))due|=1u<<RING_DIAG_E;
    }
    if(d->topology_valid) {
        uint32_t hk=(uint32_t)d->approach_stage|((uint32_t)d->left_entry<<8)|
            ((uint32_t)d->lap_valid<<9)|((uint32_t)(uint8_t)d->ring_side<<16);
        int ne=!ring_diag_have_topology || d->topology_epoch!=ring_diag_topology_epoch;
        int interesting=d->approach_stage || d->active || (d->phase>=1 && d->phase<=5);
        if((ne || d->topology_frame!=ring_diag_topology_frame) &&
           (ne || hk!=ring_diag_topology_key ||
            (interesting && now-ring_diag_topology_last_ms>=200u)))due|=1u<<RING_DIAG_H;
    }
    if(d->geometry_valid) {
        int ne=!ring_diag_have_geometry || d->geometry_epoch!=ring_diag_geometry_epoch;
        int interesting=d->active || d->branch_mode || d->inner_rows || d->associated_rows ||
            d->candidate_detected || d->candidate_allowed || (d->phase>=1 && d->phase<=5);
        if((ne || d->geometry_frame_sequence!=ring_diag_geometry_frame) &&
           (ne || (interesting && now-ring_diag_geometry_last_ms>=RING_DIAG_GEOMETRY_PERIOD_MS)))due|=1u<<RING_DIAG_G;
    }
    if(urgent&(1u<<RING_DIAG_M))return RING_DIAG_M;
    if((due&(1u<<RING_DIAG_D)) && now-ring_diag_detail_last_ms>=330u &&
       (!(due&(1u<<RING_DIAG_L)) || now-ring_diag_exit_last_ms<330u ||
        now-ring_diag_detail_last_ms>=now-ring_diag_exit_last_ms))return RING_DIAG_D;
    if((due&(1u<<RING_DIAG_L)) && now-ring_diag_exit_last_ms>=330u)return RING_DIAG_L;
    if(urgent&(1u<<RING_DIAG_K))return RING_DIAG_K;
    for(i=0;i<sizeof(order)/sizeof(order[0]);++i) {
        index=(ring_diag_optional_cursor+i)%(sizeof(order)/sizeof(order[0]));
        if(due&(1u<<order[index])) {
            ring_diag_optional_cursor=(index+1u)%(sizeof(order)/sizeof(order[0]));
            return order[index];
        }
    }
    return RING_DIAG_NONE;
}

static unsigned ring_diag_service(void)
{
    volatile stc_SCB_t *port = get_scb_module(UART_1);
    uint32_t size = Cy_SCB_GetFifoSize(port);
    uint32_t used = Cy_SCB_GetNumInTxFifo(port);
    unsigned written = 0;
    while (ring_diag_count && used < size && written < RING_DIAG_SERVICE_BYTES) {
        ring_diag_line_t *line = &ring_diag_queue[ring_diag_head];
        Cy_SCB_WriteTxFifo(port, (uint8_t)line->text[line->sent++]);
        ++used; ++written;
        if (line->sent == line->length) {
            ring_diag_head = (ring_diag_head + 1u) % RING_DIAG_QUEUE_LINES;
            --ring_diag_count;
        }
    }
    return written;
}

static int ring_diag_enqueue(const char *text, unsigned length)
{
    unsigned i;
    ring_diag_line_t *line;
    if (length >= RING_DIAG_LINE_SIZE || ring_diag_count == RING_DIAG_QUEUE_LINES) return 0;
    line = &ring_diag_queue[(ring_diag_head + ring_diag_count) % RING_DIAG_QUEUE_LINES];
    for (i = 0; i < length; ++i) line->text[i] = text[i];
    line->length = length; line->sent = 0;
    ++ring_diag_count;
    return 1;
}

/* Frozen images have the lowest priority and never renew control freshness.
 * ACK means copied into the bounded software TX queue, never UART wait.
 * The receiver checks the full image CRC32 and treats missing bytes as loss. */
static void ring_diag_image_poll(const ring_runtime_diagnostics_t *d)
{
    ring_image_packet_t packet;
    char line[RING_DIAG_LINE_SIZE];
    static const char hex[]="0123456789ABCDEF";
    unsigned offset,length,i,n,flags,reason,phase,stage,source,route;
    int count,near_row,far_row;
    if(ring_diag_count || (d->phase!=RING_NORMAL && d->phase!=RING_COOLDOWN &&
        d->phase!=RING_FAULT)) return;
    if(!ring_link_read_image(&packet) || (uint32_t)packet.word[RL_EPOCH]!=d->command_epoch ||
        ((uint32_t)packet.word[RL_SEQUENCE]==ring_diag_image_sequence &&
         (uint32_t)packet.word[RL_EPOCH]==ring_diag_image_epoch)) return;
    offset=(uint32_t)packet.word[RI_OFFSET_LENGTH]&65535u;
    length=(uint32_t)packet.word[RI_OFFSET_LENGTH]>>16;
    flags=(uint32_t)packet.word[RI_FLAGS];reason=flags&15u;phase=(flags>>4)&15u;
    stage=(flags>>8)&15u;source=(flags>>12)&15u;route=(flags>>16)&255u;
    near_row=ring_diagnostic_near_x(packet.word[RI_ROWS]);
    far_row=ring_diagnostic_look_x(packet.word[RI_ROWS]);
    if(!packet.word[RI_ID] || !packet.word[RI_COMMAND] ||
        ((uint32_t)packet.word[RI_COMMAND]&1u) ||
        ((uint32_t)packet.word[RI_RESERVED]&0xffff0000u) ||
        !length || length>RING_IMAGE_CHUNK_BYTES || offset%RING_IMAGE_CHUNK_BYTES ||
        offset+length>RING_IMAGE_BYTES || (offset+length<RING_IMAGE_BYTES && length!=RING_IMAGE_CHUNK_BYTES) ||
        (flags&0xf8000000u) || reason<1u || reason>RING_IMAGE_EXIT_WINDOW || phase>RING_FAULT ||
        stage>RING_TANGENT_EXIT || (source!=0u && source!=1u && source!=2u && source!=4u && source!=5u && source!=6u &&
        !(source==7u && ((uint32_t)packet.word[RI_RESERVED]&255u)==RING_PAIR_VERSION &&
          (((uint32_t)packet.word[RI_RESERVED]>>8)&255u)==RING_PAIR_VERSION)) ||
        (route!=0u && route!=1u && route!=255u) || near_row< -1 || near_row>=RING_H ||
        far_row< -1 || far_row>=RING_H) return;
    if(!offset) {
        count=snprintf(line,sizeof(line),"S,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%d,%d,%u,%u\r\n",
            (unsigned long)(uint32_t)packet.word[RL_EPOCH],(unsigned long)(uint32_t)packet.word[RI_ID],
            (unsigned long)(uint32_t)packet.word[RI_FRAME],(unsigned long)(uint32_t)packet.word[RI_COMMAND],
            (unsigned long)(uint32_t)packet.word[RI_MS],(unsigned long)flags,
            (long)packet.word[RI_YAW10],(long)packet.word[RI_LAP10],(long)packet.word[RI_WHEEL_MREV],
            (unsigned long)(uint32_t)packet.word[RI_CRC32],(unsigned long)(uint32_t)packet.word[RI_DROPPED],
            near_row,far_row,(unsigned)((uint32_t)packet.word[RI_RESERVED]&255u),
            (unsigned)(((uint32_t)packet.word[RI_RESERVED]>>8)&255u));
        if(count<=0 || (unsigned)count>=sizeof(line) || !ring_diag_enqueue(line,(unsigned)count)) return;
    }
    count=snprintf(line,sizeof(line),"I,%lu,%lu,%u,",
        (unsigned long)(uint32_t)packet.word[RL_EPOCH],(unsigned long)(uint32_t)packet.word[RI_ID],offset);
    if(count<=0 || (unsigned)count+length*2u+2u>=sizeof(line)) return;
    n=(unsigned)count;
    for(i=0;i<length;++i) { line[n++]=hex[packet.data[i]>>4];line[n++]=hex[packet.data[i]&15u]; }
    line[n++]='\r';line[n++]='\n';
    if(!ring_diag_enqueue(line,n)) return;
    ring_diag_image_sequence=(uint32_t)packet.word[RL_SEQUENCE];
    ring_diag_image_epoch=(uint32_t)packet.word[RL_EPOCH];
    ring_link_ack_image(&packet,&ring_diag_ack_sequence);
}

static void ring_uart_diag_poll(int steer_error, int speed_left, int speed_right)
{
    static const char *const headers[] = {
        "#RING,36,UART1,115200,8N1,TX=P04_1,RX=P04_0,CONTROL=4,PAIR=36,IMAGE=IPM,TRACK_MODE="
        RING_DIAG_STRINGIFY(RING_TRACK_MODE) ",YAW_SIGN="
        RING_DIAG_STRINGIFY(RING_YAW_SIGN) "\r\n",
        "#seq,ms,dt,loop,ph,dir,raw,out,cand,xv,cm,pv,stop,f,act,e,sel,near,deg,cap,br,ba,v0,v1,u0,u1,L,R,age,w,det,exit,drop\r\n",
        "#B,seq,ms,raw,bias_raw,rate10,cal,cal_age,cal_n,state,turn_cap10,drop;RAW_COUNTS;RATE_DPSx10;CAP_RPMx10\r\n"
        "#D,seq,ms,epoch,cmd,frame,vdt,cand_det,cand_ok,score,rej,cc,side,src,nx,lx,branch,cls,age,drop\r\n",
        "#E,seq,ms,epoch,cmd,frame,entry_flags,hold_reject,decision,lost_frames,path_age_ms,hold_turn10,entry_turn10,age,drop\r\n",
        "#G,seq,ms,epoch,cmd,frame,mode,rows,assoc,support,strong_age_ms,gx,gy,age,drop\r\n",
        "#H,seq,ms,epoch,cmd,frame,ring_side,stage,left_entry,lap_valid,near_row,far_row,origin_mdeg,lap_mdeg,age,drop,fs\r\n",
        "#J,seq,ms,epoch,cmd,frame,nx,ny,fx,fy,first_y,progress,gate,ref,nc,fc,gap,cv,vv,age,drop,nmiss,npass\r\n",
        "#K,seq,ms,epoch,cmd,active,side,target10,rate10,out10,reason,age,drop,release,run,ready,v0,v1,man,mt10,rem10,elapsed,held10,goal10,meas10,pitch100,tilt100,pwmL,pwmR,m_age;RATE_DPSx10;OUT_RPMx10\r\n",
        "#L,seq,ms,epoch,cmd,frame,reject,candidate,latched,confirmed,candidate_frames,confirm_frames,lap10,forward_mrev,age,drop\r\n",
        "#L_REJECT,1:lap|2:entry|4:geometry|8:continuity|16:forward|32:corridor|64:heading;NEW_VISION_ONLY=1;OPTIONAL=1\r\n",
        "#PAIR_SCHEMA,control_version,vision_version,pair_ok;J_CURRENT_ONLY=1;FORCE_DEG="
        RING_DIAG_STRINGIFY(RING_FORCE_ENTRY_YAW_DEG) ";FORCE_MS="
        RING_DIAG_STRINGIFY(RING_FORCE_TOTAL_MS) ";FORCE_REVS="
        RING_DIAG_STRINGIFY(RING_FORCE_ENTRY_REVS) ";TIMED_ENTRY="
        RING_DIAG_STRINGIFY(RING_TIMED_ENTRY) ";DELAY_MS="
        RING_DIAG_STRINGIFY(RING_MANEUVER_DELAY_MS) ";TURN_DEG="
        RING_DIAG_STRINGIFY(RING_MANEUVER_TURN_DEG) ";REENTRY_MS="
        RING_DIAG_STRINGIFY(RING_REENTRY_BLOCK_MS) "\r\n",
        "#LAP,MIN_DEG=" RING_DIAG_STRINGIFY(RING_EXIT_MIN_DEG)
        ";HEADING_TOL=" RING_DIAG_STRINGIFY(RING_LAP_EXIT_HEADING_TOL)
        ";MAN=1:straight|2:entry75|3:visual_follow|4:abort;YAW=EXIT_GATE_ONLY;REM=360-MT;NO_HEADING_LOCK=1;ENTRY=CONFIRMED_DELAY_SPEED;J_GATE=DIAGNOSTIC\r\n",
        "@EXIT_WINDOW",
        "#S,epoch,id,frame,cmd,ms,flags,yaw10,lap10,wheel_mrev,crc32,capture_drop,near_row,far_row,cv,vv;IMAGE=188x120,MSB,WHITE=1,BYTES=2820;REASON12=EXIT_WINDOW\r\n",
        "#I,epoch,id,offset,hex;EVENT=1:confirmed|2:near_passed|3:enter|4:branch_lost|5:F_first|6:F_progress|7:gate_change|8:approach_cancel;IDLE_ONLY=1\r\n",
        "#ENTRY_ASSIST,MIN_ERROR=" RING_DIAG_STRINGIFY(RING_ASSIST_MIN_ERROR)
        ",TARGET_ERROR=" RING_DIAG_STRINGIFY(RING_ASSIST_TARGET_ERROR)
        ",CONFIRM=" RING_DIAG_STRINGIFY(RING_ASSIST_CONFIRM)
        ",TOTAL_MS=" RING_DIAG_STRINGIFY(RING_ASSIST_TOTAL_MS)
        ",ENTRY_YAW_DEG=" RING_DIAG_STRINGIFY(RING_ASSIST_ENTRY_YAW_DEG)
        ",ENTRY_REVS=" RING_DIAG_STRINGIFY(RING_ASSIST_ENTRY_REVS) "\r\n",
        "#ENTRY_ASSIST,HOLD_MS=" RING_DIAG_STRINGIFY(RING_ASSIST_HOLD_MS)
        ",HOLD_YAW_DEG=" RING_DIAG_STRINGIFY(RING_ASSIST_HOLD_YAW_DEG)
        ",HOLD_REVS=" RING_DIAG_STRINGIFY(RING_ASSIST_HOLD_REVS)
        ",FLAGS=1024:active|2048:left|4096:spent,DECISION=11:assist|12:release|13:expired|14:blocked\r\n"
    };
    const ring_runtime_diagnostics_t *d = ring_runtime_diagnostics();
    char line[RING_DIAG_LINE_SIZE];
    uint32_t key = ((uint32_t)d->phase & 255u) | ((uint32_t)d->fault << 8) |
        ((uint32_t)d->stop << 12) | ((uint32_t)d->active << 13) |
        ((uint32_t)d->committed << 14) | ((uint32_t)d->cross_veto << 15) |
        ((uint32_t)d->path_valid << 16) | ((uint32_t)d->raw_state << 17);
    uint32_t detail_key = ((uint32_t)d->phase & 255u) |
        ((uint32_t)d->candidate_detected << 8) | ((uint32_t)d->candidate_allowed << 9) |
        ((uint32_t)d->branch_ok << 10) | ((uint32_t)d->classification << 11) |
        ((uint32_t)d->committed << 13) | ((uint32_t)d->fault << 14) |
        ((uint32_t)d->raw_state << 17);
    uint32_t elapsed = d->now_ms - ring_diag_last_ms;
    int length;
    int entry_sent = 0;
    int geometry_sent = 0;
    int topology_sent = 0;
    int mouth_sent = 0;
    int exit_sent = 0;
    int exit_force_sent = 0;
    int force_sent = 0;
    unsigned n,optional;
    ring_diag_service();
    if (ring_diag_header_line < sizeof(headers) / sizeof(headers[0])) {
        if(!ring_diag_header_line)ring_diag_optional_cursor=0u;
        const char *header = headers[ring_diag_header_line];
        if(header[0]=='@') {
            /* Integer-only formatting exposes the evaluated upper window
             * bound without enabling floating-point printf on the target. */
            length=snprintf(line,sizeof(line),
                "#EXIT_WINDOW,OBSERVE=%d;FIRST_MIN=%d;FIRST_MAX=%d;CONTINUE_YAW=%d;MS=%lu;CONFIRM=%u/%lums;NET_MIN=%d;SEARCH_MAX=%d;TARGET_ORIGINAL_STRAIGHT=360;GATE=HISTORICAL400_UNLOCK\r\n",
                (int)RING_EXIT_NET_MIN_DEG,(int)RING_EXIT_NET_MIN_DEG,
                (int)RING_EXIT_SEARCH_END_DEG,
                (int)RING_EXIT_CONTINUE_YAW_DEG,(unsigned long)RING_EXIT_TIMEOUT_MS,
                (unsigned)RING_EXIT_CANDIDATE_FRAMES,(unsigned long)RING_EXIT_CANDIDATE_MS,
                (int)RING_EXIT_NET_MIN_DEG,(int)RING_EXIT_SEARCH_END_DEG);
            if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length))
                ++ring_diag_header_line;
            ring_diag_service();return;
        }
        for (n = 0; header[n]; ++n) { }
        if (ring_diag_enqueue(header, n)) ++ring_diag_header_line;
        ring_diag_service();
        return;
    }
    {
        uint32_t pairkey=(uint32_t)d->control_version|((uint32_t)d->vision_version<<8)|((uint32_t)d->pair_ok<<16);
        if(!ring_diag_have_pair || pairkey!=ring_diag_pair_last_key) {
            length=snprintf(line,sizeof(line),"#PAIR,%u,%u,%u\r\n",
                (unsigned)d->control_version,(unsigned)d->vision_version,(unsigned)d->pair_ok);
            if(length>0 && (unsigned)length<sizeof(line) && ring_diag_count<RING_DIAG_QUEUE_LINES-1u &&
                ring_diag_enqueue(line,(unsigned)length)) {
                ring_diag_pair_last_key=pairkey;ring_diag_have_pair=1;
            }
        }
    }
    /* Keep the established R event/heartbeat behavior. Score changes are D
     * observations, not R events, so they cannot flood the legacy stream. */
    if (!ring_diag_have_last || elapsed >= RING_DIAG_PERIOD_MS || key != ring_diag_last_key) {
        length = snprintf(line, sizeof(line),
            "R,%lu,%lu,%lu,%lu,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%ld,%lu,%lu,%lu\r\n",
            (unsigned long)ring_diag_seq, (unsigned long)d->now_ms, (unsigned long)elapsed,
            (unsigned long)d->control_dt_ms,
            d->phase, d->route, d->raw_state, (unsigned)d->output_state,
            (unsigned)d->candidate, (unsigned)d->cross_veto, (unsigned)d->committed,
            (unsigned)d->path_valid, (unsigned)d->stop, (unsigned)d->fault, (unsigned)d->active,
            d->steer_error, steer_error, d->island_near, d->turn_deg, d->cap,
            d->requested_boost, d->applied_boost, d->original_target10, d->final_target10,
            d->original_turn10, d->final_turn10, speed_left, speed_right,
            (unsigned long)d->command_age_ms, (long)d->wheel_mrev,
            (unsigned long)d->detected, (unsigned long)d->exited, (unsigned long)ring_diag_dropped);
        if (length > 0 && (unsigned)length < sizeof(line) && ring_diag_enqueue(line, (unsigned)length)) {
            ring_diag_last_ms = d->now_ms;
            ring_diag_last_key = key;
            ring_diag_have_last = 1;
            ++ring_diag_seq;
        } else ++ring_diag_dropped;
    }
    optional=ring_diag_optional_select(d,detail_key);
    if(RING_TRACK_MODE && optional==RING_DIAG_M) {
        uint32_t mkey=(uint32_t)d->exit_force_stage|((uint32_t)d->exit_force_reason<<8)|
            ((uint32_t)(uint8_t)d->exit_force_side<<16);
        int new_epoch=!ring_diag_have_exit_force || d->command_epoch!=ring_diag_exit_force_epoch;
        int event=ring_diag_have_exit_force && (mkey!=ring_diag_exit_force_key ||
            (d->exit_force_stage && (new_epoch || d->detected!=ring_diag_exit_force_detected)));
        int interesting=d->exit_force_stage && d->phase>=RING_APPROACH && d->phase<=RING_RECOVER;
        int due=event || (d->exit_force_stage && (new_epoch ||
            (interesting && d->now_ms-ring_diag_exit_force_last_ms>=200u)));
        /* M is control-local telemetry. It neither enters the command
         * mailbox nor refreshes control freshness. A zero/ordinary state
         * emits only its one transition after an observed exit maneuver. */
        if(due && ring_diag_count<RING_DIAG_QUEUE_LINES-2u) {
            length=snprintf(line,sizeof(line),
                "M,%lu,%lu,%lu,%lu,%lu,%u,%u,%d,%d,%d,%d,%u,%d,%d,%lu,%lu\r\n",
                (unsigned long)ring_diag_exit_force_seq,(unsigned long)d->now_ms,
                (unsigned long)d->command_epoch,(unsigned long)d->command_sequence,
                (unsigned long)d->detected,(unsigned)d->exit_force_stage,
                (unsigned)d->exit_force_reason,(int)d->exit_force_side,
                (int)d->exit_force_start10,(int)d->exit_force_turn10,
                (int)d->exit_force_remaining10,(unsigned)d->exit_force_elapsed_ms,
                (int)d->measured_forward10,(int)d->final_turn10,
                (unsigned long)d->command_age_ms,(unsigned long)ring_diag_dropped);
            if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
                ring_diag_exit_force_epoch=d->command_epoch;
                ring_diag_exit_force_detected=d->detected;
                ring_diag_exit_force_key=mkey;ring_diag_exit_force_last_ms=d->now_ms;
                ring_diag_have_exit_force=1;exit_force_sent=1;++ring_diag_exit_force_seq;
            } else ++ring_diag_dropped;
        }
    }
    if(RING_TRACK_MODE && optional==RING_DIAG_K && !exit_force_sent) {
        uint32_t force_key=(uint32_t)d->force_active|((uint32_t)d->force_reason<<8)|
            ((uint32_t)d->release_reason<<16)|((uint32_t)d->run_requested<<24)|((uint32_t)d->standing<<25)|((uint32_t)d->maneuver_stage<<26);
        int interesting=(d->maneuver_stage>0u && d->maneuver_stage<3u) ||
            (d->active && d->maneuver_stage==3u) || d->phase==RING_ENTER || d->force_active ||
            (ring_diag_have_force && force_key!=ring_diag_force_key) ||
            (RING_TIMED_ENTRY && d->run_requested && d->standing &&
             d->now_ms-ring_diag_force_last_ms>=1000u);
        if(interesting && (!ring_diag_have_force || force_key!=ring_diag_force_key ||
            d->now_ms-ring_diag_force_last_ms>=200u) && ring_diag_count<RING_DIAG_QUEUE_LINES-2u) {
            length=snprintf(line,sizeof(line),"K,%lu,%lu,%lu,%lu,%u,%d,%d,%d,%d,%u,%lu,%lu,%u,%u,%u,%d,%d,%u,%d,%d,%u,%d,%d,%d,%d,%d,%ld,%ld,%lu\r\n",
                (unsigned long)ring_diag_force_seq,(unsigned long)d->now_ms,
                (unsigned long)d->command_epoch,(unsigned long)d->command_sequence,
                (unsigned)d->force_active,(int)d->force_side,(int)d->force_target10,
                (int)d->force_rate10,(int)d->final_turn10,(unsigned)d->force_reason,
                (unsigned long)d->command_age_ms,(unsigned long)ring_diag_dropped,
                (unsigned)d->release_reason,(unsigned)d->run_requested,(unsigned)d->standing,
                (int)d->original_target10,(int)d->final_target10,(unsigned)d->maneuver_stage,
                (int)d->maneuver_turn10,(int)d->maneuver_remaining10,(unsigned)d->maneuver_elapsed_ms,
                (int)d->held_target10,(int)d->speed_goal10,(int)d->measured_forward10,
                (int)d->pitch100,(int)d->speed_tilt100,(long)d->left_pwm,(long)d->right_pwm,
                (unsigned long)d->motion_age_ms);
            if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
                ring_diag_force_last_ms=d->now_ms;ring_diag_force_key=force_key;
                ring_diag_have_force=1;force_sent=1;++ring_diag_force_seq;
            }
        }
    }
    /* Exit-only documentation follows live M/K data and consumes a single
     * otherwise free optional slot. Boot headers have exactly the existing
     * byte/slot budget, so new text cannot postpone the first entry K. */
    if(RING_TRACK_MODE && optional==RING_DIAG_DOC && ring_diag_have_exit_force && d->exit_force_stage &&
       ring_diag_exit_force_header_line<2u && !force_sent && !exit_force_sent &&
       !ring_diag_count) {
        if(!ring_diag_exit_force_header_line) {
            static const char m_schema[]="#M,seq,ms,epoch,cmd,det,stage,reason,side,start10,turn10,rem10,elapsed,meas10,out10,age,drop;DEGx10;RPMx10;MS\r\n";
            length=(int)(sizeof(m_schema)-1u);
            if(ring_diag_enqueue(m_schema,(unsigned)length)) {
                ++ring_diag_exit_force_header_line;exit_force_sent=1;
            }
        } else {
            length=snprintf(line,sizeof(line),
                "#EXIT_FORCE,MAX_DEG=%d;EARLIEST=%d;TARGET=ENTRY_HEADING_PLUS_360;MS=%lu;STAGE=0:idle|1:turn|2:aligned|3:abort;BEEP=3x60ms_ON_60ms_GAP;END_BEEP=2;FORWARD_REQUIRED=1\r\n",
                (int)RING_EXIT_FORCE_DEG,(int)RING_EXIT_NET_MIN_DEG,
                (unsigned long)RING_EXIT_FORCE_TIMEOUT_MS);
            if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
                ++ring_diag_exit_force_header_line;exit_force_sent=1;
            }
        }
    }
    if(optional==RING_DIAG_J && d->mouth_valid && !force_sent && !exit_force_sent) {
        int fresh=!ring_diag_have_mouth || d->mouth_epoch!=ring_diag_mouth_epoch ||
            d->mouth_frame!=ring_diag_mouth_frame;
        int interesting=d->phase>=RING_APPROACH && d->phase<=RING_RECOVER;
        /* Current mouth/gate evidence is bounded at 5 Hz, including gate
         * flicker. First session observation is immediate. J does not alter
         * command validity, source identity, or the control watchdog. */
        int due=!ring_diag_have_mouth || d->mouth_epoch!=ring_diag_mouth_epoch ||
            (interesting && d->now_ms-ring_diag_mouth_last_ms>=200u) ||
            (!interesting && d->now_ms-ring_diag_mouth_last_ms>=RING_DIAG_PERIOD_MS);
        if(fresh && due && ring_diag_count<RING_DIAG_QUEUE_LINES-1u) {
            length=snprintf(line,sizeof(line),"J,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%u\r\n",
                (unsigned long)ring_diag_mouth_seq,(unsigned long)d->now_ms,(unsigned long)d->mouth_epoch,
                (unsigned long)d->mouth_command_sequence,(unsigned long)d->mouth_frame,
                (int)d->mouth_near_x,(int)d->mouth_near_y,(int)d->mouth_far_x,(int)d->mouth_far_y,
                (int)d->mouth_first_y,(int)d->mouth_progress,(unsigned)d->mouth_gate,(unsigned)d->mouth_ref,
                (unsigned)d->mouth_near_count,(unsigned)d->mouth_far_count,(unsigned)d->mouth_gap,
                (unsigned)d->control_version,(unsigned)d->vision_version,
                (unsigned long)d->mouth_age_ms,(unsigned long)ring_diag_dropped,
                (unsigned)d->mouth_near_missed,(unsigned)d->mouth_pass_reason);
            if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
                ring_diag_mouth_frame=d->mouth_frame;ring_diag_mouth_epoch=d->mouth_epoch;
                ring_diag_mouth_gate=d->mouth_gate;ring_diag_mouth_last_ms=d->now_ms;
                ring_diag_have_mouth=1;mouth_sent=1;++ring_diag_mouth_seq;
            }
        }
    }
    if (optional==RING_DIAG_E && d->entry_diagnostic_valid && !mouth_sent && !force_sent && !exit_force_sent) {
        uint32_t entry_key = ((uint32_t)d->phase << 8) | d->decision |
            ((uint32_t)(d->entry_flags & 7168u) << 6);
        unsigned previous_decision = ring_diag_entry_key & 255u;
        int new_epoch = !ring_diag_have_entry || d->entry_diagnostic_epoch != ring_diag_entry_epoch;
        int new_frame = new_epoch || d->entry_frame_sequence != ring_diag_entry_frame;
        int event = new_epoch || ((entry_key ^ ring_diag_entry_key) & 0x7ff00u) != 0u ||
            (d->decision != previous_decision && (d->decision >= 3u || previous_decision >= 3u));
        int interesting = d->active || d->decision || d->entry_flags ||
            (d->phase >= 1 && d->phase <= 5);
        int due = event || (interesting && d->now_ms - ring_diag_entry_last_ms >= RING_DIAG_ENTRY_PERIOD_MS);
        /* Major decisions precede routine D samples. MEASURED/HOLD changes
         * remain at 5 Hz, avoiding an E line for every branch flicker. */
        if (new_frame && due) {
            if (ring_diag_count >= RING_DIAG_QUEUE_LINES - 1u) ++ring_diag_dropped;
            else {
                length = snprintf(line, sizeof(line),
                    "E,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%d,%d,%lu,%lu\r\n",
                    (unsigned long)ring_diag_entry_seq, (unsigned long)d->now_ms,
                    (unsigned long)d->entry_diagnostic_epoch,
                    (unsigned long)d->entry_diagnostic_command_sequence,
                    (unsigned long)d->entry_frame_sequence,
                    (unsigned)d->entry_flags, (unsigned)d->hold_reject,
                    (unsigned)d->decision, (unsigned)d->lost_frames, (unsigned)d->path_age_ms,
                    (int)d->hold_turn10, (int)d->entry_turn10,
                    (unsigned long)d->entry_diagnostic_age_ms, (unsigned long)ring_diag_dropped);
                if (length > 0 && (unsigned)length < sizeof(line) && ring_diag_enqueue(line, (unsigned)length)) {
                    ring_diag_entry_epoch = d->entry_diagnostic_epoch;
                    ring_diag_entry_frame = d->entry_frame_sequence;
                    ring_diag_entry_key = entry_key;
                    ring_diag_entry_last_ms = d->now_ms;
                    ring_diag_have_entry = 1;
                    entry_sent = 1;
                    ++ring_diag_entry_seq;
                } else ++ring_diag_dropped;
            }
        }
    }
    if(optional==RING_DIAG_L && d->exit_diagnostic_valid && !entry_sent && !mouth_sent && !force_sent && !exit_force_sent) {
        uint32_t exit_key=(uint32_t)d->exit_candidate|((uint32_t)d->exit_latched<<1)|
            ((uint32_t)d->exit_confirmed<<2);
        int new_epoch=!ring_diag_have_exit || d->exit_diagnostic_epoch!=ring_diag_exit_epoch;
        int new_frame=new_epoch || d->exit_frame_sequence!=ring_diag_exit_frame;
        int event=ring_diag_have_exit && (new_epoch || exit_key!=ring_diag_exit_key);
        int interesting=(d->phase>=RING_APPROACH && d->phase<=RING_RECOVER) ||
            (!ring_diag_have_exit && exit_key) || event;
        int due=interesting && (new_epoch || event ||
            d->now_ms-ring_diag_exit_last_ms>=200u);
        /* L shares the existing one-line optional admission. State changes
         * are immediate; scalar reject/count updates are limited to 5 Hz. */
        if(new_frame && due) {
            if(ring_diag_count>=RING_DIAG_QUEUE_LINES-1u) ++ring_diag_dropped;
            else {
                length=snprintf(line,sizeof(line),
                    "L,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%d,%ld,%lu,%lu\r\n",
                    (unsigned long)ring_diag_exit_seq,(unsigned long)d->now_ms,
                    (unsigned long)d->exit_diagnostic_epoch,
                    (unsigned long)d->exit_diagnostic_command_sequence,
                    (unsigned long)d->exit_frame_sequence,(unsigned)d->exit_reject,
                    (unsigned)d->exit_candidate,(unsigned)d->exit_latched,
                    (unsigned)d->exit_confirmed,(unsigned)d->exit_candidate_frames,
                    (unsigned)d->exit_confirm_frames,(int)d->exit_lap10,
                    (long)d->exit_forward_mrev,(unsigned long)d->exit_diagnostic_age_ms,
                    (unsigned long)ring_diag_dropped);
                if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
                    ring_diag_exit_epoch=d->exit_diagnostic_epoch;ring_diag_exit_frame=d->exit_frame_sequence;
                    ring_diag_exit_key=exit_key;ring_diag_exit_last_ms=d->now_ms;
                    ring_diag_have_exit=1;exit_sent=1;++ring_diag_exit_seq;
                } else ++ring_diag_dropped;
            }
        }
    }
    if(optional==RING_DIAG_H && d->topology_valid && !exit_sent && !entry_sent && !mouth_sent && !force_sent && !exit_force_sent) {
        uint32_t hkey=(uint32_t)d->approach_stage|((uint32_t)d->left_entry<<8)|
            ((uint32_t)d->lap_valid<<9)|((uint32_t)(uint8_t)d->ring_side<<16);
        int new_epoch=!ring_diag_have_topology || d->topology_epoch!=ring_diag_topology_epoch;
        int new_frame=new_epoch || d->topology_frame!=ring_diag_topology_frame;
        int event=new_epoch || hkey!=ring_diag_topology_key;
        int interesting=d->approach_stage || d->active || (d->phase>=1 && d->phase<=5);
        if(new_frame && (event || (interesting && d->now_ms-ring_diag_topology_last_ms>=200u))) {
            if(ring_diag_count>=RING_DIAG_QUEUE_LINES-1u) ++ring_diag_dropped;
            else {
                length=snprintf(line,sizeof(line),"H,%lu,%lu,%lu,%lu,%lu,%d,%u,%u,%u,%d,%d,%ld,%ld,%lu,%lu,%u\r\n",
                    (unsigned long)ring_diag_topology_seq,(unsigned long)d->now_ms,
                    (unsigned long)d->topology_epoch,(unsigned long)d->topology_command_sequence,
                    (unsigned long)d->topology_frame,(int)d->ring_side,(unsigned)d->approach_stage,
                    (unsigned)d->left_entry,(unsigned)d->lap_valid,(int)d->near_row,(int)d->far_row,
                    (long)d->lap_origin_mdeg,(long)d->lap_mdeg,(unsigned long)d->topology_age_ms,
                    (unsigned long)ring_diag_dropped,(unsigned)d->mouth_source);
                if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
                    ring_diag_topology_epoch=d->topology_epoch;ring_diag_topology_frame=d->topology_frame;
                    ring_diag_topology_key=hkey;ring_diag_topology_last_ms=d->now_ms;
                    ring_diag_have_topology=1;topology_sent=1;++ring_diag_topology_seq;
                } else ++ring_diag_dropped;
            }
        }
    }
    if (optional==RING_DIAG_G && d->geometry_valid && !exit_sent && !entry_sent && !topology_sent && !mouth_sent && !force_sent && !exit_force_sent) {
        int new_epoch = !ring_diag_have_geometry || d->geometry_epoch != ring_diag_geometry_epoch;
        int new_frame = new_epoch || d->geometry_frame_sequence != ring_diag_geometry_frame;
        int interesting = d->active || d->branch_mode || d->inner_rows || d->associated_rows ||
            d->candidate_detected || d->candidate_allowed || (d->phase >= 1 && d->phase <= 5);
        int due = new_epoch || (interesting &&
            d->now_ms - ring_diag_geometry_last_ms >= RING_DIAG_GEOMETRY_PERIOD_MS);
        /* Geometry is bounded-rate telemetry. E decisions take precedence;
         * E/G/D admit at most one text record per poll after legacy R. */
        if (new_frame && due) {
            if (ring_diag_count >= RING_DIAG_QUEUE_LINES - 1u) ++ring_diag_dropped;
            else {
                length = snprintf(line, sizeof(line),
                    "G,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%d,%u,%lu,%lu\r\n",
                    (unsigned long)ring_diag_geometry_seq, (unsigned long)d->now_ms,
                    (unsigned long)d->geometry_epoch,
                    (unsigned long)d->geometry_command_sequence,
                    (unsigned long)d->geometry_frame_sequence,
                    (unsigned)d->branch_mode, (unsigned)d->inner_rows,
                    (unsigned)d->associated_rows, (unsigned)d->target_support,
                    (unsigned)d->strong_age_ms, (int)d->guide_x, (unsigned)d->guide_y,
                    (unsigned long)d->geometry_age_ms, (unsigned long)ring_diag_dropped);
                if (length > 0 && (unsigned)length < sizeof(line) && ring_diag_enqueue(line, (unsigned)length)) {
                    ring_diag_geometry_epoch = d->geometry_epoch;
                    ring_diag_geometry_frame = d->geometry_frame_sequence;
                    ring_diag_geometry_last_ms = d->now_ms;
                    ring_diag_have_geometry = 1;
                    geometry_sent = 1;
                    ++ring_diag_geometry_seq;
                } else ++ring_diag_dropped;
            }
        }
    }
    if (optional==RING_DIAG_D && d->diagnostic_valid) {
        int new_epoch = !ring_diag_have_detail || d->diagnostic_epoch != ring_diag_detail_epoch;
        int new_frame = new_epoch || d->frame_sequence != ring_diag_detail_frame;
        int event = new_epoch || detail_key != ring_diag_detail_key;
        int interesting = d->candidate_detected || d->candidate_allowed || d->active ||
            d->classification || (d->phase >= 1 && d->phase <= 5);
        int due = event || (interesting && (uint32_t)(d->frame_sequence - ring_diag_detail_frame) >= 2u) ||
            (!interesting && d->now_ms - ring_diag_detail_last_ms >= RING_DIAG_PERIOD_MS);
        /* Reserve one queue slot for R state/fault records. E/G/D share
         * one telemetry admission per poll. A deferred D reads the latest
         * available frame next time; it is not a replay of the skipped one.
         * Never wait when full; queue pressure remains visible in drop. */
        if (new_frame && due && !exit_sent && !entry_sent && !geometry_sent && !topology_sent && !mouth_sent && !force_sent && !exit_force_sent) {
            if (ring_diag_count >= RING_DIAG_QUEUE_LINES - 1u) ++ring_diag_dropped;
            else {
                length = snprintf(line, sizeof(line),
                    "D,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%d,%u,%d,%d,%u,%u,%lu,%lu\r\n",
                    (unsigned long)ring_diag_detail_seq, (unsigned long)d->now_ms,
                    (unsigned long)d->diagnostic_epoch, (unsigned long)d->diagnostic_command_sequence,
                    (unsigned long)d->frame_sequence, (unsigned)d->visual_dt_ms,
                    (unsigned)d->candidate_detected, (unsigned)d->candidate_allowed,
                    (unsigned)d->score, (unsigned)d->reject, (unsigned)d->confirm_frames,
                    (int)d->entry_side, (unsigned)d->path_source, (int)d->near_x, (int)d->look_x,
                    (unsigned)d->branch_ok, (unsigned)d->classification,
                    (unsigned long)d->diagnostic_age_ms, (unsigned long)ring_diag_dropped);
                if (length > 0 && (unsigned)length < sizeof(line) && ring_diag_enqueue(line, (unsigned)length)) {
                    ring_diag_detail_epoch = d->diagnostic_epoch;
                    ring_diag_detail_frame = d->frame_sequence;
                    ring_diag_detail_key = detail_key;
                    ring_diag_detail_last_ms = d->now_ms;
                    ring_diag_have_detail = 1;
                    ++ring_diag_detail_seq;
                } else ++ring_diag_dropped;
            }
        }
    }
    /* One compact yaw-health record per second, including standby. K
     * keeps its entry priority; no TX wait or extra ISR is introduced. */
    if((!ring_diag_have_bias || d->now_ms-ring_diag_bias_last_ms>=1000u) &&
       !force_sent && !exit_force_sent && ring_diag_count<RING_DIAG_QUEUE_LINES-2u) {
        length=snprintf(line,sizeof(line),
            "B,%lu,%lu,%d,%d,%d,%u,%lu,%lu,%u,%d,%lu\r\n",
            (unsigned long)ring_diag_bias_seq,(unsigned long)d->now_ms,
            (int)d->yaw_raw,(int)d->yaw_bias_raw,(int)d->force_rate10,
            (unsigned)d->yaw_calibrated,
            (unsigned long)(d->yaw_calibrated?d->now_ms-d->yaw_calibration_ms:UINT32_MAX),
            (unsigned long)d->yaw_calibration_count,(unsigned)d->yaw_window_state,
            (int)d->entry_turn_cap10,(unsigned long)ring_diag_dropped);
        if(length>0 && (unsigned)length<sizeof(line) && ring_diag_enqueue(line,(unsigned)length)) {
            ring_diag_bias_last_ms=d->now_ms;ring_diag_have_bias=1;
            ++ring_diag_bias_seq;
        }
    }
    ring_diag_service();
    ring_diag_image_poll(d);
}

#endif
