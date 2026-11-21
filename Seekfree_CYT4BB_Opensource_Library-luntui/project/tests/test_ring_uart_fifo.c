#include <string.h>
#include <stdlib.h>
#include "zf_driver_uart.h"
/* Keep the existing 16-byte stub file untouched. Exercise actual UART depths
 * and the hard service limit using local getter/writer replacements. */
static unsigned test_fifo_capacity=128u;
static uint32_t test_fifo_size(volatile stc_SCB_t *p) {(void)p;return test_fifo_capacity;}
static void test_fifo_write(volatile stc_SCB_t *p,uint8_t value)
{
    if(p->used<test_fifo_capacity && uart_capture_count<32767u) {
        ++p->used;uart_capture[uart_capture_count++]=(char)value;
    }
}
#define Cy_SCB_GetFifoSize test_fifo_size
#define Cy_SCB_WriteTxFifo test_fifo_write
#include "ring_uart_diag.h"
stc_SCB_t uart_port;
char uart_capture[32768];
unsigned uart_capture_count;
static ring_runtime_diagnostics_t sample;
const ring_runtime_diagnostics_t *ring_runtime_diagnostics(void){return &sample;}
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL FIFO line %d\n",__LINE__);exit(1);}}while(0)
static void reset_state(void)
{
    memset(&sample,0,sizeof(sample));memset(ring_diag_queue,0,sizeof(ring_diag_queue));
    uart_capture_count=0;uart_port.used=0;
    ring_diag_head=ring_diag_count=ring_diag_header_line=0;
    ring_diag_last_ms=ring_diag_last_key=ring_diag_seq=ring_diag_dropped=0;
    ring_diag_have_last=ring_diag_have_detail=0;
    ring_diag_optional_cursor=0u;
    ring_diag_detail_seq=ring_diag_detail_frame=ring_diag_detail_epoch=ring_diag_detail_key=ring_diag_detail_last_ms=0;
    ring_diag_entry_seq=ring_diag_entry_frame=ring_diag_entry_epoch=ring_diag_entry_key=ring_diag_entry_last_ms=0;
    ring_diag_have_entry=0;
    ring_diag_geometry_seq=ring_diag_geometry_frame=ring_diag_geometry_epoch=ring_diag_geometry_last_ms=0;
    ring_diag_have_geometry=0;
    ring_diag_topology_seq=ring_diag_topology_frame=ring_diag_topology_epoch=ring_diag_topology_last_ms=ring_diag_topology_key=0;
    ring_diag_have_topology=0;
    ring_diag_mouth_seq=ring_diag_mouth_frame=ring_diag_mouth_epoch=ring_diag_mouth_last_ms=0;
    ring_diag_bias_seq=ring_diag_bias_last_ms=0;ring_diag_have_bias=0;
    ring_diag_mouth_gate=0;ring_diag_have_mouth=0;ring_diag_have_pair=0;ring_diag_pair_last_key=0;ring_diag_image_sequence=ring_diag_image_epoch=ring_diag_ack_sequence=0;
    ring_diag_exit_seq=ring_diag_exit_frame=ring_diag_exit_epoch=ring_diag_exit_last_ms=ring_diag_exit_key=0;
    ring_diag_have_exit=0;
    sample.cap=-1;sample.island_near=-1;sample.raw_state=1;sample.output_state=1;
    sample.near_x=sample.look_x=-1;
    sample.guide_x=-1;sample.guide_y=255u;
}
static void poll_frame(unsigned frame)
{
    sample.now_ms+=46u;sample.control_dt_ms=46u;
    sample.frame_sequence=frame;sample.diagnostic_command_sequence=frame*2u;
    sample.entry_frame_sequence=frame;sample.entry_diagnostic_command_sequence=frame*2u;
    sample.geometry_frame_sequence=frame;sample.geometry_command_sequence=frame*2u;
    sample.topology_frame=frame;sample.topology_command_sequence=frame*2u;
    sample.mouth_frame=frame;sample.mouth_command_sequence=frame*2u;
    sample.exit_frame_sequence=frame;sample.exit_diagnostic_command_sequence=frame*2u;
    uart_port.used=0;ring_uart_diag_poll(0,100,100);
}
/* Optional state records are delivered by fair admission rather than a
 * fixed E/G ordering. Keep each state unchanged until its bounded send. */
static void wait_record(uint32_t *sequence,unsigned before,unsigned max_polls)
{
    unsigned i;
    for(i=0;i<max_polls && *sequence==before;++i)poll_frame(sample.frame_sequence+1u);
    CHECK(*sequence==before+1u);
}
static unsigned count_lines(const char *prefix,unsigned expected_commas)
{
    unsigned count=0;const char *p=uart_capture;
    while((p=strstr(p,prefix))!=NULL) {
        const char *end;
        p+=2;end=strstr(p,"\r\n");const char *q;unsigned commas=0;
        /* A sample may end part-way through the current bounded FIFO write;
         * only complete wire records have a field-count contract. */
        if(end==NULL)break;
        for(q=p;q<end;++q) if(*q==',') ++commas;
        CHECK(commas==expected_commas);++count;p=end;
    }
    return count;
}
int main(void)
{
    unsigned i,before,details;
    reset_state();
    for(i=0;i<100;++i)poll_frame(i);
    uart_capture[uart_capture_count]=0;
    CHECK(strstr(uart_capture,"#RING,36,UART1,115200")!=NULL);
    CHECK(strstr(uart_capture,"IMAGE=IPM")!=NULL);
    CHECK(strstr(uart_capture,"YAW_SIGN=" RING_DIAG_STRINGIFY(RING_YAW_SIGN))!=NULL);
    CHECK(strstr(uart_capture,"#D,seq,ms,epoch,cmd,frame,vdt,cand_det,cand_ok")!=NULL);
    CHECK(strstr(uart_capture,"#E,seq,ms,epoch,cmd,frame,entry_flags,hold_reject")!=NULL);
    CHECK(strstr(uart_capture,"#G,seq,ms,epoch,cmd,frame,mode,rows,assoc,support")!=NULL);
    CHECK(strstr(uart_capture,"#ENTRY_ASSIST,MIN_ERROR=" RING_DIAG_STRINGIFY(RING_ASSIST_MIN_ERROR))!=NULL);
    CHECK(strstr(uart_capture,"#EXIT_WINDOW,OBSERVE=400;FIRST_MIN=400;FIRST_MAX=430;CONTINUE_YAW=30;MS=4000;CONFIRM=3/90ms;NET_MIN=400;SEARCH_MAX=430;TARGET_ORIGINAL_STRAIGHT=360;GATE=HISTORICAL400_UNLOCK")!=NULL);
    CHECK(strstr(uart_capture,"TOTAL_MS=" RING_DIAG_STRINGIFY(RING_ASSIST_TOTAL_MS))!=NULL);
    CHECK(count_lines("\r\nR,",33u)>=4u);
    CHECK(strstr(uart_capture,"\r\nD,")==NULL);
    before=uart_capture_count;uart_port.used=test_fifo_capacity;
    sample.phase=2;sample.active=1;sample.path_valid=1;sample.now_ms+=46u;
    ring_uart_diag_poll(41,60,140);CHECK(uart_capture_count==before);
    for(i=0;i<32;++i){sample.now_ms+=46;uart_port.used=test_fifo_capacity;sample.phase=i%2?1:2;ring_uart_diag_poll(0,100,100);}
    CHECK(ring_diag_dropped>0 && ring_diag_count<=RING_DIAG_QUEUE_LINES);
    for(i=0;i<160;++i)poll_frame(i);
    CHECK(ring_diag_count<RING_DIAG_QUEUE_LINES);
    /* Per service bound is independent of physical depth. No FIFO waits. */
    reset_state();test_fifo_capacity=128u;
    {char bytes[200];memset(bytes,'x',sizeof(bytes));CHECK(ring_diag_enqueue(bytes,sizeof(bytes)));}
    CHECK(ring_diag_service()==64u && uart_port.used==64u);
    CHECK(ring_diag_service()==64u && uart_port.used==128u);
    CHECK(ring_diag_service()==0u);
    test_fifo_capacity=16u;uart_port.used=0;
    CHECK(ring_diag_service()==16u);CHECK(ring_diag_service()==0u);
    test_fifo_capacity=64u;uart_port.used=0;
    CHECK(ring_diag_service()==56u);CHECK(ring_diag_count==0u);
    reset_state();test_fifo_capacity=128u;
    for(i=0;i<20;++i)poll_frame(i);
    sample.diagnostic_valid=1;sample.diagnostic_epoch=7;sample.visual_dt_ms=23;
    sample.candidate_detected=sample.candidate_allowed=1;sample.score=5;
    sample.entry_side=-1;sample.path_source=1;sample.near_x=111;sample.look_x=128;
    sample.branch_ok=1;sample.classification=2;
    for(i=10;i<110;++i)poll_frame(i);
    sample.diagnostic_valid=0;
    for(i=0;i<30;++i)poll_frame(110u);
    uart_capture[uart_capture_count]=0;
    details=count_lines("\r\nD,",19u);
    CHECK(details==50u);CHECK(ring_diag_dropped==0u);
    CHECK(strstr(uart_capture,",23,1,1,5,0,0,-1,1,111,128,1,2,")!=NULL);
    CHECK(count_lines("\r\nR,",33u)<10u);
    /* Stable frame cannot be logged again even when polled repeatedly. */
    before=ring_diag_detail_seq;sample.diagnostic_valid=1;
    for(i=0;i<20;++i)poll_frame(108u);
    CHECK(ring_diag_detail_seq==before);
    /* Candidate/branch transition is immediate even on the odd frame. */
    sample.branch_ok=0;sample.score=6;
    poll_frame(109u);CHECK(ring_diag_detail_seq==before+1u);
    /* Ordinary/undetected geometry still reports rejection/vdt at 1 Hz. */
    reset_state();sample.diagnostic_valid=1;sample.diagnostic_epoch=9;
    sample.visual_dt_ms=65535u;sample.reject=1u;
    for(i=0;i<100;++i)poll_frame(i);
    for(i=0;i<20;++i){uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nD,",19u)==4u);
    CHECK(ring_diag_dropped==0u);
    before=ring_diag_detail_seq;sample.raw_state=2;
    poll_frame(100u);CHECK(ring_diag_detail_seq==before+1u);
    before=ring_diag_detail_seq;sample.score=4;
    poll_frame(101u);CHECK(ring_diag_detail_seq==before);
    /* E is an independent 5 Hz stream; repeated polls do not reprint it.
     * Major decisions are immediate, minor MEASURED/HOLD flicker is bounded. */
    reset_state();test_fifo_capacity=128u;
    sample.entry_diagnostic_valid=1;sample.entry_diagnostic_epoch=12u;
    sample.phase=2;sample.active=1;sample.decision=1;sample.entry_flags=75u;
    sample.hold_turn10=123;sample.entry_turn10=-321;sample.path_age_ms=248;sample.lost_frames=5;
    for(i=0;i<100u;++i)poll_frame(i);
    uart_capture[uart_capture_count]=0;
    {unsigned complete=count_lines("\r\nE,",14u);printf("E cadence count=%u drop=%lu\n",complete,(unsigned long)ring_diag_dropped);CHECK(complete>=17u && complete<=19u && ring_diag_dropped==0u);}
    before=ring_diag_entry_seq;
    sample.entry_frame_sequence=ring_diag_entry_frame;
    for(i=0;i<20u;++i){sample.now_ms+=46u;uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    CHECK(ring_diag_entry_seq==before);
    sample.decision=6;sample.hold_reject=8u;
    poll_frame(100u);wait_record(&ring_diag_entry_seq,before,4u);
    CHECK(strstr(uart_capture,",75,0,1,5,248,123,-321,")!=NULL);
    /* v11 active/direction/spent changes are events even when the assist
     * decision number is unchanged. Preserve the original hold rejection. */
    before=ring_diag_entry_seq;sample.decision=11u;sample.entry_flags=1024u;sample.hold_reject=12u;
    poll_frame(sample.frame_sequence+1u);wait_record(&ring_diag_entry_seq,before,4u);
    before=ring_diag_entry_seq;sample.entry_flags=3072u;
    poll_frame(sample.frame_sequence+1u);wait_record(&ring_diag_entry_seq,before,4u);
    before=ring_diag_entry_seq;sample.entry_flags=4096u;sample.decision=13u;
    poll_frame(sample.frame_sequence+1u);wait_record(&ring_diag_entry_seq,before,4u);
    /* Routine D yields to E; sustained realistic/worst-width observations
     * fit the existing 128-byte/poll budget without a growing queue. */
    reset_state();sample.diagnostic_valid=sample.entry_diagnostic_valid=1;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=4294967295u;
    sample.candidate_detected=sample.candidate_allowed=1;sample.classification=2u;
    sample.phase=2;sample.active=1;sample.entry_flags=8191u;sample.hold_reject=63u;
    sample.visual_dt_ms=65535u;sample.reject=65535u;sample.score=sample.confirm_frames=255u;
    sample.near_x=sample.look_x=187;sample.path_age_ms=65535u;sample.lost_frames=255u;
    sample.entry_turn10=-32768;sample.hold_turn10=32767;sample.now_ms=4000000000u;
    for(i=0;i<200u;++i){sample.branch_ok=(uint8_t)(i&1u);sample.decision=(uint8_t)(1u+(i&1u));poll_frame(4000000000u+i);}
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    CHECK(uart_capture_count<22000u); /* <2.4 KB/s over 9.2 s, headers included. */
    printf("UART pressure sample: %u bytes / 9200 ms, queue=%u, drop=%lu\n",
        uart_capture_count,ring_diag_count,(unsigned long)ring_diag_dropped);
    before=ring_diag_dropped;uart_port.used=test_fifo_capacity;sample.decision=6u;
    for(i=0;i<40u;++i){sample.now_ms+=230u;sample.entry_frame_sequence++;sample.frame_sequence++;ring_uart_diag_poll(0,100,100);}
    CHECK(ring_diag_dropped>before && ring_diag_count<=RING_DIAG_QUEUE_LINES);
    for(i=0;i<200u;++i)poll_frame(4000000300u+i);
    CHECK(ring_diag_count<RING_DIAG_QUEUE_LINES);
    /* Independent G cadence, sentinel coordinates and fixed-frame dedup. */
    reset_state();sample.geometry_valid=1;sample.geometry_epoch=14u;
    sample.branch_mode=2u;sample.inner_rows=35u;sample.associated_rows=31u;sample.target_support=5u;
    sample.strong_age_ms=123u;sample.guide_x=151;sample.guide_y=97u;
    for(i=0;i<100u;++i)poll_frame(i);
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nG,",14u)==10u && ring_diag_dropped==0u);
    CHECK(strstr(uart_capture,",2,35,31,5,123,151,97,")!=NULL);
    before=ring_diag_geometry_seq;
    sample.geometry_frame_sequence=ring_diag_geometry_frame;
    for(i=0;i<20u;++i){sample.now_ms+=46u;uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    CHECK(ring_diag_geometry_seq==before);
    /* E/G share fair admission: each pending fresh state appears within5 polls. */
    sample.entry_diagnostic_valid=1;sample.entry_diagnostic_epoch=14u;sample.decision=6u;
    {unsigned entry_before=ring_diag_entry_seq;
     poll_frame(100u);wait_record(&ring_diag_entry_seq,entry_before,4u);}
    sample.guide_x=-1;sample.guide_y=255u;
    before=ring_diag_geometry_seq;
    poll_frame(sample.frame_sequence+1u);wait_record(&ring_diag_geometry_seq,before,9u);
    /* R+D+E+G with wide values and every-frame branch/partial flicker still
     * admits only one optional line per poll, below 128 bytes/poll capacity. */
    reset_state();sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=1u;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=4294967295u;
    sample.candidate_detected=sample.candidate_allowed=1;sample.classification=2u;
    sample.phase=2;sample.active=1;sample.entry_flags=8191u;sample.hold_reject=63u;
    sample.visual_dt_ms=65535u;sample.reject=65535u;sample.score=sample.confirm_frames=255u;
    sample.near_x=sample.look_x=187;sample.path_age_ms=65535u;sample.lost_frames=255u;
    sample.entry_turn10=-32768;sample.hold_turn10=32767;sample.now_ms=4000000000u;
    sample.inner_rows=sample.associated_rows=120u;sample.target_support=7u;
    sample.strong_age_ms=65535u;sample.guide_x=187;sample.guide_y=119u;
    for(i=0;i<200u;++i){sample.branch_ok=(uint8_t)(i&1u);sample.branch_mode=(uint8_t)(1u+(i&1u));sample.decision=(uint8_t)(1u+(i&1u));poll_frame(4000000000u+i);}
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nG,",14u)>10u && count_lines("\r\nE,",14u)>20u);
    printf("UART RDEG pressure sample: %u bytes / 9200 ms, queue=%u, drop=%lu\n",
        uart_capture_count,ring_diag_count,(unsigned long)ring_diag_dropped);
    before=ring_diag_dropped;uart_port.used=test_fifo_capacity;sample.decision=6u;
    for(i=0;i<40u;++i){sample.now_ms+=460u;sample.entry_frame_sequence++;sample.geometry_frame_sequence++;sample.frame_sequence++;ring_uart_diag_poll(0,100,100);}
    CHECK(ring_diag_dropped>before && ring_diag_count<=RING_DIAG_QUEUE_LINES);
    for(i=0;i<200u;++i)poll_frame(4000000300u+i);
    CHECK(ring_diag_count<RING_DIAG_QUEUE_LINES);
    /* A stable active assist uses the same E/G cadence, not one event per
     * frame. Verify source5/mode5 reach the unchanged text layouts. */
    reset_state();sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=1u;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=23u;
    sample.phase=2;sample.active=1;sample.path_source=sample.branch_mode=5u;
    sample.decision=11u;sample.entry_flags=3072u;sample.hold_reject=12u;
    sample.visual_dt_ms=46u;sample.path_age_ms=248u;sample.strong_age_ms=248u;
    for(i=0;i<100u;++i)poll_frame(i);
    CHECK(ring_diag_dropped==0u && ring_diag_count<=2u);
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nE,",14u)<25u && count_lines("\r\nE,",14u)>15u);
    CHECK(count_lines("\r\nG,",14u)>5u);
    CHECK(strstr(uart_capture,",3072,12,11,0,248,")!=NULL);
    /* Production H adds topology at 5 Hz, sharing the same one-record
     * admission with E/G/D. Largest text values still fit the old service. */
    reset_state();sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=sample.topology_valid=1u;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=sample.topology_epoch=4294967295u;
    sample.candidate_detected=sample.candidate_allowed=1;sample.classification=2u;
    sample.phase=2;sample.active=1;sample.entry_flags=8191u;sample.hold_reject=63u;
    sample.visual_dt_ms=65535u;sample.reject=65535u;sample.score=sample.confirm_frames=255u;
    sample.near_x=sample.look_x=187;sample.path_age_ms=65535u;sample.lost_frames=255u;
    sample.entry_turn10=-32768;sample.hold_turn10=32767;sample.now_ms=4000000000u;
    sample.inner_rows=sample.associated_rows=120u;sample.target_support=7u;
    sample.strong_age_ms=65535u;sample.guide_x=187;sample.guide_y=119u;
    sample.ring_side=-1;sample.approach_stage=3;sample.left_entry=sample.lap_valid=1;
    sample.near_row=sample.far_row=119;sample.lap_origin_mdeg=-2147483647;sample.lap_mdeg=2147483647;
    for(i=0;i<200u;++i){sample.branch_ok=(uint8_t)(i&1u);sample.branch_mode=(uint8_t)(1u+(i&1u));sample.decision=(uint8_t)(1u+(i&1u));poll_frame(4000000000u+i);}
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nH,",16u)>20u && count_lines("\r\nE,",14u)>20u);
    CHECK(count_lines("\r\nG,",14u)>5u && count_lines("\r\nD,",19u)>5u);
    printf("UART RDEGH pressure sample: %u bytes / 9200 ms, queue=%u, drop=%lu\n",
        uart_capture_count,ring_diag_count,(unsigned long)ring_diag_dropped);
    sample.mouth_valid=1u;sample.mouth_epoch=4294967295u;
    sample.mouth_near_x=sample.mouth_far_x=187;sample.mouth_near_y=sample.mouth_far_y=119;
    sample.mouth_first_y=119;sample.mouth_progress=127;sample.mouth_gate=8191;
    sample.mouth_ref=1;sample.mouth_near_count=sample.mouth_far_count=sample.mouth_gap=15;
    sample.control_version=sample.vision_version=RING_PAIR_VERSION;sample.pair_ok=1;
    uart_capture_count=0;
    for(i=0;i<200u;++i){sample.branch_ok=(uint8_t)(i&1u);sample.mouth_gate=(uint16_t)(8190u+(i&1u));poll_frame(4000000600u+i);}
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nJ,",22u)>20u && count_lines("\r\nE,",14u)>20u);
    CHECK(count_lines("\r\nH,",16u)>20u && count_lines("\r\nD,",19u)>5u);
    CHECK(strstr(uart_capture,"#PAIR,36,36,1")!=NULL);
    printf("UART RDEGHJ pressure sample: %u bytes / 9200 ms, queue=%u, drop=%lu\n",
        uart_capture_count,ring_diag_count,(unsigned long)ring_diag_dropped);
    /* L is silent during normal tracking and uses one optional admission.
     * Counts/reject flicker cannot flood it; latch transitions are visible. */
    reset_state();test_fifo_capacity=128u;
    sample.exit_diagnostic_valid=1;sample.exit_diagnostic_epoch=1;
    for(i=0;i<100u;++i)poll_frame(i);
    uart_capture[uart_capture_count]=0;
    CHECK(strstr(uart_capture,"#L,seq,ms,epoch,cmd,frame,reject,candidate")!=NULL);
    CHECK(strstr(uart_capture,"\r\nL,")==NULL);
    sample.phase=RING_CIRCLE;sample.active=1;sample.exit_reject=63;
    sample.exit_lap10=3800;sample.exit_forward_mrev=-12345;
    for(i=0;i<100u;++i) {
        sample.exit_reject=(uint16_t)(i&63u);sample.exit_candidate_frames=(uint8_t)i;
        sample.exit_confirm_frames=(uint8_t)(i%12u);poll_frame(i+100u);
    }
    uart_capture[uart_capture_count]=0;
    details=count_lines("\r\nL,",15u);
    CHECK(details>=18u && details<=24u && ring_diag_dropped==0u);
    CHECK(strstr(uart_capture,",3800,-12345,")!=NULL);
    /* Existing frame replay cannot create a second L state event. */
    sample.exit_latched=1;before=ring_diag_exit_seq;poll_frame(201u);
    wait_record(&ring_diag_exit_seq,before,4u);before=ring_diag_exit_seq;
    sample.exit_latched=0;
    sample.now_ms+=10u;uart_port.used=0;ring_uart_diag_poll(0,100,100);
    CHECK(ring_diag_exit_seq==before);
    poll_frame(sample.frame_sequence+1u);wait_record(&ring_diag_exit_seq,before,4u);
    before=ring_diag_exit_seq;sample.exit_confirmed=1;
    poll_frame(sample.frame_sequence+1u);wait_record(&ring_diag_exit_seq,before,4u);
    /* Retained terminal evidence emits a transition once, no ordinary L heartbeat. */
    sample.phase=RING_COOLDOWN;sample.active=0;sample.exit_candidate=0;
    poll_frame(sample.frame_sequence+1u);before=ring_diag_exit_seq;
    for(i=0;i<40u;++i)poll_frame(sample.frame_sequence+1u);
    CHECK(ring_diag_exit_seq==before);
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    puts("PASS: bounded FIFO 16/64/128, R33/D19/E14/G14, fresh-frame cadence, fair optional admission, bandwidth/backpressure and drops");
    return 0;
}
