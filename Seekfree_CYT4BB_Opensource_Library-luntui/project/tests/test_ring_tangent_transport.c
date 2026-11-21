#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ring_link.h"
#include "ring_runtime.h"
#include "ring.h"
#include "ring_tangent_test_geometry.h"
#include "../code/ring_vision.c"
/* Include the real bounded serializer with the existing host UART stub. */
#include "ring_uart_diag.h"
stc_SCB_t uart_port;
char uart_capture[32768];
unsigned uart_capture_count;
static unsigned checks;
#define CHECK(c) do { ++checks; if(!(c)) { fprintf(stderr,"FAIL tangent transport line %d: %s\n",__LINE__,#c);exit(1); } } while(0)
static uint32_t test_command_sequence,test_epoch,test_h_sequence;
static float target,turn;
static uint8_t pixels[RING_H][RING_W];
static void ticks(unsigned n) { while(n--) (void)ring_runtime_tick(0,1); }
static void apply(int state) { target=123.0f;turn=17.0f;ring_runtime_apply(&target,&turn,0.25f,state); }
static void command(int phase,int cap,unsigned detected,int state)
{
    ring_packet_t p={{0}};
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
    p.word[RC_CAP]=ring_command_cap_turn(cap,0);p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
    p.word[RC_DETECTED]=(int32_t)detected;
    p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,0,1,24,80,1);
    ring_link_write(ring_command_mailbox,&p,&test_command_sequence);apply(state);
}
static void hpacket(uint32_t flags,uint32_t cmd)
{
    ring_packet_t p={{0}};
    p.word[RL_MAGIC]=RING_TOPOLOGY_DIAGNOSTIC_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
    p.word[RH_COMMAND_SEQUENCE]=(int32_t)cmd;p.word[RH_FRAME]=22;
    p.word[RH_FLAGS]=(int32_t)flags;p.word[RH_ORIGIN_MDEG]=-123456;p.word[RH_LAP_MDEG]=45678;
    ring_link_write(ring_topology_diagnostic_mailbox,&p,&test_h_sequence);
}
static void unchanged(void)
{
    CHECK(target==123.0f && turn==17.0f);
    CHECK(!ring_runtime_diagnostics()->active && !ring_runtime_watchdog_stop());
}
static void runtime_contract(void)
{
    int event;
    ring_runtime_init(0.1f);ring_runtime_set_operator(1,0);ticks(2100);apply(1);
    test_epoch=ring_runtime_diagnostics()->command_epoch;
    command(RING_APPROACH,-1,1,3);unchanged();
    event=ring_runtime_tick(0,1);CHECK(event==1);
    ticks(250);command(RING_APPROACH,-1,1,3);unchanged();
    CHECK(ring_runtime_tick(0,1)==-1);ticks(400);apply(3);unchanged();
    /* Waiting recognition never arms the ring command watchdog. */
    command(RING_APPROACH,100,2,3);unchanged();
    CHECK(ring_runtime_diagnostics()->detected==1u);
    command(RING_ENTER,90,1,3);CHECK(ring_runtime_diagnostics()->active && target==90.0f);
    hpacket((uint32_t)ring_topology_flags(-1,RING_TANGENT_FAR_READY,1,1,95,58),test_command_sequence);
    apply(3);CHECK(ring_runtime_diagnostics()->topology_valid);
    CHECK(ring_runtime_diagnostics()->ring_side==-1 && ring_runtime_diagnostics()->far_row==58);
    CHECK(ring_runtime_diagnostics()->lap_origin_mdeg==-123456 && ring_runtime_diagnostics()->lap_mdeg==45678);
    hpacket((uint32_t)ring_topology_flags(-1,RING_TANGENT_FAR_READY,1,1,95,58)|(2u<<23),test_command_sequence);
    apply(3);CHECK(ring_runtime_diagnostics()->topology_valid && ring_runtime_diagnostics()->mouth_source==2);
    hpacket((uint32_t)ring_topology_flags(1,0,0,0,-1,-1)|(3u<<23),test_command_sequence);
    apply(3);CHECK(ring_runtime_diagnostics()->mouth_source==2 && ring_runtime_diagnostics()->ring_side==-1);
    hpacket((uint32_t)ring_topology_flags(1,0,0,0,-1,-1)|(1u<<25),test_command_sequence);
    apply(3);CHECK(ring_runtime_diagnostics()->mouth_source==2 && ring_runtime_diagnostics()->ring_side==-1);
    {
        ring_packet_t p={{0}};
        uint32_t ds=0,gs=0;
        p.word[RL_MAGIC]=RING_DIAGNOSTIC_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
        p.word[RD_COMMAND_SEQUENCE]=(int32_t)test_command_sequence;p.word[RD_FRAME]=23;
        p.word[RD_FLAGS]=ring_diagnostic_flags(1,1,6,4,-1,6,1,2);
        p.word[RD_POINTS]=ring_diagnostic_points(112,128);
        ring_link_write(ring_diagnostic_mailbox,&p,&ds);
        p=(ring_packet_t){{0}};p.word[RL_MAGIC]=RING_GEOMETRY_DIAGNOSTIC_MAGIC;
        p.word[RL_EPOCH]=(int32_t)test_epoch;p.word[RG_COMMAND_SEQUENCE]=(int32_t)test_command_sequence;
        p.word[RG_FRAME]=23;p.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(6,14,8,3);
        p.word[RG_GUIDE]=ring_geometry_diagnostic_guide(128,97);
        ring_link_write(ring_geometry_diagnostic_mailbox,&p,&gs);apply(3);
        CHECK(ring_runtime_diagnostics()->diagnostic_valid && ring_runtime_diagnostics()->path_source==6);
        CHECK(ring_runtime_diagnostics()->geometry_valid && ring_runtime_diagnostics()->branch_mode==6);
        CHECK(ring_runtime_diagnostics()->branch_ok && target==90.0f);
    }
    /* Every telemetry channel may be corrupt/fresh while control goes stale. */
    ticks(RING_LINK_TIMEOUT_MS+1u);
    hpacket(0xffffffffu,test_command_sequence);
    memset(&ring_image_mailbox,255,sizeof(ring_image_mailbox));
    memset(ring_image_ack_mailbox,255,sizeof(ring_image_ack_mailbox));
    apply(3);CHECK(!ring_runtime_watchdog_stop() && target==123.0f && turn==17.0f);
    CHECK(!ring_runtime_diagnostics()->active && ring_runtime_diagnostics()->release_reason==4);
    CHECK(!ring_runtime_diagnostics()->topology_valid);
    command(RING_NORMAL,-1,1,1);unchanged();
    hpacket((uint32_t)ring_topology_flags(1,0,0,0,-1,-1),test_command_sequence-2u);
    apply(1);unchanged();CHECK(!ring_runtime_diagnostics()->topology_valid);
}
static void image_contract(void)
{
    ring_image_packet_t p;
    ring_packet_t ack;
    uint8_t rebuilt[RING_IMAGE_BYTES];
    uint32_t ack_seq=0,offset,part,initial_crc;
    unsigned x,y,i,before;
    epoch=test_epoch;command_sequence=100;capture_reset();ring_init();
    motion.ready=1;motion.now_ms=5000;motion.yaw_deg=15.0f;
    ring_result.phase=RING_APPROACH;ring_result.approach_stage=RING_TANGENT_NEAR_SEEN;
    ring_result.branch_ok=1;ring_result.path_source=1;ring_result.route=1;
    ring_result.near_row=80;ring_result.far_row=40;
    ring_result.mouth_far_x=ring_result.mouth_far_y=-1;
    for(y=0;y<RING_H;++y) for(x=0;x<RING_W;++x) pixels[y][x]=(uint8_t)(((x+3u*y)%7u)==0u?255:0);
    ring_result.frame_sequence=10;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==1u);
    initial_crc=(uint32_t)captures[0].meta[RI_CRC32];
    CHECK(initial_crc==ring_image_crc32(captures[0].bits,RING_IMAGE_BYTES));
    /* Camera reuse after return cannot mutate the frozen input. */
    memset(pixels,0,sizeof(pixels));
    ring_result.approach_stage=RING_TANGENT_NEAR_PASSED;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.phase=RING_ENTER;ring_result.approach_stage=RING_TANGENT_FAR_READY;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.branch_ok=0;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==4u);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,2);
    CHECK(capture_count==5u && capture_dropped==0u);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,3);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,5);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,6);
    CHECK(capture_count==8u && capture_dropped==1u);
    capture_service();CHECK(!capture_pending_sequence); /* active: no upload */
    ring_result.phase=RING_NORMAL;capture_service();
    CHECK(ring_link_read_image(&p) && p.word[RI_ID]==1);
    CHECK((uint32_t)p.word[RI_CRC32]==initial_crc);
    CHECK(((uint32_t)p.word[RI_OFFSET_LENGTH]&65535u)==0u);
    before=capture_offset;
    capture_service();CHECK(capture_offset==before); /* no ACK: retain slot */
    /* Wrong epoch/id/offset/sequence cannot consume the frozen source. */
    ring_link_ack_image(&p,&ack_seq);
    CHECK(ring_link_read_image_ack(&ack));
    ack.word[RA_ID]++;ring_link_write(ring_image_ack_mailbox,&ack,&ack_seq);
    capture_service();CHECK(capture_offset==before);
    memset(rebuilt,0,sizeof(rebuilt));
    while(capture_head==0u) {
        CHECK(ring_link_read_image(&p));
        offset=(uint32_t)p.word[RI_OFFSET_LENGTH]&65535u;
        part=(uint32_t)p.word[RI_OFFSET_LENGTH]>>16;
        CHECK(part>0u && part<=64u && offset+part<=RING_IMAGE_BYTES);
        memcpy(rebuilt+offset,p.data,part);
        ring_link_ack_image(&p,&ack_seq);capture_service();
    }
    CHECK(capture_count==7u && ring_image_crc32(rebuilt,sizeof(rebuilt))==initial_crc);
    for(i=0;i<RING_H*RING_W;++i) {
        int white=(rebuilt[i>>3]&(128u>>(i&7u)))!=0;
        CHECK(white==(((i%RING_W+3u*(i/RING_W))%7u)==0u));
    }
    CHECK(ring_image_crc32((const uint8_t *)"123456789",9u)==0xcbf43926u);
}
static void wire_contract(void)
{
    ring_image_packet_t packet;
    ring_packet_t ack;
    ring_runtime_diagnostics_t d;
    uint32_t seq=0;
    unsigned i;
    memset(&packet,0,sizeof(packet));memset(&d,0,sizeof(d));
    packet.word[RL_MAGIC]=RING_IMAGE_MAGIC;packet.word[RL_EPOCH]=(int32_t)test_epoch;
    packet.word[RI_ID]=123;packet.word[RI_FRAME]=10;packet.word[RI_COMMAND]=12;
    packet.word[RI_FLAGS]=RING_IMAGE_CONFIRMED|(RING_APPROACH<<4)|(6<<12);
    packet.word[RI_OFFSET_LENGTH]=64<<16;packet.word[RI_ROWS]=ring_diagnostic_points(80,40);
    for(i=0;i<64u;++i) packet.data[i]=(uint8_t)i;
    ring_link_write_image(&packet,&seq);d.command_epoch=test_epoch;d.phase=RING_ENTER;
    ring_diag_count=ring_diag_head=0;ring_diag_image_sequence=ring_diag_image_epoch=0;
    ring_diag_image_poll(&d);CHECK(ring_diag_count==0u); /* no active upload */
    d.phase=RING_FAULT;ring_diag_image_poll(&d);CHECK(ring_diag_count==2u);
    CHECK(strncmp(ring_diag_queue[0].text,"S,",2u)==0);
    CHECK(strncmp(ring_diag_queue[1].text,"I,",2u)==0);
    CHECK(ring_link_read_image_ack(&ack) && ack.word[RA_ID]==123);
    ring_diag_count=0;ring_diag_image_poll(&d);CHECK(ring_diag_count==0u); /* same chunk once */
    packet.word[RI_ID]=124;packet.word[RI_OFFSET_LENGTH]=65<<16;
    ring_link_write_image(&packet,&seq);ring_diag_image_poll(&d);CHECK(ring_diag_count==0u);
    /* New event reasons use the existing low four bits and S/I packets.
     * Verify serialization and ACK, not just enum packing. Unknown13 must
     * still be rejected without acknowledging a dropped chunk. */
    for(i=RING_IMAGE_HANDOVER;i<=RING_IMAGE_EXIT_WINDOW+1u;++i) {
        packet.word[RI_ID]=(int32_t)(200u+i);packet.word[RI_OFFSET_LENGTH]=64<<16;
        packet.word[RI_FLAGS]=(int32_t)(i|(RING_CIRCLE<<4)|(1<<12)|(1<<16));
        ring_diag_count=ring_diag_head=0;ring_link_write_image(&packet,&seq);
        ring_diag_image_poll(&d);
        if(i<=RING_IMAGE_EXIT_WINDOW) {
            CHECK(ring_diag_count==2u && strncmp(ring_diag_queue[0].text,"S,",2u)==0);
            CHECK(strncmp(ring_diag_queue[1].text,"I,",2u)==0);
            CHECK(ring_link_read_image_ack(&ack) && ack.word[RA_ID]==packet.word[RI_ID]);
        } else {
            CHECK(ring_diag_count==0u);
            CHECK(ring_link_read_image_ack(&ack) && ack.word[RA_ID]!=packet.word[RI_ID]);
        }
    }
    ring_diag_count=ring_diag_head=0;
    /* Exercise normal poll/header/H path, retaining the service bounds. */
    ring_diag_header_line=0;uart_port.used=0;ring_uart_diag_poll(0,100,100);
    CHECK(uart_port.used<=16u);
    /* At 16 bytes/poll, 100 polls carry only 1600 bytes. Startup schemas
     * now exceed that. Explicitly finish the bounded startup transfer
     * before measuring the independent H event latency below. */
    for(i=0;i<256u;++i) {
        unsigned before=uart_capture_count;
        ticks(46);apply(1);uart_port.used=0;ring_uart_diag_poll(0,100,100);
        CHECK(uart_capture_count-before<=16u);
        uart_capture[uart_capture_count]=0;
        if(strstr(uart_capture,"\r\nR,") && strstr(strstr(uart_capture,"\r\nR,")+2,"\r\n"))break;
    }
    CHECK(i<256u && uart_capture_count<=4096u);
    printf("UART startup delivered within %u polls, %u bytes at FIFO16; H latency measured after startup\n",i+1u,uart_capture_count);
    command(RING_APPROACH,-1,2,1);
    hpacket((uint32_t)ring_topology_flags(-1,RING_TANGENT_NEAR_PASSED,0,0,108,51),test_command_sequence);
    apply(1);
    for(i=0;i<50u;++i) { ticks(1);apply(1);uart_port.used=0;ring_uart_diag_poll(0,100,100); }
    uart_capture[uart_capture_count]=0;
    CHECK(strstr(uart_capture,"#RING,36,")!=NULL);
    CHECK(strstr(uart_capture,"TRACK_MODE=1")!=NULL);
    CHECK(strstr(uart_capture,"\r\nH,")!=NULL);
    CHECK(strstr(uart_capture,",-1,2,0,0,108,51,-123456,45678,")!=NULL);
}
static void vision_handover_contract(void)
{
    ring_packet_t feedback={{0}};
    uint32_t motion_seq=0;
    int i,e,out,entered=0,waiting=0;
    ring_vision_init();
    for(i=0;i<=RTG_APPROACH_STEPS+10;++i) {
        rtg_pose_t pose=rtg_approach(i);
        rtg_render(pixels,1,&pose);
        feedback.word[RL_MAGIC]=RING_LINK_MAGIC;feedback.word[RL_EPOCH]=(int32_t)test_epoch;
        feedback.word[RM_MS]=3000+i*20;feedback.word[RM_READY]=3;
        feedback.word[RM_WHEEL_MREV]=i*17;
        feedback.word[RM_RESERVED]=ring_motion_operator_version(RING_OPERATOR_VALID|RING_OPERATOR_RUN,RING_PAIR_VERSION);
        ring_link_write(ring_motion_mailbox,&feedback,&motion_seq);
        e=7;out=1;
        ring_vision_before_frame((const uint8_t (*)[RING_W])pixels);
        ring_vision_select(&e,&out);
        if(ring_result.phase==RING_ENTER) {
            CHECK(updated && out==3 && vision_handover);
            CHECK(e==ring_blend_error(7) && e!=7);entered=1;break;
        }
        if(ring_result.phase==RING_APPROACH) {
            CHECK(updated && e==7 && out==1 && !vision_handover);++waiting;
        }
        ring_vision_after_frame((const uint8_t (*)[RING_W])pixels);
    }
    CHECK(entered && waiting>10);
    /* A known original cross decision survives the entire waiting stage. */
    ring_init();ring_result.phase=RING_APPROACH;vision_handover=0;
    feedback.word[RM_MS]+=20;ring_link_write(ring_motion_mailbox,&feedback,&motion_seq);
    e=-13;out=2;ring_vision_before_frame((const uint8_t (*)[RING_W])pixels);
    ring_vision_select(&e,&out);CHECK(updated && e==-13 && out==2 && !vision_handover);
    ring_vision_after_frame((const uint8_t (*)[RING_W])pixels);
    /* Only an actual selected ring path may force original output to zero. */
    ring_result.phase=RING_FAULT;vision_handover=0;
    e=19;out=1;ring_vision_before_frame((const uint8_t (*)[RING_W])pixels);
    ring_vision_select(&e,&out);CHECK(e==19 && out==1);
    vision_handover=1;e=19;out=1;ring_vision_select(&e,&out);CHECK(e==0 && out==0);
    ring_result.phase=RING_NORMAL;e=19;out=1;ring_vision_select(&e,&out);
    CHECK(!vision_handover && e==19 && out==1);
}
static void pair_and_mouth_contract(void)
{
    ring_packet_t p={{0}},feedback;
    uint32_t js=0,old_sequence;
    unsigned k;const ring_runtime_diagnostics_t *d;
    ring_runtime_init(0.1f);ring_runtime_set_operator(1,0);ticks(2100);apply(1);
    d=ring_runtime_diagnostics();test_epoch=d->command_epoch;test_command_sequence=0;
    CHECK(ring_link_read(ring_motion_mailbox,&feedback));
    CHECK(ring_motion_version(feedback.word[RM_RESERVED])==RING_PAIR_VERSION);
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
    p.word[RC_PHASE]=ring_command_phase(RING_ENTER,0);p.word[RC_CAP]=ring_command_cap_turn(90,0);
    p.word[RC_DETECTED]=1;ring_link_write(ring_command_mailbox,&p,&test_command_sequence);apply(3);
    unchanged();CHECK(!d->pair_ok && d->vision_version==0 && !d->command_sequence);
    command(RING_ENTER,90,1,3);CHECK(d->pair_ok && d->vision_version==RING_PAIR_VERSION && d->active);
    old_sequence=d->command_sequence;
    p=(ring_packet_t){{0}};p.word[RL_MAGIC]=RING_MOUTH_DIAGNOSTIC_MAGIC;
    p.word[RL_EPOCH]=(int32_t)test_epoch;p.word[RJ_COMMAND_SEQUENCE]=(int32_t)old_sequence;
    p.word[RJ_FRAME]=55;p.word[RJ_POINTS]=ring_mouth_points(70,108,125,61);
    p.word[RJ_GATES]=ring_mouth_tracking(ring_mouth_gates(32767,1,2,3,4),9,2);
    p.word[RJ_PROGRESS_PAIR]=ring_mouth_progress_pair(44,17,RING_PAIR_VERSION,RING_PAIR_VERSION);
    ring_link_write(ring_mouth_diagnostic_mailbox,&p,&js);apply(3);
    CHECK(d->mouth_valid && d->mouth_frame==55u && d->mouth_far_x==125 && d->mouth_far_y==61);
    CHECK(d->mouth_gate==32767u && d->mouth_first_y==44 && d->mouth_progress==17);
    CHECK(d->mouth_ref && d->mouth_near_count==2u && d->mouth_far_count==3u && d->mouth_gap==3u);
    CHECK(d->mouth_near_missed==7u && d->mouth_pass_reason==2u);
    for(k=0;k<8u;++k) {
        ring_packet_t bad=p;
        switch(k) {
            case 0:bad.word[RL_EPOCH]++;break;
            case 1:bad.word[RJ_COMMAND_SEQUENCE]+=2;break;
            case 2:bad.word[RJ_POINTS]=ring_mouth_points(-1,10,125,61);break;
            case 3:bad.word[RJ_POINTS]=ring_mouth_points(188,108,125,61);break;
            case 4:bad.word[RJ_POINTS]=ring_mouth_points(70,108,125,120);break;
            case 5:bad.word[RJ_GATES]|=(int32_t)0x80000000u;break;
            case 6:bad.word[RJ_PROGRESS_PAIR]=ring_mouth_progress_pair(120,17,RING_PAIR_VERSION,RING_PAIR_VERSION);break;
            default:bad.word[RJ_PROGRESS_PAIR]=ring_mouth_progress_pair(44,17,0,RING_PAIR_VERSION);break;
        }
        bad.word[RJ_FRAME]=99;ring_link_write(ring_mouth_diagnostic_mailbox,&bad,&js);
        ticks(10);apply(3);CHECK(d->mouth_frame==55u && d->command_sequence==old_sequence);
        CHECK(d->command_age_ms==(k+1u)*10u);
    }
    /* Even fresh valid J every40ms cannot renew the accepted command. */
    for(k=0;k<6u;++k) {
        p.word[RJ_FRAME]=(int32_t)(100u+k);
        ring_link_write(ring_mouth_diagnostic_mailbox,&p,&js);
        ticks(40u);apply(3);CHECK(d->mouth_valid && d->mouth_age_ms==0u);
    }
    CHECK(!ring_runtime_watchdog_stop() && !d->pair_ok && target==123.0f);
    p=(ring_packet_t){{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
    p.word[RC_PHASE]=ring_command_phase(RING_ENTER,0);p.word[RC_CAP]=ring_command_cap_turn(90,0);
    p.word[RC_DETECTED]=1;ring_link_write(ring_command_mailbox,&p,&test_command_sequence);
    ticks(RING_LINK_TIMEOUT_MS);apply(3);
    CHECK(!d->pair_ok && d->command_sequence==old_sequence && !ring_runtime_watchdog_stop());
    CHECK(target==123 && turn==17); /* Old firmware cannot renew paired control or lock-stop it. */
    command(RING_NORMAL,-1,1,1);unchanged();
    ticks(RING_LINK_TIMEOUT_MS+1u);apply(1);unchanged();CHECK(!d->pair_ok);
    command(RING_NORMAL,-1,1,1);CHECK(d->pair_ok);
    p=(ring_packet_t){{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(255,0,RING_PAIR_VERSION);p.word[RC_CAP]=ring_command_cap_turn(-1,0);
    ring_link_write(ring_command_mailbox,&p,&test_command_sequence);apply(1);
    CHECK(!d->pair_ok && d->vision_version==RING_PAIR_VERSION);unchanged();
    /* New vision observing old control motion may preview; it cannot enter. */
    ring_vision_init();p=(ring_packet_t){{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;
    p.word[RL_EPOCH]=(int32_t)test_epoch;p.word[RM_READY]=3;p.word[RM_MS]=5000;
    p.word[RM_RESERVED]=(int32_t)(RING_OPERATOR_VALID|RING_OPERATOR_RUN);
    ring_link_write(ring_motion_mailbox,&p,&js);
    memset(pixels,255,sizeof(pixels));ring_vision_after_frame((const uint8_t (*)[RING_W])pixels);
    CHECK(!motion.ready && !ring_owns_path() && !vision_handover && ring_vision_preview());
    CHECK(control_version==0);
}
static void capture_budget_and_current_frames(void)
{
    uint32_t crc;
    capture_reset();ring_init();epoch=test_epoch;command_sequence=100;control_version=RING_PAIR_VERSION;
    motion.ready=0;ring_result.phase=RING_NORMAL;
    ring_result.mouth_far_x=ring_result.mouth_far_y=-1;
    memset(pixels,255,sizeof(pixels));
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,2);
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,3);
    CHECK(capture_count==2u && capture_preview_count==2u && capture_dropped==1u);
    crc=(uint32_t)captures[0].meta[RI_CRC32];
    motion.ready=1;capture_event_mask=0;
    ring_result.phase=RING_APPROACH;ring_result.approach_stage=RING_TANGENT_NEAR_SEEN;
    ring_result.frame_sequence=20;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    ring_result.approach_stage=RING_TANGENT_NEAR_PASSED;
    ring_result.frame_sequence=21;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    ring_result.mouth_far_x=122;ring_result.mouth_far_y=69;
    ring_result.frame_sequence=22;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    ring_result.mouth_progress=5;ring_result.mouth_far_count=1;
    ring_result.frame_sequence=23;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    ring_result.mouth_gate=RING_MOUTH_MOTION;ring_result.frame_sequence=24;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    CHECK(capture_count==6u); /* Optional gate image yields to reserved control events. */
    ring_result.phase=RING_ENTER;ring_result.frame_sequence=25;ring_result.branch_ok=1;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    ring_result.branch_ok=0;ring_result.frame_sequence=26;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,4);
    CHECK(capture_count==8u && (uint32_t)captures[0].meta[RI_CRC32]==crc);
    CHECK((captures[4].meta[RI_FLAGS]&15)==RING_IMAGE_FAR_CANDIDATE && captures[4].meta[RI_FRAME]==22);
    CHECK((captures[5].meta[RI_FLAGS]&15)==RING_IMAGE_FAR_PROGRESS && captures[5].meta[RI_FRAME]==23);
    CHECK((captures[6].meta[RI_FLAGS]&15)==RING_IMAGE_ENTERED && captures[6].meta[RI_FRAME]==25);
    CHECK((captures[7].meta[RI_FLAGS]&15)==RING_IMAGE_BRANCH_LOST && captures[7].meta[RI_FRAME]==26);
    CHECK(captures[4].meta[RI_RESERVED]==(RING_PAIR_VERSION|(RING_PAIR_VERSION<<8)));
}
static void exit_window_capture_contract(void)
{
    ring_result_t before;
    ring_image_packet_t pending,after;
    uint32_t crc,pending_sequence;
    unsigned i;
    capture_reset();ring_init();epoch=test_epoch;command_sequence=200;
    control_version=RING_PAIR_VERSION;motion.ready=1;motion.now_ms=1000;
    ring_result.phase=RING_CIRCLE;ring_result.committed=ring_result.lap_valid=1;
    ring_result.visual_dt_ms=17;ring_result.timed_entry=0;
    ring_result.frame_sequence=22;ring_result.lap_deg=384.9f;
    ring_result.branch_ok=ring_result.path_valid=ring_result.exit_candidate=0;
    /* Suppress unrelated event captures: inspect only the periodic input. */
    capture_detected=1;capture_event_mask=65535u;
    memset(pixels,255,sizeof(pixels));
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(!capture_count);
    ring_result.lap_deg=385.f;before=ring_result;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==1u && !memcmp(&before,&ring_result,sizeof(before)));
    CHECK((captures[0].meta[RI_FLAGS]&15)==RING_IMAGE_EXIT_WINDOW);
    CHECK(captures[0].meta[RI_LAP10]==3850 && captures[0].meta[RI_FRAME]==22);
    crc=(uint32_t)captures[0].meta[RI_CRC32];
    CHECK(crc==ring_image_crc32(captures[0].bits,RING_IMAGE_BYTES));
    memset(pixels,0,sizeof(pixels));ring_result.frame_sequence=99;
    motion.now_ms+=1499u;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==1u);
    ++motion.now_ms;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==2u && captures[1].meta[RI_FRAME]==99);
    CHECK((uint32_t)captures[0].meta[RI_CRC32]==crc &&
        (uint32_t)captures[1].meta[RI_CRC32]!=crc);
    motion.now_ms+=1500u;ring_result.lap_deg=435.f;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==3u);
    motion.now_ms+=1500u;ring_result.lap_deg=435.1f;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==3u);
    ring_result.lap_deg=400.f;
    motion.ready=0;capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    motion.ready=1;ring_result.committed=0;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.committed=1;ring_result.lap_valid=0;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.lap_valid=1;ring_result.timed_entry=1;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.timed_entry=0;ring_result.visual_dt_ms=65535u;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.visual_dt_ms=17;ring_result.phase=RING_ENTER;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==3u);
    /* A frozen chunk already awaiting ACK is never replaced or advanced
     * by periodic capture, and active operation still uploads no bytes. */
    ring_result.phase=RING_NORMAL;capture_service();
    CHECK(ring_link_read_image(&pending));pending_sequence=capture_pending_sequence;
    ring_result.phase=RING_CIRCLE;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==4u);
    capture_service();CHECK(capture_pending_sequence==pending_sequence && !capture_offset);
    CHECK(ring_link_read_image(&after) && !memcmp(&pending,&after,sizeof(pending)));
    for(i=0;i<2u;++i) {
        motion.now_ms+=1500u;
        capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    }
    CHECK(capture_count==6u && !capture_dropped);
    for(i=0;i<200u;++i) {
        motion.now_ms+=17u;
        capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    }
    CHECK(capture_count==6u && !capture_dropped);
    /* Periodic sampling leaves the last two slots for final events. */
    capture_event_mask=0;ring_result.phase=RING_EXIT;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    ring_result.phase=RING_COOLDOWN;ring_result.release_reason=2;
    capture_observe((const uint8_t (*)[RING_W])pixels,&ring_result,1);
    CHECK(capture_count==8u && !capture_dropped);
    CHECK((captures[6].meta[RI_FLAGS]&15)==RING_IMAGE_EXIT &&
        (captures[7].meta[RI_FLAGS]&15)==RING_IMAGE_RELEASE);
    CHECK((uint32_t)captures[0].meta[RI_CRC32]==crc);
    puts("PASS: v36 exit-window binary inputs at385..435deg even without candidate/branch;1500ms limit, reserved event slots, immutable pending chunk and unchanged idle-only upload");
}
int main(void)
{
    CHECK(RING_TRACK_MODE==1);CHECK(sizeof(ring_image_packet_t)==128u);
    CHECK(RING_IMAGE_ADDRESS+sizeof(ring_image_packet_t)==RING_IMAGE_ACK_ADDRESS);
    runtime_contract();image_contract();wire_contract();vision_handover_contract();
    pair_and_mouth_contract();capture_budget_and_current_frames();exit_window_capture_contract();
    printf("PASS: %u v28 reference transport checks: waiting beep/no ownership, H isolation, frozen pixels, ACK/CRC, idle-only wire\n",checks);
    return 0;
}
