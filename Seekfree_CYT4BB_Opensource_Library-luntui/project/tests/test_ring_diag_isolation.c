/* Independent manual-packet tests: no geometry/integration fixture. */
#include "ring.h"
#include "ring_link.h"
#include "ring_runtime.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ring_uart_diag.h"

stc_SCB_t uart_port;
char uart_capture[32768];
unsigned uart_capture_count;
static uint32_t command_sequence, diagnostic_sequence, current_epoch;
static uint32_t entry_diagnostic_sequence;
static uint32_t geometry_diagnostic_sequence;
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)){fprintf(stderr,"FAIL diagnostic isolation line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
static void tick(unsigned ms,int standing)
{
    unsigned i;for(i=0;i<ms;++i)(void)ring_runtime_tick(0,(uint8_t)standing);
}
static void apply_ordinary(int state)
{
    float target=87.25f,turn=-13.75f;
    ring_runtime_apply(&target,&turn,1.25f,state);
    CHECK(target==87.25f && turn==-13.75f);
    CHECK(ring_runtime_diagnostics()->active==0);
    CHECK(!ring_runtime_watchdog_stop());
}
static void boot(int clear)
{
    ring_packet_t packet;
    if(clear){memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);memset(ring_diagnostic_mailbox,0,32);memset(ring_entry_diagnostic_mailbox,0,32);memset(ring_geometry_diagnostic_mailbox,0,32);}
    ring_runtime_init(1.0f);
    command_sequence=diagnostic_sequence=0;
    entry_diagnostic_sequence=0;
    geometry_diagnostic_sequence=0;
    tick(2000u,0);tick(1u,1);apply_ordinary(1);
    CHECK(ring_link_read(ring_motion_mailbox,&packet));
    current_epoch=(uint32_t)packet.word[RL_EPOCH];
}
static void publish_command(int active)
{
    ring_packet_t packet={{0}};
    packet.word[RL_MAGIC]=RING_LINK_MAGIC;packet.word[RL_EPOCH]=(int32_t)current_epoch;
    packet.word[RC_CAP]=ring_command_cap_turn(active?90:-1,active?25:0);
    packet.word[RC_PHASE]=ring_command_phase(active?RING_ENTER:RING_NORMAL,0);
    packet.word[RC_DETECTED]=active?1:0;
    packet.word[RC_FAULT]=ring_command_diagnostics(0,active,0,active,active,0,1,active?31:0,active?80:-1,active?-1:0);
    ring_link_write(ring_command_mailbox,&packet,&command_sequence);
}
static ring_packet_t diagnostic_packet(uint32_t frame)
{
    ring_packet_t packet={{0}};
    packet.word[RL_MAGIC]=RING_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH]=(int32_t)current_epoch;
    packet.word[RD_COMMAND_SEQUENCE]=(int32_t)command_sequence;
    packet.word[RD_FRAME]=(int32_t)frame;
    packet.word[RD_TIMING_REJECT]=ring_diagnostic_timing(23u,512u);
    packet.word[RD_FLAGS]=ring_diagnostic_flags(1,1,7,5,-1,1,1,2);
    packet.word[RD_POINTS]=ring_diagnostic_points(111,128);
    return packet;
}
static void publish_diagnostic(ring_packet_t *packet)
{ring_link_write(ring_diagnostic_mailbox,packet,&diagnostic_sequence);}
static ring_packet_t entry_diagnostic_packet(uint32_t frame)
{
    ring_packet_t packet={{0}};
    packet.word[RL_MAGIC]=RING_ENTRY_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH]=(int32_t)current_epoch;
    packet.word[RE_COMMAND_SEQUENCE]=(int32_t)command_sequence;
    packet.word[RE_FRAME]=(int32_t)frame;
    packet.word[RE_FLAGS]=ring_entry_diagnostic_flags(1023u,63u,6u);
    packet.word[RE_AGE_LOST]=ring_entry_diagnostic_age(248u,5u);
    packet.word[RE_ANGLES]=ring_entry_diagnostic_angles(123,-321);
    return packet;
}
static void publish_entry_diagnostic(ring_packet_t *packet)
{ring_link_write(ring_entry_diagnostic_mailbox,packet,&entry_diagnostic_sequence);}
static ring_packet_t geometry_diagnostic_packet(uint32_t frame)
{
    ring_packet_t packet={{0}};
    packet.word[RL_MAGIC]=RING_GEOMETRY_DIAGNOSTIC_MAGIC;
    packet.word[RL_EPOCH]=(int32_t)current_epoch;
    packet.word[RG_COMMAND_SEQUENCE]=(int32_t)command_sequence;
    packet.word[RG_FRAME]=(int32_t)frame;
    packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(2u,35u,31u,5u);
    packet.word[RG_STRONG_AGE]=123u;
    packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(151,97u);
    return packet;
}
static void publish_geometry_diagnostic(ring_packet_t *packet)
{ring_link_write(ring_geometry_diagnostic_mailbox,packet,&geometry_diagnostic_sequence);}
static void test_delayed_and_invalid(void)
{
    ring_packet_t packet,copy;uint32_t age,frame;unsigned kind,state;
    boot(1);CHECK(current_epoch==1u);
    publish_command(0);apply_ordinary(1);
    CHECK(!ring_runtime_diagnostics()->diagnostic_valid);
    CHECK(ring_runtime_diagnostics()->command_age_ms==0u);
    /* Same command, later diagnostic: retry is independent of command-newness. */
    tick(47u,1);packet=diagnostic_packet(101u);publish_diagnostic(&packet);apply_ordinary(1);
    CHECK(ring_runtime_diagnostics()->diagnostic_valid);
    CHECK(ring_runtime_diagnostics()->frame_sequence==101u);
    CHECK(ring_runtime_diagnostics()->command_age_ms==47u);
    CHECK(ring_runtime_diagnostics()->visual_dt_ms==23u);
    CHECK(ring_runtime_diagnostics()->reject==512u);
    CHECK(ring_runtime_diagnostics()->entry_side==-1);
    CHECK(ring_runtime_diagnostics()->candidate_detected==1u && ring_runtime_diagnostics()->candidate_allowed==1u);
    CHECK(ring_runtime_diagnostics()->near_x==111 && ring_runtime_diagnostics()->look_x==128);
    CHECK(ring_runtime_diagnostics()->score==7u && ring_runtime_diagnostics()->confirm_frames==5u);
    CHECK(ring_runtime_diagnostics()->path_source==1u && ring_runtime_diagnostics()->classification==2u);
    /* Diagnostic-only magic cannot be consumed as an ordinary command. */
    CHECK(!ring_link_read(ring_diagnostic_mailbox,&copy));
    CHECK(ring_link_read_diagnostic(&copy));
    for(kind=0;kind<10u;++kind) {
        age=ring_runtime_diagnostics()->command_age_ms;frame=ring_runtime_diagnostics()->frame_sequence;
        packet=diagnostic_packet(200u+kind);
        switch(kind) {
            case 0:packet.word[RL_MAGIC]=RING_LINK_MAGIC;break;
            case 1:packet.word[RL_EPOCH]=(int32_t)(current_epoch+1u);break;
            case 2:packet.word[RD_COMMAND_SEQUENCE]=(int32_t)(command_sequence+2u);break;
            case 3:packet.word[RD_FLAGS]=(int32_t)((uint32_t)packet.word[RD_FLAGS]|0x80000000u);break;
            case 4:packet.word[RD_FLAGS]=(int32_t)(((uint32_t)packet.word[RD_FLAGS]&~0x300u)|0x300u);break;
            case 5:packet.word[RD_FLAGS]=ring_diagnostic_flags(1,1,7,5,-1,3,1,2);break;
            case 6:packet.word[RD_POINTS]=ring_diagnostic_points(188,128);break;
            case 7:packet.word[RD_POINTS]=ring_diagnostic_points(111,-2);break;
            case 8:packet.word[RL_MAGIC]=0;break;
            default:break;
        }
        publish_diagnostic(&packet);
        if(kind==9u)ring_diagnostic_mailbox[RL_SEQUENCE]|=1; /* in-progress writer */
        tick(11u,1);
        for(state=0;state<=2u;++state)apply_ordinary((int)state);
        CHECK(ring_runtime_diagnostics()->frame_sequence==frame);
        CHECK(ring_runtime_diagnostics()->command_age_ms==age+11u);
        CHECK(ring_runtime_diagnostics()->fault==0u);
    }
    /* Fresh command changes invalidate association until the matching D arrives. */
    publish_command(0);apply_ordinary(1);
    CHECK(!ring_runtime_diagnostics()->diagnostic_valid);
    packet=diagnostic_packet(300u);publish_diagnostic(&packet);apply_ordinary(1);
    CHECK(ring_runtime_diagnostics()->diagnostic_valid);
    tick(RING_LINK_TIMEOUT_MS+1u,1);apply_ordinary(0);
    CHECK(!ring_runtime_diagnostics()->diagnostic_valid);
    CHECK(ring_runtime_diagnostics()->diagnostic_age_ms==RING_LINK_TIMEOUT_MS+1u);
    CHECK(ring_runtime_diagnostics()->command_age_ms==RING_LINK_TIMEOUT_MS+1u);
}
static void test_watchdog_not_refreshed(void)
{
    ring_packet_t packet;unsigned i;float target,turn;
    boot(1);publish_command(1);target=100.0f;turn=12.0f;
    ring_runtime_apply(&target,&turn,0.0f,3);
    CHECK(ring_runtime_diagnostics()->active && target==90.0f && turn==12.0f);
    CHECK(!ring_runtime_watchdog_stop());
    for(i=0;i<8u;++i) {
        tick(40u,1);packet=diagnostic_packet(500u+i);publish_diagnostic(&packet);
        target=100.0f;turn=12.0f;ring_runtime_apply(&target,&turn,0.1f,3);
        CHECK(ring_runtime_diagnostics()->diagnostic_valid);
        CHECK(ring_runtime_diagnostics()->diagnostic_age_ms==0u);
        CHECK(ring_runtime_diagnostics()->command_age_ms==(i+1u)*40u);
        if(i<7u)CHECK(target==90.0f && turn==12.0f && !ring_runtime_watchdog_stop());
    }
    /* Continuously fresh diagnostic packets must never keep a stale command alive. */
    CHECK(ring_runtime_watchdog_stop());CHECK(target==0.0f && turn==0.0f);
    /* Bad packets also cannot reset the already-expired control watchdog. */
    packet=diagnostic_packet(600u);packet.word[RL_MAGIC]=0;publish_diagnostic(&packet);
    tick(1u,1);target=100.0f;turn=12.0f;ring_runtime_apply(&target,&turn,0.1f,3);
    CHECK(ring_runtime_watchdog_stop() && target==0.0f && turn==0.0f);
    CHECK(ring_runtime_diagnostics()->command_age_ms==321u);
    /* A new accepted ordinary command relinquishes only ring control. */
    publish_command(0);apply_ordinary(1);
}
static void test_epoch_restarts_and_text(void)
{
    ring_packet_t packet;uint32_t old_epoch;unsigned i,commas;const char *r,*end,*p;
    boot(1);publish_command(0);packet=diagnostic_packet(700u);publish_diagnostic(&packet);apply_ordinary(1);
    CHECK(ring_runtime_diagnostics()->diagnostic_valid);
    for(i=0;i<3u;++i) {
        old_epoch=current_epoch;boot(0);CHECK(current_epoch==old_epoch+1u);
        /* Old D and old commands remain in no-init RAM but belong to prior epoch. */
        CHECK(!ring_runtime_diagnostics()->diagnostic_valid);
        publish_command(0);apply_ordinary(1);CHECK(!ring_runtime_diagnostics()->diagnostic_valid);
        packet=diagnostic_packet(1u);publish_diagnostic(&packet);apply_ordinary(1);
        CHECK(ring_runtime_diagnostics()->diagnostic_valid);
        CHECK(ring_runtime_diagnostics()->diagnostic_epoch==current_epoch);
        CHECK(ring_runtime_diagnostics()->diagnostic_command_sequence==command_sequence);
    }
    for(i=0;i<200u;++i){uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    uart_capture[uart_capture_count]=0;
    CHECK(strstr(uart_capture,"#RING,36,UART1,115200")!=NULL);
    r=strstr(uart_capture,"\r\nR,");CHECK(r!=NULL);r+=2;end=strstr(r,"\r\n");CHECK(end!=NULL);
    commas=0;for(p=r;p<end;++p)if(*p==',')++commas;CHECK(commas==33u);
    r=strstr(uart_capture,"\r\nD,");CHECK(r!=NULL);r+=2;end=strstr(r,"\r\n");CHECK(end!=NULL);
    commas=0;for(p=r;p<end;++p)if(*p==',')++commas;CHECK(commas==19u);
    CHECK(strstr(uart_capture,",23,1,1,7,512,5,-1,1,111,128,1,2,")!=NULL);
}
static void test_entry_independent_and_invalid(void)
{
    ring_packet_t packet,copy;unsigned kind,state,i;uint32_t age,old_epoch;
    const ring_runtime_diagnostics_t *d;
    boot(1);publish_command(0);apply_ordinary(1);
    CHECK(!ring_runtime_diagnostics()->entry_diagnostic_valid);
    tick(47u,1);packet=entry_diagnostic_packet(901u);publish_entry_diagnostic(&packet);apply_ordinary(1);
    d=ring_runtime_diagnostics();
    CHECK(d->entry_diagnostic_valid && !d->diagnostic_valid);
    CHECK(d->entry_frame_sequence==901u && d->entry_diagnostic_epoch==current_epoch);
    CHECK(d->entry_diagnostic_command_sequence==command_sequence && d->command_age_ms==47u);
    CHECK(d->entry_flags==1023u && d->hold_reject==63u && d->decision==6u);
    CHECK(d->path_age_ms==248u && d->lost_frames==5u);
    CHECK(d->hold_turn10==123 && d->entry_turn10==-321);
    CHECK(!ring_link_read(ring_entry_diagnostic_mailbox,&copy));
    CHECK(ring_link_read_entry_diagnostic(&copy));
    packet=diagnostic_packet(901u);publish_diagnostic(&packet);apply_ordinary(1);
    CHECK(d->diagnostic_valid && d->entry_diagnostic_valid);
    for(kind=0;kind<9u;++kind) {
        age=d->command_age_ms;packet=entry_diagnostic_packet(950u+kind);
        switch(kind) {
            case 0:packet.word[RL_MAGIC]=RING_LINK_MAGIC;break;
            case 1:packet.word[RL_EPOCH]=(int32_t)(current_epoch+1u);break;
            case 2:packet.word[RE_COMMAND_SEQUENCE]=(int32_t)(command_sequence+2u);break;
            case 3:packet.word[RE_FLAGS]=ring_entry_diagnostic_flags(8192u,0u,1u);break;
            case 4:packet.word[RE_FLAGS]=ring_entry_diagnostic_flags(0u,64u,1u);break;
            case 5:packet.word[RE_FLAGS]=ring_entry_diagnostic_flags(0u,0u,15u);break;
            case 6:packet.word[RE_AGE_LOST]=(int32_t)0x01000000u;break;
            case 7:packet.word[RL_MAGIC]=RING_DIAGNOSTIC_MAGIC;break;
            default:break;
        }
        publish_entry_diagnostic(&packet);
        if(kind==8u)ring_entry_diagnostic_mailbox[RL_SEQUENCE]|=1;
        tick(11u,1);for(state=0;state<=2u;++state)apply_ordinary((int)state);
        CHECK(d->entry_frame_sequence==901u && d->frame_sequence==901u);
        CHECK(d->command_age_ms==age+11u && d->fault==0u);
        CHECK(d->entry_diagnostic_age_ms==d->command_age_ms-47u);
    }
    publish_command(0);apply_ordinary(1);
    CHECK(!d->entry_diagnostic_valid && !d->diagnostic_valid);
    packet=entry_diagnostic_packet(1000u);publish_entry_diagnostic(&packet);apply_ordinary(1);
    CHECK(d->entry_diagnostic_valid && !d->diagnostic_valid);
    tick(RING_LINK_TIMEOUT_MS+1u,1);apply_ordinary(0);
    CHECK(!d->entry_diagnostic_valid && d->entry_diagnostic_age_ms==RING_LINK_TIMEOUT_MS+1u);
    for(i=0;i<3u;++i) {
        old_epoch=current_epoch;boot(0);CHECK(current_epoch==old_epoch+1u);
        publish_command(0);apply_ordinary(1);CHECK(!d->entry_diagnostic_valid);
        packet=entry_diagnostic_packet(1u);publish_entry_diagnostic(&packet);apply_ordinary(1);
        CHECK(d->entry_diagnostic_valid && d->entry_diagnostic_epoch==current_epoch);
    }
}
static void test_entry_watchdog_and_text(void)
{
    ring_packet_t packet;unsigned i,commas;float target,turn;const char *p,*end;
    boot(1);publish_command(1);target=100.0f;turn=12.0f;
    ring_runtime_apply(&target,&turn,0.0f,3);
    for(i=0;i<8u;++i) {
        tick(40u,1);packet=entry_diagnostic_packet(1100u+i);publish_entry_diagnostic(&packet);
        target=100.0f;turn=12.0f;ring_runtime_apply(&target,&turn,0.1f,3);
        CHECK(ring_runtime_diagnostics()->entry_diagnostic_valid);
        CHECK(ring_runtime_diagnostics()->entry_diagnostic_age_ms==0u);
        CHECK(ring_runtime_diagnostics()->command_age_ms==(i+1u)*40u);
        if(i<7u)CHECK(target==90.0f && turn==12.0f && !ring_runtime_watchdog_stop());
    }
    CHECK(ring_runtime_watchdog_stop() && target==0.0f && turn==0.0f);
    for(i=0;i<200u;++i){uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    uart_capture[uart_capture_count]=0;
    p=strstr(uart_capture,"\r\nE,");CHECK(p!=NULL);p+=2;
    end=strstr(p,"\r\n");CHECK(end!=NULL);commas=0;
    for(;p<end;++p)if(*p==',')++commas;
    CHECK(commas==14u);
    CHECK(strstr(uart_capture,",1023,63,6,5,248,123,-321,")!=NULL);
    publish_command(0);apply_ordinary(1);
}
static void test_geometry_independent_and_invalid(void)
{
    ring_packet_t packet,copy;unsigned kind,state,i;uint32_t age,old_epoch;
    const ring_runtime_diagnostics_t *d;
    boot(1);publish_command(0);apply_ordinary(1);
    CHECK(!ring_runtime_diagnostics()->geometry_valid);
    tick(47u,1);packet=geometry_diagnostic_packet(1901u);publish_geometry_diagnostic(&packet);apply_ordinary(1);
    d=ring_runtime_diagnostics();
    CHECK(d->geometry_valid && !d->entry_diagnostic_valid && !d->diagnostic_valid);
    CHECK(d->geometry_frame_sequence==1901u && d->geometry_epoch==current_epoch);
    CHECK(d->geometry_command_sequence==command_sequence && d->command_age_ms==47u);
    CHECK(d->branch_mode==2u && d->inner_rows==35u && d->associated_rows==31u);
    CHECK(d->target_support==5u && d->strong_age_ms==123u);
    CHECK(d->guide_x==151 && d->guide_y==97u);
    CHECK(!ring_link_read(ring_geometry_diagnostic_mailbox,&copy));
    CHECK(ring_link_read_geometry_diagnostic(&copy));
    packet=diagnostic_packet(1901u);publish_diagnostic(&packet);
    packet=entry_diagnostic_packet(1901u);publish_entry_diagnostic(&packet);apply_ordinary(1);
    CHECK(d->diagnostic_valid && d->entry_diagnostic_valid && d->geometry_valid);
    for(kind=0;kind<15u;++kind) {
        age=d->command_age_ms;packet=geometry_diagnostic_packet(1950u+kind);
        switch(kind) {
            case 0:packet.word[RL_MAGIC]=RING_LINK_MAGIC;break;
            case 1:packet.word[RL_EPOCH]=(int32_t)(current_epoch+1u);break;
            case 2:packet.word[RG_COMMAND_SEQUENCE]=(int32_t)(command_sequence+2u);break;
            case 3:packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(7u,20u,20u,7u);break;
            case 4:packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(1u,121u,20u,7u);break;
            case 5:packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(1u,20u,121u,7u);break;
            case 6:packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(1u,20u,20u,8u);break;
            case 7:packet.word[RG_STRONG_AGE]=(int32_t)0x10000u;break;
            case 8:packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(188,20u);break;
            case 9:packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(-2,20u);break;
            case 10:packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(100,120u);break;
            case 11:packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(-1,97u);break;
            case 12:packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(151,255u);break;
            case 13:packet.word[RL_MAGIC]=RING_ENTRY_DIAGNOSTIC_MAGIC;break;
            default:break;
        }
        publish_geometry_diagnostic(&packet);
        if(kind==14u)ring_geometry_diagnostic_mailbox[RL_SEQUENCE]|=1;
        tick(11u,1);for(state=0;state<=2u;++state)apply_ordinary((int)state);
        CHECK(d->geometry_frame_sequence==1901u && d->frame_sequence==1901u && d->entry_frame_sequence==1901u);
        CHECK(d->command_age_ms==age+11u && d->fault==0u);
        CHECK(d->geometry_age_ms==d->command_age_ms-47u);
    }
    publish_command(0);apply_ordinary(1);
    CHECK(!d->geometry_valid && !d->entry_diagnostic_valid && !d->diagnostic_valid);
    packet=geometry_diagnostic_packet(2000u);packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(-1,255u);
    publish_geometry_diagnostic(&packet);apply_ordinary(1);
    CHECK(d->geometry_valid && d->guide_x==-1 && d->guide_y==255u);
    tick(RING_LINK_TIMEOUT_MS+1u,1);apply_ordinary(0);
    CHECK(!d->geometry_valid && d->geometry_age_ms==RING_LINK_TIMEOUT_MS+1u);
    for(i=0;i<3u;++i) {
        old_epoch=current_epoch;boot(0);CHECK(current_epoch==old_epoch+1u);
        publish_command(0);apply_ordinary(1);CHECK(!d->geometry_valid);
        packet=geometry_diagnostic_packet(1u);publish_geometry_diagnostic(&packet);apply_ordinary(1);
        CHECK(d->geometry_valid && d->geometry_epoch==current_epoch);
    }
}
static void test_geometry_watchdog_and_text(void)
{
    ring_packet_t packet;unsigned i,commas;float target,turn;const char *p,*end;
    boot(1);publish_command(1);target=100.0f;turn=12.0f;
    ring_runtime_apply(&target,&turn,0.0f,3);
    for(i=0;i<8u;++i) {
        tick(40u,1);packet=geometry_diagnostic_packet(2100u+i);publish_geometry_diagnostic(&packet);
        target=100.0f;turn=12.0f;ring_runtime_apply(&target,&turn,0.1f,3);
        CHECK(ring_runtime_diagnostics()->geometry_valid);
        CHECK(ring_runtime_diagnostics()->geometry_age_ms==0u);
        CHECK(ring_runtime_diagnostics()->command_age_ms==(i+1u)*40u);
        if(i<7u)CHECK(target==90.0f && turn==12.0f && !ring_runtime_watchdog_stop());
    }
    CHECK(ring_runtime_watchdog_stop() && target==0.0f && turn==0.0f);
    for(i=0;i<200u;++i){uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    uart_capture[uart_capture_count]=0;
    p=strstr(uart_capture,"\r\nG,");CHECK(p!=NULL);p+=2;
    end=strstr(p,"\r\n");CHECK(end!=NULL);commas=0;
    for(;p<end;++p)if(*p==',')++commas;
    CHECK(commas==14u);
    CHECK(strstr(uart_capture,",2,35,31,5,123,151,97,")!=NULL);
    publish_command(0);apply_ordinary(1);
}
static void test_assist_diagnostics_only(void)
{
    ring_packet_t packet;unsigned decision,i;float target,turn;
    const ring_runtime_diagnostics_t *d;
    boot(1);publish_command(0);apply_ordinary(1);d=ring_runtime_diagnostics();
    for(decision=11u;decision<=14u;++decision) {
        packet=diagnostic_packet(3000u+decision);
        packet.word[RD_FLAGS]=ring_diagnostic_flags(1,1,7,5,-1,5,0,2);
        publish_diagnostic(&packet);
        packet=entry_diagnostic_packet(3000u+decision);
        packet.word[RE_FLAGS]=ring_entry_diagnostic_flags(8191u,12u,decision);
        publish_entry_diagnostic(&packet);
        packet=geometry_diagnostic_packet(3000u+decision);
        packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(5u,0u,0u,0u);
        packet.word[RG_GUIDE]=ring_geometry_diagnostic_guide(-1,255u);
        publish_geometry_diagnostic(&packet);tick(11u,1);apply_ordinary(1);
        CHECK(d->diagnostic_valid && d->entry_diagnostic_valid && d->geometry_valid);
        CHECK(d->path_source==5u && !d->branch_ok && d->branch_mode==5u);
        CHECK(d->decision==decision && d->entry_flags==8191u && d->hold_reject==12u);
        CHECK(d->path_age_ms==248u && d->strong_age_ms==123u);
        CHECK(d->command_age_ms==(decision-10u)*11u && !d->active);
    }
    /* Source 3 remains invalid; diagnostic permission does not authorize a
     * control takeover, and fresh assist telemetry cannot renew a watchdog. */
    packet=diagnostic_packet(3100u);packet.word[RD_FLAGS]=ring_diagnostic_flags(1,1,7,5,-1,3,0,2);
    publish_diagnostic(&packet);apply_ordinary(1);CHECK(d->frame_sequence==3014u);
    boot(1);publish_command(1);target=100.0f;turn=12.0f;
    ring_runtime_apply(&target,&turn,0.0f,3);
    for(i=0;i<8u;++i) {
        tick(40u,1);
        packet=entry_diagnostic_packet(3200u+i);
        packet.word[RE_FLAGS]=ring_entry_diagnostic_flags(7168u,12u,11u);
        publish_entry_diagnostic(&packet);
        packet=geometry_diagnostic_packet(3200u+i);
        packet.word[RG_GEOMETRY]=ring_geometry_diagnostic_flags(5u,0u,0u,0u);
        publish_geometry_diagnostic(&packet);
        target=100.0f;turn=12.0f;ring_runtime_apply(&target,&turn,0.1f,3);
        CHECK(d->entry_diagnostic_valid && d->geometry_valid && d->decision==11u);
        CHECK(d->command_age_ms==(i+1u)*40u);
    }
    CHECK(ring_runtime_watchdog_stop() && target==0.0f && turn==0.0f);
    publish_command(0);apply_ordinary(1);
}
int main(void)
{
    test_delayed_and_invalid();test_watchdog_not_refreshed();test_epoch_restarts_and_text();
    test_entry_independent_and_invalid();test_entry_watchdog_and_text();
    test_geometry_independent_and_invalid();test_geometry_watchdog_and_text();
    test_assist_diagnostics_only();
    printf("PASS: independent D/E/G/control isolation including v11 assist, delayed/stale/malformed packets, R33/D19/E14/G14 (%u checks)\n",checks);
    return 0;
}
