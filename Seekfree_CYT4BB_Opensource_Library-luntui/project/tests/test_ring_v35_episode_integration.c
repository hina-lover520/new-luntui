/* Production publisher/controller integration with explicitly prepared
 * recognition/committed-entry history. This does not claim a driven lap.
 * Real gyro integrates75 -> blank beyond250ms -> ordinary steering with
 * retained lap identity -> current tilted exit -> final360 -> handover. */
#include "../code/ring.c"
#include "../code/ring_vision.c"
#include "ring_runtime.h"
#include <stdio.h>

static uint8_t camera[120][188];
static int direction,track,error;
static unsigned pulses;
static float revs,speed,turn,normal_turn=17.f,measured=100.f;
static const ring_runtime_diagnostics_t *diag(void){return ring_runtime_diagnostics();}
#define VERIFY(c) do{if(!(c)){fprintf(stderr,"FAIL v35 episode:%d %s side%d phase%d man%u force%u lap%.3f active%u out%.2f\n",\
 __LINE__,#c,direction,ring_result.phase,diag()->maneuver_stage,diag()->exit_force_stage,ring_result.lap_deg,diag()->active,turn);exit(1);}}while(0)
static void ticks(int raw,unsigned count)
{while(count--){if(ring_runtime_tick((int16_t)raw,1)==1)++pulses;}}
static void apply(void)
{
    speed=100.f;turn=normal_turn;
    ring_runtime_observe_motion(measured,0.f,0.f,0,0);
    ring_runtime_apply(&speed,&turn,revs,track);
}
static void road(float relative)
{
    int x,y;float a=relative*.01745329252f;
    memset(camera,0,sizeof(camera));
    for(y=0;y<120;++y)for(x=0;x<188;++x) {
        float dx=(x-94.f)*cosf(a)-(108.f-y)*sinf(a);
        if(fabsf(dx)<=29.5f)camera[y][x]=255;
    }
}
static void frame(void)
{
    track=1;error=12;
    ring_vision_before_frame((const uint8_t(*)[188])camera);
    ring_vision_select(&error,&track);
    ring_vision_after_frame((const uint8_t(*)[188])camera);
    apply();
}
static ring_packet_t command(void)
{ring_packet_t p;VERIFY(ring_link_read(ring_command_mailbox,&p));return p;}
static void prepare75(int side,int committed,int scheduled)
{
    unsigned i,n;ring_packet_t feedback;
    direction=side;normal_turn=17.f;measured=100.f;
    memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);
    memset(ring_diagnostic_mailbox,0,32);memset(ring_entry_diagnostic_mailbox,0,32);
    memset(ring_geometry_diagnostic_mailbox,0,32);memset(ring_topology_diagnostic_mailbox,0,32);
    memset(ring_mouth_diagnostic_mailbox,0,32);memset(ring_exit_diagnostic_mailbox,0,32);
    pulses=0;revs=0.f;track=1;
    ring_runtime_init(.1f);ring_runtime_set_operator(0,0);
    for(i=0;i<2200u;++i)(void)ring_runtime_tick(0,0);
    ring_runtime_set_operator(1,0);ticks(0,1u);apply();
    ring_vision_init();road(0.f);frame();
    VERIFY(ring_link_read(ring_motion_mailbox,&feedback));
    VERIFY(epoch==(uint32_t)feedback.word[RL_EPOCH] && motion.ready);
    /* Prepare recognition only; original entry timer and real gyro are used. */
    ring_result.phase=RING_APPROACH;ring_result.detected_events=1;
    ring_result.route=(int8_t)-side;ring_result.ring_side=(int8_t)side;
    ring_result.entry_side=(int8_t)side;ring_result.timed_entry=1;
    ring_result.force_entry=ring_result.path_valid=1;ring_result.steer_error=0;
    publish();apply();
    for(i=0;i<RING_MANEUVER_DELAY_MS/20u+3u;++i) {
        ticks(0,20u);revs+=.03f;publish();apply();
        if(diag()->maneuver_stage==2u)break;
    }
    VERIFY(diag()->maneuver_stage==2u && pulses==1u);
    ring_result.phase=RING_ENTER;ring_result.force_entry=0;
    for(n=0;n<752u;) {
        unsigned step=752u-n>20u?20u:752u-n;
        ticks(side*1000,step);revs+=step*.0015f;publish();apply();n+=step;
    }
    VERIFY(diag()->maneuver_stage==3u && diag()->maneuver_turn10>=750);
    /* Explicitly prepare proved branch identity, not new camera evidence. */
    ring_result.phase=committed?RING_CIRCLE:RING_ENTER;ring_result.timed_entry=0;
    ring_result.committed=ring_result.left_entry=(uint8_t)committed;
    ring_result.lap_valid=1;ring_result.lap_origin_yaw=0;ring_result.lane_width=60;
    ring_result.approach_stage=RING_TANGENT_LEFT;
    timed_started=scheduled;have_path=1;seed_identity_lost=0;lap_completed=0;
    ring_since=phase_since=last_path_fault_ms=diag()->now_ms;
    last_path_ms=vision_ms;tangent_lap_revs=0.f;
    trusted_steer=0;trusted_near=trusted_look=94;
    last_yaw=side*75.2f;support_last_revs=revs;
    for(i=0;i<RING_H;++i)previous_mid[i]=ring_result.mid[i]=94;
}
static void observe_then_exit(int side)
{
    unsigned n,declared=0;float origin;uint32_t event;ring_packet_t p;
    prepare75(side,1,1);origin=ring_result.lap_origin_yaw;event=ring_result.detected_events;
    memset(camera,0,sizeof(camera));
    for(n=0;n<20u;++n) {
        ticks(side*1000,20u);revs+=.03f;frame();
        VERIFY(ring_result.phase==RING_CIRCLE && !ring_result.release_reason &&
            ring_result.committed && ring_result.left_entry && ring_result.lap_valid);
        VERIFY(diag()->active && diag()->maneuver_stage==3u && !diag()->force_active);
        VERIFY(ring_result.lap_origin_yaw==origin && ring_result.detected_events==event &&
            !ring_result.exited_events && ring_result.lap_deg>75.f);
    }
    VERIFY(!ring_result.branch_ok && !ring_result.path_valid && !ring_owns_path());
    VERIFY(diag()->entry_diagnostic_valid && diag()->decision==RING_DECISION_OBSERVE);
    VERIFY(track==1 && error==12 && turn==normal_turn && speed>90.f && speed<=100.f);
    p=command();VERIFY(!ring_command_diag_path_valid(p.word[RC_FAULT]) &&
        ring_command_cap_value(p.word[RC_CAP])==-1 &&
        !ring_command_boost(p.word[RC_PHASE]) && ring_command_diag_committed(p.word[RC_FAULT]));
    /* Measured wheel-speed limiting must not clip ordinary steering while
     * observing the episode; the ring no longer owns a visual target. */
    normal_turn=60.f;measured=20.f;ticks(0,20u);revs+=.03f;frame();
    VERIFY(track==1 && error==12 && turn==60.f && !diag()->force_active && diag()->active);
    normal_turn=17.f;measured=100.f;road(0.f);
    while(diag()->maneuver_turn10<3150) {
        int raw=diag()->maneuver_turn10<3130?1000:100;
        ticks(side*raw,20u);revs+=.03f;frame();
        VERIFY(ring_result.phase==RING_CIRCLE && !ring_result.branch_ok &&
            !ring_result.exited_events && !ring_result.exit_latched && !diag()->force_active);
        VERIFY(ring_result.lap_origin_yaw==origin && ring_result.detected_events==event && diag()->active);
        VERIFY(track==1 && error==12 && turn==normal_turn);
    }
    for(n=0;n<10u && ring_result.phase!=RING_EXIT;++n) {
        ticks(0,20u);revs+=.03f;
        road(side*(360.f-diag()->maneuver_turn10*.1f));frame();
    }
    VERIFY(ring_result.phase==RING_EXIT && ring_result.exit_turn_request && ring_result.exit_latched);
    VERIFY(diag()->active && diag()->exit_force_stage==1u && diag()->force_active);
    VERIFY((uint32_t)command().word[RC_FAULT]&RING_COMMAND_EXIT_TURN);
    for(n=0;n<40u && diag()->exit_force_stage==1u;++n) {
        ticks(side*1000,20u);revs+=.03f;
        road(side*(360.f-diag()->maneuver_turn10*.1f-2.f));frame();
        VERIFY(diag()->active && speed>0.f && !ring_result.exited_events);
    }
    VERIFY(diag()->exit_force_stage==2u && diag()->maneuver_turn10>=3600);
    for(n=0;n<45u && !ring_result.exited_events;++n) {
        ticks(0,20u);revs+=.03f;road(side*(360.f-diag()->maneuver_turn10*.1f));frame();
        VERIFY(speed>0.f && !ring_result.release_reason);
        if(ring_result.phase==RING_RECOVER)declared=1;
    }
    VERIFY(declared && ring_result.phase==RING_COOLDOWN && ring_result.exited_events==1u &&
        ring_result.exit_confirmed && !ring_result.exit_turn_request);
    VERIFY(diag()->phase==RING_COOLDOWN && diag()->exited==1u && !diag()->active &&
        track==1 && error==12 && turn==normal_turn);
    ticks(0,600u);apply();VERIFY(pulses==6u);
}
static void unproved_still_releases(int scheduled)
{
    unsigned n;prepare75(-1,scheduled?0:1,scheduled);memset(camera,0,sizeof(camera));
    for(n=0;n<45u && ring_result.phase!=RING_COOLDOWN;++n) {
        ticks(0,20u);revs+=.03f;frame();
    }
    VERIFY(ring_result.phase==RING_COOLDOWN && !ring_result.circle_observe_only &&
        !ring_result.exited_events && !diag()->active && !diag()->force_active);
}
static void aligned360_after_long_blank(int side)
{
    unsigned n,declared=0;float origin;uint32_t event;
    prepare75(side,1,1);origin=ring_result.lap_origin_yaw;event=ring_result.detected_events;
    memset(camera,0,sizeof(camera));
    while(diag()->maneuver_turn10<3602) {
        int raw=diag()->maneuver_turn10<3580?1000:100;
        ticks(side*raw,20u);revs+=.03f;frame();
        VERIFY(ring_result.phase==RING_CIRCLE && diag()->active &&
            diag()->maneuver_stage==3u && !diag()->force_active &&
            !ring_result.exited_events && !ring_result.exit_latched);
        VERIFY(ring_result.lap_origin_yaw==origin && ring_result.detected_events==event &&
            track==1 && error==12 && turn==normal_turn);
    }
    /* Consume the controller's newest gyro feedback before asking vision
     * to re-acquire; the mailbox is one production frame behind apply. */
    ticks(0,20u);revs+=.03f;frame();
    VERIFY(ring_result.circle_observe_only && !ring_result.branch_ok &&
        !ring_result.path_valid && ring_result.lap_deg>=360.f);
    road(0.f);
    for(n=0;n<12u && ring_result.phase!=RING_EXIT;++n) {
        ticks(0,20u);revs+=.03f;frame();
    }
    VERIFY(ring_result.phase==RING_EXIT && ring_result.exit_latched &&
        !ring_result.circle_observe_only && !ring_result.exit_turn_request &&
        diag()->active && !diag()->force_active && diag()->exit_force_stage==0u);
    for(n=0;n<45u && !ring_result.exited_events;++n) {
        ticks(0,20u);revs+=.03f;frame();
        VERIFY(speed>0.f && !ring_result.release_reason);
        if(ring_result.phase==RING_RECOVER)declared=1;
    }
    VERIFY(declared && ring_result.phase==RING_COOLDOWN && ring_result.exited_events==1u &&
        diag()->exited==1u && !diag()->active && track==1 && error==12 && turn==normal_turn);
    ticks(0,600u);apply();VERIFY(pulses==6u);
}
static void observation_does_not_bypass_release(int how)
{
    unsigned n;prepare75(-1,1,1);memset(camera,0,sizeof(camera));
    for(n=0;n<20u;++n){ticks(0,20u);revs+=.03f;frame();}
    VERIFY(ring_result.circle_observe_only && diag()->active && !diag()->force_active);
    if(how==0) {
        ring_runtime_set_operator(0,0);ticks(0,20u);apply();
        for(n=0;n<4u && ring_result.phase!=RING_COOLDOWN;++n) {
            ticks(0,20u);frame();
        }
    } else if(how==1) {
        ticks(0,RING_LINK_TIMEOUT_MS+1u);
        ring_runtime_poll_timeout(&speed,&turn);apply();
        ticks(0,20u);frame();
    } else {
        for(n=0;n<RING_TOTAL_TIMEOUT_MS/20u+2u && ring_result.phase!=RING_COOLDOWN;++n) {
            ticks(0,20u);revs+=.03f;frame();
        }
    }
    VERIFY(ring_result.phase==RING_COOLDOWN && !ring_result.circle_observe_only &&
        !ring_result.exited_events && !diag()->active && !diag()->force_active && pulses==1u);
}
int main(void)
{
    observe_then_exit(-1);observe_then_exit(1);
    aligned360_after_long_blank(-1);aligned360_after_long_blank(1);
    unproved_still_releases(1);unproved_still_releases(0);
    observation_does_not_bypass_release(0);observation_does_not_bypass_release(1);
    observation_does_not_bypass_release(2);
    puts("PASS: v35 production vision publish/runtime receive retains committed75deg episode through blank>250ms and ordinary path0, ordinary turn+cap preserved, final315deg corridor forces real360, long-blank360 reacquires aligned exit without force, EXIT/RECOVER/single success with entry1+exitfound3+success2; no-commit/legacy/STOP/stale/30s still release, both sides");
    return 0;
}
