/* Prepared-state production integration, not a claimed driven full lap.
 * Recognition/commit/history are explicitly prepared. Real gyro integration
 * advances the runtime through the original75deg entry to315deg. Current
 * tilted corridor images then execute ring_update -> production publish ->
 * runtime accept -> real final gyro turn -> visual EXIT/RECOVER/COOLDOWN.
 * No replacement publisher, command encoder or controller is used here. */
#include "../code/ring.c"
#include "../code/ring_vision.c"
#include "ring_runtime.h"
#include <stdio.h>

static uint8_t camera[120][188];
static int direction,track,error;
static unsigned pulses,pair_first_ms,pair_second_ms,success_ms;
static float revs,speed,turn;
static const ring_runtime_diagnostics_t *control_diag(void)
{return ring_runtime_diagnostics();}
#define VERIFY(c) do{if(!(c)){fprintf(stderr,"FAIL v34 publish:%d %s side%d ph%d man%u force%u lap%.3f active%u out%.2f\n",\
    __LINE__,#c,direction,ring_result.phase,control_diag()->maneuver_stage,\
    control_diag()->exit_force_stage,ring_result.lap_deg,control_diag()->active,turn);exit(1);}}while(0)

static void gyro_ticks(int raw,unsigned count)
{
    while(count--) {
        int8_t beep=ring_runtime_tick((int16_t)raw,1);
        if(beep==1) {
            ++pulses;
            if(pulses==2u)pair_first_ms=control_diag()->now_ms+1u;
            if(pulses==3u)pair_second_ms=control_diag()->now_ms+1u;
            if(pulses==5u)success_ms=control_diag()->now_ms+1u;
        }
    }
}
static void apply_runtime(void)
{
    speed=100.f;turn=17.f;
    ring_runtime_observe_motion(100.f,0.f,0.f,0,0);
    ring_runtime_apply(&speed,&turn,revs,track);
}
static void road(float relative_angle)
{
    int x,y;float a=relative_angle*.01745329252f;
    memset(camera,0,sizeof(camera));
    for(y=0;y<120;++y)for(x=0;x<188;++x) {
        float dx=(x-94.f)*cosf(a)-(108.f-y)*sinf(a);
        if(fabsf(dx)<=29.5f)camera[y][x]=255;
    }
}
static void visual_frame(void)
{
    track=1;error=12;
    ring_vision_before_frame((const uint8_t(*)[188])camera);
    ring_vision_select(&error,&track);
    ring_vision_after_frame((const uint8_t(*)[188])camera);
    apply_runtime();
}
static ring_packet_t command_snapshot(void)
{
    ring_packet_t packet;
    VERIFY(ring_link_read(ring_command_mailbox,&packet));
    return packet;
}
static void prepared_entry_to_315(void)
{
    unsigned i,n;ring_packet_t feedback;
    memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);
    memset(ring_diagnostic_mailbox,0,32);memset(ring_entry_diagnostic_mailbox,0,32);
    memset(ring_geometry_diagnostic_mailbox,0,32);memset(ring_topology_diagnostic_mailbox,0,32);
    memset(ring_mouth_diagnostic_mailbox,0,32);memset(ring_exit_diagnostic_mailbox,0,32);
    pulses=pair_first_ms=pair_second_ms=success_ms=0;revs=0.f;track=1;
    ring_runtime_init(.1f);ring_runtime_set_operator(0,0);
    for(i=0;i<2200u;++i)(void)ring_runtime_tick(0,0);
    ring_runtime_set_operator(1,0);gyro_ticks(0,1u);apply_runtime();
    ring_vision_init();road(0.f);visual_frame();
    VERIFY(ring_link_read(ring_motion_mailbox,&feedback));
    VERIFY(epoch==(uint32_t)feedback.word[RL_EPOCH] && motion.ready);
    /* Prepare the recognition decision; all packets below are built by the
     * production vision publisher, including entrytiming/flags/version. */
    ring_result.phase=RING_APPROACH;ring_result.detected_events=1;
    ring_result.route=(int8_t)-direction;ring_result.ring_side=(int8_t)direction;
    ring_result.entry_side=(int8_t)direction;ring_result.timed_entry=1;
    ring_result.force_entry=ring_result.path_valid=1;ring_result.steer_error=0;
    publish();apply_runtime();
    for(i=0;i<RING_MANEUVER_DELAY_MS/20u+3u;++i) {
        gyro_ticks(0,20u);revs+=.03f;publish();apply_runtime();
        if(control_diag()->maneuver_stage==2u)break;
    }
    VERIFY(control_diag()->maneuver_stage==2u && pulses==1u);
    ring_result.phase=RING_ENTER;ring_result.force_entry=0;
    for(n=0;n<3152u;) {
        unsigned step=3152u-n>20u?20u:3152u-n;
        gyro_ticks(direction*1000,step);revs+=step*.0015f;
        publish();apply_runtime();n+=step;
    }
    VERIFY(control_diag()->maneuver_stage==3u && control_diag()->maneuver_turn10>=3150);
    /* Prepare already proven branch commitment/history. This test is about
     * the actual outbound control protocol, not camera entry recognition. */
    ring_result.phase=RING_CIRCLE;ring_result.timed_entry=0;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lap_origin_yaw=0;ring_result.lane_width=60;
    ring_result.approach_stage=RING_TANGENT_LEFT;
    timed_started=1;have_path=1;seed_identity_lost=0;lap_completed=0;
    ring_since=phase_since=last_path_fault_ms=control_diag()->now_ms;
    last_path_ms=vision_ms;tangent_lap_revs=0.f;
    trusted_steer=0;trusted_near=trusted_look=94;
    for(i=0;i<RING_H;++i)previous_mid[i]=ring_result.mid[i]=94;
}
static void execute_side(int side)
{
    unsigned n,declared=0,started_ms=0;ring_packet_t packet;
    direction=side;prepared_entry_to_315();
    for(n=0;n<9u && ring_result.phase!=RING_EXIT;++n) {
        gyro_ticks(0,20u);revs+=.03f;
        road(side*(360.f-control_diag()->maneuver_turn10*.1f));visual_frame();
    }
    VERIFY(ring_result.phase==RING_EXIT && ring_result.exit_turn_request &&
        ring_result.exit_latched && ring_result.path_valid);
    VERIFY(control_diag()->active && control_diag()->phase==RING_EXIT &&
        control_diag()->exit_force_stage==1u && control_diag()->force_active);
    packet=command_snapshot();
    VERIFY((uint32_t)packet.word[RC_FAULT]&RING_COMMAND_EXIT_TURN);
    VERIFY(!ring_command_boost(packet.word[RC_PHASE]));
    started_ms=control_diag()->now_ms;
    /* Rotate the current exit picture consistently with real gyro yaw.
     * Neither the control completion nor its feedback is injected. */
    for(n=0;n<40u && control_diag()->exit_force_stage==1u;++n) {
        gyro_ticks(side*1000,20u);revs+=.03f;
        road(side*(360.f-control_diag()->maneuver_turn10*.1f-2.f));visual_frame();
        VERIFY(control_diag()->active && !ring_result.exited_events && speed>0.f);
        VERIFY((uint32_t)command_snapshot().word[RC_FAULT]&RING_COMMAND_EXIT_TURN);
    }
    VERIFY(control_diag()->exit_force_stage==2u && control_diag()->maneuver_turn10>=3600);
    VERIFY(control_diag()->now_ms-started_ms<1000u);
    VERIFY(!ring_result.exited_events && !control_diag()->exited);
    for(n=0;n<40u && !ring_result.exited_events;++n) {
        gyro_ticks(0,20u);revs+=.03f;
        road(side*(360.f-control_diag()->maneuver_turn10*.1f));visual_frame();
        VERIFY(speed>0.f && !ring_result.release_reason);
        packet=command_snapshot();
        if(ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER) {
            VERIFY((uint32_t)packet.word[RC_FAULT]&RING_COMMAND_EXIT_TURN);
            VERIFY(control_diag()->active);
        }
        if(ring_result.phase==RING_RECOVER)declared=1;
    }
    VERIFY(declared && ring_result.exited_events==1u && ring_result.exit_confirmed);
    VERIFY(ring_result.phase==RING_COOLDOWN && !ring_result.exit_turn_request);
    packet=command_snapshot();
    VERIFY(!((uint32_t)packet.word[RC_FAULT]&RING_COMMAND_EXIT_TURN));
    VERIFY(control_diag()->command_sequence==(uint32_t)packet.word[RL_SEQUENCE] &&
        control_diag()->phase==RING_COOLDOWN && control_diag()->exited==1u && !control_diag()->active);
    VERIFY(track==1 && error==12 && turn==17.f);
    gyro_ticks(0,600u);apply_runtime();
    VERIFY(pulses==6u && pair_first_ms && pair_second_ms && success_ms);
    /* One fast entry pulse + three exit-found pulses + two success pulses. */
    for(n=0;n<10u;++n) {
        gyro_ticks(0,20u);revs+=.03f;road(0.f);visual_frame();
    }
    VERIFY(pulses==6u && ring_result.exited_events==1u && control_diag()->exited==1u);
}
int main(void)
{
    execute_side(-1);execute_side(1);
    puts("PASS: v34 prepared315deg current corridor through PRODUCTION vision publish/runtime receive, real gyro final360, feedback-controlled EXIT/RECOVER, COOLDOWN clears request and accepts one success; entry1+exitfound3+success2 exactly once, both sides");
    return 0;
}
