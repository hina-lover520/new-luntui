#include "../code/ring.c"
#include "ring_link.h"
#include "ring_runtime.h"
#include "ring_vision.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL release:%d %s ph%d cause%d\n",__LINE__,#c,ring_result.phase,ring_result.release_reason);exit(1);}}while(0)
static uint8_t pixels[120][188];
static ring_motion_t m;
static uint32_t cmdseq,control_epoch;
static float forward,turn;
static void lost_or_deadline(int phase,int cause,int raw)
{
 int i;ring_init();memset(pixels,0,sizeof(pixels));
 ring_result.phase=(ring_phase_t)phase;ring_result.ring_side=1;ring_result.route=-1;
 ring_result.left_entry=ring_result.committed=1;ring_result.lap_valid=1;
 ring_result.detected_events=1;ring_result.turn_deg=55;ring_result.force_entry=1;
 m=(ring_motion_t){cause==2?30001u:100u,55,.5f,1};
 for(i=0;i<40 && ring_owns_path();++i){
  m.now_ms+=20;ring_observe_track(raw);ring_observe_frame_interval(20);
  ring_update((const uint8_t(*)[188])pixels,&m);
  CHECK(!ring_result.stop && !ring_result.fault && !ring_result.exited_events);
 }
 CHECK(ring_result.phase==RING_COOLDOWN && ring_result.release_reason==cause);
 CHECK(ring_result.decision==RING_DECISION_RELEASE && !ring_result.force_entry);
 CHECK(!ring_owns_path() && !ring_blocks_cross() && ring_speed_limit()==-1);
 CHECK(ring_blend_error(17)==17 && !ring_turn_boost());
}
static void exit_deadline(void)
{
 int x,y;ring_init();for(y=0;y<120;++y)for(x=0;x<188;++x)pixels[y][x]=(uint8_t)(x>=64 && x<=124?255:0);
 ring_result.phase=RING_EXIT;ring_result.ring_side=1;ring_result.route=-1;
 ring_result.left_entry=ring_result.committed=1;ring_result.lap_valid=1;
 m=(ring_motion_t){RING_EXIT_TIMEOUT_MS+1u,370,3,1};
 ring_observe_track(1);ring_observe_frame_interval(20);ring_update((const uint8_t(*)[188])pixels,&m);
 CHECK(ring_result.release_reason==3 && ring_result.phase==RING_COOLDOWN);
 CHECK(!ring_result.exited_events && !ring_result.stop && !ring_result.fault && ring_speed_limit()==-1);
}
static void tick(unsigned ms){while(ms--)(void)ring_runtime_tick(0,1);}
static void apply(void){forward=97.25f;turn=-17.5f;ring_runtime_apply(&forward,&turn,1,3);}
static void command(int phase,int release)
{
 ring_packet_t p={{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)control_epoch;
 p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
 p.word[RC_CAP]=ring_command_cap_turn(phase==RING_ENTER?90:-1,0);p.word[RC_DETECTED]=1;
 p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,0,1,26,70,-1);
 if(phase==RING_ENTER)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
 p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|((uint32_t)release<<RING_COMMAND_RELEASE_SHIFT));
 ring_link_write(ring_command_mailbox,&p,&cmdseq);apply();
}
static void stale_interrupt_fallback(void)
{
 ring_packet_t feedback;int raw,steer,x,y;unsigned i;
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);
 ring_runtime_init(.1f);ring_runtime_set_operator(1,0);
 for(i=0;i<2000;++i)(void)ring_runtime_tick(0,0);
 tick(1);apply();control_epoch=ring_runtime_diagnostics()->command_epoch;cmdseq=0;
 for(y=0;y<120;++y)for(x=0;x<188;++x)pixels[y][x]=(uint8_t)(x>=64 && x<=124?255:0);
 ring_vision_init();ring_vision_before_frame((const uint8_t(*)[188])pixels);
 steer=13;raw=1;ring_vision_select(&steer,&raw);ring_vision_after_frame((const uint8_t(*)[188])pixels);
 command(RING_ENTER,0);tick(46);command(RING_ENTER,0);CHECK(forward==90 && turn>0);
 tick(RING_LINK_TIMEOUT_MS+1);ring_runtime_poll_timeout(&forward,&turn);
 CHECK(forward==97.25f && turn==-17.5f && !ring_runtime_watchdog_stop());
 /* One-time restore cannot override a later original stop or a normal turn. */
 forward=0;turn=3;ring_runtime_poll_timeout(&forward,&turn);CHECK(forward==0 && turn==3);
 apply();CHECK(forward==97.25f && turn==-17.5f && !ring_runtime_diagnostics()->active);
 CHECK(ring_runtime_diagnostics()->release_reason==4);
 CHECK(ring_link_read(ring_motion_mailbox,&feedback));CHECK((uint32_t)feedback.word[RM_RESERVED]&RING_OPERATOR_ABORT_RING);
 command(RING_ENTER,0);CHECK(forward==97.25f && turn==-17.5f && !ring_runtime_diagnostics()->active);
 /* Vision consumes the explicit stale-episode notice and returns the
  * CURRENT ordinary error/state, with no fake exit or second beep. */
 ring_result.phase=RING_ENTER;ring_result.left_entry=1;ring_result.committed=1;
 ring_result.route=-1;ring_result.ring_side=1;
 ring_vision_before_frame((const uint8_t(*)[188])pixels);steer=-11;raw=1;
 ring_vision_select(&steer,&raw);ring_vision_after_frame((const uint8_t(*)[188])pixels);
 CHECK(steer==-11 && raw==1 && ring_result.phase==RING_COOLDOWN && ring_result.release_reason==4);
 CHECK(!ring_result.exited_events && !ring_result.stop && !ring_result.fault);
 apply();CHECK(forward==97.25f && turn==-17.5f && ring_runtime_diagnostics()->release_reason==4);
 CHECK(ring_link_read(ring_motion_mailbox,&feedback));CHECK(!((uint32_t)feedback.word[RM_RESERVED]&RING_OPERATOR_ABORT_RING));
 command(RING_NORMAL,4);CHECK(!ring_runtime_diagnostics()->pair_ok); /* wrong release phase rejected */
}
int main(void)
{
 int phase,raw;CHECK(RING_TRACK_MODE==1);
 for(phase=RING_ENTER;phase<=RING_EXIT;++phase)for(raw=0;raw<=2;++raw)lost_or_deadline(phase,1,raw);
 lost_or_deadline(RING_CIRCLE,2,1);exit_deadline();stale_interrupt_fallback();
 puts("PASS: production lost branch/total/exit/stale release without lock-stop; original errors/outputs preserved, 1ms one-shot fallback, no stale reacquisition, vision handshake, no fake exit");
 return 0;
}
