#include "ring.h"
#include "ring_link.h"
#include "ring_runtime.h"
#include "ring_vision.h"
#include "ring_uart_diag.h"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
stc_SCB_t uart_port;char uart_capture[32768];unsigned uart_capture_count;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL timed integration:%d %s ph%d man%u active%u turn%.1f\n",__LINE__,#c,ring_runtime_diagnostics()->phase,ring_runtime_diagnostics()->maneuver_stage,ring_runtime_diagnostics()->active,turn);exit(1);}}while(0)
static uint8_t pixels[120][188];static float turn,speed,revs;static int track,error,beeps;static uint32_t start;static int exercise_visual_pid;static float err_filter,last_error,turn_filter,ordinary_turn;
static void ticks(int raw,int n)
{while(n--){int b=ring_runtime_tick((int16_t)raw,1);if(b==1)++beeps;}}
static void control(void)
{
 speed=100;turn=40;
 if(exercise_visual_pid){
  float proportional=1.35f*error/(1.f+fabsf((float)error)/40.f),differential;
  err_filter+=.6f*(error-err_filter);differential=1.5f*(err_filter-last_error);last_error=err_filter;
  turn_filter+=.4f*(proportional+differential-turn_filter);turn=turn_filter;
 }
 ordinary_turn=turn;ring_runtime_observe_motion(100,0,0,0,0);
 ring_runtime_apply(&speed,&turn,revs,track);uart_port.used=0;ring_uart_diag_poll(error,100,-100);
}
static void frame(void)
{track=1;error=12;ring_vision_before_frame((const uint8_t(*)[188])pixels);ring_vision_select(&error,&track);ring_vision_after_frame((const uint8_t(*)[188])pixels);control();}
static void boot(void)
{
 int i;memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);beeps=0;revs=0;track=1;exercise_visual_pid=0;err_filter=last_error=turn_filter=0;
 ring_runtime_init(.1f);ring_runtime_set_operator(1,0);
 for(i=0;i<2000;++i)(void)ring_runtime_tick(0,0);
 (void)ring_runtime_tick(0,1);control();ring_vision_init();rtg_straight(pixels);frame();
}
static void acquire(int side)
{
 int i;boot();for(i=0;i<120;++i){rtg_pose_t pose=rtg_approach(i);rtg_render(pixels,side,&pose);ticks(0,20);revs+=.02f;frame();if(ring_result.detected_events)break;}
 CHECK(ring_result.timed_entry && ring_runtime_diagnostics()->maneuver_stage==1);
 start=ring_runtime_diagnostics()->now_ms;
}
static void route(int side)
{
 int i;acquire(side);rtg_straight(pixels);
 /* Recognition is already structural and side-locked. Loss of N/F after
  * the beep must not veto the scheduled75. A cold straight still never arms. */
 for(i=1;i<=(int)(RING_MANEUVER_DELAY_MS/20u);++i){ticks(0,20);revs+=.02f;frame();
  CHECK(speed>0);
  if(i*20u<RING_MANEUVER_DELAY_MS)CHECK(ring_runtime_diagnostics()->maneuver_stage==1 && !ring_runtime_diagnostics()->force_active);
 }
 CHECK(beeps==1 && ring_runtime_diagnostics()->maneuver_stage==2 && ring_runtime_diagnostics()->force_active);
 for(i=0;i<6;++i){ticks(0,20);revs+=.02f;frame();}
 CHECK(side*turn>0 && !ring_result.committed && !ring_result.exited_events);
}
static void interruption(int stop)
{
 float forward=100,output=25;acquire(1);ticks(0,200);frame();
 if(stop){ring_runtime_set_operator(0,0);ticks(0,20);frame();CHECK(ring_runtime_diagnostics()->maneuver_stage==4 && !ring_runtime_diagnostics()->force_active);}
 else {ticks(0,301);ring_runtime_poll_timeout(&forward,&output);CHECK(forward==100 && output==40);control();CHECK(!ring_runtime_diagnostics()->active && ring_runtime_diagnostics()->release_reason==4);}
 CHECK(!ring_runtime_watchdog_stop());
}
static void cold_and_preview(void)
{
 int i;rtg_pose_t pose=rtg_approach(45);boot();rtg_cross(pixels);
 for(i=0;i<30;++i){ticks(0,20);frame();CHECK(speed==100 && turn==40 && !ring_runtime_diagnostics()->active && !ring_runtime_diagnostics()->maneuver_stage && !beeps);}
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);
 ring_runtime_init(.1f);ring_runtime_set_operator(0,0);beeps=0;
 for(i=0;i<2000;++i)(void)ring_runtime_tick(0,0);
 control();ring_vision_init();rtg_render(pixels,1,&pose);
 for(i=0;i<40;++i){int k;for(k=0;k<20;++k)if(ring_runtime_tick(0,0)==1)++beeps;
  frame();CHECK(!ring_runtime_diagnostics()->active && !ring_runtime_diagnostics()->maneuver_stage && speed==100 && turn==40);
 }
 CHECK(beeps==1 && ring_vision_preview());
}
/* Fixed-world images, paired controller feedback, physically consistent yaw
 * and wheel displacement. The synthetic speed deliberately reaches F in
 * 1000ms; this checks software handoff/lap, not real camera timing. */
static rtg_pose_t last_pose;static unsigned visual_samples,post_entry_frames;
static void drive(int side,rtg_pose_t pose)
{
 /* Finish at360.02deg so float gyro integration/1mdeg wire quantization
  * cannot turn an exact mathematical360 into359.994 and hide exit tests. */
 if(side*pose.heading_deg>=359.9f)pose.heading_deg+=side*.02f;
 float dx=pose.x-last_pose.x,dy=pose.y-last_pose.y;
 int raw=(int)lroundf((pose.heading_deg-last_pose.heading_deg)*500.f);
 revs+=sqrtf(dx*dx+dy*dy)/RTG_WHEEL_CIRCUMFERENCE;
 ticks(raw,20);rtg_render(pixels,side,&pose);frame();last_pose=pose;
 CHECK(!ring_result.stop && !ring_result.fault);
 if(ring_runtime_diagnostics()->active && ring_runtime_diagnostics()->maneuver_stage==3) {
  ++post_entry_frames;
  if(!ring_result.exit_turn_request)CHECK(!ring_runtime_diagnostics()->force_active);
  else CHECK((ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER) &&
      ring_result.exit_latched && !ring_result.exit_confirmed);
  if(post_entry_frames>2)CHECK(!ring_result.timed_entry);
  if(!ring_result.exit_turn_request)CHECK(ring_runtime_diagnostics()->force_target10==0);
  if(post_entry_frames*20u>RING_ENTRY_HANDOVER_MS+20u && !ring_result.exit_turn_request)
   CHECK(fabsf(turn-(ordinary_turn+ring_runtime_diagnostics()->applied_boost))<.11f);
  if(!ring_result.timed_entry && ring_result.path_valid && ring_result.phase!=RING_RECOVER) {
   CHECK(error==ring_result.steer_error);
   if(error)++visual_samples;
  }
 }
}
static void full_world(int side)
{
 int i,shared_completed=0;uint32_t entered_ms;uint8_t recognized[120][188];
 rtg_pose_t pose={0,-180,0};float first_y;boot();exercise_visual_pid=1;visual_samples=post_entry_frames=0;last_pose=pose;
 for(i=0;i<120;++i){pose=rtg_approach(i);drive(side,pose);if(ring_result.detected_events)break;}
 CHECK(ring_result.timed_entry);first_y=pose.y;
 entered_ms=ring_runtime_diagnostics()->now_ms;memcpy(recognized,pixels,sizeof(recognized));
 for(i=1;i<=(int)(RING_MANEUVER_DELAY_MS/20u);++i){pose=(rtg_pose_t){0,first_y+(60-first_y)*i/(RING_MANEUVER_DELAY_MS/20.f),0};drive(side,pose);}
 for(i=0;i<12 && ring_runtime_diagnostics()->maneuver_stage==1;++i){pose.y+=.2f;drive(side,pose);}
 if(ring_runtime_diagnostics()->maneuver_stage!=2)fprintf(stderr,"entry blocked stage%u gate%u N%d F%d count%d speed%d\n",ring_result.approach_stage,ring_result.mouth_gate,ring_result.near_row,ring_result.far_row,ring_result.mouth_far_count,ring_runtime_diagnostics()->final_target10);
 CHECK(ring_runtime_diagnostics()->maneuver_stage==2);
 for(i=1;i<=RTG_JOIN_STEPS;++i){drive(side,rtg_join(side,i));CHECK(!ring_result.exited_events && ring_owns_path());}
 CHECK(ring_result.phase==RING_CIRCLE && ring_result.committed && ring_result.left_entry);
 CHECK(!ring_result.timed_entry && ring_runtime_diagnostics()->maneuver_stage==3);
 for(i=1;i<=RTG_ARC_STEPS;++i){drive(side,rtg_outer_arc(side,i));CHECK(ring_owns_path() && !ring_result.exited_events && ring_result.phase==RING_CIRCLE);}
 for(i=1;i<=RTG_JOIN_STEPS;++i){drive(side,rtg_return_near(side,i));CHECK(ring_owns_path() && !ring_result.exited_events);}
 /* A confirmed current exit can finish on this straight before the old
  * far-mouth event. Unconfirmed release remains a failure. The preceding
  * return-near loop still requires continuous ownership before full360. */
 for(i=1;i<RTG_SHARED_STEPS;++i){
  drive(side,rtg_shared_return(side,i));
  CHECK(speed>0 && !ring_result.stop && !ring_result.fault);
  if(ring_owns_path()) {
   CHECK(!ring_result.exited_events && !shared_completed);
  } else {
   CHECK(ring_result.exited_events==1 && ring_result.exit_confirmed &&
       ring_result.exit_latched && !ring_result.release_reason);
   CHECK(fabsf(last_pose.x)<.01f && side*last_pose.heading_deg>=360.f && ring_result.lap_deg>=360.f);
   CHECK(!ring_runtime_diagnostics()->active && track==1 && error==12 && turn==ordinary_turn);
   if(!shared_completed) {
    CHECK(ring_result.path_valid && ring_result.path_source==4 &&
        abs(ring_result.near_x-94)<=2 && abs(ring_result.look_x-94)<=2 &&
        abs(ring_result.mid[RING_NEAR_ROW]-94)<=2 && abs(ring_result.mid[RING_LOOK_ROW]-94)<=2);
    CHECK(pixels[RING_NEAR_ROW][ring_result.mid[RING_NEAR_ROW]] &&
        pixels[RING_LOOK_ROW][ring_result.mid[RING_LOOK_ROW]]);
    printf("PASS: side%d first shared exit pose(%.2f,%.2f,%.2f), net%.3fdeg, locked near/look%d/%d, ordinary error%d restored\n",
        side,last_pose.x,last_pose.y,last_pose.heading_deg,ring_result.lap_deg,
        ring_result.near_x,ring_result.look_x,error);
    shared_completed=1;
   }
  }
 }
 drive(side,rtg_shared_return(side,RTG_SHARED_STEPS));
 for(i=1;i<=RTG_DEPART_STEPS;++i){
  drive(side,rtg_depart(side,i));
  if(shared_completed)CHECK(ring_result.exited_events==1 && !ring_owns_path() &&
      !ring_runtime_diagnostics()->active && speed>0 && !ring_result.stop && !ring_result.fault &&
      !ring_result.release_reason && track==1 && error==12 && turn==ordinary_turn);
 }
 if(!ring_result.exited_events)fprintf(stderr,"end side%d stage%u lap%.4f gate%u N%d F%d first%d prog%d ref%u count%u\n",side,ring_result.approach_stage,ring_result.lap_deg,ring_result.mouth_gate,ring_result.near_row,ring_result.far_row,ring_result.mouth_first_y,ring_result.mouth_progress,ring_result.mouth_ref,ring_result.mouth_far_count);
 CHECK(ring_result.exited_events==1 && !ring_owns_path());
 exercise_visual_pid=0;
 ticks(0,600);frame();CHECK(beeps==6);
 CHECK(visual_samples>100);
 printf("PASS: default timed full-world side%d:75 hands nonzero path error to original PID, measured CIRCLE/360/shared straight/exit; no angular heading lock\n",side);
 /* The existing1.2s/0.5rev post-exit cleanup can finish, but cannot
  * authorize another mode entry before15s from that first recognition. */
 rtg_straight(pixels);
 for(i=0;i<70;++i){ticks(0,20);revs+=.04f;frame();}
 CHECK(ring_result.phase==RING_NORMAL && !ring_runtime_diagnostics()->active);
 CHECK(ring_runtime_diagnostics()->now_ms-entered_ms<RING_REENTRY_BLOCK_MS-100u);
 memcpy(pixels,recognized,sizeof(pixels));
 while(ring_runtime_diagnostics()->now_ms-entered_ms<14960u){
  ticks(0,20);revs+=.04f;frame();
  CHECK(ring_result.detected_events==1 && !ring_runtime_diagnostics()->active && !ring_runtime_diagnostics()->maneuver_stage);
  CHECK(speed==100 && turn==40 && beeps==6);
 }
 ticks(0,(int)(15000u-(ring_runtime_diagnostics()->now_ms-entered_ms)));frame();
 CHECK(ring_result.detected_events==1);
 for(i=0;i<4;++i){ticks(0,20);revs+=.04f;frame();}
 CHECK(ring_result.detected_events==2 && ring_runtime_diagnostics()->active && ring_runtime_diagnostics()->maneuver_stage==1);
 ticks(0,20);frame();CHECK(beeps==7);
 printf("PASS: side%d completed exit cannot re-enter/beep/force until15s from first mode entry; fresh detection then arms once\n",side);
}
int main(void)
{route(-1);route(1);interruption(0);interruption(1);cold_and_preview();full_world(-1);full_world(1);puts("PASS: default paired end-to-end beep -> configured heading hold -> bounded75 with live gyro then measured visual follow; blank frames cannot cause blind360, repeated commands do not reset, K29 motion telemetry, branchless completion releases original outputs without a fake exit, operator/stale releases without ring lock-stop, default standby beep without motion/cold cross isolation");return 0;}
