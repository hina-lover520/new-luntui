#include "../code/ring.c"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL timed vision:%d %s ph%d timed%d feedback%u lap%.1f\n",__LINE__,#c,ring_result.phase,ring_result.timed_entry,maneuver_feedback,ring_result.lap_deg);exit(1);}}while(0)
static uint8_t pixels[120][188],saved[120][188];
static ring_motion_t m;
static void step(unsigned feedback,int progress)
{
 ring_observe_maneuver_feedback(feedback,(int16_t)progress);
 ring_observe_frame_interval(20);ring_observe_track(1);
 memcpy(saved,pixels,sizeof(saved));ring_update((const uint8_t(*)[188])pixels,&m);
 CHECK(!memcmp(saved,pixels,sizeof(saved)) && ring_scan_pixels<=3u*188u*120u);
}
static void recognize(int side)
{
 int i;ring_init();m=(ring_motion_t){0,0,0,1};
 for(i=0;i<120;++i){rtg_pose_t p=rtg_approach(i);rtg_render(pixels,side,&p);m.now_ms+=20;m.wheel_revs+=.02f;step(0,0);if(ring_result.detected_events)break;}
 CHECK(ring_result.detected_events==1 && ring_result.phase==RING_APPROACH);
 CHECK(ring_result.timed_entry && ring_result.ring_side==side && ring_owns_path());
 CHECK(ring_speed_limit()==100 && ring_blend_error(35)==ring_result.steer_error && ring_turn_boost()==0);
}
static void continuous_lap(int side)
{
 int i;float origin=side*4.f;recognize(side);
 rtg_straight(pixels);
 for(i=0;i<50;++i){m.now_ms+=20;m.wheel_revs+=.02f;m.yaw_deg=origin;step(1,0);CHECK(ring_result.phase==RING_APPROACH && !ring_result.lap_valid && ring_result.timed_entry);}
 m.now_ms+=20;step(2,0);CHECK(ring_result.phase==RING_ENTER && ring_result.lap_valid);
 CHECK(fabsf(ring_result.lap_origin_yaw-origin)<.01f);
 memset(pixels,0,sizeof(pixels));
 for(i=1;i<=15;++i){m.now_ms+=40;m.yaw_deg=origin+side*i*4.f;m.wheel_revs+=.04f;step(2,i*40);
  CHECK(ring_result.phase==RING_ENTER && ring_result.timed_entry && !ring_result.committed && !ring_result.branch_ok && !ring_result.stop && !ring_result.exited_events);
 }
 CHECK(!ring_result.force_entry); /* only the new control maneuver may turn */
 m.now_ms+=40;m.yaw_deg=origin+side*75;step(3,750);
 CHECK(!ring_result.timed_entry && !ring_result.committed && !ring_result.exited_events);
 CHECK(ring_result.phase==RING_COOLDOWN && ring_speed_limit()==-1 && !ring_result.stop);
 CHECK(ring_blend_error(17)==17); /* never spin blindly to360 */

}
static void ordinary(void)
{
 int k;ring_init();m=(ring_motion_t){0,0,0,1};rtg_cross(pixels);
 for(k=0;k<40;++k){m.now_ms+=20;step(0,0);CHECK(!ring_result.detected_events && !ring_result.timed_entry && !ring_owns_path());}
 rtg_straight(pixels);
 for(k=0;k<40;++k){m.now_ms+=20;step(0,0);CHECK(ring_blend_error(17)==17 && ring_speed_limit()==-1);}
}
int main(void)
{continuous_lap(-1);continuous_lap(1);ordinary();puts("PASS: default vision recognition arms without old F gate; configured straight then origin-correct ENTER,75 hands steering to visual PID, absent branch releases without heading lock or false exit, plain cross/straight untouched");return 0;}
