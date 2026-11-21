/* Confirm from real rendered ring structure; release through the public
 * production path. No direct assignment of phase or lock state. */
#include "../code/ring.c"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL reentry:%d %s ph%d now%lu start%lu count%lu\n",__LINE__,#c,ring_result.phase,(unsigned long)m.now_ms,(unsigned long)first,(unsigned long)ring_result.detected_events);exit(1);}}while(0)
static uint8_t pixels[120][188],entry_image[120][188];
static ring_motion_t m;
static uint32_t first;
static void step(void)
{
 ring_observe_frame_interval(20);ring_observe_track(1);
 ring_update((const uint8_t(*)[188])pixels,&m);
}
static void acquire(int side,uint32_t start)
{
 int i;ring_init();m=(ring_motion_t){start,0,0,1};
 for(i=0;i<120;++i){rtg_pose_t pose=rtg_approach(i);rtg_render(pixels,side,&pose);m.now_ms+=20;m.wheel_revs+=.02f;step();if(ring_result.detected_events)break;}
 first=m.now_ms;
 CHECK(ring_result.detected_events==1 && ring_result.phase==RING_APPROACH && ring_result.timed_entry);
 memcpy(entry_image,pixels,sizeof(pixels));
}
static void cancel_and_observe(int reset)
{
 unsigned i;
 ring_release_control(1,&m);CHECK(ring_result.phase==RING_COOLDOWN);
 if(reset)ring_reset_episode();
 rtg_straight(pixels);
 for(i=0;i<70;++i){m.now_ms+=20;m.wheel_revs+=.04f;step();CHECK(!ring_owns_path() && ring_speed_limit()==-1 && ring_blend_error(17)==17 && !ring_result.stop);}
 CHECK(ring_result.phase==RING_NORMAL);
}
static void retry(int side,int reset,uint32_t start)
{
 unsigned i;uint32_t previous;
 acquire(side,start);cancel_and_observe(reset);
 memcpy(pixels,entry_image,sizeof(pixels));
 /* Repeated valid candidates for the entire blocked interval cannot
  * confirm, turn, beep, extend the timer, or limit ordinary speed. */
 for(i=0;i<300;++i){m.now_ms+=20;m.wheel_revs+=.04f;step();
  CHECK(ring_result.phase==RING_NORMAL && !ring_owns_path() && ring_speed_limit()==-1);
  CHECK(!ring_result.confirm_frames && ring_result.detected_events==(reset?0u:1u));
 }
 previous=m.now_ms;CHECK(previous-first<RING_REENTRY_BLOCK_MS);
 m.now_ms=first+14999u;step();CHECK(!ring_owns_path() && !ring_result.confirm_frames);
 /* At15s only fresh confirmation starts. Do not bank blocked candidates. */
 m.now_ms=first+15000u;step();CHECK(!ring_owns_path() && ring_result.confirm_frames==1);
 for(i=1;i<RING_CONFIRM_FRAMES;++i){m.now_ms+=20;step();}
 CHECK(ring_owns_path() && ring_result.timed_entry && ring_result.detected_events==(reset?1u:2u));
 /* A second accepted entry starts its own15 seconds. */
 first=m.now_ms;cancel_and_observe(0);memcpy(pixels,entry_image,sizeof(pixels));
 for(i=0;i<50;++i){m.now_ms+=20;step();CHECK(!ring_owns_path());}
}
static void active_expiry(void)
{
 acquire(1,0);m.now_ms+=20;ring_observe_maneuver_feedback(2,0);step();
 CHECK(ring_result.phase==RING_ENTER && ring_owns_path());
 m.now_ms=first+15000u;ring_observe_maneuver_feedback(2,0);step();
 CHECK(ring_result.phase==RING_ENTER && ring_owns_path() && ring_result.detected_events==1);
}
int main(void)
{
 CHECK(RING_REENTRY_BLOCK_MS==15000u);
 retry(-1,0,0);retry(1,0,0);retry(-1,1,0);retry(1,1,0xfffff000u);
 active_expiry();
 puts("PASS:15s from accepted entry, both directions, cancel and temporary standby resets cannot unlock; candidates cannot renew/bank;14999ms blocked,15000ms fresh confirm;second entry re-arms;uint32 wrap;active ring survives expiry;ordinary outputs unchanged");
 return 0;
}
