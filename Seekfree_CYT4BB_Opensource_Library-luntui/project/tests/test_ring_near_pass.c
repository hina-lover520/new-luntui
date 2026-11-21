/* Include the real core for bounded-history unit checks; world scenarios
 * below still feed only images/motion through the public ring_update API. */
#include "../code/ring.c"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL near-pass:%d %s ph%d stage%d N%d/%d missed%d clear%d corridor%d gate%u\n",__LINE__,#c,ring_result.phase,ring_result.approach_stage,mouth_near.x,mouth_near.y,mouth_near.missed,mouth_near_cleared,mouth_corridor_rows,ring_result.mouth_gate);exit(1);}}while(0)
static uint8_t pixels[RING_H][RING_W],before[RING_H][RING_W];
static ring_motion_t movement;
static unsigned frames;
static void observe(uint16_t dt,int raw)
{
 movement.now_ms+=40;ring_observe_frame_interval(dt);ring_observe_track(raw);
 memcpy(before,pixels,sizeof(before));ring_update((const uint8_t (*)[RING_W])pixels,&movement);
 CHECK(!memcmp(before,pixels,sizeof(before)));++frames;
}
static void reset(void){ring_init();movement=(ring_motion_t){0,0,0,1};}
static void ordinary(void)
{
 CHECK(!ring_owns_path()&&!ring_blocks_cross());
 CHECK(ring_blend_error(17)==17&&ring_turn_boost()==0&&ring_speed_limit()==-1);
 CHECK(!ring_result.stop&&!ring_result.fault&&!ring_result.left_entry&&!ring_result.committed);
}
/* Fixed annulus/straight world, same independent F region as the original
 * tests. A fixed camera mapping expands only the near band. It represents
 * incomplete rectification/cropping, NOT a claim of real-camera calibration.
 * Every through-road width in rows94..117 exceeds 90px: the old narrow
 * corridor cannot supply 18 good rows at default learned width60. */
static void render_wide(int side,float world_y,float scale)
{
 int x,y;
 for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){
  float k=y<=85?1.f:(y>=94?scale:1.f-(1.f-scale)*(y-85)/9.f);
  pixels[y][x]=(uint8_t)(rtg_world_white(side,(x-94)*k,world_y+119-y)?255:0);
 }
}
static void wide_world_enters_at_far(int side,float scale)
{
 int i,passed=0,saw_clipped=0;reset();
 for(i=0;i<=125;++i){
  float wy=-180.f+2.f*i;render_wide(side,wy,scale);movement.wheel_revs=i/60.f;observe(40,1);
  if(ring_result.phase==RING_APPROACH){
   CHECK(mouth_corridor_rows<18);if(mouth_corridor_clipped)saw_clipped=1;
   if(ring_result.approach_stage==RING_TANGENT_NEAR_PASSED){
    CHECK(wy>=-60.f);passed=1;CHECK(ring_result.mouth_pass_reason==2);
   }
  }
  if(ring_result.phase==RING_ENTER){
   int y;CHECK(saw_clipped&&passed);CHECK(wy>=RTG_FAR_MIN_WORLD&&wy<=RTG_FAR_MAX_WORLD);
   CHECK(ring_result.ring_side==side&&side*ring_result.steer_error>0);
   CHECK(ring_result.guide_x>=0&&pixels[ring_result.guide_y][ring_result.guide_x]);
   for(y=0;y<RING_H;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);
   CHECK(!ring_result.committed&&!ring_result.left_entry);return;
  }
  ordinary();
 }
 CHECK(0);
}
/* Stop immediately before the actually observed N-bottom transition.
 * Frozen image, an opaque dropout, a straight/cross replacement and
 * unknown capture time must not manufacture this transition. */
static void missing_is_not_passage(int side,int kind)
{
 int i;reset();
 for(i=0;i<=60;++i){render_wide(side,-180.f+2.f*i,.65f);movement.wheel_revs=i/60.f;observe(40,1);ordinary();}
 CHECK(ring_result.phase==RING_APPROACH&&!mouth_near_cleared);
 if(kind==1)memset(pixels,0,sizeof(pixels));
 if(kind==2)rtg_straight(pixels);
 if(kind==3)rtg_cross(pixels);
 for(i=0;i<18;++i){
  if(kind==4)render_wide(side,-60.f+2.f*i,.65f);
  movement.wheel_revs+=.02f;movement.yaw_deg+=side*.2f;
  observe(kind==4?65535u:40,kind==3?2:1);
  CHECK(ring_result.approach_stage==RING_TANGENT_NEAR_SEEN);
  CHECK(!mouth_near_cleared&&ring_result.phase!=RING_ENTER);ordinary();
 }
}
static void bounded_corner_recovery(void)
{
 mouth_track_t t;int i;
 ring_init();vision_ms=100;mouth_seed(&t,174,99,0);
 for(i=0;i<6;++i){vision_ms+=40;++t.missed;}
 CHECK(t.missed==6);CHECK(mouth_associate(&t,177,102,0,1));CHECK(t.missed==0);
 CHECK(t.progress==3&&t.count==2); /* measured net displacement, no timeout credit */
 vision_ms+=RING_OBSERVE_MS+1;t.missed=8;
 CHECK(!mouth_associate(&t,178,103,0,1));
 CHECK(!mouth_associate(&t,90,103,0,1));
}
/* An expired actual N observation can be detected afresh, but cannot
 * inherit the prior progress/F credits. Feed real public geometry again. */
static void expired_near_restarts(int side)
{
 int i;reset();
 for(i=0;i<=50;++i){render_wide(side,-180.f+2.f*i,.65f);movement.wheel_revs=i/60.f;observe(40,1);}
 CHECK(ring_result.phase==RING_APPROACH&&mouth_near.progress>=8);
 memset(pixels,0,sizeof(pixels));
 for(i=0;i<9;++i){observe(40,1);ordinary();}
 CHECK(mouth_near.missed>5);
 render_wide(side,-80.f,.65f);observe(40,1);
 CHECK(ring_result.mouth_near_y>=0&&ring_result.mouth_pass_reason==3);
 CHECK(mouth_near.count==1&&mouth_near.progress==0&&!tangent_near_close);
 CHECK(!mouth_near_cleared);ordinary();
}
int main(void)
{
 int side,k;bounded_corner_recovery();
 for(side=-1;side<=1;side+=2){
  wide_world_enters_at_far(side,.6f);wide_world_enters_at_far(side,.62f);wide_world_enters_at_far(side,.65f);
  for(k=0;k<5;++k)missing_is_not_passage(side,k);
  expired_near_restarts(side);
 }
 printf("PASS: %u near-pass frames; mirrored clipped wide mouths reach F without narrow corridor, bounded reacquisition, expiry restart, frozen/blank/straight/cross/unknown cannot pass N\n",frames);
 return 0;
}

