/* Real frozen v19 images test geometry, not a reconstructed video or car.
 * The route/phase are prepared so this cannot claim recognition success.
 * Independent fixtures fix the visible island and intended right bend.
 * Mirrors exercise left entry; public world tests still enforce full lap. */
#include "../code/ring.c"
#include "ring_force.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL normal:%d %s id%d side%d width%d e%d\n",__LINE__,#c,ident,side,width,ring_result.steer_error);exit(1);}}while(0)
static int ident,side,width;
static unsigned observations;
static uint8_t pixels[120][188],original[120][188];
static uint32_t crc32(const uint8_t*b,size_t n)
{uint32_t c=~0u;size_t i;int k;for(i=0;i<n;++i){c^=b[i];for(k=0;k<8;++k)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));}return ~c;}
static void load(uint32_t crc)
{
 char path[2048];const char*s=__FILE__,*slash=strrchr(s,'/'),*back=strrchr(s,'\\');FILE*f;uint8_t bits[2820];int x,y;
 if(!slash||(back&&back>slash))slash=back;
 CHECK(slash);snprintf(path,sizeof(path),"%.*s/fixtures/v19_real/entry_%d.bin",(int)(slash-s),s,ident);
 f=fopen(path,"rb");CHECK(f);CHECK(fread(bits,1,2820,f)==2820 && fgetc(f)==EOF);fclose(f);CHECK(crc32(bits,2820)==crc);
 for(y=0;y<120;++y)for(x=0;x<188;++x){int k=y*188+(side>0?x:187-x);pixels[y][x]=(uint8_t)((bits[k>>3]&(128>>(k&7)))?255:0);}
 memcpy(original,pixels,sizeof(pixels));
}
static void measured_bend(uint32_t crc)
{
 ring_motion_t m={100,side*40.f,1,1};island_t a;int y,old,normal,accepted;ring_force_t f;float output;
 load(crc);ring_init();ring_result.lane_width=(int16_t)width;
 ring_result.phase=RING_ENTER;ring_result.route=(int8_t)-side;ring_result.ring_side=(int8_t)side;
 ring_result.left_entry=1;ring_result.approach_stage=RING_TANGENT_LEFT;
 scan_geometry((const uint8_t(*)[188])pixels,&m);a=find_island();
 accepted=build_path(&a,a.valid&&a.growing&&a.returning,0,0,0,&m);old=ring_result.steer_error;
 /* Fixture24 is the directly reproduced v19 accepted reverse command. */
 if(ident==24 && width==105 && side>0)CHECK(accepted==1 && old<0);
 CHECK(tangent_normal_path());normal=ring_result.steer_error;
 CHECK(side*normal>0 && ring_result.guide_y<RING_LOOK_ROW);
 ring_result.timed_entry=1;CHECK(ring_blend_error(0)==0);
 ring_observe_maneuver_feedback(3,800);
 CHECK(!tangent_timed_entry(&a,&m,0) && !ring_result.timed_entry);
 CHECK(ring_blend_error(0)==normal); /* v27 incorrectly returned zero */
 CHECK(side*(ring_result.guide_x-RING_W/2)>0);
 for(y=0;y<120;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);
 CHECK(!memcmp(pixels,original,sizeof(pixels)));
 CHECK(!ring_result.committed && !ring_result.exited_events && !ring_result.branch_ok);
 if(ident==41) {
  /* Prepared previous same edge is shifted one pixel, explicitly a local
   * association test, not an assertion of actual unseen vehicle frames. */
  mouth_local_valid=1;mouth_local_ms=mouth_local_curve_ms=vision_ms;
  mouth_local_yaw=m.yaw_deg;mouth_local_revs=m.wheel_revs;
  ring_result.committed=0;
  for(y=0;y<120;++y)mouth_local_edge[y]=(int16_t)(
      observed_left[y]>=0?(side>0?observed_right[y]:observed_left[y])-side:-1);
  /* This geometry-only fixture deliberately has no earlier strong
   * observation. A tail-supported local path cannot acquire history. */
  CHECK(!tangent_local_path(&m));
  have_path=1;last_path_ms=vision_ms;last_path_fault_ms=m.now_ms;
  last_yaw=m.yaw_deg;support_last_revs=m.wheel_revs;ring_result.visual_dt_ms=20;
  CHECK(tangent_local_path(&m)>0);
  CHECK(side*ring_result.steer_error>0 && mouth_local_curve_ms==vision_ms);
  CHECK(!ring_result.committed && !ring_result.exited_events);
 }
 /* The SAME finite force budget expires; its successor uses the measured
  * bend, not the reverse error above. No budget extension or sign clamp. */
 ring_force_begin(&f,0,side,0,0);
 (void)ring_force_apply(&f,0,100,side*10,.1f,side*100,side,1,1,0);
 output=ring_force_apply(&f,normal*.8f,1200,side*101,1.2f,side*100,side,1,1,1);
 CHECK(!f.active && f.spent && f.reason==4 && output==normal*.8f && output*side>0);
 observations++;
}
static void unavailable_is_not_projected(void)
{
 int y;ring_init();ring_result.phase=RING_ENTER;ring_result.route=-1;ring_result.ring_side=1;
 for(y=0;y<120;++y)inner[y]=-1;
 CHECK(!tangent_normal_path() && !ring_result.committed && !ring_result.exited_events);
}
int main(void)
{
 CHECK(RING_TRACK_MODE==1);
 for(side=-1;side<=1;side+=2)for(width=90;width<=110;width+=5){
  ident=24;measured_bend(4173809944u);ident=41;measured_bend(3845543662u);
 }
 unavailable_is_not_projected();
 printf("PASS: %u real-image/mirror normal-centre paths stay white; v19 reverse error reproduced and corrected, finite force handoff keeps measured bend, no recognition/commit/lap credit\n",observations);
 return 0;
}
