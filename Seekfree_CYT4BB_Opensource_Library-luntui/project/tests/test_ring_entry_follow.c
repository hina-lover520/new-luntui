/* CRC-checked field snapshots, followed by explicitly synthetic local
 * transformations with prepared N/F/edge history. No missing field frames
 * or vehicle dynamics are reconstructed. Full-world public tests remain
 * responsible for the independently declared F region and entire route. */
#include "../code/ring.c"
#include <stdio.h>
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL entry-follow:%d %s label=%s ph%d stage%d src%d fs%d gate%u local%d\n",__LINE__,#c,label,ring_result.phase,ring_result.approach_stage,ring_result.path_source,ring_result.mouth_source,ring_result.mouth_gate,mouth_local_valid);exit(1);}}while(0)
static uint8_t original[RING_H][RING_W],pixels[RING_H][RING_W],saved[RING_H][RING_W];
static const char*label="load";
static unsigned frames;
static uint32_t crc32(const uint8_t*b,size_t n)
{uint32_t c=0xffffffffu;size_t i;int k;for(i=0;i<n;++i){c^=b[i];for(k=0;k<8;++k)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));}return ~c;}
static void load(int id,uint32_t crc)
{
 char path[2048];const char*s=__FILE__,*slash=strrchr(s,'/'),*back=strrchr(s,'\\');uint8_t b[2820];FILE*f;int x,y;
 if(!slash||(back&&back>slash))slash=back;
 CHECK(slash);snprintf(path,sizeof(path),"%.*s/fixtures/v16_real/mouth_%d.bin",(int)(slash-s),s,id);
 f=fopen(path,"rb");CHECK(f);CHECK(fread(b,1,sizeof(b),f)==sizeof(b));CHECK(fgetc(f)==EOF);fclose(f);CHECK(crc32(b,sizeof(b))==crc);
 for(y=0;y<120;++y)for(x=0;x<188;++x){int k=y*188+x;original[y][x]=(uint8_t)((b[k>>3]&(128>>(k&7)))?255:0);}
}
static void transform(int side,int dx,int dy)
{int x,y;for(y=0;y<120;++y)for(x=0;x<188;++x){int xx=side>0?x:187-x;xx-=dx;pixels[y][x]=(uint8_t)(y>=dy&&xx>=0&&xx<188?original[y-dy][xx]:0);}}
static void step(ring_motion_t*m,unsigned dt)
{
 m->now_ms+=20;ring_observe_track(1);ring_observe_frame_interval((uint16_t)dt);memcpy(saved,pixels,sizeof(saved));
 ring_update((const uint8_t(*)[188])pixels,m);++frames;CHECK(!memcmp(saved,pixels,sizeof(saved)));
 if(ring_result.path_source==6){int y;for(y=0;y<120;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);}
}
static void shoulder_sequence(int side,int kind)
{
 int i,entered=0,saw_source=0;ring_motion_t m={0,0,1,1};label="actual aperture without width closure";load(4,1128064939u);ring_init();
 ring_result.phase=RING_APPROACH;ring_result.ring_side=(int8_t)side;ring_result.route=(int8_t)-side;
 ring_result.approach_stage=(uint8_t)(kind==3?RING_TANGENT_NEAR_SEEN:RING_TANGENT_NEAR_PASSED);
 tangent_near_close=mouth_corridor_seen=1;mouth_seed(&mouth_near,side>0?145:42,117,0);mouth_near.count=4;mouth_near.progress=12;
 for(i=0;i<24;++i){transform(side,0,kind==1?0:i);m.wheel_revs+=.02f;step(&m,kind==2?65535u:20);
  if(ring_result.mouth_source==2)saw_source=1;
  if(ring_result.phase==RING_ENTER){entered=1;CHECK(kind==0 && i>=8);CHECK(ring_result.mouth_source==2);CHECK(side*ring_result.steer_error>0);break;}
  CHECK(!ring_owns_path() && ring_result.phase!=RING_CIRCLE);
 }
 CHECK(saw_source);CHECK(entered==(kind==0));
}
static void prepare_local(int mirror,ring_motion_t*m)
{
 int y;load(34,823807684u);transform(mirror,0,0);ring_init();vision_ms=100;
 ring_result.phase=RING_ENTER;ring_result.route=(int8_t)mirror;ring_result.ring_side=(int8_t)-mirror;
 ring_result.left_entry=1;ring_result.approach_stage=RING_TANGENT_LEFT;ring_result.lap_valid=1;
 ring_result.lap_origin_yaw=0;ring_result.visual_dt_ms=20;
 *m=(ring_motion_t){100,-mirror*40.f,1,1};scan_geometry((const uint8_t(*)[188])pixels,m);
 for(y=24;y<=117;++y)mouth_local_edge[y]=observed_left[y]>=0?(mirror>0?observed_left[y]:observed_right[y]):-1;
 mouth_local_valid=1;mouth_local_ms=mouth_local_curve_ms=100;mouth_local_yaw=m->yaw_deg;mouth_local_revs=1;
 phase_since=ring_since=last_path_ms=last_path_fault_ms=100;last_yaw=m->yaw_deg;support_last_revs=1;
 have_path=1;trusted_steer=0;tangent_lap_revs=entry_revs=1;
}
static void current_tangent_is_bounded(int mirror)
{
 int i,seen=0;ring_motion_t m;label="same local near edge, distant noise excluded";prepare_local(mirror,&m);
 for(i=0;i<20;++i){transform(mirror,1+(i%2),0);m.wheel_revs+=.003f;m.yaw_deg-=mirror*.1f;step(&m,20);
  if(ring_result.path_source==6)seen=1;
  CHECK(!ring_result.stop && !ring_result.fault && !ring_result.committed && ring_result.phase!=RING_CIRCLE);
  CHECK(mouth_local_curve_ms==100); /* tangent does not refresh strong-curve origin */
 }
 CHECK(seen);vision_ms=100+RING_ASSIST_TOTAL_MS+1;
 CHECK(!tangent_local_path(&m));
}
static void local_negative(int mirror,int kind)
{
 int y,x;ring_motion_t m;label="local source missing/frozen/unrelated/expired";prepare_local(mirror,&m);
 if(kind==0){m.yaw_deg-=mirror*2.f;}
 if(kind==1)mouth_local_valid=0;
 if(kind==2){transform(mirror,40,0);}
 if(kind==3)vision_ms+=RING_OBSERVE_MS+1;
 if(kind==4){for(y=0;y<120;++y)for(x=0;x<188;++x)pixels[y][x]=(uint8_t)(x>=64&&x<=123?255:0);}
 if(kind==5)memset(pixels,0,sizeof(pixels));
 scan_geometry((const uint8_t(*)[188])pixels,&m);
 CHECK(!tangent_local_path(&m));CHECK(!ring_result.committed && !ring_result.exited_events);
}
static void circle_local_handover(int mirror)
{
 int i,y,seen=0;ring_motion_t m;label="v30 measured circle accepts same bounded tangent";
 prepare_local(mirror,&m);timed_started=1;ring_result.phase=RING_CIRCLE;ring_result.committed=1;
 for(i=0;i<8;++i){
  transform(mirror,1+(i%2),0);m.now_ms+=20;vision_ms+=20;m.yaw_deg-=mirror*.1f;m.wheel_revs+=.003f;
  scan_geometry((const uint8_t(*)[188])pixels,&m);
  if(tangent_local_path(&m)){
   seen=1;CHECK(ring_result.committed && ring_result.phase==RING_CIRCLE);
   CHECK(mouth_local_curve_ms==100);
   for(y=0;y<120;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);
  }
 }
 CHECK(seen);
 vision_ms=100+RING_ASSIST_TOTAL_MS+1;CHECK(!tangent_local_path(&m));
 prepare_local(mirror,&m);ring_result.phase=RING_CIRCLE;ring_result.committed=1;timed_started=0;
 transform(mirror,1,0);scan_geometry((const uint8_t(*)[188])pixels,&m);CHECK(tangent_local_path(&m));
 CHECK(mouth_local_curve_ms==100 && ring_result.committed);
 ring_result.committed=0;CHECK(!tangent_local_path(&m));
 ring_result.committed=1;ring_result.approach_stage=RING_TANGENT_SHARED;CHECK(!tangent_local_path(&m));
 prepare_local(mirror,&m);ring_result.phase=RING_CIRCLE;ring_result.committed=1;timed_started=1;
 for(y=0;y<120;++y){int x;for(x=0;x<188;++x)pixels[y][x]=(uint8_t)(x>=64&&x<=123?255:0);}
 scan_geometry((const uint8_t(*)[188])pixels,&m);CHECK(!tangent_local_path(&m));
}
int main(void)
{
 int side,kind;CHECK(RING_TRACK_MODE==1);
 for(side=-1;side<=1;side+=2){for(kind=0;kind<4;++kind)shoulder_sequence(side,kind);
  current_tangent_is_bounded(side);circle_local_handover(side);for(kind=0;kind<6;++kind)local_negative(side,kind);}
 printf("PASS: %u entry-follow frames; real shoulder source/mirrors without width proxy, local near-edge white targets, frozen/yaw/ordinary/expired rejects and tangent cannot renew or commit (synthetic local sequences)\n",frames);
 return 0;
}
