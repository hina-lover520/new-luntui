/* CRC snapshots are real; translations/mirrors and prepared N history
 * below are synthetic local observations, not replayed driving frames. */
#include "../code/ring.c"
#include <stdio.h>
static uint8_t original[120][188],pixels[120][188],saved[120][188];
static unsigned frames;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL clipped portal:%d %s ph%d src%d gate%u tip%d\n",__LINE__,#c,ring_result.phase,ring_result.mouth_source,ring_result.mouth_gate,mouth_portal_row);exit(1);}}while(0)
static void load(int id,uint32_t expected)
{
 const char*s=__FILE__,*q=strrchr(s,'/'),*b=strrchr(s,'\\');char path[2048];uint8_t bits[2820];
 FILE*f;uint32_t crc=0xffffffffu;int i,k,x,y;if(!q || (b && b>q))q=b;CHECK(q);
 snprintf(path,sizeof(path),"%.*s/fixtures/v17_real/mouth_%d.bin",(int)(q-s),s,id);
 f=fopen(path,"rb");CHECK(f);CHECK(fread(bits,1,2820,f)==2820);CHECK(fgetc(f)==EOF);fclose(f);
 for(i=0;i<2820;++i){crc^=bits[i];for(k=0;k<8;++k)crc=(crc>>1)^(0xedb88320u&(0u-(crc&1u)));}CHECK(~crc==expected);
 for(y=0;y<120;++y)for(x=0;x<188;++x){int j=y*188+x;original[y][x]=(uint8_t)((bits[j>>3]&(128>>(j&7)))?255:0);}
}
static void transform(int side,int dy)
{int x,y;for(y=0;y<120;++y)for(x=0;x<188;++x)pixels[y][x]=(uint8_t)(y>=dy?original[y-dy][side>0?x:187-x]:0);}
static void prepare(int side,int stage)
{
 ring_init();ring_result.phase=RING_APPROACH;ring_result.ring_side=(int8_t)side;
 ring_result.route=(int8_t)-side;ring_result.approach_stage=(uint8_t)stage;
 ring_result.lane_width=60;tangent_near_close=mouth_corridor_seen=1;
 mouth_seed(&mouth_near,side>0?146:41,117,0);mouth_near.count=4;mouth_near.progress=12;
}
static void sequence(int side,int kind)
{
 ring_motion_t m={0,0,1,1};int i,entered=0;
 load(18,4093000696u);prepare(side,kind==3?RING_TANGENT_NEAR_SEEN:RING_TANGENT_NEAR_PASSED);
 for(i=0;i<24;++i){
  transform(side,kind==1?0:i);m.now_ms+=20;m.wheel_revs+=.02f;
  ring_observe_frame_interval((uint16_t)(kind==2?65535u:20));ring_observe_track(1);
  memcpy(saved,pixels,sizeof(saved));ring_update((const uint8_t(*)[188])pixels,&m);++frames;
  CHECK(!memcmp(saved,pixels,sizeof(saved)));CHECK(ring_scan_pixels<=188u*120u*3u);
  if(ring_result.phase==RING_ENTER){int y;entered=1;CHECK(kind==0 && i>=8);CHECK(ring_result.force_entry);
   CHECK(ring_result.lap_valid && !ring_result.committed && !ring_result.exited_events);
   for(y=0;y<120;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);
   break;}
  CHECK(!ring_result.force_entry && !ring_owns_path() && ring_speed_limit()==-1);
 }
 CHECK(entered==(kind==0));
}
int main(void)
{
 int side,kind,ok;ring_motion_t m={0,0,0,1};
 for(side=-1;side<=1;side+=2){
  load(18,4093000696u);transform(side,0);prepare(side,RING_TANGENT_NEAR_PASSED);
  scan_geometry((const uint8_t(*)[188])pixels,&m);CHECK(tangent_reference());CHECK(tangent_portal_path());CHECK(mouth_portal_row==57);
  load(9,1312508242u);transform(side,0);prepare(side,RING_TANGENT_NEAR_PASSED);
  scan_geometry((const uint8_t(*)[188])pixels,&m);CHECK(tangent_reference());ok=tangent_portal_path();
  CHECK(!ok && mouth_portal_row==49); /* visible corner, no current safe portal path */
  CHECK(!tangent_far_window(mouth_portal_row,&ok) && !ok);
  for(kind=0;kind<4;++kind)sequence(side,kind);
 }
 printf("PASS: %u clipped-portal observations; CRC shoulders, side clipping, feature/path separation, moving local entry, frozen/unknown/N-before-F rejects\n",frames);
 return 0;
}
