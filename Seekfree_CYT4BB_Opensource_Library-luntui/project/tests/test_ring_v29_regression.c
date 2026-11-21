/* Independent CRC-verified event snapshots: NOT continuous replay.
 * Check current pixels, corridor identity and clipped-view rejection only. */
#include "../code/ring.c"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL v29 real:%d %s id%d side%d\n",__LINE__,#c,ident,side);exit(1);}}while(0)
static int ident,side;static uint8_t pixels[120][188],original[120][188];
static void fixture(int id,uint32_t crc){
 const char*s=__FILE__,*q=strrchr(s,'/'),*b=strrchr(s,'\\');char path[2048];uint8_t bits[2820];FILE*f;unsigned i,k;uint32_t sum=0xffffffffu;
 if(!q || (b && b>q))q=b;
 CHECK(q);ident=id;snprintf(path,sizeof(path),"%.*s/fixtures/v29_real/image_%d.bin",(int)(q-s),s,id);
 f=fopen(path,"rb");CHECK(f);CHECK(fread(bits,1,2820,f)==2820 && fgetc(f)==EOF);fclose(f);
 for(i=0;i<2820;++i){sum^=bits[i];for(k=0;k<8;++k)sum=(sum>>1)^(0xedb88320u&(0u-(sum&1u)));}CHECK(~sum==crc);
 for(i=0;i<22560;++i)pixels[i/188][i%188]=(uint8_t)((bits[i>>3]&(128>>(i&7)))?255:0);
 memcpy(original,pixels,sizeof(pixels));
}
static void snapshot(int id,uint32_t crc,int ref_expected){
 ring_motion_t m={100,0,0,1};int ref,y,old_error;side=id<48?1:-1;fixture(id,crc);ring_init();
 ring_result.phase=RING_APPROACH;ring_result.timed_entry=1;ring_result.ring_side=(int8_t)side;ring_result.route=(int8_t)-side;
 ring_result.lane_width=60;scan_geometry((const uint8_t(*)[188])pixels,&m);ref=tangent_reference();CHECK(ref==ref_expected);
 if(ref){CHECK(tangent_line_path(0));old_error=ring_result.steer_error;
  CHECK(ring_blend_error(side*60)==old_error); /* gyro no longer zeros measured correction */
  for(y=0;y<120;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);
 }else{ring_result.path_valid=0;CHECK(ring_blend_error(17)==17);}
 CHECK(!ring_result.force_entry && !ring_result.committed && !ring_result.exited_events);
 CHECK(!memcmp(pixels,original,sizeof(pixels)));
}
int main(void){
 snapshot(41,620856840u,1);snapshot(42,3070222297u,0);snapshot(44,4151148011u,1);
 snapshot(48,4100961368u,1);snapshot(49,4051366633u,0);snapshot(51,1396036763u,1);
 puts("PASS: 6 real CRC snapshots, same-frame incoming corridor target and current white support, clipped reference stays unarmed; no route-success claim from sparse images");return 0;}

