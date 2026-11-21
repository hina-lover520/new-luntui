/* Local observation regression built from a CRC-checked real v14 image.
 * Translations below are synthetic, NOT a continuous field replay. The
 * existing public fixed-world tests independently check the complete route. */
#include "../code/ring.c"
#include <stdio.h>
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL bottom-pass:%d %s count%d N%d,%d\n",__LINE__,#c,mouth_bottom_count,mouth_near.x,mouth_near.y);exit(1);}}while(0)
static uint8_t original[RING_H][RING_W],pixels[RING_H][RING_W];
static unsigned observations;
static uint32_t crc32(const uint8_t *b,size_t n)
{uint32_t c=0xffffffffu;size_t i;int k;for(i=0;i<n;++i){c^=b[i];for(k=0;k<8;++k)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));}return ~c;}
static void load(int id,uint32_t expected)
{
 char path[2048];const char *source=__FILE__,*slash=strrchr(source,'/'),*back=strrchr(source,'\\');
 uint8_t bits[2820];FILE*f;int x,y;if(!slash||(back&&back>slash))slash=back;CHECK(slash);
 snprintf(path,sizeof(path),"%.*s/fixtures/v14_real/near_%d.bin",(int)(slash-source),source,id);
 f=fopen(path,"rb");CHECK(f);CHECK(fread(bits,1,sizeof(bits),f)==sizeof(bits));CHECK(fgetc(f)==EOF);fclose(f);
 CHECK(crc32(bits,sizeof(bits))==expected);
 for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){int k=y*RING_W+x;original[y][x]=(uint8_t)((bits[k>>3]&(128>>(k&7)))?255:0);}
}
static void translated(int side,int dy)
{int x,y;for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x)pixels[y][x]=(uint8_t)(y>=dy?original[y-dy][side>0?x:RING_W-1-x]:0);}
static void setup(int side)
{
 int y;ring_init();vision_ms=100;ring_result.ring_side=(int8_t)side;ring_result.route=(int8_t)-side;
 mouth_seed(&mouth_near,side>0?128:59,108,0);mouth_near.count=4;mouth_near.progress=12;
 tangent_near_close=mouth_open_seen=1;
 for(y=0;y<RING_H;++y)mouth_previous_lower[y]=(int16_t)(side>0?187:0);
}
static int contact(int side,int dy,int save,int reference)
{
 int y;ring_motion_t m={0,0,0,1};translated(side,dy);scan_geometry((const uint8_t (*)[RING_W])pixels,&m);
 CHECK(tangent_reference());vision_ms+=20;++observations;
 y=mouth_bottom_contact(reference,&m);
 if(save)for(int k=0;k<RING_H;++k)mouth_previous_lower[k]=(int16_t)(side>0?mouth_right[k]:mouth_left[k]);
 return y;
}
static void interior_bottom(int side)
{
 int i,passed=0;setup(side);
 for(i=29;i<=38;++i){
  int p=contact(side,i,1,1);
  if(i==29){CHECK(!mouth_near_clearance(0,1,&(ring_motion_t){0,0,0,1}));CHECK(!p);}
  if(p){passed=1;CHECK(mouth_near.y==117);CHECK(mouth_near.x>25&&mouth_near.x<162);break;}
 }
 CHECK(passed);
}
static void frozen_or_missing(int side,int kind)
{
 int i;setup(side);
 /* A current reference cannot be replaced by black/missing pixels; fixed
  * lower geometry cannot earn further credit even if the top flickers. */
 CHECK(!contact(side,29,1,kind!=1));
 for(i=0;i<12;++i){
  if(kind==2){
   int y;ring_motion_t m={0,0,0,1};translated(side,29);pixels[10][20]=(uint8_t)((i&1)?255:0);
   scan_geometry((const uint8_t (*)[RING_W])pixels,&m);CHECK(tangent_reference());vision_ms+=20;++observations;
   CHECK(!mouth_bottom_contact(1,&m));
   for(y=0;y<RING_H;++y)mouth_previous_lower[y]=(int16_t)(side>0?mouth_right[y]:mouth_left[y]);
  } else CHECK(!contact(side,29,1,kind!=1));
 }
 CHECK(mouth_bottom_count<3);
}
static void expired_and_other_edge(int side)
{
 setup(side);vision_ms+=RING_OBSERVE_MS+1;CHECK(!contact(side,29,1,1));CHECK(!mouth_bottom_count);
 setup(side);mouth_near.x=side>0?60:127;CHECK(!contact(side,29,1,1));CHECK(!mouth_bottom_count);
}
static void actual_corner_association(void)
{
 /* Real images53->54: N=(161,67) at yaw7363.0 becomes local returning
  * corner(115,61) at yaw7387.7. Both are current endpoint pixels. This is
  * only an association check; those sparse snapshots cannot prove N/F. */
 ring_init();vision_ms=100;mouth_seed(&mouth_near,161,67,7363.f);vision_ms+=100;
 CHECK(mouth_associate(&mouth_near,115,61,7387.7f,1));CHECK(mouth_near.x==115&&mouth_near.y==61);
 CHECK(mouth_near.count==2&&mouth_near.progress>0);
}
static void actual_wide_corner_tracking(void)
{
 int i;ring_motion_t m={100,7363.f,0,1};island_t s;
 load(53,1632447164u);memcpy(pixels,original,sizeof(pixels));ring_init();vision_ms=100;
 ring_result.phase=RING_APPROACH;ring_result.ring_side=1;ring_result.route=-1;
 ring_result.approach_stage=RING_TANGENT_NEAR_SEEN;
 scan_geometry((const uint8_t (*)[RING_W])pixels,&m);s=find_island();
 CHECK(s.valid&&s.bottom==67&&s.left==161);
 mouth_seed(&mouth_near,s.left,s.bottom,m.yaw_deg);mouth_previous_yaw=m.yaw_deg;
 /* Missing intermediate images are deliberately opaque, NOT invented
  * field frames. The actual next endpoint must be reacquired from pixels. */
 memset(pixels,0,sizeof(pixels));
 for(i=0;i<3;++i){m.yaw_deg+=6.f;m.now_ms+=40;ring_observe_frame_interval(20);ring_observe_track(1);ring_update((const uint8_t (*)[RING_W])pixels,&m);}
 load(54,2861146557u);memcpy(pixels,original,sizeof(pixels));m.yaw_deg=7387.7f;m.now_ms+=40;
 ring_observe_frame_interval(20);ring_observe_track(1);ring_update((const uint8_t (*)[RING_W])pixels,&m);
 CHECK(ring_result.mouth_near_x==115&&ring_result.mouth_near_y==61);
 CHECK(mouth_near.missed==0&&mouth_near.progress>0);
 CHECK(!ring_owns_path()&&!ring_result.left_entry&&!ring_result.committed);
}
int main(void)
{
 int side,k;actual_corner_association();actual_wide_corner_tracking();load(19,2722520982u);
 for(side=-1;side<=1;side+=2){interior_bottom(side);for(k=0;k<3;++k)frozen_or_missing(side,k);expired_and_other_edge(side);}
 printf("PASS: %u local real-shape observations; interior bottom passage mirrored; fixed/missing/expired/unrelated boundary denied; actual corner association (not full field replay)\n",observations);
 return 0;
}
