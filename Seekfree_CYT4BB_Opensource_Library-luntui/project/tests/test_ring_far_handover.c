/* Real images are individual CRC-checked observations. Mirroring and
 * translating them below is a SYNTHETIC local sequence with an explicitly
 * prepared prior N-passed state, not a field replay or a vehicle model.
 * Public full-route tests independently enforce the original world F
 * region, white targets, complete outer arc and shared-straight exit. */
#include "../code/ring.c"
#include <stdio.h>
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL far-handover:%d %s case=%s side%d ph%d stage%d F%d gate%u ready%d guide%d,%d\n",__LINE__,#c,label,ring_result.ring_side,ring_result.phase,ring_result.approach_stage,ring_result.mouth_far_y,ring_result.mouth_gate,mouth_ready,ring_result.guide_x,ring_result.guide_y);exit(1);}}while(0)
static const char *label="load";
static uint8_t real_image[RING_H][RING_W],pixels[RING_H][RING_W],saved[RING_H][RING_W];
static unsigned observations;
static uint32_t crc32(const uint8_t *bytes,size_t n)
{uint32_t c=0xffffffffu;size_t i;int k;for(i=0;i<n;++i){c^=bytes[i];for(k=0;k<8;++k)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));}return ~c;}
static void load(int id,uint32_t crc)
{
    char path[2048];const char *source=__FILE__,*slash=strrchr(source,'/'),*back=strrchr(source,'\\');
    uint8_t bits[2820];FILE *f;int x,y;if(!slash || (back && back>slash))slash=back;CHECK(slash);
    snprintf(path,sizeof(path),"%.*s/fixtures/v15_real/mouth_%d.bin",(int)(slash-source),source,id);
    f=fopen(path,"rb");CHECK(f);CHECK(fread(bits,1,sizeof(bits),f)==sizeof(bits));CHECK(fgetc(f)==EOF);fclose(f);
    CHECK(crc32(bits,sizeof(bits))==crc);
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){int k=y*RING_W+x;real_image[y][x]=(uint8_t)((bits[k>>3]&(128>>(k&7)))?255:0);}
}
static void translated(int side,int dy)
{int x,y;for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x)pixels[y][x]=(uint8_t)(y>=dy?real_image[y-dy][side>0?x:RING_W-1-x]:0);}
static void setup(int side,int stage)
{
    ring_init();ring_result.phase=RING_APPROACH;ring_result.ring_side=(int8_t)side;
    ring_result.route=(int8_t)-side;ring_result.approach_stage=(uint8_t)stage;
    tangent_near_close=mouth_corridor_seen=1;
}
static void observe(ring_motion_t *motion,unsigned interval)
{
    int y;motion->now_ms+=20;ring_observe_track(1);ring_observe_frame_interval((uint16_t)interval);
    memcpy(saved,pixels,sizeof(saved));ring_update((const uint8_t (*)[RING_W])pixels,motion);
    CHECK(!memcmp(saved,pixels,sizeof(saved)));++observations;
    CHECK(ring_scan_pixels<=RING_W*RING_H*3u);
    CHECK(!ring_result.stop && !ring_result.fault && !ring_result.exited_events);
    if(ring_owns_path()) {for(y=0;y<RING_H;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);}
    else CHECK(ring_blend_error(19)==19 && ring_turn_boost()==0 && ring_speed_limit()==-1);
}
static void actual_aperture(int side,int id,uint32_t crc)
{
    int fx=-1,fy,ok=0,y;ring_motion_t m={0,0,0,1};
    label="actual returning shoulder supplies current-white portal";load(id,crc);translated(side,0);setup(side,RING_TANGENT_NEAR_PASSED);
    scan_geometry((const uint8_t (*)[RING_W])pixels,&m);CHECK(tangent_reference());fy=tangent_far_closure(&fx);
    CHECK(tangent_portal_path());
    if(id==27)CHECK(fy>=0 && fy<74);
    if(fy>=0)CHECK(tangent_far_window(fy,&ok) && ok);
    CHECK(mouth_portal_row>=28 && ring_result.guide_y<=RING_LOOK_ROW);
    CHECK(side*(ring_result.guide_x-mouth_centre[ring_result.guide_y])>=ring_result.lane_width/4);
    for(y=0;y<RING_H;++y)if(ring_result.mid[y]>=0)CHECK(pixels[y][ring_result.mid[y]]);
    CHECK(!ring_result.branch_ok && !ring_result.left_entry && !ring_result.committed);
}
static void advancing_or_frozen(int side,int kind)
{
    int i,entered=0;ring_motion_t m={100,0,1,1};
    label=kind==0?"advancing local aperture":kind==1?"frozen aperture plus sensor motion":kind==2?"unknown capture interval":"N has not passed";
    load(27,2327019547u);setup(side,kind==3?RING_TANGENT_NEAR_SEEN:RING_TANGENT_NEAR_PASSED);
    for(i=0;i<20;++i){
        translated(side,kind==1?0:i);m.wheel_revs+=.02f;observe(&m,kind==2?65535u:20u);
        if(ring_result.phase==RING_ENTER){entered=1;CHECK(kind==0 && i>=8);CHECK(ring_result.mouth_far_y<74);
            CHECK(ring_result.lap_valid && !ring_result.committed && !ring_result.left_entry);
            CHECK(side*ring_result.steer_error>0 && ring_owns_path());break;}
        CHECK(!ring_owns_path() && ring_result.phase!=RING_CIRCLE);
        if(kind==1 && i>1)CHECK(ring_result.mouth_gate&RING_MOUTH_VISUAL_PROGRESS);
    }
    CHECK(entered==(kind==0));
}
static void no_portal_no_turn(int side,int kind)
{
    int y,x,ok=1,fx=-1,fy;ring_motion_t m={0,0,0,1};
    label="no aperture / wrong side / disconnected upper white";
    if(kind==0){load(13,2484059816u);translated(side,0);}
    else {load(27,2327019547u);translated(side,0);}
    setup(kind==1?-side:side,RING_TANGENT_NEAR_PASSED);
    if(kind==2)memset(pixels[44],0,RING_W);
    if(kind==3)for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){int half=28+(y-24)/2;if(half<20)half=20;pixels[y][x]=(uint8_t)(abs(x-94)<=half?255:0);}
    scan_geometry((const uint8_t (*)[RING_W])pixels,&m);CHECK(tangent_reference());
    CHECK(!tangent_portal_path());fy=tangent_far_closure(&fx);
    if(kind==0)CHECK(fy>=60 && fy<74); /* old false perspective neck */
    CHECK(!tangent_far_window(fy<0?65:fy,&ok) && !ok);
}
static void confirmation_is_not_window(int side)
{
    int i,saw_ready_only=0;ring_motion_t m={100,0,1,1};
    label="independent location, current path and consecutive readiness";load(27,2327019547u);setup(side,RING_TANGENT_NEAR_PASSED);
    for(i=0;i<15;++i){translated(side,i);m.wheel_revs+=.02f;observe(&m,20);
        if(ring_result.mouth_gate==RING_MOUTH_READY){saw_ready_only=1;CHECK(mouth_ready>0 && mouth_ready<RING_TANGENT_CONFIRM);}
        if(ring_result.phase==RING_ENTER)break;
    }
    CHECK(saw_ready_only && ring_result.phase==RING_ENTER && !ring_result.mouth_gate);
}
int main(void)
{
    int side,kind;CHECK(RING_TRACK_MODE==1);
    for(side=-1;side<=1;side+=2){
        actual_aperture(side,27,2327019547u);actual_aperture(side,36,79719106u);
        for(kind=0;kind<4;++kind){advancing_or_frozen(side,kind);no_portal_no_turn(side,kind);}
        confirmation_is_not_window(side);
    }
    printf("PASS: %u far-handover observations; CRC real shoulders/mirrors, current white paths, evolving local entry, frozen/unknown/N-before-F/false neck rejects, separate ready gate (local synthetic sequence, not field replay)\n",observations);
    return 0;
}
