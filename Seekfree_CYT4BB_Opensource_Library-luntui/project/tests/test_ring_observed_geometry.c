#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL observed-geometry:%d %s sample=%s ph=%d stage=%u side=%d near=%d far=%d rej=%u\n",__LINE__,#c,label,ring_result.phase,ring_result.approach_stage,ring_result.ring_side,ring_result.near_row,ring_result.far_row,ring_result.reject); exit(1); } } while (0)
static const char *label="initial";
static uint8_t image[RING_H][RING_W],saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;

static uint32_t crc32_bytes(const uint8_t *bytes,size_t n)
{
    uint32_t crc=0xffffffffu;size_t i;int b;
    for(i=0;i<n;++i){crc^=bytes[i];for(b=0;b<8;++b)crc=(crc>>1)^(0xedb88320u&(0u-(crc&1u)));}
    return ~crc;
}
static void fixture(int id,uint32_t expected_crc)
{
    char path[2048];const char *source=__FILE__,*slash=strrchr(source,'/'),*back=strrchr(source,'\\');
    uint8_t bits[2820];FILE *file;size_t n;int x,y;
    if(!slash || (back && back>slash))slash=back;
    CHECK(slash && (size_t)(slash-source)<1800u);
    snprintf(path,sizeof(path),"%.*s/fixtures/v12_real/image_%d.bin",(int)(slash-source),source,id);
    file=fopen(path,"rb");CHECK(file!=NULL);n=fread(bits,1,sizeof(bits),file);
    CHECK(n==2820u && fgetc(file)==EOF);fclose(file);CHECK(crc32_bytes(bits,n)==expected_crc);
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){int k=y*RING_W+x;image[y][x]=(uint8_t)((bits[k>>3]&(128>>(k&7)))?255:0);}
}
static void observe(int track)
{
    motion.now_ms+=20u;ring_observe_track(track);ring_observe_frame_interval(20);
    memcpy(saved,image,sizeof(saved));ring_update((const uint8_t (*)[RING_W])image,&motion);
    CHECK(!memcmp(saved,image,sizeof(saved)));++frames;
}
static void reset(void){ring_init();motion=(ring_motion_t){0,0,0,1};}
static void ordinary_output(void)
{
    CHECK(!ring_owns_path() && !ring_blocks_cross());
    CHECK(ring_blend_error(17)==17 && ring_turn_boost()==0 && ring_speed_limit()==-1);
    CHECK(!ring_result.stop && !ring_result.fault && !ring_result.left_entry && !ring_result.committed);
}

/* The CRC checked camera samples are frozen observations, NOT a labelled
 * route to F. Even large yaw/displacement cannot create a new mouth. */
static void frozen_real_cannot_enter(int id,uint32_t crc)
{
    int i;char name[32];snprintf(name,sizeof(name),"real-%d-frozen",id);label=name;
    reset();fixture(id,crc);
    for(i=0;i<120;++i){motion.yaw_deg+=3.f;motion.wheel_revs+=.03f;observe(1);
        CHECK(ring_result.phase!=RING_ENTER && ring_result.phase!=RING_CIRCLE);
        CHECK(!ring_result.exited_events);ordinary_output();}
}

/* Independent pixel facts establish a local neck/opening, not a true F
 * label. Source7 has a connected white neck x108..166 at y48 and a broad
 * current road x0..180 at y83. Source10's short neck is x37..97 at y73;
 * upper rows then disconnect, so the observation must remain uncertain. */
static void real_pixels(void)
{
    int x;
    label="real7-neck";fixture(7,312265816u);
    for(x=108;x<=166;++x)CHECK(image[48][x]);
    CHECK(!image[48][107] && !image[48][167]);
    for(x=0;x<=180;++x)CHECK(image[83][x]);
    label="real10-short-neck";fixture(10,3947681586u);
    for(x=37;x<=97;++x)CHECK(image[73][x]);
    CHECK(!image[73][36] && !image[73][98]);
    for(x=2;x<=119;++x)CHECK(image[82][x]);
}
static void local_real_observation(int id,uint32_t crc,int side,int clipped)
{
    int dx,dy,support=0;char name[32];snprintf(name,sizeof(name),"real-%d-local-observation",id);label=name;
    fixture(id,crc);memcpy(saved,image,sizeof(saved));
    ring_test_observe_mouth((const uint8_t (*)[RING_W])image,side,60);
    CHECK(!memcmp(image,saved,sizeof(saved)));
    if(clipped){CHECK(!ring_result.mouth_ref && ring_result.mouth_far_y<0);return;}
    CHECK(ring_result.mouth_ref && ring_result.mouth_far_y>=0);
    /* Independent image support: the reported side boundary must have a
     * current white pixel and a dark pixel immediately into the opening.
     * The event row sits beside the narrow neck, hence two rows tolerance. */
    for(dy=-2;dy<=2;++dy)for(dx=-1;dx<=1;++dx){
        int x=ring_result.mouth_far_x+dx,y=ring_result.mouth_far_y+dy;
        if(x>0 && x<RING_W-1 && y>=0 && y<RING_H && image[y][x] && !image[y][x+side])support=1;
    }
    CHECK(support);
    CHECK(!ring_result.left_entry && !ring_result.committed && !ring_result.exited_events);
}

/* A fixed world is rendered through a moving camera. A smooth lateral
 * bend deforms the whole road/ring together, retaining road topology.
 * No production phase/route is used to manufacture these pixels. */
static float bend(float y,float amplitude){return amplitude*sinf(y/110.f);}
static float bend_heading(float y,float amplitude){return atanf(amplitude/110.f*cosf(y/110.f))*180.f/RTG_PI;}
static void render_bent(int side,float world_y,float amplitude,float camera_skew,int top_crop)
{
    int x,y;float theta=(bend_heading(world_y,amplitude)+camera_skew)*RTG_PI/180.f;
    float px=bend(world_y,amplitude);
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){
        float right=(float)(x-RING_W/2),forward=(float)(RING_H-1-y);
        float wx=px+cosf(theta)*right+sinf(theta)*forward;
        float wy=world_y-sinf(theta)*right+cosf(theta)*forward;
        image[y][x]=(uint8_t)(y>=top_crop && rtg_world_white(side,wx-bend(wy,amplitude),wy)?255:0);
    }
}
static void short_visible_approach(int side,int top_crop)
{
    int i,saw_near=0,saw_passed=0;char name[64];
    snprintf(name,sizeof(name),"short-vision-side%d-crop%d",side,top_crop);label=name;
    reset();
    for(i=0;i<=125;++i){float wy=-180.f+2.f*i;
        render_bent(side,wy,0,0,top_crop);motion.wheel_revs=(float)i/60.f;observe(1);
        if(ring_result.approach_stage>=RING_TANGENT_NEAR_SEEN)saw_near=1;
        if(ring_result.approach_stage>=RING_TANGENT_NEAR_PASSED)saw_passed=1;
        CHECK(!ring_result.left_entry && !ring_result.committed && !ring_result.exited_events);
        if(ring_result.phase==RING_ENTER){CHECK(saw_near && saw_passed);CHECK(wy>=60.f && wy<=70.f);return;}
        ordinary_output();
    }
    CHECK(0);
}
static void overlapping_world_mouths(int side,float scale)
{
    int i,x,y,saw_near=0,saw_passed=0,saw_pending_far=0;char name[80];reset();
    /* The fixed-world mouths are 120 units apart; this camera covers
     * 166.6 units in front, so the F closing can coexist with near N.
     * Both are real pixels in one unchanged world, not phase-labelled
     * artificial features. The old width learner only accepts >=2/3 of
     * its previous estimate. Gradual ordinary widths exercise that actual
     * public learner, without mutating its initial value or parameters. */
    label="overlap-camera-width-calibration";
    for(i=60;i>=(int)(60.f/scale);--i){int t;
        for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x)
            image[y][x]=(uint8_t)(x>=94-i/2 && x<94-i/2+i?255:0);
        for(t=0;t<4;++t){observe(1);ordinary_output();}
    }
    for(i=0;i<=170;++i){float wy=-260.f+2.f*i;
        snprintf(name,sizeof(name),"overlap-fov-side%d-scale%.1f-y%.0f",side,scale,wy);label=name;
        for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x)
            image[y][x]=(uint8_t)(rtg_world_white(side,(x-94)*scale,wy+(119-y)*scale)?255:0);
        motion.wheel_revs=(float)i/60.f;observe(1);
        if(ring_result.approach_stage>=RING_TANGENT_NEAR_SEEN)saw_near=1;
        if(ring_result.approach_stage>=RING_TANGENT_NEAR_PASSED)saw_passed=1;
        if(ring_result.approach_stage==RING_TANGENT_NEAR_SEEN &&
           ring_result.mouth_first_y>=0 && ring_result.mouth_far_y>=0)saw_pending_far=1;
        if(ring_result.phase==RING_ENTER){
            int t,gy=ring_result.guide_y,gx=ring_result.guide_x;
            float best=1.e6f,wx=(gx-94)*scale,goal_y=wy+(119-gy)*scale;
            CHECK(saw_near && saw_passed && wy>0.f && wy<90.f);
            /* This camera has a different pixel scale. Validate the
             * selected place against the unchanged WORLD instead of
             * copying the 1:1 pixel fixture's entry-position window. */
            CHECK(gx>=0 && gx<RING_W && gy>=0 && gy<RING_H && image[gy][gx]);
            for(y=0;y<RING_H;++y)if(ring_result.mid[y]>=0)CHECK(image[y][ring_result.mid[y]]);
            for(t=0;t<=200;++t){
                rtg_pose_t q=rtg_bezier_at(side,(float)t/200.f,0,wy);
                float dx=wx-q.x,dy=goal_y-q.y,distance=sqrtf(dx*dx+dy*dy);
                CHECK(rtg_world_white(side,q.x,q.y));if(distance<best)best=distance;
            }
            CHECK(best<=34.f && side*wx>0);
            if(scale>1.7f)CHECK(saw_pending_far);
            return;
        }
        ordinary_output();
    }
    CHECK(0);
}

static void frozen_near_is_not_far(int side)
{
    int i;rtg_pose_t p={0,-100,0};label="frozen-near-closing";reset();rtg_render(image,side,&p);
    for(i=0;i<200;++i){motion.wheel_revs+=.025f;motion.yaw_deg+=side*1.7f;observe(1);
        CHECK(ring_result.phase!=RING_ENTER && ring_result.phase!=RING_CIRCLE);ordinary_output();}
}
static void slow_entry_keeps_portal(int side)
{
    int i;float far_y=0;rtg_pose_t p;label="slow-entry-does-not-select-through-road";reset();
    for(i=0;i<=125;++i){p=rtg_approach(i);rtg_render(image,side,&p);motion.wheel_revs=(float)i/60.f;observe(1);
        if(ring_result.phase==RING_ENTER){far_y=p.y;break;}ordinary_output();}
    CHECK(far_y>=60.f && far_y<=70.f);
    /* Deliberately weak response: the body remains in the legal straight
     * corridor and yaw rises by only one degree per frame. This geometry
     * still needs a turn towards the PREDECLARED side portal, so a new
     * measured straight-road target must not reverse that entry request.
     * It is not a calibrated vehicle or a general e!=0 requirement. */
    for(i=0;i<20;++i){
        float theta,wx,wy;int gx,gy;
        p.x=0;p.y=far_y+.5f*i;p.heading_deg=(float)(side*i);
        rtg_render(image,side,&p);motion.yaw_deg=p.heading_deg;motion.wheel_revs+=.5f/RTG_WHEEL_CIRCUMFERENCE;observe(1);
        CHECK(ring_result.phase==RING_ENTER && ring_owns_path() && !ring_result.stop && !ring_result.fault);
        CHECK(!ring_result.left_entry && !ring_result.committed && !ring_result.exited_events);
        gx=ring_result.guide_x;gy=ring_result.guide_y;CHECK(gx>=0 && gx<RING_W && gy<119);
        CHECK(image[gy][gx]);theta=p.heading_deg*RTG_PI/180.f;
        wx=p.x+cosf(theta)*(gx-94)+sinf(theta)*(119-gy);
        wy=p.y-sinf(theta)*(gx-94)+cosf(theta)*(119-gy);
        CHECK(side*wx>=20.f && wy>=far_y && wy<=110.f);
        CHECK(side*ring_result.steer_error>0);
    }
}
static void clipped_ordinary_and_cross(int side)
{
    int kind,i,x,y;label="clipped-straight-and-cross";
    for(kind=0;kind<2;++kind){
        reset();
        for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){
            int centre=94+side*(117-y)*3/4;
            image[y][x]=(uint8_t)((abs(x-centre)<=30 || (kind && y>=45 && y<=75))?255:0);
        }
        for(i=0;i<60;++i){motion.wheel_revs+=.02f;motion.yaw_deg+=side*.3f;
            observe(kind?2:1);ordinary_output();CHECK(!ring_result.detected_events);
        }
    }
}
static void bent_and_skewed_observation(int side,float amplitude,float skew)
{
    int i,saw_near=0;char name[100];
    reset();
    for(i=0;i<100;++i){float wy=-180.f+2.f*i;
        snprintf(name,sizeof(name),"world-side%d-bend%.0f-skew%.0f-y%.0f",side,amplitude,skew,wy);label=name;
        render_bent(side,wy,amplitude,skew,0);
        motion.yaw_deg=bend_heading(wy,amplitude)+skew;motion.wheel_revs=(float)i/60.f;
        observe(1);ordinary_output();CHECK(ring_result.phase!=RING_ENTER && !ring_result.exited_events);
        if(ring_result.approach_stage>=RING_TANGENT_NEAR_SEEN)saw_near=1;
    }
    CHECK(saw_near);
}
int main(void)
{
    int side;
    CHECK(RING_TRACK_MODE==1);real_pixels();
    local_real_observation(5,1271433325u,1,0);local_real_observation(6,3627003665u,-1,1);
    local_real_observation(7,312265816u,-1,0);local_real_observation(9,2147060907u,-1,1);local_real_observation(10,3947681586u,-1,0);
    local_real_observation(105,1704324264u,1,0);local_real_observation(111,3945351821u,-1,0);local_real_observation(112,1294611206u,-1,0);
    frozen_real_cannot_enter(5,1271433325u);frozen_real_cannot_enter(6,3627003665u);
    frozen_real_cannot_enter(7,312265816u);frozen_real_cannot_enter(9,2147060907u);frozen_real_cannot_enter(10,3947681586u);
    frozen_real_cannot_enter(105,1704324264u);frozen_real_cannot_enter(111,3945351821u);frozen_real_cannot_enter(112,1294611206u);
    for(side=-1;side<=1;side+=2){
        slow_entry_keeps_portal(side);
        frozen_near_is_not_far(side);
        clipped_ordinary_and_cross(side);
        bent_and_skewed_observation(side,20,0);bent_and_skewed_observation(side,-20,0);
        bent_and_skewed_observation(side,0,15);bent_and_skewed_observation(side,0,-15);
    }
    for(side=-1;side<=1;side+=2)short_visible_approach(side,50);
    for(side=-1;side<=1;side+=2){overlapping_world_mouths(side,1.4f);overlapping_world_mouths(side,1.8f);}
    printf("PASS: %u observed geometry frames; CRC-verified actual images, fixed world bend/skew, frozen neck rejects, short sight N/F sequence\n",frames);
    return 0;
}
