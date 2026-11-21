#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned checks;
#define CHECK(c) do { if(!(c)) { fprintf(stderr,"FAIL field line %d: %s ph=%d dir=%d near=%d cc=%u rej=%u src=%u branch=%u\n",__LINE__,#c,ring_result.phase,ring_result.route,ring_result.island_near,ring_result.confirm_frames,ring_result.reject,ring_result.path_source,ring_result.branch_ok); exit(1); } } while(0)

static void reset(void) {ring_init();motion=(ring_motion_t){0,0,0,1};}
static void step(void)
{
    int y;
    ring_motion_t sensor;
    motion.now_ms+=20;
    ring_observe_frame_interval(20);ring_observe_track(1);
    memcpy(saved,image,sizeof(image));
    sensor=motion;sensor.yaw_deg=motion.yaw_deg*RING_YAW_SIGN;
    ring_update((const uint8_t (*)[RING_W])image,&sensor);
    CHECK(!memcmp(saved,image,sizeof(image)));
    if(ring_result.path_source==1 && ring_result.path_valid)
        for(y=0;y<RING_H;++y) if(ring_result.mid[y]>=0) CHECK(image[y][ring_result.mid[y]]);
    ++checks;
}
static void mirror(void)
{
    int x,y;
    for(y=0;y<RING_H;++y) for(x=0;x<RING_W/2;++x) {
        uint8_t v=image[y][x];image[y][x]=image[y][RING_W-1-x];image[y][RING_W-1-x]=v;
    }
}
static void check_ordinary(int frames)
{
    int i;
    for(i=0;i<frames;++i) {
        step();CHECK(ring_result.phase==RING_NORMAL);
        CHECK(!ring_result.detected_events && !ring_result.stop);
        CHECK(ring_speed_limit()==-1 && ring_blend_error(17)==17 && !ring_turn_boost());
    }
}
int main(void)
{
    int i,side,cy,slope;
    /* Original field reproduction: confirmation must NOT turn 8..19 visible
     * edge rows into an impossible >=20-row historical association. */
    reset();rg_annulus(image,108,31,10,70,1,0);
    check_ordinary(12);
    CHECK(ring_result.reject&RING_REJ_SHORT);
    rg_annulus(image,108,45,16,86,1,0);
    for(i=0;i<4;++i) step();
    CHECK(ring_result.detected_events==1 && ring_result.branch_ok && ring_result.path_source==1);
    for(i=0;i<15;++i) {step();CHECK(ring_result.branch_ok && !ring_result.stop);}
    puts("PASS: far short island observes without beeping/stopping; larger reliable island acquires and persists");

    /* IPM-like side cropping: no opposite white branch exists in the frame.
     * The visible arc and the white merge below its tip provide identity. */
    for(side=-1;side<=1;side+=2) for(cy=20;cy<=40;cy+=10) {
        reset();rg_annulus(image,192,cy,55,130,1,0);
        if(side<0) mirror();
        for(i=0;i<4;++i) step();
        CHECK(ring_owns_path() && ring_result.detected_events==1);
        CHECK(ring_result.route==-side && ring_result.branch_ok && !ring_result.stop);
        for(i=0;i<12;++i) {step();CHECK(ring_result.branch_ok && ring_result.path_source==1 && !ring_result.stop);}
    }
    puts("PASS: both side-clipped curved islands acquire without requiring two white branches");

    /* A cold tangent view may be indistinguishable from an ordinary bend;
     * it must not invent an entry event from one curved edge alone. */
    for(side=-1;side<=1;side+=2) {
        reset();rg_orbit(image,side,90);check_ordinary(12);
        reset();rg_road(image,0);
        for(i=0;i<30;++i) {
            rg_rect(image,side>0?124:0,30,side>0?187:63,65,255);
            step();CHECK(!ring_result.detected_events && !ring_result.stop);
        }
    }
    for(slope=-1;slope<=1;++slope) {
        reset();rg_cross(image,slope);check_ordinary(30);
        /* White seams and detached bright flecks cannot make a closed island. */
        rg_rect(image,4,8,15,27,255);rg_rect(image,170,12,180,29,255);
        check_ordinary(30);
    }
    puts("PASS: tangent cold start, side opening, slanted cross and detached flecks leave ordinary control unchanged");
    for(side=-1;side<=1;side+=2) {
        reset();rg_annulus(image,192,20,55,130,1,0);if(side<0) mirror();
        for(i=0;i<4;++i) step();
        for(i=1;i<=20;++i) {
            rg_annulus(image,192-i*8/20,20+i*99/20,55+i*5/20,130-i*10/20,1,0);
            if(side<0) mirror();
            motion.yaw_deg=-side*i*2.5f;motion.wheel_revs+=0.025f;
            step();CHECK(ring_result.branch_ok && ring_result.path_source==1 && !ring_result.stop);
        }
        rg_orbit(image,-side,90);
        for(i=0;i<12;++i) {
            int x,edge,edge3;
            float slope_value,distance;
            motion.yaw_deg-=side*2.0f;motion.wheel_revs+=0.025f;step();
            CHECK(ring_result.branch_ok && ring_result.path_source==1 && !ring_result.stop);
            x=ring_result.mid[RING_LOOK_ROW];
            edge=x;edge3=ring_result.mid[RING_LOOK_ROW+3];
            if(ring_result.route>0) {
                while(edge>0 && image[RING_LOOK_ROW][edge-1]) --edge;
                while(edge3>0 && image[RING_LOOK_ROW+3][edge3-1]) --edge3;
            } else {
                while(edge<RING_W-1 && image[RING_LOOK_ROW][edge+1]) ++edge;
                while(edge3<RING_W-1 && image[RING_LOOK_ROW+3][edge3+1]) ++edge3;
            }
            slope_value=(edge-edge3)/3.0f;
            distance=abs(x-edge)/sqrtf(1.0f+slope_value*slope_value);
            CHECK(fabsf(distance-ring_result.lane_width*0.5f)<=2.0f);
        }
        CHECK(ring_result.phase==RING_CIRCLE && ring_result.committed);
    }
    puts("PASS: clipped entry continuously becomes a single-edge orbit; path maintains normal distance from inner edge");
    printf("PASS: %u field-failure geometry regression frames (synthetic, not photo replay)\n",checks);
    return 0;
}
