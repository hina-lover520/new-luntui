#include "ring_test_geometry.h"
#include "ring_control.h"
#include "ring_link.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL isolation line %d: %s phase=%d turn=%.1f source=%u branch=%u reject=%u\n", __LINE__, #x, ring_result.phase, (double)ring_result.turn_deg,ring_result.path_source,ring_result.branch_ok,ring_result.reject); exit(1); } } while (0)
static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;
static void reset(void) { ring_init(); motion=(ring_motion_t){0,0,0,1}; }
static void observe(int state, unsigned dt, int advance_clock)
{
    int y;
    ring_motion_t sensor;
    if (advance_clock) motion.now_ms += dt;
    ring_observe_track(state); ring_observe_frame_interval((uint16_t)dt);
    memcpy(saved,image,sizeof(image));
    sensor=motion;sensor.yaw_deg=motion.yaw_deg*RING_YAW_SIGN;
    ring_update((const uint8_t (*)[RING_W])image,&sensor);
    CHECK(!memcmp(saved,image,sizeof(image)));
    if (ring_result.path_valid && (ring_result.path_source==1 || ring_result.path_source==4))
        for(y=0;y<RING_H;++y) if(ring_result.mid[y]>=0) CHECK(image[y][ring_result.mid[y]]);
    ++frames;
}
static void step(int state) { observe(state,20,1); }
static void confirm(int route, int state)
{
    int i;
    rg_entry(image,route,0,0);
    for(i=0;i<RING_CONFIRM_FRAMES;++i) step(state);
    CHECK(ring_result.phase==RING_APPROACH && ring_result.route==route);
    CHECK(ring_result.branch_ok && ring_result.path_source==1);
}
static void enter(int route)
{
    int i;
    static const int a[]={3,6,9,13};
    confirm(route,1);
    for(i=0;i<4;++i) { rg_entry(image,route,a[i],0); step(1); }
    CHECK(ring_result.phase==RING_ENTER);
}
static void transition(int route,int last)
{
    int i;
    for(i=1;i<=last;++i) {
        rg_transition(image,route,i,12);
        motion.yaw_deg=route*4.0f*i; motion.wheel_revs+=0.025f; step(1);
        CHECK(ring_result.branch_ok && ring_result.path_source==1);
    }
}
static void circle(int route)
{
    int i;
    enter(route); transition(route,12); rg_orbit(image,route,90);
    for(i=0;i<6;++i) { motion.yaw_deg+=route*2.0f; motion.wheel_revs+=0.03f; step(1); }
    CHECK(ring_result.phase==RING_CIRCLE && ring_result.committed);
}
int main(void)
{
    int i,route,slope,old_look;
    for(route=-1;route<=1;route+=2) {
        reset(); confirm(route,2);
        CHECK(!ring_result.cross_veto && ring_result.detected_events==1);
    }
    for(slope=-1;slope<=1;++slope) {
        reset(); rg_cross(image,slope);
        for(i=0;i<12;++i) step(1);
        CHECK(ring_result.phase==RING_NORMAL && ring_result.cross_veto);
        CHECK(!ring_result.detected_events && ring_speed_limit()==-1);
    }
    reset(); rg_conflict(image,1);
    for(i=0;i<12;++i) step(2);
    CHECK(ring_result.phase==RING_NORMAL && !ring_result.detected_events);
    CHECK(ring_result.classification==3 && (ring_result.reject&RING_REJ_CONFLICT));
    puts("PASS: strong curved ring survives raw=2; slanted cross and conflicting geometry do not acquire");

    /* The entry side locks despite the island tip crossing the centre pixel. */
    reset();
    for(i=0;i<RING_CONFIRM_FRAMES;++i) {
        static const int centers[]={93,95,94,93};
        rg_annulus(image,centers[i],45,26,86,1,0); step(i%2?2:1);
    }
    CHECK(ring_result.phase==RING_APPROACH && ring_result.route==1);
    puts("PASS: centre jitter does not reverse an observed entry side");

    for(route=-1;route<=1;route+=2) {
        reset(); circle(route);
        CHECK(!ring_result.stop && ring_result.branch_ok && ring_result.path_source==1);
    }
    reset(); enter(1); rg_road(image,0);
    for(i=0;i<RING_CANCEL_FRAMES;++i) {
        motion.yaw_deg=45; motion.wheel_revs+=0.05f; step(1);
        CHECK(ring_result.phase!=RING_CIRCLE && !ring_result.committed);
    }
    CHECK(ring_result.phase==RING_COOLDOWN && !ring_result.stop && !ring_result.fault);
    CHECK(!ring_result.exited_events && ring_speed_limit()==-1);
    puts("PASS: straight road plus supplied yaw/displacement cannot establish circle following");

    /* A visible island still wholly AHEAD is not proof that the car left
     * the entry, even if yaw and wheel feedback exceed the motion gates. */
    reset(); enter(1);
    for(i=0;i<8;++i) {
        motion.yaw_deg=45;motion.wheel_revs+=0.03f;step(1);
        CHECK(ring_result.branch_ok && !ring_result.committed);
        CHECK(ring_result.phase==RING_ENTER);
    }
    puts("PASS: an unchanged island ahead plus supplied motion cannot commit or enter CIRCLE");

    reset(); enter(1); transition(1,9);
    CHECK(ring_result.phase==RING_ENTER && ring_result.committed);
    memset(image,0,sizeof(image));
    for(i=0;i<(int)RING_HOLD_FRAMES;++i) {
        motion.yaw_deg+=1; motion.wheel_revs+=0.01f; step(1);
        CHECK(ring_result.path_source==2 && !ring_result.branch_ok);
        CHECK(ring_result.phase==RING_ENTER);
    }
    step(1); CHECK(ring_result.path_source==0 && ring_result.stop);
    CHECK(ring_result.phase==RING_ENTER && !ring_result.branch_ok);
    puts("PASS: held/predicted paths never provide branch or CIRCLE confirmation");

    reset(); enter(1); transition(1,9); rg_road(image,0);
    for(i=0;i<2;++i) { step(2); CHECK(ring_result.phase==RING_ENTER && ring_result.committed); }
    rg_transition(image,1,10,12); motion.yaw_deg=40; motion.wheel_revs+=0.025f; step(1);
    CHECK(ring_result.branch_ok && ring_result.path_source==1 && !ring_result.stop);
    puts("PASS: short normal-looking slice after actual entry does not cancel a committed car");

    reset(); confirm(1,1); rg_road(image,0);
    for(i=0;i<RING_CANCEL_FRAMES;++i) { motion.wheel_revs+=0.02f; step(1); }
    CHECK(ring_result.phase==RING_COOLDOWN && !ring_result.stop && !ring_result.fault);
    CHECK(ring_speed_limit()==-1 && !ring_result.exited_events);
    observe(1,RING_TOTAL_TIMEOUT_MS+1,1);
    CHECK(ring_result.phase!=RING_FAULT && ring_speed_limit()==-1);
    reset(); confirm(1,1); rg_cross(image,0);
    for(i=0;i<RING_CANCEL_FRAMES;++i) step(2);
    CHECK(ring_result.phase==RING_COOLDOWN && !ring_result.exited_events && !ring_result.stop);
    puts("PASS: uncommitted normal/cross fallback releases control without false exit or latched stop");

    reset(); rg_entry(image,1,0,0);
    for(i=0;i<4;++i) observe(1,110,0);
    CHECK(!ring_result.detected_events && ring_result.phase==RING_NORMAL);
    CHECK(ring_result.confirm_frames<RING_CONFIRM_FRAMES);
    reset(); rg_entry(image,1,0,0);
    for(i=0;i<3;++i) observe(1,20,0);
    observe(1,65535,0);
    CHECK(!ring_result.detected_events && ring_result.confirm_frames<RING_CONFIRM_FRAMES);
    /* Both visual timers must still expire a held path when the original
     * controller ALSO reports lost line. A valid original state would now
     * correctly release this uncommitted ring instead of parking the car. */
    reset(); enter(1); memset(image,0,sizeof(image));
    for(i=0;i<5;++i) observe(0,60,0);
    CHECK(!ring_result.branch_ok && ring_result.path_source==0 && ring_result.stop);
    CHECK(ring_result.phase==RING_ENTER && !ring_result.committed);
    reset(); enter(1); memset(image,0,sizeof(image));
    for(i=0;i<6;++i) observe(0,0,0);
    CHECK(!ring_result.branch_ok && ring_result.path_source==0 && ring_result.stop);
    CHECK(ring_result.phase==RING_ENTER && !ring_result.committed);
    puts("PASS: visual time and frame count expire evidence even while motion timestamp is frozen");

    /* Unknown visual timing invalidates old evidence, but cannot fabricate
     * half a second of total time per frame and compress the 30s guard. */
    reset();circle(1);rg_orbit(image,1,90);
    for(i=0;i<80;++i) {motion.now_ms+=20;observe(1,65535,0);}
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.fault);
    motion.now_ms+=RING_TOTAL_TIMEOUT_MS+1;observe(1,65535,0);
    CHECK(ring_result.phase==RING_FAULT && ring_result.fault==1);
    puts("PASS: unknown vdt does not shorten the real 30-second protection deadline");

    reset(); circle(1); rg_orbit(image,-1,90); step(1);
    CHECK(!ring_result.branch_ok && ring_result.path_source!=1);
    CHECK(ring_result.reject&(RING_REJ_CORRIDOR|RING_REJ_BRANCH));
    puts("PASS: opposite-curvature/look branch cannot refresh acquired ring corridor");

    reset(); circle(1); rg_exit(image,1); motion.wheel_revs=1;
    for(i=0;i<8;++i) { motion.yaw_deg=-400; step(1); }
    CHECK(ring_result.phase==RING_CIRCLE && ring_result.turn_deg<0);
    for(i=0;i<8;++i) { motion.yaw_deg=355; step(1); }
    CHECK(ring_result.phase==RING_CIRCLE);
    for(i=0;i<RING_EXIT_CONFIRM_FRAMES;++i) { motion.yaw_deg=370; step(1); }
    CHECK(ring_result.phase==RING_EXIT);
    old_look=ring_result.look_x; step(1);
    CHECK(ring_result.phase==RING_EXIT && ring_result.path_source==4);
    CHECK(ring_result.look_x>=old_look); /* Same-pixel tangent is valid. */
    rg_road(image,0);
    for(i=0;i<RING_RECOVER_FRAMES*2;++i) step(1);
    CHECK(ring_result.phase==RING_COOLDOWN && ring_result.exited_events==1);
    CHECK(ring_result.decision==RING_DECISION_EXIT);
    step(1);CHECK(ring_result.decision==RING_DECISION_EXIT);
    puts("PASS: signed full lap, genuine outer opening, tangent exit guide and ordinary-road recovery");

    reset(); circle(1); memset(image,0,sizeof(image));
    observe(1,RING_PATH_TIMEOUT_MS+1,1);
    CHECK(ring_result.phase==RING_FAULT && ring_speed_limit()==0);
    puts("PASS: actual committed path loss retains bounded-stop protection");
    for(i=-RING_ENTRY_BOOST_RPM;i<=RING_ENTRY_BOOST_RPM;++i) {
        int32_t encoded=ring_command_phase(RING_ENTER,i);
        CHECK(ring_command_boost(encoded)==i);
        CHECK(((uint32_t)encoded&65535u)==RING_ENTER);
        CHECK(ring_control_turn(237.5f,(float)i,0)==237.5f);
        CHECK(ring_control_turn(110,45,1)==RING_TURN_LIMIT_RPM);
    }
    printf("PASS: isolated protocol/control; %u additional synthetic geometry frames\n",frames);
    return 0;
}
