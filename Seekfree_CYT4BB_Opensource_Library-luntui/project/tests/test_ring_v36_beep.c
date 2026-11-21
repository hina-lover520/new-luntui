#include "ring_event_beep.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if(!(c)) { fprintf(stderr,"FAIL v36 beep:%d %s\n",__LINE__,#c);exit(1); } } while(0)

static void fast_one_three_two(uint32_t start)
{
    ring_event_beep_t s;
    unsigned t,on=0,off=0;
    static const unsigned starts[]={0u,120u,240u,360u,480u,600u};
    ring_event_beep_init(&s);
    ring_event_beep_fast_single(&s);
    CHECK(ring_event_beep_tick(&s,start,0)==1);on=1;
    /* Later semantic events may queue while the first pulse is sounding. */
    ring_event_beep_triple(&s);ring_event_beep_pair(&s);
    for(t=1u;t<=800u;++t) {
        int8_t action=ring_event_beep_tick(&s,start+t,0);
        if(action==1) {
            CHECK(on<6u && t==starts[on]);++on;
        } else if(action==0) {
            CHECK(off<6u && t==starts[off]+60u);++off;
        }
    }
    CHECK(on==6u && off==6u);
    CHECK(ring_event_beep_tick(&s,start+1000u,0)==-1);
}
static void stop_and_startup(void)
{
    ring_event_beep_t s;
    ring_event_beep_init(&s);
    ring_event_beep_triple(&s);
    CHECK(ring_event_beep_tick(&s,1u,0)==1);
    CHECK(ring_event_beep_tick(&s,61u,0)==0);
    ring_event_beep_cancel_pairs(&s);
    CHECK(ring_event_beep_tick(&s,80u,0)==-1);
    CHECK(ring_event_beep_tick(&s,200u,0)==-1);
    ring_event_beep_single(&s);
    CHECK(ring_event_beep_tick(&s,201u,0)==1);
    ring_event_beep_fast_single(&s);ring_event_beep_triple(&s);ring_event_beep_pair(&s);
    ring_event_beep_cancel_pairs(&s);
    CHECK(ring_event_beep_tick(&s,220u,0)==-1); /* Parked preview not canceled. */
    CHECK(ring_event_beep_tick(&s,321u,0)==0);
    CHECK(ring_event_beep_tick(&s,1000u,0)==-1);
    ring_event_beep_fast_single(&s);ring_event_beep_triple(&s);ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,1001u,1)==-1);
    CHECK(ring_event_beep_tick(&s,1600u,0)==-1); /* No queued startup leak. */
    ring_event_beep_triple(&s);
    CHECK(ring_event_beep_tick(&s,1601u,0)==1);
    CHECK(ring_event_beep_tick(&s,1602u,1)==0);
    CHECK(ring_event_beep_tick(&s,1603u,1)==-1);
}
int main(void)
{
    fast_one_three_two(1u);fast_one_three_two(UINT32_MAX-30u);
    stop_and_startup();
    puts("PASS: v36 entry1/exit-found3/success2 groups each60ms with60ms gaps, ordered queued events exactly6 pulses, wrap, STOP cancels fast events while preserving parked preview, startup ownership");
    return 0;
}

