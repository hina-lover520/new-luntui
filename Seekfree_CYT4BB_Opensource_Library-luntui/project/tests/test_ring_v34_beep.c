#include <stdio.h>
#include <stdlib.h>
#include "ring_event_beep.h"

#define CHECK(c) do { if(!(c)) { fprintf(stderr,"FAIL v34-beep:%d\n",__LINE__);exit(1); } } while(0)

static void ordinary_and_pair(void)
{
    ring_event_beep_t s;
    ring_event_beep_init(&s);
    CHECK(ring_event_beep_tick(&s,0,0)==-1);
    ring_event_beep_single(&s);
    CHECK(ring_event_beep_tick(&s,10,0)==1);
    /* Exit recognition can arrive during the original entry pulse. It
     * queues exactly two short pulses and preserves the current tone. */
    ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,129,0)==-1);
    CHECK(ring_event_beep_tick(&s,130,0)==0);
    CHECK(ring_event_beep_tick(&s,249,0)==-1);
    CHECK(ring_event_beep_tick(&s,250,0)==1);
    CHECK(ring_event_beep_tick(&s,309,0)==-1);
    CHECK(ring_event_beep_tick(&s,310,0)==0);
    CHECK(ring_event_beep_tick(&s,369,0)==-1);
    CHECK(ring_event_beep_tick(&s,370,0)==1);
    ring_event_beep_single(&s); /* Original completed-exit event stays single. */
    CHECK(ring_event_beep_tick(&s,429,0)==-1);
    CHECK(ring_event_beep_tick(&s,430,0)==0);
    CHECK(ring_event_beep_tick(&s,489,0)==-1);
    CHECK(ring_event_beep_tick(&s,490,0)==1);
    CHECK(ring_event_beep_tick(&s,609,0)==-1);
    CHECK(ring_event_beep_tick(&s,610,0)==0);
    CHECK(ring_event_beep_tick(&s,730,0)==-1);
    CHECK(ring_event_beep_tick(&s,900,0)==-1);
}

static void startup_owner(void)
{
    ring_event_beep_t s;
    ring_event_beep_init(&s);
    ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,1,0)==1);
    CHECK(ring_event_beep_tick(&s,20,1)==0);
    CHECK(ring_event_beep_tick(&s,21,1)==-1);
    ring_event_beep_single(&s);
    CHECK(ring_event_beep_tick(&s,22,1)==-1);
    CHECK(ring_event_beep_tick(&s,500,0)==-1);
    ring_event_beep_single(&s); /* Parked preview remains allowed. */
    CHECK(ring_event_beep_tick(&s,501,0)==1);
    CHECK(ring_event_beep_tick(&s,621,0)==0);
    CHECK(ring_event_beep_tick(&s,1000,0)==-1);
}

static void wrap_and_backlog(void)
{
    ring_event_beep_t s;
    uint32_t start=UINT32_MAX-20u;
    ring_event_beep_init(&s);
    ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,start,0)==1);
    CHECK(ring_event_beep_tick(&s,start+59u,0)==-1);
    CHECK(ring_event_beep_tick(&s,start+60u,0)==0);
    CHECK(ring_event_beep_tick(&s,start+119u,0)==-1);
    CHECK(ring_event_beep_tick(&s,start+120u,0)==1);
    CHECK(ring_event_beep_tick(&s,start+180u,0)==0);
    CHECK(ring_event_beep_tick(&s,start+240u,0)==-1);
    /* Long idle periods cannot leave a stale gap deadline that blocks
     * a new event after the signed millisecond half-range. */
    ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,0x80001000u,0)==1);
    CHECK(ring_event_beep_tick(&s,0x8000103cu,0)==0);
    CHECK(ring_event_beep_tick(&s,0x80001078u,0)==1);
    CHECK(ring_event_beep_tick(&s,0x800010b4u,0)==0);
}

static void stop_discards_pairs_keeps_preview(void)
{
    ring_event_beep_t s;
    ring_event_beep_init(&s);
    ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,1,0)==1);
    ring_event_beep_cancel_pairs(&s); /* Stop while first short tone is on. */
    CHECK(ring_event_beep_tick(&s,20,0)==0);
    CHECK(ring_event_beep_tick(&s,200,0)==-1);
    ring_event_beep_single(&s);
    CHECK(ring_event_beep_tick(&s,201,0)==1);
    ring_event_beep_pair(&s);
    ring_event_beep_cancel_pairs(&s); /* Preview tone belongs to single. */
    CHECK(ring_event_beep_tick(&s,220,0)==-1);
    CHECK(ring_event_beep_tick(&s,321,0)==0);
    CHECK(ring_event_beep_tick(&s,500,0)==-1);
    ring_event_beep_pair(&s);
    CHECK(ring_event_beep_tick(&s,501,0)==1);
    CHECK(ring_event_beep_tick(&s,561,0)==0);
    ring_event_beep_cancel_pairs(&s); /* Stop in the gap skips second pulse. */
    CHECK(ring_event_beep_tick(&s,570,0)==-1);
    CHECK(ring_event_beep_tick(&s,700,0)==-1);
    ring_event_beep_pair(&s); /* A future exit can independently request again. */
    CHECK(ring_event_beep_tick(&s,701,0)==1);
    CHECK(ring_event_beep_tick(&s,761,0)==0);
    CHECK(ring_event_beep_tick(&s,821,0)==1);
    CHECK(ring_event_beep_tick(&s,881,0)==0);
}

int main(void)
{
    ordinary_and_pair();startup_owner();wrap_and_backlog();stop_discards_pairs_keeps_preview();
    puts("PASS: v34 nonblocking 60/60ms exit pair, original120ms singles, startup ownership, operatorstop cancels pairs/preserves preview, queue and clock wrap");
    return 0;
}
