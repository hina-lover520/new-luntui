#ifndef CODE_RING_EVENT_BEEP_H_
#define CODE_RING_EVENT_BEEP_H_

#include <stdint.h>
#include <string.h>
#include "ring_link.h"

/* Main queues semantic events; the existing1ms owner alone drives pulses.
 * Running entry/exit-found/success are1/3/2 fast pulses. Parked preview
 * retains its original single tone and startup owns the buzzer unchanged. */
#ifndef RING_EXIT_BEEP_MS
#define RING_EXIT_BEEP_MS 60u
#endif
#ifndef RING_EXIT_BEEP_GAP_MS
#define RING_EXIT_BEEP_GAP_MS 60u
#endif

typedef struct {
    volatile uint32_t singles_requested,fast_singles_requested,pairs_requested,triples_requested;
    volatile uint32_t pair_cancels_requested;
    uint32_t singles_started,fast_singles_started,pairs_started,triples_started;
    uint32_t deadline_ms,next_ms,pair_cancels_seen;
    uint8_t on,fast,pair_remaining,waiting;
} ring_event_beep_t;

static inline void ring_event_beep_init(ring_event_beep_t *s)
{ memset(s,0,sizeof(*s)); }

static inline void ring_event_beep_single(ring_event_beep_t *s)
{ ++s->singles_requested; }

static inline void ring_event_beep_fast_single(ring_event_beep_t *s)
{ ++s->fast_singles_requested; }

static inline void ring_event_beep_pair(ring_event_beep_t *s)
{ ++s->pairs_requested; }

static inline void ring_event_beep_triple(ring_event_beep_t *s)
{ ++s->triples_requested; }

/* Kept as the existing STOP-facing API: cancel all running-event groups,
 * including a pending entry or third exit tone, while parked preview stays. */
static inline void ring_event_beep_cancel_pairs(ring_event_beep_t *s)
{ ++s->pair_cancels_requested; }

/* -1 leaves the original buzzer unchanged,0 releases an owned pulse,
 * 1 starts a pulse. Deadline comparisons remain valid across clock wrap. */
static inline int8_t ring_event_beep_tick(ring_event_beep_t *s,uint32_t now,
    int starting)
{
    if(starting) {
        s->singles_started=s->singles_requested;
        s->fast_singles_started=s->fast_singles_requested;
        s->pairs_started=s->pairs_requested;
        s->triples_started=s->triples_requested;
        s->pair_cancels_seen=s->pair_cancels_requested;
        s->pair_remaining=s->waiting=0;
        if(s->on) { s->on=0;return 0; }
        return -1;
    }
    if(s->pair_cancels_seen!=s->pair_cancels_requested) {
        s->pair_cancels_seen=s->pair_cancels_requested;
        s->fast_singles_started=s->fast_singles_requested;
        s->pairs_started=s->pairs_requested;
        s->triples_started=s->triples_requested;s->pair_remaining=0;
        if(s->fast) {
            s->waiting=0;
            if(s->on) {s->on=0;return 0;}
        }
    }
    if(s->on && (int32_t)(now-s->deadline_ms)>=0) {
        s->on=0;s->waiting=1;
        s->next_ms=now+(s->fast?RING_EXIT_BEEP_GAP_MS:RING_EVENT_BEEP_MS);
        return 0;
    }
    if(s->waiting && (int32_t)(now-s->next_ms)>=0)s->waiting=0;
    if(s->on || s->waiting)return -1;
    if(s->pair_remaining) {
        --s->pair_remaining;s->fast=1;
    } else if(s->fast_singles_started!=s->fast_singles_requested) {
        ++s->fast_singles_started;s->fast=1;
    } else if(s->triples_started!=s->triples_requested) {
        ++s->triples_started;s->pair_remaining=2;s->fast=1;
    } else if(s->pairs_started!=s->pairs_requested) {
        ++s->pairs_started;s->pair_remaining=1;s->fast=1;
    } else if(s->singles_started!=s->singles_requested) {
        ++s->singles_started;s->fast=0;
    } else return -1;
    s->on=1;
    s->deadline_ms=now+(s->fast?RING_EXIT_BEEP_MS:RING_EVENT_BEEP_MS);
    return 1;
}

#endif
