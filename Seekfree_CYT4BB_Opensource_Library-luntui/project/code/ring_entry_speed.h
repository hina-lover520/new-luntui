#ifndef CODE_RING_ENTRY_SPEED_H_
#define CODE_RING_ENTRY_SPEED_H_
#include "ring.h"
#include <string.h>

/* Confirmed-entry cap profile starts at recognition, before the turn.
 * A separate continuous forward profile covers ordinary/release targets.
 * No PID, wheel command, tilt or timer ISR is changed. */
typedef struct {
    uint32_t previous_ms;
    float held, output, goal;
    unsigned valid;
} ring_entry_speed_t;

static inline void ring_entry_speed_begin(ring_entry_speed_t *s,
    uint32_t now, float preceding_target)
{
    memset(s,0,sizeof(*s));
    if(preceding_target<0.f)return;
    s->valid=1;s->previous_ms=now;
    s->held=s->output=s->goal=preceding_target;
}

static inline float ring_entry_speed_apply(ring_entry_speed_t *s,
    float ordinary,int cap,unsigned stage,uint32_t now,int eligible)
{
    uint32_t dt=now-s->previous_ms;
    float step,limit;
    s->previous_ms=now;
    if(!s->valid || !eligible || cap<=0 || ordinary<=0.f || !stage || stage==4u){
        s->valid=0;return ordinary;
    }
    s->goal=ordinary<(float)cap?ordinary:(float)cap;
    limit=RING_ENTRY_SPEED_SLEW_RPM_S*(float)(dt>100u?100u:dt)*.001f;
    step=s->goal-s->output;
    if(step>limit)step=limit;
    if(step< -limit)step= -limit;
    s->output+=step;
    return s->output;
}
#endif
