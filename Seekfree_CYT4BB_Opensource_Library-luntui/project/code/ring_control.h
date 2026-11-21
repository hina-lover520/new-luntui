#ifndef CODE_RING_CONTROL_H_
#define CODE_RING_CONTROL_H_

#include "ring.h"

/* Invoked only for an acquired, fresh ring with actual steering ownership.
 * Return the caller's value exactly when no ring owns it. */
static inline float ring_control_turn(float original_turn, float boost, int owns)
{
    float value;
    if (!owns) return original_turn;
    value = original_turn + boost;
    if (value > RING_TURN_LIMIT_RPM) return RING_TURN_LIMIT_RPM;
    if (value < -RING_TURN_LIMIT_RPM) return -RING_TURN_LIMIT_RPM;
    return value;
}

/* Apply only AFTER the original controller has calculated its output.
 * A negative cap means no confirmed ring: return the original speed exactly. */
static inline int ring_control_stop(int cap, int state, int fault, int fresh)
{
    return cap >= 0 && (cap == 0 || state == 0 || fault || !fresh);
}

static inline float ring_control_speed(float original_speed, int cap,
                                       int state, int fault, int fresh)
{
    if (cap < 0) return original_speed;
    if (ring_control_stop(cap, state, fault, fresh)) return 0.0f;
    return original_speed > cap ? (float)cap : original_speed;
}

#endif
