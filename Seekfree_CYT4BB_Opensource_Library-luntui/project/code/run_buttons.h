#ifndef CODE_RUN_BUTTONS_H_
#define CODE_RUN_BUTTONS_H_

#include <stdint.h>

/* Call exactly once per millisecond. Inputs are pressed booleans, not GPIO
 * levels. This module owns no GPIO, timer, motor, display or shared memory. */
#define RUN_BUTTON_DEBOUNCE_MS 20u
#define RUN_BUTTON_LONG_MS     1000u
#define RUN_BUTTON_EVENT_START   1u
#define RUN_BUTTON_EVENT_DISPLAY 2u

typedef struct {
    uint8_t run_enabled;
    uint8_t display_ring;
    uint8_t key_a, key_b;
    uint8_t debounce_a, debounce_b;
    uint8_t gesture_active, inhibit_start, chord_done, start_done;
    uint8_t release_ms;
    uint16_t held_ms;
} run_buttons_t;

static inline void run_buttons_init(run_buttons_t *s)
{
    s->run_enabled = s->display_ring = 0;
    s->key_a = s->key_b = s->debounce_a = s->debounce_b = 0;
    s->gesture_active = s->inhibit_start = s->chord_done = s->start_done = 0;
    s->release_ms = 0;
    s->held_ms = 0;
}

static inline void run_buttons_debounce(uint8_t raw, uint8_t *stable,
                                        uint8_t *count)
{
    if (raw == *stable) {
        *count = 0;
    } else if (++*count >= RUN_BUTTON_DEBOUNCE_MS) {
        *stable = raw;
        *count = 0;
    }
}

/* P20_2 alone held for one second starts once. A chord toggles the display
 * once. Seeing P20_3 at ANY point suppresses starting for the whole gesture;
 * releasing one chord key cannot turn its remaining key into a start hold.
 * A fresh gesture requires BOTH raw keys released for the debounce period. */
static inline uint8_t run_buttons_tick(run_buttons_t *s,
                                        uint8_t p20_2_pressed,
                                        uint8_t p20_3_pressed)
{
    uint8_t a = (uint8_t)(p20_2_pressed != 0);
    uint8_t b = (uint8_t)(p20_3_pressed != 0);
    uint8_t event = 0;

    run_buttons_debounce(a, &s->key_a, &s->debounce_a);
    run_buttons_debounce(b, &s->key_b, &s->debounce_b);

    if (a || b) {
        s->gesture_active = 1;
        s->release_ms = 0;
    } else if (s->gesture_active) {
        if (++s->release_ms >= RUN_BUTTON_DEBOUNCE_MS) {
            s->gesture_active = s->inhibit_start = 0;
            s->chord_done = s->start_done = 0;
            s->release_ms = 0;
            s->held_ms = 0;
        }
    }

    if (b) s->inhibit_start = 1;
    if (s->gesture_active && a && b && s->key_a && s->key_b &&
        !s->chord_done) {
        s->display_ring ^= 1u;
        s->chord_done = 1;
        s->inhibit_start = 1;
        event |= RUN_BUTTON_EVENT_DISPLAY;
    }

    if (a && !s->inhibit_start && !s->start_done) {
        if (s->held_ms < RUN_BUTTON_LONG_MS) ++s->held_ms;
        if (s->key_a && s->held_ms >= RUN_BUTTON_LONG_MS) {
            s->start_done = 1;
            if (!s->run_enabled) {
                s->run_enabled = 1;
                event |= RUN_BUTTON_EVENT_START;
            }
        }
    } else {
        s->held_ms = 0;
    }
    return event;
}

#endif
