#include <assert.h>
#include <stdio.h>
#include "run_buttons.h"

static unsigned starts, displays;

static void sample(run_buttons_t *s, unsigned milliseconds, int a, int b)
{
    unsigned i;
    for (i = 0; i < milliseconds; ++i) {
        unsigned event = run_buttons_tick(s, (uint8_t)a, (uint8_t)b);
        if (event & RUN_BUTTON_EVENT_START) ++starts;
        if (event & RUN_BUTTON_EVENT_DISPLAY) ++displays;
        assert(event != (RUN_BUTTON_EVENT_START | RUN_BUTTON_EVENT_DISPLAY));
    }
}

static void reset(run_buttons_t *s)
{
    run_buttons_init(s);
    starts = displays = 0;
}

int main(void)
{
    run_buttons_t s;
    unsigned i;

    reset(&s);
    sample(&s, 10000, 0, 0);
    assert(!s.run_enabled && !s.display_ring && !starts && !displays);
    sample(&s, 999, 1, 0);
    assert(!s.run_enabled && !starts);
    sample(&s, 1, 1, 0);
    assert(s.run_enabled && starts == 1);
    sample(&s, 10000, 1, 0);
    sample(&s, 20, 0, 0);
    sample(&s, 2000, 1, 0);
    assert(starts == 1 && !displays);

    reset(&s);
    sample(&s, 19, 1, 1);
    assert(!s.display_ring && !displays);
    sample(&s, 1, 1, 1);
    assert(s.display_ring && displays == 1 && !s.run_enabled);
    sample(&s, 10000, 1, 1);
    sample(&s, 2000, 1, 0); /* Release B first: must not start. */
    assert(!s.run_enabled && displays == 1);
    sample(&s, 100, 1, 1);
    assert(displays == 1); /* Both keys were never fully released. */
    sample(&s, 20, 0, 0);
    sample(&s, 20, 1, 1);
    assert(!s.display_ring && displays == 2 && !starts);

    reset(&s);
    sample(&s, 990, 1, 0);
    sample(&s, 1, 1, 1); /* Raw B, even before debounce, defeats long A. */
    sample(&s, 2000, 1, 0);
    assert(!s.run_enabled && !starts && !displays);
    sample(&s, 19, 0, 0);
    sample(&s, 2000, 1, 0); /* Too short a release cannot rearm. */
    assert(!s.run_enabled && !starts);
    sample(&s, 20, 0, 0);
    sample(&s, 1000, 1, 0);
    assert(s.run_enabled && starts == 1);

    reset(&s);
    sample(&s, 50, 0, 1); /* B first, then A: chord is still recognized. */
    sample(&s, 20, 1, 1);
    sample(&s, 2000, 0, 1); /* Release A first: no repeat display event. */
    assert(!s.run_enabled && s.display_ring && displays == 1);

    reset(&s);
    for (i = 0; i < 50; ++i) {
        sample(&s, 1, 1, 1);
        sample(&s, 1, 0, 0);
    }
    assert(!s.run_enabled && !s.display_ring && !starts && !displays);
    sample(&s, 20, 0, 0);
    for (i = 0; i < 100; ++i) {
        sample(&s, 9, 1, 0);
        sample(&s, 1, 0, 0);
    }
    assert(!s.run_enabled && !starts);
    sample(&s, 20, 0, 0);
    sample(&s, 1000, 1, 0);
    assert(s.run_enabled && starts == 1);
    sample(&s, 20, 0, 0);
    sample(&s, 20, 1, 1);
    assert(s.run_enabled && s.display_ring && displays == 1);

    puts("PASS: button idle, long start, debounce, chord priority/release, run latch");
    return 0;
}
