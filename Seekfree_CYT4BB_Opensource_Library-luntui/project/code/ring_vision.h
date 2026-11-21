#ifndef CODE_RING_VISION_H_
#define CODE_RING_VISION_H_

#include "ring.h"

void ring_vision_init(void);
uint8_t ring_vision_binary_display(void);
uint8_t ring_vision_run_requested(void);
uint8_t ring_vision_preview(void);
const ring_result_t *ring_vision_display_result(void);
/* before_frame only remembers the image. select observes the original track
 * decision before updating an active ring; normal detection stays AFTER the
 * original frame has been published. These hooks never modify input pixels. */
void ring_vision_before_frame(const uint8_t image[RING_H][RING_W]);
void ring_vision_select(int *steer_error, int *track_state);
void ring_vision_after_frame(const uint8_t image[RING_H][RING_W]);

#endif
