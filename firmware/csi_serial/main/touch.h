#ifndef TOUCH_H
#define TOUCH_H

#include <stdint.h>

typedef enum {
    GESTURE_NONE = 0,
    GESTURE_TAP,
    GESTURE_SWIPE_LEFT,
    GESTURE_SWIPE_RIGHT,
    GESTURE_SWIPE_UP,
    GESTURE_SWIPE_DOWN,
} touch_gesture_t;

void touch_init(void);
touch_gesture_t touch_read(void);

#endif
