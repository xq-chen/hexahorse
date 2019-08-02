#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

typedef enum {
    DIR_EXPAND = 0,
    DIR_CONTRACT,
    DIR_STOP,
    DIR_BREAK
} LEG_DIRECTION;

typedef enum {
    LEG_SET_A = 0,  // Left 2 (0, 2), Right 1 (4)
    LEG_SET_B,      // Left 1 (1)   , Right(3, 5)
    LEG_SET_TRANSIT, // During transition
    LEG_SET_MM     // 強制的に水平補正motion()を動かす
} LEG_SET;

void legMotion (int num, LEG_DIRECTION dir, int speed);
void legMotionf(int num, LEG_DIRECTION dir, float pwm);

void motion(LEG_SET legSet,
            uint32_t initialBodyHeight,
            uint32_t bodyHeight,
            float roll, float pitch);

void motionf(LEG_SET legSet,
            uint32_t initialBodyHeight,
            uint32_t bodyHeight,
            float roll, float pitch);
#endif
