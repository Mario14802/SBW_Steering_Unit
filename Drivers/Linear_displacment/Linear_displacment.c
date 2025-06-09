/*
 * Linear_displacment.c
 *
 *  Created on: Jun 9, 2025
 *      Author: MARIO
 */


#include "Linear_displacment.h"   // your sensor’s header

float get_linear_position(float sensor_count)
{
    // 1) Clamp raw reading just in case
    if (sensor_count < COUNT_MIN) sensor_count = COUNT_MIN;
    else if (sensor_count > COUNT_MAX) sensor_count = COUNT_MAX;

    // 2) Linearly map [COUNT_MIN…COUNT_MAX] → [POS_MIN…POS_MAX]:
    //     pos = (sensor_count - COUNT_MIN) * (POS_MAX - POS_MIN) / (COUNT_MAX - COUNT_MIN) + POS_MIN
    float pos = (float)(sensor_count - COUNT_MIN) * (POS_MAX - POS_MIN)/ (float)(COUNT_MAX - COUNT_MIN)
              + POS_MIN;

    return pos;
}
