/*
 * Linear_displacment.h
 *
 *  Created on: Jun 9, 2025
 *      Author: MARIO
 */

#ifndef LINEAR_DISPLACMENT_LINEAR_DISPLACMENT_H_
#define LINEAR_DISPLACMENT_LINEAR_DISPLACMENT_H_

//Distance from center 28.13

// Raw count limits from the KTC200
#define COUNT_MIN        464
#define COUNT_MAX        3910

// Desired physical range
#define POS_MIN         -74.0f
#define POS_MAX         +74.0f

float get_linear_position(float sensor_count);

#endif /* LINEAR_DISPLACMENT_LINEAR_DISPLACMENT_H_ */
