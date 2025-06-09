/*
 * Feedback_CAN.h
 *
 *  Created on: Apr 27, 2025
 *      Author: M.W Laptop
 */

#ifndef INC_STEERING_CAN_H_
#define INC_STEERING_CAN_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TX_SIZE 8

void encode_data(uint8_t* buffer, uint8_t bit_start, uint8_t length, int16_t value, uint8_t is_signed);

uint16_t decode_data(uint8_t *rxdata, uint8_t start_bit,/* float offset*/uint8_t length, uint8_t IsSigned);


void PrepareCANMessage(uint8_t *txdata, int16_t motor_current,int16_t rack_position, int16_t rack_force) ;

#endif /* INC_STEERING_CAN_H_ */
