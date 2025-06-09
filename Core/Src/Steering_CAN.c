/*
 * Feedback_CAN.c
 *
 *  Created on: Apr 27, 2025
 *      Author: M.W Laptop
 */

#include <Steering_CAN.h>
uint16_t Bitmasks[] = {
    0x0000, 0x0001, 0x0003, 0x0007,
    0x000F, 0x001F, 0x003F, 0x007F,
    0x00FF, 0x01FF, 0x03FF, 0x07FF,
    0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF,
    0xFFFF
};

void encode_data(uint8_t* buffer, uint8_t bit_start, uint8_t length, int16_t value, uint8_t is_signed) {
    uint16_t to_write;

    if (is_signed) {
        to_write = value & Bitmasks[length - 1];
        if (value < 0) {
            to_write |= (1 << (length - 1));
        }
    }
    else {
        to_write = (uint16_t)value & Bitmasks[length];
    }

    uint8_t bit_offset = bit_start % 8;
    uint8_t byte_index = bit_start / 8;

    uint8_t bits_written = 0;
    while (bits_written < length) {
        uint8_t bits_available = 8 - bit_offset;
        uint8_t bits_this_round = (length - bits_written < bits_available) ? (length - bits_written) : bits_available;

        uint8_t mask = ((1 << bits_this_round) - 1) << bit_offset;
        buffer[byte_index] &= ~mask;
        buffer[byte_index] |= ((to_write >> bits_written) << bit_offset) & mask;

        bits_written += bits_this_round;
        bit_offset = 0;
        byte_index++;
    }
}


// decode function
uint16_t decode_data(uint8_t *rxdata, uint8_t start_bit,/* float offset*/uint8_t length, uint8_t IsSigned) {
  uint16_t value = 0;
  for (uint8_t i = 0; i < length; i++) {
    uint8_t byte_index = (start_bit + i) / 8;
    uint8_t bit_position = (start_bit + i) % 8;
    if (rxdata[byte_index] & (1 << bit_position)) {
      value |= (1 << i);
    }
  }
  if (IsSigned && (value & (1 << (length - 1)))) {
    value |= ~((1 << length) - 1);
  }
 /* if (offset != 0) {
    value *= offset;

  }*/
  return value;
}


// Function to prepare CAN message
void PrepareCANMessage(uint8_t *txdata, int16_t motor_current,
		int16_t steering_wheel_angle, int16_t steering_wheel_speed,
		int16_t PWM_output) {
	memset(txdata, 0, TX_SIZE); // Clear txdata before encoding ***********************************

	// Encode parameters into the message
	Encode(txdata, 0, 13, motor_current, 1);   // Speed: 12-bit, unsigned
	Encode(txdata, 13,  12, steering_wheel_angle, 1); // Angle: 10-bit, signed
	Encode(txdata, 25,  14, steering_wheel_speed, 1); // Current: 8-bit, unsigned
	Encode(txdata, 39,  15, PWM_output, 1);
}
