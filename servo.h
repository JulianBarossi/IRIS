#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

#define SERVO_PING       0x01
#define SERVO_READ       0x02
#define SERVO_WRITE      0x03
#define SERVO_ADDR_POS   0x2A

uint8_t servo_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                       uint8_t *params, uint8_t param_count);

void servo_write_position(uint8_t id, uint16_t position,
                          uint16_t time, uint16_t speed);

void servo_ping(uint8_t id);

bool servo_ping_read(uint8_t id);

#endif