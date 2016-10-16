

#ifndef _I2C_SLAVE_LITE_H_
#define _I2C_SLAVE_LITE_H_

#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#elif defined(__PIC24H__)
#include "p24Hxxxx.h"
#endif

#include <aversive.h>

#define I2C_BUFFER_SIZE 32

void i2c_init(uint8_t addr);
void i2c_register_write_event(void (*event)(uint8_t, uint8_t *, uint16_t));
void i2c_register_read_event(void (*event)(uint8_t, uint8_t *));
#endif

