//*****************************
// Raspberry Pi 2 I2C
//*****************************

#ifndef RPI_I2C_H
#define RPI_I2C_H

#include <stdint.h>

//I2C RX/TX Buffer Sizes
#define RXBUFFERSIZE 12
#define TXBUFFERSIZE 13

uint8_t i2cRXBuffer[RXBUFFERSIZE];
uint8_t i2cTXBuffer[TXBUFFERSIZE];

int init_I2C(void);
int open_Port(int devId);
int terminate_I2C(void);
int i2cMasterTransmit(uint16_t addr);
int i2cMasterReceive(uint16_t addr);

#endif