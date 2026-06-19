/**
  ******************************************************************************
  * @file    Wire.h
  * @brief   I2C stub — POD, no constructor
  ******************************************************************************
  */
#ifndef WIRE_H
#define WIRE_H

#include <stdint.h>

struct TwoWire {
    void begin() {}
    void beginTransmission(uint8_t) {}
    uint8_t endTransmission() { return 0; }
    void write(uint8_t) {}
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    uint8_t read() { return 0; }
};

extern TwoWire Wire;

#endif
