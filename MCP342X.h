/*
Originally written by B@tto
Contact : batto@hotmail.fr
Edited for mbed by www.github.com/pilotak


  MCP342X.h - ADC 18 bits i2c library for Wiring & Arduino
  Copyright (c) 2012 Yann LEFEBVRE.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef MCP342X_H
#define MCP342X_H

#include <climits>
#include "mbed.h"
#include <chrono>
using namespace std::chrono;

#define MCP342X_DEFAULT_ADDRESS 0x68 << 1
#define MCP342X_DEFAULT_TIMEOUT 100ms

class MCP342X {
 public:
  typedef enum {
    OneShot = 0,
    Continuous
  } Conversion;

  typedef enum {
    x1 = 0,
    x2,
    x4,
    x8
  } PGA;

  typedef enum {
    _12bit = 0,
    _14bit,
    _16bit,
    _18bit
  } Resolution;

  MCP342X(I2C * i2c_obj, uint8_t slave_adr = MCP342X_DEFAULT_ADDRESS);
  MCP342X(PinName sda, PinName scl, uint8_t slave_adr = MCP342X_DEFAULT_ADDRESS, int32_t freq = 400000);
  virtual ~MCP342X(void);
  bool init(I2C * i2c_obj = NULL);
  bool config(uint8_t channel, Resolution res = _12bit, Conversion mode = Continuous, PGA gain = x1);
  int32_t read(uint8_t channel);
  int32_t readVoltage(uint8_t channel);
  bool newConversion(uint8_t channel);
  int32_t getResult(uint8_t channel);
  bool isConversionFinished(uint8_t channel);

 protected:
  I2C * _i2c;

 private:
  uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];
  uint8_t _address;
  char _config[4];
  char _Buffer[4];
};

#endif  // MCP342X_H
