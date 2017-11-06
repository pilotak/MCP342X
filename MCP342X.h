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

#include "mbed.h"

#define MCP3422_DEFAULT_ADDRESS 0x68 << 1

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

  MCP342X(uint8_t slave_adr = MCP3422_DEFAULT_ADDRESS);
  virtual ~MCP342X(void);
  void init(I2C * i2c_obj);
  bool config(uint8_t channel, Resolution res = _12bit, Conversion mode = Continuous, PGA gain = x1);
  int32_t read(uint8_t channel);
  int32_t readVoltage(uint8_t channel);
  void newConversion(uint8_t channel);

 protected:
  I2C * i2c;
  uint8_t address;
  char _config[4];
  char _Buffer[4];

  bool isConversionFinished(uint8_t channel);
};

#endif

