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
#include <climits>

#define MCP3422_DEFAULT_ADDRESS 0x68 << 1

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

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
  void init(I2C * i2c_obj, Callback<void(uint8_t)> callback = NULL);
  bool config(uint8_t channel, Resolution res = _12bit, Conversion mode = Continuous, PGA gain = x1);
  void read(uint8_t channel, Callback<void(int32_t)> callback);
  int32_t readVoltage(uint8_t channel);
  void process();

 private:
  void internal_cb_handler(int event);
  void isConversionFinished();
  void test();

  Callback<void(int32_t)> done_cb;
  Callback<void(uint8_t)> error_cb;

  uint8_t _address;
  char _config[4];
  char _Buffer[4];
  uint8_t _current_channel;
  uint8_t _requested_bytes;

 protected:
  I2C * i2c;
  Timeout timeout;

  bool transfer(const char *data, uint8_t rx_len, uint8_t tx_len = 1);
};

#endif

