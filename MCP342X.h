/*
MIT License

Copyright (c) 2018 Pavel Slama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MCP342X_H
#define MCP342X_H

#include <climits>
#include "mbed.h"
#include "mbed_events.h"

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

  typedef enum {
    None,
    Init,
    Reading,
    Waiting
  } Stage;

  typedef enum {
    NoSlave,
    EventError,
    TransferError,
    Timeout
  } ErrorType;

  explicit MCP342X(uint8_t slave_adr = MCP3422_DEFAULT_ADDRESS);
  virtual ~MCP342X(void);
  void init(I2C * i2c_obj, EventQueue * queue, Callback<void(ErrorType)> callback = NULL);
  bool config(uint8_t channel, Resolution res = _12bit, Conversion mode = Continuous, PGA gain = x1);
  void process();
  bool read(uint8_t channel, Callback<void(uint8_t, int32_t)> callback);
  int32_t toVoltage(uint8_t channel, int32_t value);

 private:
  void cbHandler(int event);
  void isConversionFinished();

  Callback<void(uint8_t, int32_t)> done_cb;
  Callback<void(ErrorType)> error_cb;

  uint8_t _address;
  char _config[4];
  char _Buffer[4];
  uint8_t _current_channel;
  uint8_t _requested_bytes;
  Stage _stage;
  uint16_t _wait_time[4];

 protected:
  I2C * _i2c;
  EventQueue * _queue;

  bool transfer(const char *data, uint8_t rx_len = 0, uint8_t tx_len = 1);
};

#endif

