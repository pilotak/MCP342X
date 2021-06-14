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

    MCP342X(I2C *i2c_obj, uint8_t slave_adr = MCP342X_DEFAULT_ADDRESS);
    MCP342X(PinName sda, PinName scl, uint8_t slave_adr = MCP342X_DEFAULT_ADDRESS, int32_t freq = 400000);
    virtual ~MCP342X(void);
    bool init(I2C *i2c_obj = NULL);
    bool config(uint8_t channel, Resolution res = _12bit, Conversion mode = Continuous, PGA gain = x1);
    int32_t read(uint8_t channel);
    int32_t readVoltage(uint8_t channel);
    bool newConversion(uint8_t channel);
    int32_t getResult(uint8_t channel);

    /**
     * @brief Check in conversion is done
     * 
     * @param channel 
     * @return 0 if not done, 1 if done, -1 in case of error
     */
    int8_t isConversionFinished(uint8_t channel);

  protected:
    I2C *_i2c;

  private:
    uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];
    uint8_t _address;
    char _config[4];
    char _Buffer[4];
};

#endif  // MCP342X_H
