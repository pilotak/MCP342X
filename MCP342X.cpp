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

#include "MCP342X.h"

MCP342X::MCP342X(I2C *i2c_obj, uint8_t slave_adr):
    _address(slave_adr) {
    _i2c = i2c_obj;
}

MCP342X::MCP342X(PinName sda, PinName scl, uint8_t slave_adr, int32_t freq):
    _address(slave_adr) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(freq);
}

MCP342X::~MCP342X(void) {
    if (_i2c == reinterpret_cast<I2C *>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool MCP342X::init(I2C *i2c_obj) {
    int32_t ack = -1;

    if (i2c_obj != NULL) {
        _i2c = i2c_obj;
    }

    _config[0] = 0b00010000;  // channel 1, continuous mode, 12bit
    _config[1] = 0b00110000;  // channel 2, continuous mode, 12bit
    _config[2] = 0b01010000;  // channel 3, continuous mode, 12bit
    _config[3] = 0b01110000;  // channel 4, continuous mode, 12bit

    memset(_Buffer, 0, sizeof(_Buffer));

    // test if device is on the bus
    if (_i2c != NULL) {
        _i2c->lock();
        ack = _i2c->write(_address, &_config[0], 1);
        _i2c->unlock();
    }

    return (ack == 0);
}

bool MCP342X::config(uint8_t channel, Resolution res, Conversion mode, PGA gain) {
    int32_t ack = -1;
    _config[channel] |= ((res << 2) | gain);
    _config[channel] ^= (-mode ^ _config[channel]) & (1 << 4);

    if (_i2c != NULL) {
        _i2c->lock();
        ack = _i2c->write(_address, &_config[channel], 1);
        _i2c->unlock();

        return (ack == 0);
    }

    return false;
}

bool MCP342X::newConversion(uint8_t channel) {
    char byte = _config[channel] |= 128;
    int32_t ack = -1;

    memset(_Buffer, 0, sizeof(_Buffer));

    _i2c->lock();
    ack = _i2c->write(_address, &byte, 1);
    _i2c->unlock();

    return (ack == 0);
}

bool MCP342X::isConversionFinished(uint8_t channel) {
    char requested_bytes = 4;
    int32_t ack = -1;

    if (((_config[channel] >> 2) & 0b11) != 0b11) {  // not 18bit
        requested_bytes = 3;
    }

    memset(_Buffer, 0, sizeof(_Buffer));

    _i2c->lock();
    ack = _i2c->read(_address, _Buffer, requested_bytes);
    _i2c->unlock();

    if (ack == 0 && (_Buffer[requested_bytes - 1] >> 7) == 0) {
        if (((_Buffer[requested_bytes - 1] >> 5) & 0b11) == channel) {
            _i2c->read(0);  // send NACK
            return false;  // data ready
        }
    }

    return true;  // not ready
}

int32_t MCP342X::read(uint8_t channel) {
    uint8_t resolution = ((_config[channel] >> 2) & 0b11);
    auto delay = 4ms;

    if (((_config[channel] >> 4) & 1) == OneShot) {
        if (!newConversion(channel)) {
            return LONG_MIN;
        }
    }

    delay = (resolution == _12bit ? 4ms : (resolution == _14bit ? 16ms : (resolution == _16bit ? 66ms : 266ms)));
    ThisThread::sleep_for(delay);

    Timer timer;
    timer.start();

    while (isConversionFinished(channel) == 1) {
        if (timer.elapsed_time() >= MCP342X_DEFAULT_TIMEOUT) {
            timer.stop();
            return LONG_MIN;
        }

        wait_us(250);
    }

    timer.stop();

    return getResult(channel);
}

int32_t MCP342X::getResult(uint8_t channel) {
    int32_t result = LONG_MIN;
    int16_t tmp = 0;
    uint8_t resolution = ((_config[channel] >> 2) & 0b11);

    switch (resolution) {
        case _12bit: {
            tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x0FFF;  // substract 12bit

            result = (int32_t)tmp;

            break;
        }

        case _14bit: {
            tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x3FFF;  // substract 14bit

            result = (int32_t)tmp;

            break;
        }

        case _16bit: {
            tmp = ((_Buffer[0] << 8) | _Buffer[1]);

            result = (int32_t)tmp;

            break;
        }

        case _18bit: {
            result = ((_Buffer[0] << 16) | (_Buffer[1] << 8) | _Buffer[2]);
            result &= 0x3FFFF;  // substract 18bit

            break;
        }
    }

    return result;
}

int32_t MCP342X::readVoltage(uint8_t channel) {
    int32_t result = read(channel);
    uint8_t resolution = ((_config[channel] >> 2) & 0b11);
    uint8_t pga = (_config[channel] & 0b11);

    switch (pga) {
        case x1:
            pga = 1;
            break;

        case x2:
            pga = 2;
            break;

        case x4:
            pga = 4;
            break;

        case x8:
            pga = 8;
            break;
    }

    switch (resolution) {
        case _12bit:
            result /= pga;
            result *= 1000;

            break;

        case _14bit:
            result /= pga;
            result *= 250;

            break;

        case _16bit:
            result /= pga;
            result *= 62.5;

            break;

        case _18bit:
            result /= pga;
            result *= 15.625;

            break;
    }

    return result;
}
