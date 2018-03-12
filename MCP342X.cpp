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

#include "mbed.h"
#include "mbed_events.h"
#include "MCP342X.h"

MCP342X::MCP342X(uint8_t slave_adr):
    _address(slave_adr),
    _current_channel(UCHAR_MAX),
    _stage(None),
    _i2c(NULL),
    _queue(NULL) {
}

MCP342X::~MCP342X(void) {
}

void MCP342X::init(I2C * i2c_obj, EventQueue * queue, Callback<void(ErrorType)> callback) {
    _i2c = i2c_obj;
    error_cb = callback;

    _config[0] = 0b00010000;  // channel 1, continuous mode, 12bit
    _config[1] = 0b00110000;  // channel 2, continuous mode, 12bit
    _config[2] = 0b01010000;  // channel 3, continuous mode, 12bit
    _config[3] = 0b01110000;  // channel 4, continuous mode, 12bit

    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit
    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit
    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit
    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit

    memset(_Buffer, 0xFF, sizeof(_Buffer));

    _queue = queue;
}

bool MCP342X::config(uint8_t channel, Resolution res, Conversion mode, PGA gain) {

    _config[channel] |= ((res << 2) | gain);
    _config[channel] ^= (-mode ^ _config[channel]) & (1 << 4);

    switch (res) {
        case _12bit:
            _wait_time[channel] = 5;  // 4.166ms@12bit
            break;

        case _14bit:
            _wait_time[channel] = 17;  // 16.666ms@14bit
            break;

        case _16bit:
            _wait_time[channel] = 67;  // 66.666ms@16bit
            break;

        case _18bit:
            _wait_time[channel] = 267;  // 266.666ms@18bit
            break;
    }

    if (_i2c != NULL && _queue != NULL) {
        _stage = Init;

        if (mode == Continuous) {
            transfer(&_config[channel]);
        }

        return true;
    }

    return false;
}

void MCP342X::process() {
    int32_t result = 0;
    uint8_t resolution = ((_config[_current_channel] >> 2) & 0b11);

    switch (resolution) {
        case _12bit: {
            int16_t tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x0FFF;  // substract 12bit

            result = (int32_t)tmp;

            break;
        }

        case _14bit: {
            int16_t tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x3FFF;  // substract 14bit

            result = (int32_t)tmp;

            break;
        }

        case _16bit: {
            int16_t tmp = ((_Buffer[0] << 8) | _Buffer[1]);

            result = (int32_t)tmp;

            break;
        }

        case _18bit: {
            result = ((_Buffer[0] << 16) | (_Buffer[1] << 8) | _Buffer[2]);
            result &= 0x3FFFF;  // substract 18bit

            break;
        }
    }

    uint8_t channel = _current_channel;
    _current_channel = UCHAR_MAX;
    done_cb.call(channel, result);
}

bool MCP342X::read(uint8_t channel, Callback<void(uint8_t, int32_t)> callback) {
    done_cb = callback;


    if (_current_channel == UCHAR_MAX && _stage == Init) {
        _current_channel = channel;

        _requested_bytes = 4;

        if (((_config[_current_channel] >> 2) & 0b11) != 0b11) {  // is not 18bit
            _requested_bytes = 3;
        }

        if (((_config[channel] >> 4) & 1) == OneShot) {
            _stage = Reading;
            _config[channel] |= 128;  // new conversion
            transfer(&_config[channel]);

        } else if (((_config[channel] >> 4) & 1) == Continuous) {
            isConversionFinished();
        }

        return true;

    }

    return false;
}

bool MCP342X::transfer(const char *data, uint8_t rx_len, uint8_t tx_len) {
    memset(_Buffer, 0xFF, sizeof(_Buffer));  // null buffer

    if (_i2c->transfer(_address, data, tx_len, reinterpret_cast<char *>(_Buffer), rx_len, event_callback_t(this, &MCP342X::cbHandler),
                       I2C_EVENT_ALL) != 0) {
        _stage = Init;
        _current_channel = UCHAR_MAX;

        if (error_cb) {
            error_cb.call(TransferError);
        }

        return false;
    }

    return true;
}

void MCP342X::isConversionFinished() {
    _stage = Waiting;
    _config[_current_channel] &= ~(128);  // clear new conversion

    transfer(&_config[_current_channel], _requested_bytes);
}

void MCP342X::cbHandler(int event) {
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        _stage = Init;
        _current_channel = UCHAR_MAX;

        if (error_cb) {
            error_cb.call(NoSlave);
        }

    } else if (event & I2C_EVENT_ERROR) {
        _stage = Init;
        _current_channel = UCHAR_MAX;

        if (error_cb) {
            error_cb.call(EventError);
        }

    } else {
        if (_stage == Reading) {
            _queue->call_in(_wait_time[_current_channel], callback(this, &MCP342X::isConversionFinished));

        } else if (_stage == Waiting) {
            if ((_Buffer[_requested_bytes - 1] >> 7) == 0) {
                _stage = Init;
                process();

            } else {
                _stage = Init;
                _current_channel = UCHAR_MAX;

                if (error_cb) {
                    error_cb.call(Timeout);
                }
            }
        }
    }
}

int32_t MCP342X::toVoltage(uint8_t channel, int32_t value) {
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
            value /= pga;
            value *= 1000;

            break;

        case _14bit:
            value /= pga;
            value *= 250;

            break;

        case _16bit:
            value /= pga;
            value *= 62.5;

            break;

        case _18bit:
            value /= pga;
            value *= 15.625;

            break;
    }

    return value;
}
