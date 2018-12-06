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

MCP342X::MCP342X(PinName sda, PinName scl, EventQueue * queue, uint8_t slave_adr, int32_t freq):
    _address(slave_adr),
    _current_channel(UCHAR_MAX),
    _stage(Init),
    _queue_id(-1) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(freq);
    _queue = queue;
}

MCP342X::MCP342X(I2C * i2c_obj, EventQueue * queue, uint8_t slave_adr):
    _address(slave_adr),
    _current_channel(UCHAR_MAX),
    _stage(Init),
    _queue_id(-1) {
    _i2c = i2c_obj;
    _queue = queue;
}

MCP342X::~MCP342X(void) {
    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

void MCP342X::init(const Callback<void(uint8_t, int32_t)> &done_callback, const Callback<void(ErrorType)> &err_callback) {
    _done_cb = done_callback;
    _error_cb = err_callback;

    _config[0] = 0b00010000;  // channel 1, continuous mode, 12bit
    _config[1] = 0b00110000;  // channel 2, continuous mode, 12bit
    _config[2] = 0b01010000;  // channel 3, continuous mode, 12bit
    _config[3] = 0b01110000;  // channel 4, continuous mode, 12bit

    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit
    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit
    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit
    _wait_time[0] = 5;  // one samples takes 4.166ms@12bit

    memset(_Buffer, 0xFF, sizeof(_Buffer));
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
        _stage = Ready;

        if (mode == Continuous) {
            return transfer(&_config[channel]);

        } else {
            return true;
        }
    }

    return false;
}

void MCP342X::process() {
    int32_t result = 0;
    int16_t tmp;
    uint8_t resolution = ((_config[_current_channel] >> 2) & 0b11);
    _queue_id = -1;

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

    uint8_t channel = _current_channel;
    reset();
    _done_cb.call(channel, result);
}

int8_t MCP342X::read(uint8_t channel) {
    if (_stage == Ready) {
        reset();

        _current_channel = channel;
        _requested_bytes = 4;
        _stage = Reading;

        if (((_config[_current_channel] >> 2) & 0b11) != 0b11) {  // is not 18bit
            _requested_bytes = 3;
        }

        if (((_config[channel] >> 4) & 1) == OneShot) {
            _config[channel] |= 128;  // new conversion
            return transfer(&_config[channel]);
        }

        // Continuous
        isConversionFinished();
        return 1;
    }

    if (_error_cb) {
        _error_cb.call(NotResponding);
    }

    return _stage;
}

bool MCP342X::transfer(const char *data, uint8_t rx_len, uint8_t tx_len) {
    memset(_Buffer, 0xFF, sizeof(_Buffer));  // null buffer

    if (_i2c->transfer(_address, data, tx_len, reinterpret_cast<char *>(_Buffer), rx_len, event_callback_t(this, &MCP342X::cbHandler),
                       I2C_EVENT_ALL) != 0) {
        reset();

        if (_error_cb) {
            _error_cb.call(TransferError);
        }

        return false;
    }

    return true;
}

void MCP342X::isConversionFinished() {
    _queue_id = -1;

    if (_current_channel < 4) {
        _stage = Request;
        _config[_current_channel] &= ~(128);  // clear new conversion

        transfer(&_config[_current_channel], _requested_bytes);
    }
}

void MCP342X::cbHandler(int event) {
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        reset();

        if (_error_cb) {
            _error_cb.call(NoSlave);
        }

    } else if (event & I2C_EVENT_ERROR) {
        reset();

        if (_error_cb) {
            _error_cb.call(EventError);
        }

    } else {
        if (_stage == Reading) {
            _stage = Waiting;

            _queue_id = _queue->call_in(_wait_time[_current_channel], callback(this, &MCP342X::isConversionFinished));

            if (_queue_id <= 0) {
                reset();  // prevent no space left in queue
            }

        } else if (_stage == Request) {
            if ((_Buffer[_requested_bytes - 1] >> 7) == 0) {
                _queue_id = _queue->call(callback(this, &MCP342X::process));

                if (_queue_id <= 0) {
                    reset();  // prevent no space left in queue
                }

            } else {
                reset();

                if (_error_cb) {
                    _error_cb.call(Timeout);
                }
            }
        }
    }
}

void MCP342X::reset() {
    _stage = Ready;
    _current_channel = UCHAR_MAX;

    if (_queue_id > -1) {
        _queue->cancel(_queue_id);
        _queue_id = -1;
    }

    _i2c->abort_transfer();
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
