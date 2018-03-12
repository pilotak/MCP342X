/*
Originally written by B@tto
Contact : batto@hotmail.fr
Edited for mbed by www.github.com/pilotak


  MCP342X.cpp - ADC 16/18 bits i2c library for mbed
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
#include <MCP342X.h>
#include "mbed.h"


MCP342X::MCP342X(uint8_t slave_adr):
    _address(slave_adr),
    _current_channel(UCHAR_MAX) {
    i2c = NULL;
}

MCP342X::~MCP342X(void) {
}

void MCP342X::init(I2C * i2c_obj, Callback<void(uint8_t)> callback) {
    i2c = i2c_obj;
    error_cb = callback;

    _config[0] = 0b00010000; // channel 1, continuous mode, 12bit
    _config[1] = 0b00110000; // channel 2, continuous mode, 12bit
    _config[2] = 0b01010000; // channel 3, continuous mode, 12bit
    _config[3] = 0b01110000; // channel 4, continuous mode, 12bit

    memset(_Buffer, 0xFF, sizeof(_Buffer));
}

bool MCP342X::config(uint8_t channel, Resolution res, Conversion mode, PGA gain) {

    _config[channel] |= ((res << 2) | gain);
    _config[channel] ^= (-mode ^ _config[channel]) & (1 << 4);

    if (i2c != NULL) {
        if (mode == Continuous) {
            i2c->lock();
            i2c->write(_address, &_config[channel], 1);
            i2c->unlock();
        }

        return true;
    }

    return false;
}

void MCP342X::isConversionFinished() {
    i2c->lock();
    i2c->read(_address, _Buffer, _requested_bytes);
    i2c->unlock();

    if ((_Buffer[_requested_bytes - 1] >> 7) == 0) {
        process();

    } else {
        Thread::wait(60);
        isConversionFinished();
    }
}

void MCP342X::process() {
    int32_t result = 0;
    uint8_t resolution = ((_config[_current_channel] >> 2) & 0b11);

    switch (resolution) {

        case _12bit: {
            int16_t tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x0FFF; // substract 12bit

            result = (int32_t)tmp;

            break;
        }

        case _14bit: {
            int16_t tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x3FFF; // substract 14bit

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
            result &= 0x3FFFF; // substract 18bit

            break;
        }
    }

    uint8_t channel = _current_channel;
    _current_channel = UCHAR_MAX;
    done_cb.call(channel, result);
}

void MCP342X::read(uint8_t channel, Callback<void(uint8_t, int32_t)> callback) {
    done_cb = callback;
    printf("read\n");

    if (_current_channel == UCHAR_MAX) {
        _current_channel = channel;

        _requested_bytes = 4;

        if (((_config[_current_channel] >> 2) & 0b11) != 0b11) {  // 18bit
            _requested_bytes = 3;
        }

        if (((_config[channel] >> 4) & 1) == OneShot) {
            char byte = _config[channel] |= 128;

            i2c->lock();
            i2c->write(_address, &byte, 1);
            i2c->unlock();
        }

        isConversionFinished();
    }
}

/*bool MCP342X::transfer(const char *data, uint8_t rx_len, uint8_t tx_len) {
    if (i2c->transfer(_address, data, tx_len, (char*)_Buffer, rx_len, event_callback_t(this, &MCP342X::internal_cb_handler),
                      I2C_EVENT_ALL) != 0) {
        if (error_cb) {
            _current_channel = UCHAR_MAX;
            error_cb.call(1);
        }

        return false;
    }

    return true;
}

void MCP342X::internal_cb_handler(int event) {
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        if (error_cb) {
            _current_channel = UCHAR_MAX;
            error_cb.call(3);
        }

    } else if (event & I2C_EVENT_ERROR) {
        if (error_cb) {
            _current_channel = UCHAR_MAX;
            error_cb.call(2);
        }

    } else {

    }
}
*/
/*int32_t MCP342X::readVoltage(uint8_t channel) {

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
}*/


