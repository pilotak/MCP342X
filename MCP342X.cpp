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

MCP342X::MCP342X(I2C * i2c_obj, uint8_t slave_adr):
    _address(slave_adr) {
    _i2c = i2c_obj;
}

MCP342X::MCP342X(PinName sda, PinName scl, uint8_t slave_adr, int32_t freq):
    _address(slave_adr) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(freq);
}

MCP342X::~MCP342X(void) {
    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool MCP342X::init() {
    int32_t ack;
    _config[0] = 0b00010000;  // channel 1, continuous mode, 12bit
    _config[1] = 0b00110000;  // channel 2, continuous mode, 12bit
    _config[2] = 0b01010000;  // channel 3, continuous mode, 12bit
    _config[3] = 0b01110000;  // channel 4, continuous mode, 12bit

    memset(_Buffer, 0, sizeof(_Buffer));

    // test if device is on the bus
    _i2c->lock();
    ack = _i2c->write(_address, &_config[0], 1);
    _i2c->unlock();

    return (ack == 0 ? true : false);
}

bool MCP342X::config(uint8_t channel, Resolution res, Conversion mode, PGA gain) {
    int32_t ack;
    _config[channel] |= ((res << 2) | gain);
    _config[channel] ^= (-mode ^ _config[channel]) & (1 << 4);

    if (_i2c != NULL) {
        _i2c->lock();
        ack = _i2c->write(_address, &_config[channel], 1);
        _i2c->unlock();

        return (ack == 0 ? true : false);
    }

    return false;
}

bool MCP342X::newConversion(uint8_t channel) {
    char byte = _config[channel] |= 128;
    int32_t ack;

    memset(_Buffer, 0, sizeof(_Buffer));

    _i2c->lock();
    ack = _i2c->write(_address, &byte, 1);
    _i2c->unlock();

    return (ack == 0 ? true : false);
}

bool MCP342X::isConversionFinished(uint8_t channel) {
    char requested_bytes = 4;
    int32_t ack;

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
    uint8_t delay = 4;

    if (((_config[channel] >> 4) & 1) == OneShot) {
        if (!newConversion(channel)) {
            return LONG_MIN;
        }
    }

    delay = (resolution == _12bit ? 4 : (resolution == _14bit ? 16 : (resolution == _16bit ? 66 : 266)));
    ThisThread::sleep_for(delay);

    LowPowerTimer timer;
    timer.start();

    while (isConversionFinished(channel) == 1) {
        if (timer.read_ms() >= MCP342X_DEFAULT_TIMEOUT) {
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
