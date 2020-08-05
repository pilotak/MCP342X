# MCP342x ADC library for mbed
[![Framework Badge mbed](https://img.shields.io/badge/framework-mbed-008fbe.svg)](https://os.mbed.com/)

Originally an Arduino library ported to mbedOS 6 to support Microchip MCP342x ADC over I2C

## Supported devices

*   MCP3422: 2 channel, 12, 14, 16, or 18 bit
*   MCP3423: 2 channel, 12, 14, 16, or 18 bit
*   MCP3424: 4 channel, 12, 14, 16, or 18 bit
*   MCP3426: 2 channel, 12, 14, or 16 bit
*   MCP3427: 2 channel, 12, 14, or 16 bit
*   MCP3428: 4 channel, 12, 14, or 16 bit

## Example
```cpp
#include "mbed.h"
#include "mbed_events.h"
#include "MCP342X.h"

I2C i2c(PB_9, PB_8);
MCP342X adc(&i2c);

MCP342X adc(&i2c, &queue);

void done(uint8_t channel, int32_t value) {
    int32_t voltage = adc.toVoltage(channel, value);
    printf("Result - channel: %u, value: %li, voltage: %li uV\n", channel, value, voltage);
}

void error(MCP342X::ErrorType e) {
    printf("ADC error: %u\n", e);
}

int main() {
    adc.init();
    adc.config(0, MCP342X::_12bit, MCP342X::OneShot, MCP342X::x2); // channel, precision, mode, PGA
    adc.config(1, MCP342X::_18bit, MCP342X::OneShot, MCP342X::x1);

    t.start(callback(&queue, &EventQueue::dispatch_forever));  // dispatch queue

    while (1) {
        adc.read(0);  // read channel 0
        wait_ms(250);

        printf("ADC1 value: %ld\t", adc1);
        printf("ADC2 voltage: %ld uV\n", adc2_v); // convert uV->V
        ThisThread::sleep_for(500ms);
    }
}
```
