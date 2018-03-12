# MCP342x ADC library for mbed

I2C async library for Microchip MCP342x ADC devices 

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

I2C i2c(PB_7, PB_6);
MCP342X adc;

Thread t;
EventQueue queue(1 * EVENTS_EVENT_SIZE);  // only one event is required

void done(uint8_t channel, int32_t value) {
    int32_t voltage = adc.toVoltage(channel, value);
    printf("Result - channel: %u, value: %li, voltage: %li uV\n", channel, value, voltage);
}

void error(MCP342X::ErrorType e) {
    printf("ADC error: %u\n", e);
}

int main() {
    adc.init(&i2c, &queue, error);
    adc.config(0, MCP342X::_16bit, MCP342X::OneShot, MCP342X::x8); //channel, precision, mode, PGA
    adc.config(1, MCP342X::_12bit, MCP342X::OneShot, MCP342X::x1);

    t.start(callback(&queue, &EventQueue::dispatch_forever));  // dispatch queue

    while (1) {
        adc.read(0, done);  // read channel 0, pass callback
        wait_ms(250);

        adc.read(1, done);  // read channel 1, pass callback
        wait_ms(250);
    }
}
```
