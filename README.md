# MCP342x ADC library for mbed

Mbed library to support Microchip MCP342x ADC over I2C

## Supported devices

*   MCP3422: 2 channel, 12, 14, 16, or 18 bit
*   MCP3423: 2 channel, 12, 14, 16, or 18 bit
*   MCP3424: 4 channel, 12, 14, 16, or 18 bit
*   MCP3426: 2 channel, 12, 14, or 16 bit
*   MCP3427: 2 channel, 12, 14, or 16 bit
*   MCP3428: 4 channel, 12, 14, or 16 bit

## Example
```cpp
#include <mbed.h>
#include "MCP342X.h"

I2C i2c(PB_9, PB_8);
MCP342X adc;

int main() {
    adc.init(&i2c);
    adc.config(0, MCP342X::_12bit, MCP342X::OneShot, MCP342X::x2); //channel, precision, mode, PGA
    adc.config(1, MCP342X::_18bit, MCP342X::OneShot, MCP342X::x1);

    while (1) {
        int32_t adc1 = adc.read(0);
        int32_t adc2_v = adc.readVoltage(1);

        printf("ADC1 value: %ld\t", adc1);
        printf("ADC2 voltage: %lf V\n", (double)adc2_v/1000000); // convert uV->V
        wait_ms(500);
    }
}
```

[Original code](https://github.com/battosai30/MCP3424/)


