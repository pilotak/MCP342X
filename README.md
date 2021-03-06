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
#include "MCP342X.h"

I2C i2c(PB_9, PB_8);
MCP342X adc(&i2c);

int main() {
    if(adc.init()){
        adc.config(0, MCP342X::_12bit, MCP342X::OneShot, MCP342X::x2); // channel, precision, mode, PGA
        adc.config(1, MCP342X::_18bit, MCP342X::OneShot, MCP342X::x1);

        while (1) {
            printf("ADC1 value: %ld\t", adc.read(0)); // read channel 0
            printf("ADC2 voltage: %ld uV\n", adc.readVoltage(1)); // read voltage at channel 1
            ThisThread::sleep_for(500ms);
        }
    }

    printf("failed to init\n");
}
```
