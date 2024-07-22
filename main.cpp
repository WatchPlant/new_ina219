#include "mbed.h"
#include <cstdint>
#include <cstring>
#include "INA219.hpp"

INA219 ina(ARDUINO_UNO_I2C_SDA, ARDUINO_UNO_I2C_SCL); //, 0x40, 400000, RES_10BITS);
//SDA is D14, SCL is D15

int main()
{
    float current;
    printf("starting...\n");

    while (true) {
        current = ina.read_current_mA();
        printf("I = %f [mA]\n", current);

        wait_us(1000000);
    }
}
