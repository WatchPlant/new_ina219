#include "mbed.h"
#include <cstdint>
#include <cstring>
#include "INA219.hpp"

//INA219 ina(ARDUINO_UNO_I2C_SDA, ARDUINO_UNO_I2C_SCL); //, 0x40, 400000, RES_10BITS);
//SDA is D14, SCL is D15
//addr is used for reading I2C

// ^ this INA library might be too old to use (mbed 2? (2014), current version is mbed 6)
int main()
{
    ina219_init();
    float current;
    printf("starting...\n");

    while (true) {
        current = ina219_busvoltage();
        //something in the read function doesnt work...
        printf("I = %f [mA]\n", current);
        //only prints out the max current (calibrated to 400 mA in init)
        wait_us(1000000);
    }
}
