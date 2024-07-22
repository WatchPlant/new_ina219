#include "mbed.h"
#include "INA219_consts.h"

#define DEBUG

/// Represents the different supported voltage levels
typedef enum {
    BUS_16V=16,
    BUS_32V=32
} busvoltage_t;

/** @enum Resolution typedef
 *  @brief Represents the different ADC resolutions supported by the INA219.
 */
typedef enum {
    RES_12BITS=12,  ///< 12 bit precision (532us)
    RES_11BITS=11,  ///< 11 bit precision (276us)
    RES_10BITS=10,  ///< 10 bit precision (148us)
    RES_9BITS =9    ///< 9 bit precision (84us)
} resolution_t;

/** @class INA219
 *  @brief A class to represent the INA219 breakout board from Adafruit.
 */
class INA219: protected I2C
{
public:
    /** Creates an instance of the INA219 device.
     *
     *  @param sda An I2C data pin.
     *  @param scl An I2C clock pin.
     *  @param addr The I2C address of the INA219. (Default = 0x40)
     *  @param freq The I2C clock frequency.
     *  @param res The desired resolution of the INA219's ADC.
     */
    INA219 (PinName sda, PinName scl, int addr=0x40, int freq=100000, resolution_t res=RES_12BITS);

    /** Reads the current raw value of current from the INA219.
     *
     *  @see read_current_mA
     *
     *  @returns
     *      A value between -32768 and +32768. Depending on the current calibration and configuration register values, the corresponding current can be calculated.
     */
    int16_t read_current_raw();
    /** Reads the current from the INA219 and calculates the actual value in mA.
     *
     *  @see read_current_raw
     *
     *  @returns
     *      A floating point value corresponding to the current flowing through the current shunt, in mA.
     */
    float read_current_mA();

    /** Reads the raw power value from the INA219.
     *
     *  @see read_power_mW
     *
     *  @returns
     *      A value between -32768 and +32768. Depending on the calibration and configuration register values, the actual power can be calculated.
     */
    //int16_t read_power_raw();
    /** Reads the power from the INA219 and calculates the actual value in mW.
     *
     *  @see read_power_raw
     *
     *  @returns
     *      A floating point value corresponding to the power being used in the circuit, in mW.
     */
    //float read_power_mW();

    /** Reads the raw shunt voltage value from the INA219.
     *
     *  @see read_shunt_voltage_mV
     *
     *  @returns
     *      A value between -32768 and +32768. Depending on the calibration and configuration register values, the actual shunt voltage can be calculated.
     */
    //int16_t read_shunt_voltage_raw();
    /** Reads the shunt voltage from the INA219 and calculates the actual value in mV.
     *
     *  @see read_shunt_voltage_raw
     *
     *  @returns
     *      A floating point value corresponding to the potential difference across the current shunt, in mV.
     */
    //float read_shunt_voltage_mV();

    /** Reads the raw bus voltage.
     *
     *  @see read_bus_voltage
     *
     *  @returns
     *      A value between -32768 and +32768 corresponding to the bus voltage.
     */
    //int16_t read_bus_voltage_raw();
    /** Reads the bus voltage and uses it to calculate the actual bus voltage.
     *
     *  @see read_bus_voltage_raw
     *
     *  @returns
     *      A floating point value corresponding to the voltage of V+ (in V).
     */
    //float read_bus_voltage();

    /** Sets the calibration register.
     *     
     *  Specifies a maximum bus voltage of 16V and maximum current of 400mA.
     *  
     */
    void calibrate_16v_400mA();

protected:
    resolution_t resolution;

    int i2c_addr;
    int current_divider;
    int power_divider;
    //...

    /** Writes a uint8_t array to the specified I2C register.
     *
     *  @see write_register_u16
     *
     *  @param reg (8-Bit) Register address to be written to.
     *  @param data An array of bytes with the data to be written.
     *  @param length The length of the array.
     */
    void write_register(uint8_t reg, uint8_t* data, int length);
    /** Writes a uint16_t to the specified I2C register.
     *
     *  @see write_register
     *
     *  @param reg (8-Bit) Register address to be written to.
     *  @param data (16-Bit) Data to be written.
     */
    void write_register_u16(uint8_t reg, uint16_t data);
    /** Reads a value from the specified I2C register.
     *
     *  @see write_register
     *  @see write_register_u16
     *
     *  @param reg (8-Bit) Register to be read from.
     *
     *  @returns
     *      The contents of the specified register, as a 16 bit integer.
     */
    uint16_t read_register_u16(uint8_t reg);
    
    /** Writes nothing to a specified register. (Used to tell the chip that we want to read from that register)
     *
     *  @see read_register_u16
     *
     *  @param reg The register for nothing to be written to.
     */
     void write_null(uint8_t reg);
};
