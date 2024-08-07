#include "INA219.hpp"

/*
* CALIBRATION REGISTER Calculation
* max load (bus) voltage = 32 V
* max shunt voltage = 0.32 V  (voltage across shunt resistor)
* shunt resistor = 0.01 Ohm
*
* max shunt current = vshunt_max / rshunt (0.32/0.01) = 32 A
* max expected current = 32 A
*
* current_lsb = max expected current / 32768 (32/32768) = 0.00097 A per bit
*       = 0.001 A per bit (current resolution)
*
* power_lsb = 20 * current_lsb
*       = 0.02 W per bit
*
* calibration register = 0.04096 / (current_lsb * rshunt)
*       = 0.04096 / (0.001 * 0.1)
*        
* CALIBRATION REGISTER VALUE = 409.6
*/

float CURRENT_LSB, POWER_LSB;
I2C i2c(PB_9, PB_8);    //ARDUINO_UNO_I2C_SDA, ARDUINO_UNO_I2C_SCL);

void ina219_init(){
    CURRENT_LSB = 0.001;
    POWER_LSB = 0.02;
    
    //when writing the 16 bit register, send it as 2 bytes
    //1st byte is bits 15-8
    //2nd byte is 0-7
    char calibration_data[3] = {INA219_REG_CALIBRATION, 0x01, 0x99}; //409 in 2 bytes = 0x0199

    uint16_t config_value = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV |
        INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    char config_data[3] = {INA219_REG_CONFIG, (char)config_value, (char)(config_value >> 8)};

//~~~~~ WRITING DOES NOT WORK!!!! pin problem or code problem?
    //i2c setup
    int st;

    i2c.start();
    st = i2c.write(INA219_WRITE);        //first send the slave address with WRITE bit (to indicate writing)
    printf("\nCALIB status: %d", st);
    st = i2c.write(calibration_data[0]); //send address to be written to (CALIB)
    printf("\nstatus: %d", st);
    st = i2c.write(calibration_data[1]); //send 1st byte of data
    printf("\nstatus: %d", st);
    st = i2c.write(calibration_data[2]); //send 2nd byte of data
    printf("\nstatus: %d", st);
    i2c.stop();
    // i2c.write(INA219_WRITE, calibration_data, 3);
    //send config value to config register
    // i2c.write(INA219_WRITE, config_data, 3);
    i2c.start();
    st =i2c.write(INA219_WRITE);        //first send the slav address with WRITE bit
    printf("\nCONFIG status: %d", st);
    st=i2c.write(config_data[0]);      //send address to be written to (CONFIG)
    printf("\nstatus: %d", st);
    st=i2c.write(config_data[1]);
    printf("\nstatus: %d", st);
    st=i2c.write(config_data[2]);
    printf("\nstatus: %d", st);
    i2c.stop();
}

float ina219_busvoltage(){
    uint16_t bvoltage;

    //array to read data into
    char data[2] = {0,0};
    
    //to read from the ina219, first must send which address will be read from
    i2c.start();
    i2c.write(INA219_WRITE);
    i2c.write(INA219_REG_BUSVOLTAGE);
    i2c.stop();
    //i2c.write(INA219_WRITE, (char*)INA219_REG_BUSVOLTAGE, 1);

    //start the read by sending START condition
    i2c.start();

    i2c.write(INA219_READ);
    data[0] = i2c.read(1);
    data[1] = i2c.read(1);
    //send the register to read
    //i2c.read(INA219_READ, data, 2);
    i2c.stop();

    printf("\ndata 0: %d", data[0]);
    printf("\ndata 1: %d\n", data[1]);

    bvoltage = data[0];
    bvoltage = bvoltage << 8;
    bvoltage = bvoltage | data[1];
    //dont need the last 3 bits
    bvoltage = bvoltage >> 3;

    float ret = bvoltage * 0.004; //multiply by LSB value
    
    return ret;
}

/*
INA219::INA219 (PinName sda, PinName scl, int addr, int freq, resolution_t res) : I2C(sda, scl), resolution(res), i2c_addr(addr << 1)
{
    I2C::frequency(freq);
    
    // by default, calibrate to this level.
    calibrate_16v_400mA();
}

// Private Methods

void INA219::write_register (uint8_t reg, uint8_t* data, int length)
{
    char* transmission = (char*)malloc(length + 1);
    memcpy(transmission + 1, data, length);

    transmission[0] = reg;
    I2C::write(i2c_addr, transmission, length + 1);

    free(transmission);
}

void INA219::write_register_u16 (uint8_t reg, uint16_t data)
{
    char transmission[3];
    transmission[0] = reg;
    transmission[1] = (data >> 8) & 0xff;
    transmission[2] = data & 0xff;

    I2C::write(i2c_addr, transmission, 3);
}

void INA219::write_null(uint8_t reg) {
    I2C::write(i2c_addr, (char*)&reg, 1);    
}

uint16_t INA219::read_register_u16 (uint8_t reg)
{
    write_null(reg);

    char data[2];
    I2C::read(i2c_addr, data, 2);

    uint16_t ret_val = data[0] << 8 | data[1];
    return ret_val;
}

// Public Methods

void INA219::calibrate_16v_400mA()
{
    // ASSUMING A 0.1 OHM RESISTOR!
    write_register_u16(INA219_REG_CALIBRATION, 8192);

    // Write to config register

    uint16_t resolution_mask = 0x0000;
    
    if (resolution == RES_12BITS)
        resolution_mask = INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US;
    else if (resolution == RES_11BITS)
        resolution_mask = INA219_CONFIG_BADCRES_11BIT | INA219_CONFIG_SADCRES_11BIT_1S_276US;
    else if (resolution == RES_10BITS)
        resolution_mask = INA219_CONFIG_BADCRES_10BIT | INA219_CONFIG_SADCRES_10BIT_1S_148US;
    else // resolution == RES_9BITS
        resolution_mask = INA219_CONFIG_BADCRES_9BIT | INA219_CONFIG_SADCRES_9BIT_1S_84US;

    write_register_u16(INA219_REG_CONFIG, INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV |
                    resolution_mask |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
                    
    // Set current divider
    current_divider = 20;
    power_divider = 1;
}

int16_t INA219::read_current_raw()
{
    return (int16_t)read_register_u16(INA219_REG_CURRENT);
}

float INA219::read_current_mA()
{
    float raw_current = read_current_raw();
    return raw_current / current_divider;
}
*/
