/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef FXAS21000_H
#define FXAS21000_H

#include "mbed.h"


/**
 * MMA8652 Slave Address
 */
#define FXAS21000_SLAVE_ADDR 0x40

// MMA8652 internal register addresses
#define FXAS21000_STATUS 0x00
#define FXAS21000_WHOAMI 0x0C
#define FXAS21000_XYZ_DATA_CFG 0x0E
#define FXAS21000_CTRL_REG0 0x0D
#define FXAS21000_CTRL_REG1 0x13
#define FXAS21000_WHOAMI_VAL 0xD1

/**
 * FXAS21000 gyroscope on I2C
 */
class FXAS21000
{
public:
    /**
     * FXAS21000 constructor
     *
     * @param sda SDA pin
     * @param sdl SCL pin
     */
    FXAS21000(PinName sda, PinName scl);
 
    /**
     * Get the Gyro values as floating point degrees / second
     *
     * @param floating point array where the results will be placed
     */
    void ReadXYZ(float * a);

    /**
     * Get the Gyro values as signed 16 bit value
     *
     * @param int16_t point array where the results will be placed
     */
    void ReadXYZraw(int16_t * t);
   
    /**
     * Get the value of the WHO_AM_I register
     *
     * @returns DEVICE_ID value == 0xD1
     */
    char getWhoAmI(void);
    
private:
    I2C _i2c;

    /**
     * Set the device in active mode
     */
    void begin( void);
    
    void RegRead( char reg, char * d, int len);
};

#endif
