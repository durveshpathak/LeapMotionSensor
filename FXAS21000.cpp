 #include "FXAS21000.h"
 
FXAS21000::FXAS21000(PinName sda, PinName scl) : _i2c(sda, scl) {
 
    begin();
}
              
void FXAS21000::RegRead( char reg, char * d, int len)
{
    char cmd[1];
    cmd[0] = reg;
char i2c_addr = FXAS21000_SLAVE_ADDR;
_i2c.write( i2c_addr, cmd, 1, true);
_i2c.read ( i2c_addr, d, len);
}

void FXAS21000::begin(void)
{
    char data[2];
    // write 0000 1000 = 0x08 to gyro control register 1 to place FXAS21000 into
    // standby
    // [7-1] = 0000 000
    // [0]: active=0
    data[0] = FXAS21000_CTRL_REG1;
    data[1] = 0x08;
    _i2c.write( FXAS21000_SLAVE_ADDR, data, 2);
    
    // write 0001 1011 to CRTL_REG0 register
    data[0] = FXAS21000_CTRL_REG0;
    data[1] = 0x1B;
    _i2c.write( FXAS21000_SLAVE_ADDR, data, 2);

    // write 0000 1001 to gyro control register 1
    data[0] = FXAS21000_CTRL_REG1;
    data[1] = 0x0A;
    _i2c.write( FXAS21000_SLAVE_ADDR, data, 2);
}

char FXAS21000::getWhoAmI(void)
{
    char d;
    RegRead( FXAS21000_WHOAMI, &d, 1);
    return(d);
}

void FXAS21000::ReadXYZ(float * a)
{
    char d[7];
    int16_t t[6];

    RegRead( FXAS21000_STATUS, d, 7);
    t[0] = ((d[1] * 256) + ((unsigned short) d[2]));
    t[1] = ((d[3] * 256) + ((unsigned short) d[4]));
    t[2] = ((d[5] * 256) + ((unsigned short) d[6]));
    //printf("%X\r\n", (int) d[0]);

    a[0] = (float) t[0] * 0.003125;
    a[1] = (float) t[1] * 0.003125;
    a[2] = (float) t[2] * 0.003125;
  
}

void FXAS21000::ReadXYZraw(int16_t * t)
{
    char d[7];

    RegRead( FXAS21000_STATUS, d, 7);
    t[0] = ((d[1] * 256) + ((unsigned short) d[2]));
    t[1] = ((d[3] * 256) + ((unsigned short) d[4]));
    t[2] = ((d[5] * 256) + ((unsigned short) d[6]));
}