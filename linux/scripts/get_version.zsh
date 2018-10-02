
I2C_ADD_PISTON=0x38
I2C_ADD_POWER=0x39
I2C_ADD_ENGINE=0x20
I2C_ADD_TEMP=0x45

V_PISTON=`i2cget -y 1 $I2C_ADD_PISTON 0xC0`
V_POWER=`i2cget -y 1 $I2C_ADD_POWER 0xC0`
V_ENGINE=`i2cget -y 1 $I2C_ADD_ENGINE 0xC0`
V_TEMP=`i2cget -y 1 $I2C_ADD_TEMP 0xC0`

echo "version piston = " $V_PISTON
echo "version power = " $V_POWER
echo "version engine = " $V_ENGINE
echo "version temp = " $V_TEMP

