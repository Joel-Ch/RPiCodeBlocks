#include "pi2c.h"

void SetArduino(int rightSpeed, int leftSpeed, int servoAngle)
{
    nano.i2cWriteArduinoInt(rightSpeed);
    nano.i2cWriteArduinoInt(leftSpeed);
    nano.i2cWriteArduinoInt(servoAngle);
    return;
}

/*int ReadEncoders(int *rightPos, int *leftPos)
{
    Wire.requestFrom(I2C_ARDUINO_ADDR, 2);
    bool messageFlag = false;
    while (Wire.available())
    {

        *rightPos = Wire.read();
        *leftPos = Wire.read();
        messageFlag = true;
    }
    Serial.print("reading encoders");
    if (messageFlag = true)
    {
        return 0;
    }

}*/
