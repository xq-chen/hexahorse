#include "Bmx055.h"

extern bool debugMode;

Bmx055::Bmx055(I2C *i2c, int addrAccl, int addrGyro, int addrMag, bool debugMode) :
    mAddrAccl(addrAccl << 1),
    mAddrGyro(addrGyro << 1),
    mAddrMag(addrMag << 1),
    mI2C(i2c),
    mDebugMode(debugMode)
{
    mInitialized = false;
    filter = new Madgwick();
    filter->begin(10);
}

Bmx055::~Bmx055()
{
}

#define I2C_WRITE_2BYTE(addr, b1, b2, errMsg) \
    buf[0] = b1; \
    buf[1] = b2; \
    if ((status = mI2C->write(addr, buf, 2, 0)) != 0) { \
        if (mDebugMode) { \
            printf("%s status=%d\r\n", errMsg, status); \
        } \
        return false; \
    }
bool Bmx055::Initialize(void)
{
    char buf[2];
    int status;

    // Range = +/- 2g
    I2C_WRITE_2BYTE(mAddrAccl, 0x0F, 0x03, "Accl Range")
    // Bandwidth = 7.81 Hz
    I2C_WRITE_2BYTE(mAddrAccl, 0x10, 0x08, "Accl Bandwidth")
    // Normal mode, Sleep duration = 0.5ms
    I2C_WRITE_2BYTE(mAddrAccl, 0x11, 0x00, "Accl Mode")

    // Full scale = +/- 125 degree/s
    I2C_WRITE_2BYTE(mAddrGyro, 0x0F, 0x04, "Gyro Range")
    // ODR = 100 Hz
    I2C_WRITE_2BYTE(mAddrGyro, 0x10, 0x07, "Gyro ODR")
    // Normal mode, Sleep duration = 2ms
    I2C_WRITE_2BYTE(mAddrGyro, 0x11, 0x00, "Gyro Mode")

    // Soft reset 1/2
    I2C_WRITE_2BYTE(mAddrMag, 0x4B, 0x83, "Mag Soft reset 1/2")
    // Soft reset 2/2
    I2C_WRITE_2BYTE(mAddrMag, 0x4B, 0x01, "Mag Soft reset 2/2")
    // Normal Mode, ODR = 10 Hz
    I2C_WRITE_2BYTE(mAddrMag, 0x4C, 0x00, "Mag Mode")
    // X, Y, Z-Axis enabled
    I2C_WRITE_2BYTE(mAddrMag, 0x4E, 0x84, "Mag Axis")
    // No. of Repetitions for X-Y Axis = 9
    I2C_WRITE_2BYTE(mAddrMag, 0x51, 0x04, "Mag X-Y Repetitions")
    // No. of Repetitions for Z-Axis = 15
    I2C_WRITE_2BYTE(mAddrMag, 0x52, 0x16, "Mag Z Repetitions")

    mInitialized = true;

    return true;
}

bool Bmx055::read(float& roll, float& pitch, float& yaw)
{
    float xAccl, yAccl, zAccl;
    float xGyro, yGyro, zGyro;
    float xMag, yMag, zMag;

    readAccl(xAccl, yAccl, zAccl);
    // pc.printf("Accl= %lf, %lf, %lf\r\n", xAccl, yAccl, zAccl);
    readGyro(xGyro, yGyro, zGyro);
    // pc.printf("Gyro= %lf, %lf, %lf\r\n", xGyro, yGyro, zGyro);
    readMag(xMag, yMag, zMag);
    // pc.printf("Mag= %lf, %lf, %lf\r\n", xMag, yMag, zMag);

    filter->update(xGyro, yGyro, zGyro, xAccl, yAccl, zAccl, xMag, yMag, zMag);
    // filter->updateIMU(xGyro,yGyro,zGyro,xAccl,yAccl,zAccl);
    roll = filter->getRoll();
    pitch = filter->getPitch();
    yaw = filter->getYaw();

    return true;
}

void Bmx055::readAccl(float& x, float& y, float& z)
{
    char data_write[1];
    char data_read[1];
    int data[6];

    for (int i = 0; i < 6; i++) {
        data_write[0] = 2 + i;
        mI2C->write(mAddrAccl, data_write, 1, 1); // no stop
        mI2C->read(mAddrAccl, data_read, 1, 0);
        data[i] = (unsigned char)data_read[0];
    }

    // Convert the data to 12-bits
    x = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (x > 2047)
        x -= 4096;
    y = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (y > 2047)
        y -= 4096;
    z = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (z > 2047)
        z -= 4096;
    x *= 0.0098f; // renge +-2g
    y *= 0.0098f; // renge +-2g
    z *= 0.0098f; // renge +-2g
}

void Bmx055::readGyro(float& x, float& y, float& z)
{
    char data_write[1];
    char data_read[1];
    int data[6];

    for (int i = 0; i < 6; i++) {
        data_write[0] = 2 + i;
        mI2C->write(mAddrGyro, data_write, 1, 1); // no stop
        mI2C->read(mAddrGyro, data_read, 1, 0);
        data[i] = (unsigned char)data_read[0];
    }

    // Convert the data
    x = (data[1] * 256) + data[0];
    if (x > 32767)
        x -= 65536;
    y = (data[3] * 256) + data[2];
    if (y > 32767)
        y -= 65536;
    z = (data[5] * 256) + data[4];
    if (z > 32767)
        z -= 65536;

    x *= 0.0038f; //  Full scale = +/- 125 degree/s
    y *= 0.0038f; //  Full scale = +/- 125 degree/s
    z *= 0.0038f; //  Full scale = +/- 125 degree/s
}

void Bmx055::readMag(float& x, float& y, float& z)
{
    char data_write[1];
    char data_read[1];
    int data[8];

    for (int i = 0; i < 8; i++) {
        data_write[0] = 0x42 + i;
        mI2C->write(mAddrMag, data_write, 1, 1); // no stop
        mI2C->read(mAddrMag, data_read, 1, 0);
        data[i] = (unsigned char)data_read[0];
    }

    // Convert the data
    x = ((data[1] << 8) | data[0]) >> 3;
    if (x > 4095)
        x -= 8192;
    y = ((data[3] << 8) | data[2]) >> 3;
    if (y > 4095)
        y -= 8192;
    z = ((data[5] << 8) | data[4]) >> 3;
    if (z > 4095)
        z -= 8192;
}
