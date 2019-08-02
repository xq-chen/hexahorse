#ifndef BMX055_H
#define BMX055_H

#include "mbed.h"
#include "MadgwickAHRS.h"

typedef int (*PrintLog)(const char *, ...);
class Bmx055
{
private:
    int mAddrAccl;
    int mAddrGyro;
    int mAddrMag;
    I2C *mI2C;
    bool mDebugMode;

    bool mInitialized;
    Madgwick* filter;

    void readAccl(float& x, float& y, float& z);
    void readGyro(float& x, float& y, float& z);
    void readMag(float& x, float& y, float& z);

public:
    Bmx055(I2C *i2c, int addrAccl, int addrGyro, int addrMag, bool debugMode = false);
    ~Bmx055();

    bool Initialize(void);
    bool read(float& roll, float& pitch, float& yaw);
};

#endif

