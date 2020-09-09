////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTIMUSETTINGS_H
#define _RTIMUSETTINGS_H

#include "RTMath.h"
#include "RTIMULibDefs.h"
#include "CalLib.h"

class RTIMUSettings
{
public:
    RTIMUSettings(){}

    bool init();
    virtual bool saveSettings();
    void setDefaults();
    bool loadSettings();


    //  These are the local variables
    bool m_busIsI2C;                                        // Using I2C for communication
    int m_fusionType;                                       // fusion algorithm type code
    unsigned char m_I2CSlaveAddress;                        // I2C slave address of the imu

    unsigned char m_I2CPressureAddress;                     // I2C slave address of the pressure sensor
    unsigned char m_I2CHumidityAddress;                     // I2C slave address of the humidity sensor

    bool m_compassCalValid;                                 // true if there is valid compass calibration data
    RTVector3 m_compassCalMin;                              // the minimum values
    RTVector3 m_compassCalMax;                              // the maximum values

    bool m_compassCalEllipsoidValid;                        // true if the ellipsoid calibration data is valid
    RTVector3 m_compassCalEllipsoidOffset;                  // the ellipsoid offset
    float m_compassCalEllipsoidCorr[3][3];                  // the correction matrix

    float m_compassAdjDeclination;                          // magnetic declination adjustment - subtracted from measured

    bool m_accelCalValid;                                   // true if there is valid accel calibration data
    RTVector3 m_accelCalMin;                                // the minimum values
    RTVector3 m_accelCalMax;                                // the maximum values

    bool m_gyroBiasValid;                                   // true if the recorded gyro bias is valid
    RTVector3 m_gyroBias;                                   // the recorded gyro bias



    //  IMU-specific vars

#if defined(MPU9150_68) || defined(MPU9150_69)
    //  MPU9150

    int m_MPU9150GyroAccelSampleRate;                       // the sample rate (samples per second) for gyro and accel
    int m_MPU9150CompassSampleRate;                         // same for the compass
    int m_MPU9150GyroAccelLpf;                              // low pass filter code for the gyro and accel
    int m_MPU9150GyroFsr;                                   // FSR code for the gyro
    int m_MPU9150AccelFsr;                                  // FSR code for the accel
#endif

#if defined(MPU9250_68) || defined(MPU9250_69)
    //  MPU9250

    int m_MPU9250GyroAccelSampleRate;                       // the sample rate (samples per second) for gyro and accel
    int m_MPU9250CompassSampleRate;                         // same for the compass
    int m_MPU9250GyroLpf;                                   // low pass filter code for the gyro
    int m_MPU9250AccelLpf;                                  // low pass filter code for the accel
    int m_MPU9250GyroFsr;                                   // FSR code for the gyro
    int m_MPU9250AccelFsr;                                  // FSR code for the accel

#endif

#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)
    //  LSM9DS0

    int m_LSM9DS0GyroSampleRate;                            // the gyro sample rate
    int m_LSM9DS0GyroBW;                                    // the gyro bandwidth code
    int m_LSM9DS0GyroHpf;                                   // the gyro high pass filter cutoff code
    int m_LSM9DS0GyroFsr;                                   // the gyro full scale range

    int m_LSM9DS0AccelSampleRate;                           // the accel sample rate
    int m_LSM9DS0AccelFsr;                                  // the accel full scale range
    int m_LSM9DS0AccelLpf;                                  // the accel low pass filter

    int m_LSM9DS0CompassSampleRate;                         // the compass sample rate
    int m_LSM9DS0CompassFsr;                                // the compass full scale range
#endif

#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)
    int m_GD20HM303DGyroSampleRate;                         // the gyro sample rate
    int m_GD20HM303DGyroBW;                                 // the gyro bandwidth code
    int m_GD20HM303DGyroHpf;                                // the gyro high pass filter cutoff code
    int m_GD20HM303DGyroFsr;                                // the gyro full scale range

    int m_GD20HM303DAccelSampleRate;                        // the accel sample rate
    int m_GD20HM303DAccelFsr;                               // the accel full scale range
    int m_GD20HM303DAccelLpf;                               // the accel low pass filter

    int m_GD20HM303DCompassSampleRate;                      // the compass sample rate
    int m_GD20HM303DCompassFsr;                             // the compass full scale range
#endif

#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)
    int m_GD20M303DLHCGyroSampleRate;                       // the gyro sample rate
    int m_GD20M303DLHCGyroBW;                               // the gyro bandwidth code
    int m_GD20M303DLHCGyroHpf;                              // the gyro high pass filter cutoff code
    int m_GD20M303DLHCGyroFsr;                              // the gyro full scale range

    int m_GD20M303DLHCAccelSampleRate;                      // the accel sample rate
    int m_GD20M303DLHCAccelFsr;                             // the accel full scale range

    int m_GD20M303DLHCCompassSampleRate;                    // the compass sample rate
    int m_GD20M303DLHCCompassFsr;                           // the compass full scale range
#endif

#if defined(GD20HM303DLHC_6a) || defined(GD20HM303DLHC_6b)
    int m_GD20HM303DLHCGyroSampleRate;                      // the gyro sample rate
    int m_GD20HM303DLHCGyroBW;                              // the gyro bandwidth code
    int m_GD20HM303DLHCGyroHpf;                             // the gyro high pass filter cutoff code
    int m_GD20HM303DLHCGyroFsr;                             // the gyro full scale range

    int m_GD20HM303DLHCAccelSampleRate;                     // the accel sample rate
    int m_GD20HM303DLHCAccelFsr;                            // the accel full scale range

    int m_GD20HM303DLHCCompassSampleRate;                   // the compass sample rate
    int m_GD20HM303DLHCCompassFsr;                          // the compass full scale range
#endif


private:
CalLibEEPROM eeprom;
CALLIB_DATA calData_;                                  // the calibration data

};

#endif // _RTIMUSETTINGS_H

