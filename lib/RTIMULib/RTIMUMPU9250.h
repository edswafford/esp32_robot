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

#ifndef _RTIMUMPU9250_H
#define	_RTIMUMPU9250_H
#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

#include "RTIMU.h"

//  MPU9250 I2C Slave Addresses

#define MPU9250_ADDRESS0            0x68
#define MPU9250_ADDRESS1            0x69
#define MPU9250_ID                  0x71

#define AK8963_ADDRESS              0x0c

//  Register map

#define MPU9250_SMPRT_DIV           0x19
#define MPU9250_GYRO_LPF            0x1a
#define MPU9250_GYRO_CONFIG         0x1b
#define MPU9250_ACCEL_CONFIG        0x1c
#define MPU9250_ACCEL_LPF           0x1d
#define MPU9250_FIFO_EN             0x23
#define MPU9250_I2C_MST_CTRL        0x24
#define MPU9250_I2C_SLV0_ADDR       0x25
#define MPU9250_I2C_SLV0_REG        0x26
#define MPU9250_I2C_SLV0_CTRL       0x27
#define MPU9250_I2C_SLV1_ADDR       0x28
#define MPU9250_I2C_SLV1_REG        0x29
#define MPU9250_I2C_SLV1_CTRL       0x2a
#define MPU9250_I2C_SLV2_ADDR       0x2b
#define MPU9250_I2C_SLV2_REG        0x2c
#define MPU9250_I2C_SLV2_CTRL       0x2d
#define MPU9250_I2C_SLV4_CTRL       0x34
#define MPU9250_INT_PIN_CFG         0x37
#define MPU9250_INT_ENABLE          0x38
#define MPU9250_INT_STATUS          0x3a
#define MPU9250_ACCEL_XOUT_H        0x3b
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_EXT_SENS_DATA_00    0x49

const uint8_t MPU9250_I2C_SLV0_DO = 0x63;

#define MPU9250_I2C_SLV1_DO         0x64
#define MPU9250_I2C_MST_DELAY_CTRL  0x67
#define MPU9250_USER_CTRL           0x6a
#define MPU9250_PWR_MGMT_1          0x6b
#define MPU9250_PWR_MGMT_2          0x6c
#define MPU9250_FIFO_COUNT_H        0x72
#define MPU9250_FIFO_R_W            0x74
#define MPU9250_WHO_AM_I            0x75


// Register Fields

// Reset value: 0x00 enable sensors
const uint8_t MPU9250_SEN_ENABLE = 0x00;

// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
const uint8_t MPU9250_CLOCK_SEL_PLL = 0x01;

// I2C Master clock speed 400KHz
const uint8_t MPU9250_I2C_MST_CLK = 0x0D;

//Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.
const uint8_t MPU9250_I2C_MST_EN = 0x20;

// Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA register, 
//which is always EXT_SENS_DATA_00 for I2C slave 0.
const uint8_t MPU9250_I2C_SLV0_EN = 0x80;

// Transfer is a read
const uint8_t MPU9250_I2C_READ_FLAG = 0x80;

const uint8_t MPU9250_PWR_RESET = 0x80;


    // AK8963 registers
    const uint8_t MPU9250_AK8963_I2C_ADDR = 0x0C;
    const uint8_t MPU9250_AK8963_HXL = 0x03; 
    const uint8_t MPU9250_AK8963_CNTL1 = 0x0A;
    const uint8_t MPU9250_AK8963_PWR_DOWN = 0x00;
    const uint8_t MPU9250_AK8963_CNT_MEAS1 = 0x12;
    const uint8_t MPU9250_AK8963_CNT_MEAS2 = 0x16;
    const uint8_t MPU9250_AK8963_FUSE_ROM = 0x0F;
    const uint8_t MPU9250_AK8963_CNTL2 = 0x0B;
    const uint8_t MPU9250_AK8963_RESET = 0x01;
    const uint8_t MPU9250_AK8963_ASA = 0x10;
    const uint8_t MPU9250_AK8963_WHO_AM_I = 0x00;



//  sample rate defines (applies to gyros and accels, not mags)

#define MPU9250_SAMPLERATE_MIN      5                       // 5 samples per second is the lowest
#define MPU9250_SAMPLERATE_MAX      1000                    // 1000 samples per second is the absolute maximum

//  compass rate defines

#define MPU9250_COMPASSRATE_MIN     1                       // 1 samples per second is the lowest
#define MPU9250_COMPASSRATE_MAX     100                     // 100 samples per second is maximum

//  Gyro LPF options

#define MPU9250_GYRO_LPF_8800       0x11                    // 8800Hz, 0.64mS delay
#define MPU9250_GYRO_LPF_3600       0x10                    // 3600Hz, 0.11mS delay
#define MPU9250_GYRO_LPF_250        0x00                    // 250Hz, 0.97mS delay
#define MPU9250_GYRO_LPF_184        0x01                    // 184Hz, 2.9mS delay
#define MPU9250_GYRO_LPF_92         0x02                    // 92Hz, 3.9mS delay
#define MPU9250_GYRO_LPF_41         0x03                    // 41Hz, 5.9mS delay
#define MPU9250_GYRO_LPF_20         0x04                    // 20Hz, 9.9mS delay
#define MPU9250_GYRO_LPF_10         0x05                    // 10Hz, 17.85mS delay
#define MPU9250_GYRO_LPF_5          0x06                    // 5Hz, 33.48mS delay

//  Gyro FSR options

#define MPU9250_GYROFSR_250         0                       // +/- 250 degrees per second
#define MPU9250_GYROFSR_500         8                       // +/- 500 degrees per second
#define MPU9250_GYROFSR_1000        0x10                    // +/- 1000 degrees per second
#define MPU9250_GYROFSR_2000        0x18                    // +/- 2000 degrees per second

//  Accel FSR options

#define MPU9250_ACCELFSR_2          0                       // +/- 2g
#define MPU9250_ACCELFSR_4          8                       // +/- 4g
#define MPU9250_ACCELFSR_8          0x10                    // +/- 8g
#define MPU9250_ACCELFSR_16         0x18                    // +/- 16g

//  Accel LPF options

#define MPU9250_ACCEL_LPF_1130      0x08                    // 1130Hz, 0.75mS delay
#define MPU9250_ACCEL_LPF_460       0x00                    // 460Hz, 1.94mS delay
#define MPU9250_ACCEL_LPF_184       0x01                    // 184Hz, 5.80mS delay
#define MPU9250_ACCEL_LPF_92        0x02                    // 92Hz, 7.80mS delay
#define MPU9250_ACCEL_LPF_41        0x03                    // 41Hz, 11.80mS delay
#define MPU9250_ACCEL_LPF_20        0x04                    // 20Hz, 19.80mS delay
#define MPU9250_ACCEL_LPF_10        0x05                    // 10Hz, 35.70mS delay
#define MPU9250_ACCEL_LPF_5         0x06                    // 5Hz, 66.96mS delay

//  AK8963 compass registers

#define AK8963_DEVICEID             0x48                    // the device ID
#define AK8963_ST1                  0x02                    // status 1
#define AK8963_CNTL                 0x0a                    // control reg
#define AK8963_ASAX                 0x10                    // start of the fuse ROM data

//  FIFO transfer size

#define MPU9250_FIFO_CHUNK_SIZE     12                      // gyro and accels take 12 bytes

class RTIMUMPU9250 : public RTIMU
{
public:
    RTIMUMPU9250() = delete;
    RTIMUMPU9250(RTIMUSettings& settings);

    ~RTIMUMPU9250();

    bool setGyroLpf(unsigned char lpf);
    bool setAccelLpf(unsigned char lpf);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelFsr(unsigned char fsr);

    virtual const char *IMUName() { return "MPU-9250"; }
    virtual int IMUType() { return RTIMU_TYPE_MPU9250; }
    virtual int IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

  protected:
    // i2c
    uint8_t _address;
    TwoWire *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes; // number of bytes received from I2C
    
    // spi
    SPIClass *_spi;
    uint8_t _csPin;
    bool _useSPI;
    bool _useSPIHS;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
    const uint32_t SPI_HS_CLOCK = 15000000; // 15 MHz

    //sample rate divider
    uint8_t _srd;

    // buffer for reading from sensor
    uint8_t _buffer[21];
    // track success of interacting with sensor
    int _status;

private:

    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int writeAK8963Register(uint8_t subAddress, uint8_t data);
    int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
    int whoAmIAK8963();
    int setSrd(uint8_t srd);

    bool setGyroConfig();
    bool setAccelConfig();
    bool setSampleRate();
    bool compassSetup();
    bool setCompassRate();
    bool resetFifo();
  //  bool bypassOn();
  //  bool bypassOff();

    bool m_firstTime;                                       // if first sample

   // unsigned char m_bus;                                    // I2C bus (usually 1 for Raspberry Pi for example)

    unsigned char m_gyroLpf;                                // gyro low pass filter setting
    unsigned char m_accelLpf;                               // accel low pass filter setting
    int m_compassRate;                                      // compass sample rate in Hz
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;

    RTFLOAT m_compassAdjust[3];                             // the compass fuse ROM values converted for use
};

#endif // _RTIMUMPU9250_H
