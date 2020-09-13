#ifndef _CAL_IMU_H_
#define _CAL_IMU_H_

#include <Arduino.h>
#include "imu.h"

class CalImu
{
public:
    CalImu(IMU &imu) : _imu(imu) {}
    CalImu() = delete;

    void doCalibration();

private:
    IMU &_imu;
    const float G = 9.807f;

    // magnetometer bias and scale factor estimation
    uint16_t _maxCounts{1000};
    float _deltaThresh{0.3f};
    uint8_t _coeff{8};
    uint16_t _counter;
    float _framedelta, _delta;
    float _hxfilt, _hyfilt, _hzfilt;
    float _hxmax, _hymax, _hzmax;
    float _hxmin, _hymin, _hzmin;
    float _hxb{0.0};
    float _hyb{0.0};
    float _hzb{0.0};
    float _hxs{1.0f};
    float _hys{1.0f};
    float _hzs{1.0f};
    float _avgs;

    float _prev_hxmin{0.0};
    float _prev_hxmax{0.0};
    float _prev_hymin{0.0};
    float _prev_hymax{0.0};
    float _prev_hzmin{0.0};
    float _prev_hzmax{0.0};

    // gyro bias estimation
    size_t _numSamples{100};
    // accel bias and scale factor estimation
    double _axbD, _aybD, _azbD;
    float _axmax, _aymax, _azmax;
    float _axmin, _aymin, _azmin;
    float _ax;
    float _ay;
    float _az;
    float _axb{0.0f};
    float _ayb{0.0f};
    float _azb{0.0f};
    float _axs{1.0f};
    float _ays{1.0f};
    float _azs{1.0f};
    bool accelEnables[3];
    int accelCurrentAxis;
    int previousAccelAxis;

    int calibrateMag();
    int calibrateAccel();

    char get_char();
    char getUserChar();
    void displayMenu();
    void displayMagMinMax();
    void displayAccelMinMax();
};

#endif // _CAL_IMU_H_
