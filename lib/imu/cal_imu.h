#ifndef _CAL_IMU_H_
#define _CAL_IMU_H_

#include <Arduino.h>
#include "imu.h"

class CalImu
{
public:
    CalImu(IMU &imu) : imu_(imu) {}
    CalImu() = delete;

    void doCalibration();

private:
    IMU &imu_;
    const float G_ = 9.807f;

    // magnetometer bias and scale factor estimation
    uint16_t maxCounts_{1000};
    float deltaThresh_{0.3f};
    uint8_t coeff_{8};
    uint16_t counter_;
    float framedelta_, delta_;
    float hxfilt_, hyfilt_, hzfilt_;
    float hxmax_, hymax_, hzmax_;
    float hxmin_, hymin_, hzmin_;
    float hxb_{0.0};
    float hyb_{0.0};
    float hzb_{0.0};
    float hxs_{1.0f};
    float hys_{1.0f};
    float hzs_{1.0f};
    float avgs_;

    float prev_hxmin_{0.0};
    float prev_hxmax_{0.0};
    float prev_hymin_{0.0};
    float prev_hymax_{0.0};
    float prev_hzmin_{0.0};
    float prev_hzmax_{0.0};

    // gyro bias estimation
    size_t numSamples_{100};
    // accel bias and scale factor estimation
    double axbD_, aybD_, azbD_;
    float axmax_, aymax_, azmax_;
    float axmin_, aymin_, azmin_;
    float ax_;
    float ay_;
    float az_;
    float axb_{0.0f};
    float ayb_{0.0f};
    float azb_{0.0f};
    float axs_{1.0f};
    float ays_{1.0f};
    float azs_{1.0f};
    bool accelEnables_[3];
    int accelCurrentAxis_;
    int previousAccelAxis_;

    int calibrateMag();
    int calibrateAccel();

    char get_char();
    char getUserChar();
    void displayMenu();
    void displayMagMinMax();
    void displayAccelMinMax();
    void resetAccel();

};

#endif // _CAL_IMU_H_
