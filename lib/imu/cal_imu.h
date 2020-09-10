#ifndef _CAL_IMU_H_
#define _CAL_IMU_H_

#include <Arduino.h>
#include "imu.h"

class CalImu
{
public:
    CalImu() = delete;
    CalImu(RTIMU* imu, RTIMUSettings& settings) : imu(imu), settings_(settings){}

    void doCalibration();

private:
 
    bool accelEnables[3];
    int accelCurrentAxis;
    int previousAccelAxis;

    void calibrateMag();
    void calibrateAccel();

    bool pollIMU();
    char get_char();
    char getUserChar();
    void displayMenu();
    void displayMagMinMax( RTIMUMagCal magCal);
    void displayAccelMinMax(RTIMUAccelCal& accelCal);
    
    RTIMU_DATA imuData;
    RTIMU* imu;
    RTIMUSettings& settings_;
    bool magMinMaxDone{false};
};

#endif // _CAL_IMU_H_
