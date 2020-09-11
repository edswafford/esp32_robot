#ifndef _IMU_H_
#define _IMU_H_

#include <MPU9250.h>

#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"


enum CalibCmds
{
    SAVE,
    END
};

class IMU : public MPU9250
{
public:
    IMU() : MPU9250(Wire, 0x68){ }

    int getSrd() { return _srd; }
    IMU::AccelRange getAccelRange(){return _accelRange;}
    IMU::DlpfBandwidth getDlpfBandwidth() {return _bandwidth;}

    int begin()
    {
        int status = MPU9250::begin();
        if (status > 0)
        {
            // setting the accelerometer full scale range to +/-8G
            if (setAccelRange(MPU9250::ACCEL_RANGE_2G) < 0)
            {
                return -7;
            }
            // setting the gyroscope full scale range to +/-250 deg/s
            if (setGyroRange(MPU9250::GYRO_RANGE_250DPS) < 0)
            {
                return -8;
            }

            // setting DLPF bandwidth to 20 Hz
            if (setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ) < 0)
            {
                return -9;
            }
            // setting SRD to 19 for a 50 Hz update rate
            if (setSrd(19) < 0)
            {
                return -11;
            }
        }

        return status;
    }
   

    void fetch_imu_data(neo_msgs::Imu &raw_imu_msg);


private:

};

#endif      //_IMU_H_