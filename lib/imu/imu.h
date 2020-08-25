#include <MPU9250.h>
#include <EEPROM.h>
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"

enum CalibCmds {CAL_ACCEL, CAL_MAG, SAVE, END};

class IMU : public MPU9250
{
public:
    IMU() : MPU9250(Wire, 0x68) {}

    int begin()
    {
        int status = MPU9250::begin();
        if (status > 0)
        {
            // setting the accelerometer full scale range to +/-8G
            setAccelRange(MPU9250::ACCEL_RANGE_2G);
            // setting the gyroscope full scale range to +/-250 deg/s
            setGyroRange(MPU9250::GYRO_RANGE_250DPS);

            setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
            //setSrd(19);
        }
        return status;
    }

    void fetch_imu_data(neo_msgs::Imu& raw_imu_msg);

    void calibrate_accelerometer(neo_msgs::ImuCal& cal_imu_msg)
    {
        int status = MPU9250::calibrateAccel();
        cal_imu_msg.imu_status = static_cast<int8_t>(status);
        if (status > 0)
        {
            cal_imu_msg.max_acceleration.x = _axmax;
            cal_imu_msg.max_acceleration.y = _aymax;
            cal_imu_msg.max_acceleration.z = _azmax;

            cal_imu_msg.min_acceleration.x = _axmin;
            cal_imu_msg.min_acceleration.y = _aymin; 
            cal_imu_msg.min_acceleration.z = _azmin;
        }
    }

private:
    short minVal[3];
    short maxVal[3];
};