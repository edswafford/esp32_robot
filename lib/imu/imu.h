#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <EEPROM.h>
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "calibration_storage.h"

enum CalibCmds {CAL_ACCEL, CAL_MAG, SAVE, END};

class IMU
{
public:
    IMU()
    {
    }
    ~IMU()
    {
        delete rt_imu_;
    }
    /*
    bool storeMagCalData(bool valid){
        _calib_data.compassCalValid = valid;
        
        _calib_data.compassMin[0] = getMagBiasX_uT();
        _calib_data.compassMin[1] = getMagBiasY_uT();
        _calib_data.compassMin[2] = getMagBiasZ_uT();

        _calib_data.compassMax[0] = getMagScaleFactorX();
        _calib_data.compassMax[1] = getMagScaleFactorY();
        _calib_data.compassMax[2] = getMagScaleFactorZ();

        return storage.write(&_calib_data);
    }

   bool storeAccelCalData(bool valid){
        _calib_data.accelCalValid = valid;
        
        _calib_data.accelMin[0] = getAccelBiasX_mss();
        _calib_data.accelMin[1] = getAccelBiasY_mss();
        _calib_data.accelMin[2] = getAccelBiasZ_mss();

        _calib_data.accelMax[0] = getAccelScaleFactorX();
        _calib_data.accelMax[1] = getAccelScaleFactorY();
        _calib_data.accelMax[2] = getAccelScaleFactorZ();

        return storage.write(&_calib_data);
    }
*/
    bool init()
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

        rt_imu_->setSlerpPower(0.02);
        rt_imu_->setGyroEnable(true);
        rt_imu_->setAccelEnable(true);
        rt_imu_->setCompassEnable(true);
        return valid_;
    }
    bool isValid() { return valid_; }

    RTIMU* getRTIMU() {return rt_imu_;}
    RTIMUSettings& getSettings(){return settings_;}
private:
    bool valid_{false};
    RTIMUSettings settings_;
    RTIMU *rt_imu_;

    CALLIB_DATA _calib_data;
    CalibrationStorage _storage;
};

#endif      //_IMU_H_
