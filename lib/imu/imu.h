<<<<<<< HEAD
#include <EEPROM.h>
#include <Arduino.h>
=======
#ifndef _IMU_H_
#define _IMU_H_

#include <MPU9250.h>
#include <EEPROM.h>
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "calibration_storage.h"
>>>>>>> 9e4fdeb23d7a4a4f8a5450e08b276f5239e1a87d

#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"
#include "RTIMUMPU9250.h"

class IMU
{
public:
<<<<<<< HEAD
    IMU()
    {
    }
    ~IMU()
    {
        delete rt_imu_;
=======
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
        if (storage.init())
        {
            storage.read(&_calib_data);
        }
        else
        {
            _calib_data.compassCalValid = false;
            _calib_data.accelCalValid = false;

            _calib_data.compassBias[0] = 0.0f;
            _calib_data.compassBias[1] = 0.0f;
            _calib_data.compassBias[2] = 0.0f;
            _calib_data.compassScaleFactor[0] = 1.0f;
            _calib_data.compassScaleFactor[1] = 1.0f;
            _calib_data.compassScaleFactor[2] = 1.0f;

            _calib_data.accelBias[0] = 0.0f;
            _calib_data.accelBias[1] = 0.0f;
            _calib_data.accelBias[2] = 0.0f;
            _calib_data.accelScaleFactor[0] = 1.0f;
            _calib_data.accelScaleFactor[1] = 1.0f;
            _calib_data.accelScaleFactor[2] = 1.0f;
        }

                // Initialize IMU
        setMagCalX(_calib_data.compassBias[0], _calib_data.compassScaleFactor[0]);
        setMagCalY(_calib_data.compassBias[1], _calib_data.compassScaleFactor[1]);
        setMagCalZ(_calib_data.compassBias[2], _calib_data.compassScaleFactor[2]);

        setAccelCalX(_calib_data.accelBias[0], _calib_data.accelScaleFactor[0]);
        setAccelCalY(_calib_data.accelBias[1], _calib_data.accelScaleFactor[1]);
        setAccelCalZ(_calib_data.accelBias[2], _calib_data.accelScaleFactor[2]);

        return status;
>>>>>>> 9e4fdeb23d7a4a4f8a5450e08b276f5239e1a87d
    }
    bool storeMagCalData(bool valid){
        _calib_data.compassCalValid = valid;
        
        _calib_data.compassBias[0] = getMagBiasX_uT();
        _calib_data.compassBias[1] = getMagBiasY_uT();
        _calib_data.compassBias[2] = getMagBiasZ_uT();

        _calib_data.compassScaleFactor[0] = getMagScaleFactorX();
        _calib_data.compassScaleFactor[1] = getMagScaleFactorY();
        _calib_data.compassScaleFactor[2] = getMagScaleFactorZ();

        return storage.write(&_calib_data);
    }

   bool storeAccelCalData(bool valid){
        _calib_data.accelCalValid = valid;
        
        _calib_data.accelBias[0] = getAccelBiasX_mss();
        _calib_data.accelBias[1] = getAccelBiasY_mss();
        _calib_data.accelBias[2] = getAccelBiasZ_mss();

        _calib_data.accelScaleFactor[0] = getAccelScaleFactorX();
        _calib_data.accelScaleFactor[1] = getAccelScaleFactorY();
        _calib_data.accelScaleFactor[2] = getAccelScaleFactorZ();

        return storage.write(&_calib_data);
    }

    bool init()
    {

        // Initialize Calibration (Load from EEPROM)
        bool settings_valid = settings_.init();

        rt_imu_ = new RTIMUMPU9250(settings_);
        bool imu_valid = rt_imu_->IMUInit();

        rt_imu_->setSlerpPower(0.02);
        rt_imu_->setGyroEnable(true);
        rt_imu_->setAccelEnable(true);
        rt_imu_->setCompassEnable(true);
        valid_ = settings_valid && imu_valid;

        return valid_;
    }
    bool isValid() { return valid_; }

    void read() {
        rt_imu_->IMURead();
    }
    void getIMUData(){
        rt_imu_->getIMUData();
    }
private:
<<<<<<< HEAD
    bool valid_{false};
    RTIMUSettings settings_;
    RTIMU *rt_imu_;
};
=======
    CALLIB_DATA _calib_data;
    CalibrationStorage storage;
};

#endif      //_IMU_H_
>>>>>>> 9e4fdeb23d7a4a4f8a5450e08b276f5239e1a87d
