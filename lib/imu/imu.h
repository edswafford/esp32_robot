#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <EEPROM.h>
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "calibration_storage.h"

#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"
#include "RTIMUMPU9250.h"

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
        if(_storage.init() && _storage.read()) {
            Serial.printf("Storage Calibration values are available\n");
        }
        else {
            Serial.printf("Failed to initialize Calibration Storage memory.  Calibration values are not available\n");
        }

        // Initialize Calibration
        settings_.init(&_storage);

        rt_imu_ = new RTIMUMPU9250(settings_);
        valid_ = rt_imu_->IMUInit();

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
