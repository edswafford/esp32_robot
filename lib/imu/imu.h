#include <EEPROM.h>
#include <Arduino.h>

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
    bool valid_{false};
    RTIMUSettings settings_;
    RTIMU *rt_imu_;
};