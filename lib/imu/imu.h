#ifndef _IMU_H_
#define _IMU_H_

#include <MPU9250.h>
#include <EEPROM.h>
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "calibration_storage.h"

enum CalibCmds
{
    SAVE,
    END
};

class IMU : public MPU9250
{
public:
    IMU() : MPU9250(Wire, 0x68)
    {
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
    }

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
                // Initialize IMU
        setMagCalX(_calib_data.accelBias[0], _calib_data.accelScaleFactor[0]);
        setMagCalX(_calib_data.accelBias[1], _calib_data.accelScaleFactor[1]);
        setMagCalX(_calib_data.accelBias[2], _calib_data.accelScaleFactor[2]);

        setAccelCalX(_calib_data.accelBias[0], _calib_data.accelScaleFactor[0]);
        setAccelCalX(_calib_data.accelBias[1], _calib_data.accelScaleFactor[1]);
        setAccelCalX(_calib_data.accelBias[2], _calib_data.accelScaleFactor[2]);

        return status;
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

    void fetch_imu_data(neo_msgs::Imu &raw_imu_msg);


private:
    CALLIB_DATA _calib_data;
    CalibrationStorage storage;
};

#endif      //_IMU_H_