#ifndef _CALLIBRATION_STORAGE_H_
#define _CALLIBRATION_STORAGE_H_

#include <Arduino.h>
#include "RTIMUCalDefs.h"

#define CALLIB_DATA_VALID_LOW 0xfa  // pattern to detect valid config - low byte
#define CALLIB_DATA_VALID_HIGH 0x85 // pattern to detect valid config - high byte

typedef struct
{
  unsigned char validL;          // 0   should contain the valid pattern if a good config
  unsigned char validH;          // 1   should contain the valid pattern if a good config
  unsigned char compassCalValid; // 2   true if data valid
  unsigned char accelCalValid;   // 3

  float compassMin[3]; // 4
  float compassMax[3]; // 16
  float accelMin[3];   // 28
  float accelMax[3];   // 40
                       // 52   

} CALLIB_DATA;

class CalibrationStorage
{

public:
  bool init();

  //  erases any current data in the EEPROM
  bool erase();

  //  writes new data to the EEPROM
  bool write();

  //reads existing data and returns true if valid else false in not.
  bool read();

   // Getters and Setters
     unsigned char getCompassCalValid() const {
        return _calData.compassCalValid;
    }

    void setCompassCalValid(unsigned char compassCalValid) {
        _calData.compassCalValid = compassCalValid;
    }

    unsigned char getAccelCalValid() const {
        return _calData.accelCalValid;
    }

    void setAccelCalValid(unsigned char accelCalValid) {
        _calData.accelCalValid = accelCalValid;
    }

    const float getCompassMin(int index) const {
        return _calData.compassMin[index];
    }

    const float getCompassMax(int index) const {
        return _calData.compassMax[index];
    }

    const float getAccelMin(int index) const {
        return _calData.accelMin[index];
    }

    const float getAccelMax(int index) const {
        return _calData.accelMax[index];
    } 
    

    void setCompassMin(float x, float y, float z)  {
         _calData.compassMin[0] = x;
         _calData.compassMin[1] = y;
         _calData.compassMin[2] = z;
    }

    void setCompassMax(float x, float y, float z)  {
        _calData.compassMax[0] = x;
         _calData.compassMax[1] = y;
          _calData.compassMax[2] = z;
    }

    void setAccelMin(float x, float y, float z)  {
        _calData.accelMin[0] = x;
        _calData.accelMin[1] = y;
        _calData.accelMin[2] = z;
    }

    void setAccelMax(float x, float y, float z)  {
        _calData.accelMax[0] = x;
        _calData.accelMax[1] = y;
        _calData.accelMax[2] = z;
    }  

private:
  unsigned _length{sizeof(CALLIB_DATA)};
  bool _eeprom_valid{false};
  CALLIB_DATA _calData{
      // set Calibration data to defaults
      0,
      0,
      0,
      0,
      {RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN},
      {RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX},

      {RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN},
      {RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX},
  };
};

#endif // _CALLIBRATION_STORAGE_H_
