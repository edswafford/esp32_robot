#include "calibration_storage.h"
#include <SPIFFS.h>


bool CalibrationStorage::write(CALLIB_DATA *calData)
{
  File cal = SPIFFS.open("/imu_calibration.bin", FILE_WRITE);
  if (SPIFFS.exists("/imu_calibration.bin"))
  {
    byte *ptr = (byte *)calData;

    calData->validL = CALLIB_DATA_VALID_LOW;
    calData->validH = CALLIB_DATA_VALID_HIGH;

    for (byte i = 0; i < _length; i++)
    {
      cal.write(*ptr++);
    }
    cal.close();

    // Validate
    ptr = (byte *)calData;
    File cal = SPIFFS.open("/imu_calibration.bin");
    for (byte i = 0; i < _length; i++)
    {
      Serial.println(*ptr);
      if (*ptr++ != cal.read())
      {
        return false;
      }
    }
  }
  else {
    return false;
  }
  return true;
}

bool CalibrationStorage::read(CALLIB_DATA *calData)
{
  if (SPIFFS.exists("/imu_calibration.bin"))
  {
    File cal = SPIFFS.open("/imu_calibration.bin");
    byte *ptr = (byte *)calData;
    for (byte i = 0; i < _length; i++)
    {
      *ptr++ = cal.read();
    }

    if ((calData->validL != CALLIB_DATA_VALID_LOW) ||
        (calData->validH != CALLIB_DATA_VALID_HIGH))
    {
      return false; // invalid data
    }
    Serial.printf("Read IMU storage is valid\n");
  }
  else {
    Serial.printf("Read IMU storage is INVALID!\n");
    return false;
  }

  return true;
}
