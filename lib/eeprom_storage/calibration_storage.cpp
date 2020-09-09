#include "calibration_storage.h"

#include <EEPROM.h>

bool CalibrationStorage::init()
{

  // initialize EEPROM with predefined size
  _valid = EEPROM.begin(_length);
  return _valid;
}

bool CalibrationStorage::erase()
{
  if (_valid)
  {
    EEPROM.write(0, 0); // just destroy the valid byte
  }
  EEPROM.commit();
  return _valid;
}

bool CalibrationStorage::write(CALLIB_DATA *calData)
{
  if (_valid)
  {
    byte *ptr = (byte *)calData;

    calData->validL = CALLIB_DATA_VALID_LOW;
    calData->validH = CALLIB_DATA_VALID_HIGH;

    for (byte i = 0; i < _length; i++){
      EEPROM.write(i, *ptr++);
    }
    EEPROM.commit();

    // Validate
    ptr = (byte *)calData;
    for (byte i = 0; i < _length; i++){
      if(*ptr++ != EEPROM.read(i)) {
        return false;
      }
    }
  }
  return _valid;
}

bool CalibrationStorage::read(CALLIB_DATA *calData)
{
  if (_valid)
  {
    byte *ptr = (byte *)calData;

    if ((EEPROM.read(0) != CALLIB_DATA_VALID_LOW) ||
        (EEPROM.read(1) != CALLIB_DATA_VALID_HIGH))
    {
      return false; // invalid data
    }

    for (byte i = 0; i < _length; i++){
      *ptr++ = EEPROM.read(i);
    }
  }
  return _valid;
}
