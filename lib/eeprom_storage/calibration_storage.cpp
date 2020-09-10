#include "calibration_storage.h"

#include <EEPROM.h>

bool CalibrationStorage::init()
{

  // initialize EEPROM with predefined size
  _eeprom_valid = EEPROM.begin(_length);
  
  return _eeprom_valid;
}

bool CalibrationStorage::erase()
{
  if (_eeprom_valid)
  {
    EEPROM.write(0, 0); // just destroy the valid byte
  }
  EEPROM.commit();
  return _eeprom_valid;
}

bool CalibrationStorage::write()
{
  if (_eeprom_valid)
  {
    byte *ptr = (byte *)&_calData;

    _calData.validL = CALLIB_DATA_VALID_LOW;
    _calData.validH = CALLIB_DATA_VALID_HIGH;

    for (byte i = 0; i < _length; i++){
      EEPROM.write(i, *ptr++);
    }
    EEPROM.commit();

    // Validate
    ptr = (byte *)&_calData;
    for (byte i = 0; i < _length; i++){
      if(*ptr++ != EEPROM.read(i)) {
        return false;
      }
    }
  }
  return _eeprom_valid;
}

bool CalibrationStorage::read()
{
  if (_eeprom_valid)
  {
    byte *ptr = (byte *)&_calData;

    if ((EEPROM.read(0) != CALLIB_DATA_VALID_LOW) ||
        (EEPROM.read(1) != CALLIB_DATA_VALID_HIGH))
    {
      _calData.compassCalValid = false;
      _calData.accelCalValid = false;
      return false; // invalid data
    }

    for (byte i = 0; i < _length; i++){
      *ptr++ = EEPROM.read(i);
    }
  }
  return _eeprom_valid;
}
