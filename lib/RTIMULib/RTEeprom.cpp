////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTEeprom.h"


bool RTEEPROM::init()
{

  // initialize EEPROM with predefined size
  valid_ = EEPROM.begin(length_);
  return valid_;
}

bool RTEEPROM::erase()
{
  if (valid_)
  {
    EEPROM.write(0, 0); // just destroy the valid byte
  }
  EEPROM.commit();
  return valid_;
}

bool RTEEPROM::write(CALLIB_DATA *calData)
{
  if (valid_)
  {
    byte *ptr = (byte *)calData;

    calData->validL = CALLIB_DATA_VALID_LOW;
    calData->validH = CALLIB_DATA_VALID_HIGH;

    for (byte i = 0; i < length_; i++){
      EEPROM.write(i, *ptr++);
    }
    EEPROM.commit();

    // Validate
    ptr = (byte *)calData;
    for (byte i = 0; i < length_; i++){
      if(*ptr++ != EEPROM.read(i)) {
        return false;
      }
    }
  }
  return valid_;
}

bool RTEEPROM::read(CALLIB_DATA *calData)
{
  if (valid_)
  {
    byte *ptr = (byte *)calData;

    if ((EEPROM.read(0) != CALLIB_DATA_VALID_LOW) ||
        (EEPROM.read(1) != CALLIB_DATA_VALID_HIGH))
    {
      return false; // invalid data
    }

    for (byte i = 0; i < length_; i++){
      *ptr++ = EEPROM.read(i);
    }
  }
  return valid_;
}
