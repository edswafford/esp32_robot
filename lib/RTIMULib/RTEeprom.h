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

#ifndef _RTEEPROM_H_
#define _RTEEPROM_H_

#include <Arduino.h>
#include <EEPROM.h>

#define CALLIB_DATA_VALID_LOW     0xfc // pattern to detect valid config - low byte
#define CALLIB_DATA_VALID_HIGH    0x15 // pattern to detect valid config - high byte

typedef struct
{
  unsigned char validL;                      // 0   should contain the valid pattern if a good config
  unsigned char validH;                      // 1   should contain the valid pattern if a good config
  unsigned char compassCalValid;             // 2   true if data valid
  unsigned char compassCalEllipsoidValid;    // 3

  float gyroBias[3];                         // 4   Gyro Bias
  float compassCalMax[3];                    // 16  min values
  float compassCalMin[3];                    // 28  max values
  float compassCalEllipsoidOffset[3];        // 40
  float compassCalEllipsoidCorr[3][3];       // 52
  float accelCalMin[3];                      // 88
  float accelCalMax[3];                      // 100
   
  unsigned char accelCalValid;               // 112
  unsigned char gyroBiasValid;               // 113
              // 114

              
} CALLIB_DATA;

class RTEEPROM{

  public:
  RTEEPROM(){}
  ~RTEEPROM(){}
  
  bool init();

  //  erases any current data in the EEPROM
  bool erase();

  //  writes new data to the EEPROM
  bool write(CALLIB_DATA * calData);

  //reads existing data and returns true if valid else false in not.
  bool read(CALLIB_DATA * calData);


    private:
    unsigned length_{sizeof(CALLIB_DATA)};
    bool valid_{false};
};




#endif // _RTEEPROM_H_
