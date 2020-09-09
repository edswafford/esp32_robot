#ifndef _CALLIBRATION_STORAGE_H_
#define _CALLIBRATION_STORAGE_H_

#include <Arduino.h>

#define CALLIB_DATA_VALID_LOW     0xfa // pattern to detect valid config - low byte
#define CALLIB_DATA_VALID_HIGH    0x85 // pattern to detect valid config - high byte

typedef struct
{
  unsigned char validL;                      // 0   should contain the valid pattern if a good config
  unsigned char validH;                      // 1   should contain the valid pattern if a good config
  unsigned char compassCalValid;             // 2   true if data valid
  unsigned char accelCalValid;               // 3

  float compassBias[3];                      // 4   Compass Bias
  float compassScaleFactor[3];               // 16  Compass Scale Factor
  float accelBias[3];                        // 28  
  float accelScaleFactor[3];                 // 40
         // 52              
} CALLIB_DATA;

class CalibrationStorage{

  public:
  
  bool init();

  //  erases any current data in the EEPROM
  bool erase();

  //  writes new data to the EEPROM
  bool write(CALLIB_DATA * calData);

  //reads existing data and returns true if valid else false in not.
  bool read(CALLIB_DATA * calData);


    private:
    unsigned _length{sizeof(CALLIB_DATA)};
    bool _valid{false};
};




#endif // _CALLIBRATION_STORAGE_H_
