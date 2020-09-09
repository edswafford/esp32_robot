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

#include "RTIMUMPU9250.h"
#include "RTIMUSettings.h"

/************************** Default Settings **************************************
 * 
 * 
    m_MPU9250GyroAccelSampleRate = 80;
    m_MPU9250CompassSampleRate = 40;
    m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;      // 41Hz, 5.9mS delay
    m_MPU9250AccelLpf = MPU9250_ACCEL_LPF_41;    // 41Hz, 11.80mS delay
    m_MPU9250GyroFsr = MPU9250_GYROFSR_1000;     // +/- 1000 degrees per second
    m_MPU9250AccelFsr = MPU9250_ACCELFSR_8;      // +/- 8g
    m_I2CSlaveAddress = MPU9250_ADDRESS0;         0x68

***********************************************************************************/


#if defined(MPU9250_68) || defined(MPU9250_69)

    RTIMUMPU9250::RTIMUMPU9250(RTIMUSettings& settings) : RTIMU(settings) {

     }



RTIMUMPU9250::~RTIMUMPU9250()
{
}

bool RTIMUMPU9250::setSampleRate(int rate)
{
    if ((rate < MPU9250_SAMPLERATE_MIN) || (rate > MPU9250_SAMPLERATE_MAX)) {
        return false;
    }

       //  Note: rates interact with the lpf settings

    if ((rate < MPU9250_SAMPLERATE_MAX) && (rate >= 8000))
        rate = 8000;

    if ((rate < 8000) && (rate >= 1000))
        rate = 1000;

    if (rate < 1000) {
        int sampleDiv = (1000 / rate) - 1;
        m_sampleRate = 1000 / (1 + sampleDiv);
    } else {
        m_sampleRate = rate;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    if (m_sampleInterval == 0){
        m_sampleInterval = 1;
    }
    return true;
}

bool RTIMUMPU9250::setGyroLpf(unsigned char lpf)
{
    switch (lpf)
    {
    case MPU9250_GYRO_LPF_8800:
    case MPU9250_GYRO_LPF_3600:
    case MPU9250_GYRO_LPF_250:
    case MPU9250_GYRO_LPF_184:
    case MPU9250_GYRO_LPF_92:
    case MPU9250_GYRO_LPF_41:
    case MPU9250_GYRO_LPF_20:
    case MPU9250_GYRO_LPF_10:
    case MPU9250_GYRO_LPF_5:
        m_gyroLpf = lpf;
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9250::setAccelLpf(unsigned char lpf)
{
    switch (lpf)
    {
    case MPU9250_ACCEL_LPF_1130:
    case MPU9250_ACCEL_LPF_460:
    case MPU9250_ACCEL_LPF_184:
    case MPU9250_ACCEL_LPF_92:
    case MPU9250_ACCEL_LPF_41:
    case MPU9250_ACCEL_LPF_20:
    case MPU9250_ACCEL_LPF_10:
    case MPU9250_ACCEL_LPF_5:
        m_accelLpf = lpf;
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9250::setCompassRate(int rate)
{
    if ((rate < MPU9250_COMPASSRATE_MIN) || (rate > MPU9250_COMPASSRATE_MAX))
    {
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9250::setGyroFsr(unsigned char fsr)
{
    switch (fsr)
    {
    case MPU9250_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9250_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9250_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9250_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9250::setAccelFsr(unsigned char fsr)
{
    switch (fsr)
    {
    case MPU9250_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0 / 16384.0;
        return true;

    case MPU9250_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0 / 8192.0;
        return true;

    case MPU9250_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0 / 4096.0;
        return true;

    case MPU9250_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0 / 2048.0;
        return true;

    default:
        return false;
    }
}

int RTIMUMPU9250::IMUInit()
{
    m_firstTime = true;

#ifdef MPU9250_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    setSampleRate(m_settings.m_MPU9250GyroAccelSampleRate);
    setCompassRate(m_settings.m_MPU9250CompassSampleRate);
    setGyroLpf(m_settings.m_MPU9250GyroLpf);
    setAccelLpf(m_settings.m_MPU9250AccelLpf);
    setAccelFsr(m_settings.m_MPU9250AccelFsr);
    setGyroFsr(m_settings.m_MPU9250GyroFsr);
    
    _useSPI = !m_settings.m_busIsI2C;
    
    if (_useSPI) // using SPI for communication
    {
        // use low speed SPI for register setting
        _useSPIHS = false;
        // setting CS pin to output
        pinMode(_csPin, OUTPUT);
        // setting CS pin high
        digitalWrite(_csPin, HIGH);
        // begin SPI communication
        _spi->begin();
    }
    else // using I2C for communication
    {
      _i2c = &Wire;
      _address = m_settings.m_I2CSlaveAddress;

        // starting the I2C bus
        _i2c->begin();
        // setting the I2C clock
        _i2c->setClock(_i2cRate);
    }

    setCalibrationData();

    //  reset the MPU9250

   // select clock source to gyro
  if(writeRegister(MPU9250_PWR_MGMT_1, MPU9250_CLOCK_SEL_PLL) < 0){
    return -1;
  }
  // enable I2C master mode
  if(writeRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN) < 0){
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if(writeRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK) < 0){
    return -3;
  }
  // set AK8963 to Power Down
  writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(MPU9250_PWR_MGMT_1, MPU9250_PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(1);
  // reset the AK8963
  writeAK8963Register(MPU9250_AK8963_CNTL2, MPU9250_AK8963_RESET);
  // select clock source to gyro
  if(writeRegister(MPU9250_PWR_MGMT_1, MPU9250_CLOCK_SEL_PLL) < 0){
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if((whoAmI() != 113)&&(whoAmI() != 115)){
    return -5;
  }

    /*  OLD ....
    //  power reset
    if (!writeRegister( MPU9250_PWR_MGMT_1, 0x80))
        return -1;
    delay(100);
    // set clock to Internal 20MHz oscillator
    if (!writeRegister( MPU9250_PWR_MGMT_1, 0x00))
        return -4;
    if (!I2Cdev::readByte(m_slaveAddr, MPU9250_WHO_AM_I, &result))
        return -5;
    if (result != MPU9250_ID)
    {
        return -6;
    }
    */

    //  now configure the various components
     // enable accelerometer and gyro
    if(writeRegister(MPU9250_PWR_MGMT_2, MPU9250_SEN_ENABLE) < 0){
      return -6;
   }
    unsigned char gyroConfig = m_gyroFsr + ((m_gyroLpf >> 3) & 3);
    unsigned char gyroLpf = m_gyroLpf & 7;

    // default MPU9250_GYROFSR_1000 --  +/- 1000 dps (degrees per second)
    if (!writeRegister(MPU9250_GYRO_CONFIG, gyroConfig)){
        return -7;
    }
    // Default MPU9250_GYRO_LPF_41  -- 41Hz
    if (!writeRegister( MPU9250_GYRO_LPF, gyroLpf)){
        return -8;
    }
    // Default MPU9250_ACCELFSR_8  --  +/- 8g
    if (!writeRegister( MPU9250_ACCEL_CONFIG, m_accelFsr)){
                return -9;
    }
    // Default MPU9250_ACCEL_LPF_41  --  41Hz
    if (!writeRegister( MPU9250_ACCEL_LPF, m_accelLpf)){
        return -10;
    }
    // default m_MPU9250GyroAccelSampleRate == 80Hz
    if(!setSrd(1000 / m_sampleRate - 1))
    {
      return -11;
    }

    // enable I2C master mode
    if(writeRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN) < 0){
  	  return -12;
    }

	// set the I2C bus speed to 400 kHz
	if( writeRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK) < 0){
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 72 ){
    return -14;
	}

    //  now configure compass

    /**** get fuse ROM data  ****
     * The AK8963 has a read-only Fuse ROM Access mode that allows the access to the Fuse ROM data. 
     * The Fuse ROM contains the sensitivity adjustment data for each axis. The Control 1 register 
     * of the AK8963 controls the operation mode
     * 
     */

  // set AK8963 to Power Down
  if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0){
    return -15;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_FUSE_ROM) < 0){
    return -16;
  }
  delay(100); // long wait between AK8963 mode changes
  
  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(MPU9250_AK8963_ASA, 3, _buffer);
    m_compassAdjust[0] = ((float)_buffer[0] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[1] = ((float)_buffer[1] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[2] = ((float)_buffer[2] - 128.0) / 256.0 + 1.0f;

  
  // set AK8963 to Power Down
  if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0){
    return -17;
  }
  delay(100); // long wait between AK8963 mode changes  
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if(writeRegister(MPU9250_PWR_MGMT_1, MPU9250_CLOCK_SEL_PLL) < 0){
    return -19;
  }       
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(MPU9250_AK8963_HXL, 7, _buffer);


    //  now set up MPU9250 to talk to the compass chip

    if (!writeRegister( MPU9250_I2C_MST_CTRL, 0x40))
        return -17;

    if (!writeRegister( MPU9250_I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS))
        return -18;

    if (!writeRegister( MPU9250_I2C_SLV0_REG, AK8963_ST1))
        return -19;

    if (!writeRegister( MPU9250_I2C_SLV0_CTRL, 0x88))
        return -20;

    if (!writeRegister( MPU9250_I2C_SLV1_ADDR, AK8963_ADDRESS))
        return -21;

    if (!writeRegister( MPU9250_I2C_SLV1_REG, AK8963_CNTL))
        return -22;

    if (!writeRegister( MPU9250_I2C_SLV1_CTRL, 0x81))
        return -23;

    if (!writeRegister( MPU9250_I2C_SLV1_DO, 0x1))
        return -24;

    if (!writeRegister( MPU9250_I2C_MST_DELAY_CTRL, 0x3))
        return -25;

    if (!setCompassRate())
        return -27;

    //  enable the sensors

    if (!writeRegister( MPU9250_PWR_MGMT_1, 1))
        return -28;

    if (!writeRegister( MPU9250_PWR_MGMT_2, 0))
        return -29;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return -30;

    gyroBiasInit();
    return 1;
}

/* writes a byte to MPU9250 register given a register address and data */
int RTIMUMPU9250::writeRegister(uint8_t subAddress, uint8_t data){
  /* write data to device */
  if( _useSPI ){
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the MPU9250 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    _spi->endTransaction(); // end the transaction
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  }

  delay(10);
  
  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int RTIMUMPU9250::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  if( _useSPI ){
    // begin the transaction
    if(_useSPIHS){
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin,LOW); // select the MPU9250 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++){
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++){ 
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}

/* writes a register to the AK8963 given a register address and data */
int RTIMUMPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
	if (writeRegister(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address 
	if (writeRegister(MPU9250_I2C_SLV0_REG, subAddress) < 0) {
    return -2;
  }
  // store the data for write
	if (writeRegister(MPU9250_I2C_SLV0_DO, data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
	if (writeRegister(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
	// read the register and confirm
	if (readAK8963Registers(subAddress, 1, _buffer) < 0) {
    return -5;
  }
	if(_buffer[0] == data) {
  	return 1;
  } else{
  	return -6;
  }
}

/* reads registers from the AK8963 */
int RTIMUMPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
  // set slave 0 to the AK8963 and set for read
	if (writeRegister(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ_FLAG) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (writeRegister(MPU9250_I2C_SLV0_REG, subAddress) < 0) {
    return -2;
  }
  // enable I2C and request the bytes
	if (writeRegister(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | count) < 0) {
    return -3;
  }
	delay(1); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
	_status = readRegisters(MPU9250_EXT_SENS_DATA_00, count, dest); 
  return _status;
}


/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int RTIMUMPU9250::whoAmI(){
  // read the WHO AM I register
  if (readRegisters(MPU9250_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int RTIMUMPU9250::whoAmIAK8963(){
  // read the WHO AM I register
  if (readAK8963Registers(MPU9250_AK8963_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}


/* sets the sample rate divider to values other than default */
int RTIMUMPU9250::setSrd(uint8_t srd) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  if(writeRegister(MPU9250_SMPRT_DIV, 19) < 0){ // setting the sample rate divider
    return -1;
  }
  if(srd > 9){
    // set AK8963 to Power Down
    if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0){
      return -2;
    }
    delay(100); // long wait between AK8963 mode changes  
    // set AK8963 to 16 bit resolution, 8 Hz update rate
    if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS1) < 0){
      return -3;
    }
    delay(100); // long wait between AK8963 mode changes     
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(MPU9250_AK8963_HXL, 7, _buffer);
  } else {
    // set AK8963 to Power Down
    if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0){
      return -2;
    }
    delay(100); // long wait between AK8963 mode changes  
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if(writeAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS2) < 0){
      return -3;
    }
    delay(100); // long wait between AK8963 mode changes     
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(MPU9250_AK8963_HXL, 7, _buffer);    
  } 
  /* setting the sample rate divider */
  if(writeRegister(MPU9250_SMPRT_DIV, srd) < 0){ // setting the sample rate divider
    return -4;
  } 
  _srd = srd;
  return 1; 
}




bool RTIMUMPU9250::resetFifo()
{
    if (!writeRegister( MPU9250_INT_ENABLE, 0))
        return false;
    if (!writeRegister( MPU9250_FIFO_EN, 0))
        return false;
    if (!writeRegister( MPU9250_USER_CTRL, 0))
        return false;

    if (!writeRegister( MPU9250_USER_CTRL, 0x04))
        return false;

    if (!writeRegister( MPU9250_USER_CTRL, 0x60))
        return false;

    delay(50);

    if (!writeRegister( MPU9250_INT_ENABLE, 1))
        return false;

    if (!writeRegister( MPU9250_FIFO_EN, 0x78))
        return false;

    return true;
}


bool RTIMUMPU9250::setGyroConfig()
{
    unsigned char gyroConfig = m_gyroFsr + ((m_gyroLpf >> 3) & 3);
    unsigned char gyroLpf = m_gyroLpf & 7;

    if (!writeRegister(MPU9250_GYRO_CONFIG, gyroConfig))
        return false;

    if (!writeRegister( MPU9250_GYRO_LPF, gyroLpf))
        return false;
    return true;
}

bool RTIMUMPU9250::setAccelConfig()
{
    if (!writeRegister( MPU9250_ACCEL_CONFIG, m_accelFsr))
        return false;

    if (!writeRegister( MPU9250_ACCEL_LPF, m_accelLpf))
        return false;
    return true;
}

bool RTIMUMPU9250::setSampleRate()
{
    if (m_sampleRate > 1000)
        return true; // SMPRT not used above 1000Hz

    if (!writeRegister( MPU9250_SMPRT_DIV, (unsigned char)(1000 / m_sampleRate - 1)))
        return false;

    return true;
}





bool RTIMUMPU9250::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31){
        rate = 31;
    }
    if (!writeRegister( MPU9250_I2C_SLV4_CTRL, rate)){
        return false;
    }
    return true;  
}

int RTIMUMPU9250::IMUGetPollInterval()
{
    if (m_sampleRate > 400){
        return 1;
    }
    else {
        return (400 / m_sampleRate);
    }
} 

bool RTIMUMPU9250::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!readRegisters( MPU9250_FIFO_COUNT_H, 2, fifoCount))
         return false;

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    if (count >= 512) {
        resetFifo();
        m_imuData.timestamp += m_sampleInterval * (512 / MPU9250_FIFO_CHUNK_SIZE + 1); // try to fix timestamp
        return false;
    }

#ifdef MPU9250_CACHE_MODE

# else
    if (count > MPU9250_FIFO_CHUNK_SIZE * 40) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= MPU9250_FIFO_CHUNK_SIZE * 10) {
            if (!readRegisters( MPU9250_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData))
                return false;
            count -= MPU9250_FIFO_CHUNK_SIZE;
            m_imuData.timestamp += m_sampleInterval;
        }
    }

    if (count < MPU9250_FIFO_CHUNK_SIZE)
      return false;

    if (!readRegisters( MPU9250_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData))
        return false;

    if (!readRegisters( MPU9250_EXT_SENS_DATA_00, 8, compassData))
        return false;
#endif

    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.6f, false);

    //  sort out gyro axes
    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());


    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  use the fuse data adjustments for compass

    m_imuData.compass.setX(m_imuData.compass.x() * m_compassAdjust[0]);
    m_imuData.compass.setY(m_imuData.compass.y() * m_compassAdjust[1]);
    m_imuData.compass.setZ(m_imuData.compass.z() * m_compassAdjust[2]);

    //  sort out compass axes

    float temp;

    temp = m_imuData.compass.x();
    m_imuData.compass.setX(m_imuData.compass.y());
    m_imuData.compass.setY(-temp);

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    if (m_firstTime)
        m_imuData.timestamp = millis();
    else
        m_imuData.timestamp += m_sampleInterval;

    m_firstTime = false;

    //  now update the filter
    updateFusion();
    return true;
}
#endif
