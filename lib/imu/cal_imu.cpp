#include "cal_imu.h"

void CalImu::doCalibration()
{
    bool mustExit = false;

    while (!mustExit)
    {
        displayMenu();
        switch (tolower(get_char()))
        {
        case 'x':
            mustExit = true;
            break;

        case 'm':
            calibrateMag();
            break;

        case 'a':
            calibrateAccel();
            break;
        }
    }
}

int CalImu::calibrateMag()
{
    unsigned long displayTimer;
    unsigned long now;
    char input = 0;
    uint8_t srd = _imu.getSrd();

    // set the srd to 50Hz (sensors read every 20ms)
    if (_imu.setSrd(19) < 0)
    {
        return -1;
    }

    bool done = false;
    while (!done)
    {
        Serial.printf("\n\nMagnetometer min/max calibration\n");
        Serial.printf("--------------------------------\n");
        Serial.printf("Waggle the IMU chip around, ensuring that all six axes\n");
        Serial.printf("(+x, -x, +y, -y and +z, -z) go through their extrema.\n");
        Serial.printf("When all extrema have been achieved, enter 's' to save, 'r' to reset\n");
        Serial.printf("or 'x' to abort and discard the data.\n");
        Serial.printf("\nPress any key to start...");
        get_char();

        displayTimer = millis();

        // get a starting set of data
        _imu.readSensor();
        _hxmax = _imu.getMagX_uT();
        _hxmin = _imu.getMagX_uT();
        _hymax = _imu.getMagY_uT();
        _hymin = _imu.getMagY_uT();
        _hzmax = _imu.getMagZ_uT();
        _hzmin = _imu.getMagZ_uT();

        _counter = 0;
        while (_counter < _maxCounts)
        {
            _delta = 0.0f;
            _framedelta = 0.0f;
            _imu.readSensor();
            _hxfilt = (_hxfilt * ((float)_coeff - 1) + (_imu.getMagX_uT() / _hxs + _hxb)) / ((float)_coeff);
            _hyfilt = (_hyfilt * ((float)_coeff - 1) + (_imu.getMagY_uT() / _hys + _hyb)) / ((float)_coeff);
            _hzfilt = (_hzfilt * ((float)_coeff - 1) + (_imu.getMagZ_uT() / _hzs + _hzb)) / ((float)_coeff);
            if (_hxfilt > _hxmax)
            {
                _delta = _hxfilt - _hxmax;
                _hxmax = _hxfilt;
            }
            if (_delta > _framedelta)
            {
                _framedelta = _delta;
            }
            if (_hyfilt > _hymax)
            {
                _delta = _hyfilt - _hymax;
                _hymax = _hyfilt;
            }
            if (_delta > _framedelta)
            {
                _framedelta = _delta;
            }
            if (_hzfilt > _hzmax)
            {
                _delta = _hzfilt - _hzmax;
                _hzmax = _hzfilt;
            }
            if (_delta > _framedelta)
            {
                _framedelta = _delta;
            }
            if (_hxfilt < _hxmin)
            {
                _delta = abs(_hxfilt - _hxmin);
                _hxmin = _hxfilt;
            }
            if (_delta > _framedelta)
            {
                _framedelta = _delta;
            }
            if (_hyfilt < _hymin)
            {
                _delta = abs(_hyfilt - _hymin);
                _hymin = _hyfilt;
            }
            if (_delta > _framedelta)
            {
                _framedelta = _delta;
            }
            if (_hzfilt < _hzmin)
            {
                _delta = abs(_hzfilt - _hzmin);
                _hzmin = _hzfilt;
            }
            if (_delta > _framedelta)
            {
                _framedelta = _delta;
            }
            if (_framedelta > _deltaThresh)
            {
                _counter = 0;
            }
            else
            {
                _counter++;
            }

            now = millis();

            //  display 1 time per second

            if ((now - displayTimer) > 1000)
            {
                displayMagMinMax();
                displayTimer = now;
            }

            if ((input = Serial.read()) != 0)
            {
                switch (input)
                {
                case 's':
                case 'S':
                    done = true;
                    break;

                case 'x':
                case 'X':
                    Serial.printf("\nAborting.\n");
                    return 0;

                case 'r':
                case 'R':
                    Serial.printf("\nResetting min/max data.\n");
                    break;
                }
            }
            delay(20);
        }
    }

    if (input != 's' || input != 'S')
    {
        Serial.printf("All extrems have been achieved, enter 's' to save, 'x' to abort\n");
        input = get_char();
        switch (input)
        {
        case 's':
        case 'S':
            break;

        case 'x':
        case 'X':
        default:
            Serial.printf("\nAborting.\n");
            return 0;
        }
    }
    // find the magnetometer bias
    _hxb = (_hxmax + _hxmin) / 2.0f;
    _hyb = (_hymax + _hymin) / 2.0f;
    _hzb = (_hzmax + _hzmin) / 2.0f;

    // find the magnetometer scale factor
    _hxs = (_hxmax - _hxmin) / 2.0f;
    _hys = (_hymax - _hymin) / 2.0f;
    _hzs = (_hzmax - _hzmin) / 2.0f;
    _avgs = (_hxs + _hys + _hzs) / 3.0f;
    _hxs = _avgs / _hxs;
    _hys = _avgs / _hys;
    _hzs = _avgs / _hzs;

    _imu.setMagCalX(_hxb, _hxs);
    _imu.setMagCalY(_hyb, _hys);
    _imu.setMagCalZ(_hzb, _hzs);

    Serial.printf("\nSaving Compass Calibration data.\n\n");
    if (_imu.storeMagCalData(true))
    {
        Serial.printf("EEPROM Storage Success\n");
    }
    else
    {
        Serial.printf("EEPROM Storage FAILED!\n");
    }

    // set the srd back to what it was
    if (_imu.setSrd(srd) < 0)
    {
        return -2;
    }
    return 1;
}

int CalImu::calibrateAccel()
{

    uint64_t displayTimer;
    uint64_t now;
    char input;

    // set the range, bandwidth, and srd
    IMU::AccelRange accelRange = _imu.getAccelRange();
    if (_imu.setAccelRange(_imu.ACCEL_RANGE_2G) < 0)
    {
        return -1;
    }
    IMU::DlpfBandwidth bandwidth = _imu.getDlpfBandwidth();
    if (_imu.setDlpfBandwidth(_imu.DLPF_BANDWIDTH_20HZ) < 0)
    {
        _imu.setAccelRange(accelRange);
        return -2;
    }
    uint8_t srd = _imu.getSrd();
    if (_imu.setSrd(19) < 0)
    {
        _imu.setAccelRange(accelRange);
        _imu.setDlpfBandwidth(bandwidth);
        return -3;
    }

    Serial.printf("\n\nAccelerometer Calibration\n");
    Serial.printf("-------------------------\n");
    Serial.printf("The code normally ignores readings until an axis has been enabled.\n");
    Serial.printf("The idea is to orient the IMU near the current extrema (+x, -x, +y, -y, +z, -z)\n");
    Serial.printf("and then enable the axis, moving the IMU very gently around to find the\n");
    Serial.printf("extreme value. Now disable the axis again so that the IMU can be inverted.\n");
    Serial.printf("When the IMU has been inverted, enable the axis again and find the extreme\n");
    Serial.printf("point. Disable the axis again and press the space bar to move to the next\n");
    Serial.printf("axis and repeat. The software will display the current axis and enable state.\n");
    Serial.printf("Available options are:\n");
    Serial.printf("  e - enable the current axis.\n");
    Serial.printf("  d - disable the current axis.\n");
    Serial.printf("  space bar - move to the next axis (x then y then z then x etc.\n");
    Serial.printf("  r - reset the current axis (if enabled).\n");
    Serial.printf("  s - save the data once all 6 extrema have been collected.\n");
    Serial.printf("  x - abort and discard the data.\n");
    Serial.printf("\nPress any key to start...");
    get_char();

    accelCurrentAxis = 0;

    for (int i = 0; i < 3; i++)
    {
        accelEnables[i] = false;
    }
    displayTimer = millis();

    bool done = false;
    bool save_accelCal = false;
    while (!done)
    {
        int sampleCount = 0;
        _axbD = 0;
        _aybD = 0;
        _azbD = 0;
        previousAccelAxis = accelCurrentAxis;

        while (!done && (previousAccelAxis == accelCurrentAxis))
        {
            _imu.readSensor();

            _ax = _imu.getAccelX_mss();
            _ay = _imu.getAccelY_mss();
            _az = _imu.getAccelZ_mss();

            if (accelCurrentAxis == 0)
            {
                if (accelEnables[0])
                {
                    sampleCount += 1;
                    _axbD += (_ax / _axs + _axb) / ((double)_numSamples);
                    if (sampleCount == _numSamples)
                    {
                        sampleCount = 0;
                        if (_axbD > 9.0f)
                        {
                            _axmax = (float)_axbD;
                        }
                        if (_axbD < -9.0f)
                        {
                            _axmin = (float)_axbD;
                        }
                    }
                }
            }
            else if (accelCurrentAxis == 1)
            {
                if (accelEnables[0])
                {
                    sampleCount += 1;
                    _aybD += (_ay / _ays + _ayb) / ((double)_numSamples);
                    if (sampleCount == _numSamples)
                    {
                        sampleCount = 0;
                        if (_aybD > 9.0f)
                        {
                            _aymax = (float)_aybD;
                        }
                        if (_aybD < -9.0f)
                        {
                            _aymin = (float)_aybD;
                        }
                    }
                }
            }
            else
            {
                if (accelEnables[0])
                {
                    sampleCount += 1;
                    _azbD += (_az / _azs + _azb) / ((double)_numSamples);
                    if (sampleCount == _numSamples)
                    {
                        sampleCount = 0;
                        if (_azbD > 9.0f)
                        {
                            _azmax = (float)_azbD;
                        }
                        if (_azbD < -9.0f)
                        {
                            _azmin = (float)_azbD;
                        }
                    }
                }
            }

            now = millis();
            //  display once per second
            if ((now - displayTimer) > 1000)
            {
                displayAccelMinMax();
                displayTimer = now;
            }
            if ((input = getUserChar()) != 0)
            {
                switch (input)
                {
                case 'e':
                    accelEnables[accelCurrentAxis] = true;
                    break;

                case 'd':
                    accelEnables[accelCurrentAxis] = false;
                    break;

                case 'r':
                    accelCurrentAxis = 0;
                    accelEnables[0] = false;
                    accelEnables[1] = false;
                    accelEnables[2] = false;
                    sampleCount = 0;
                    _axbD = 0;
                    _aybD = 0;
                    _azbD = 0;
                    _axmax = _axmin = 0.0f;
                    _aymax = _aymin = 0.0f;
                    _azmax = _azmin = 0.0f;
                    break;

                case ' ':
                    if (++accelCurrentAxis == 3)
                    {
                        accelCurrentAxis = 0;
                    }
                    break;

                case 's':
                    save_accelCal = true;
                    done = true;

                case 'x':
                    Serial.printf("\nAborting.\n");
                    done = true;
                }
            }
            delay(20);
        }
    }

    if (save_accelCal)
    {
        // find bias and scale factor
        if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f))
        {
            _axb = (_axmin + _axmax) / 2.0f;
            _axs = G / ((abs(_axmin) + abs(_axmax)) / 2.0f);
        }
        if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f))
        {
            _ayb = (_aymin + _aymax) / 2.0f;
            _ays = G / ((abs(_aymin) + abs(_aymax)) / 2.0f);
        }
        if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f))
        {
            _azb = (_azmin + _azmax) / 2.0f;
            _azs = G / ((abs(_azmin) + abs(_azmax)) / 2.0f);
        }

        _imu.setAccelCalX(_axb, _axs);
        _imu.setAccelCalY(_ayb, _ays);
        _imu.setAccelCalZ(_azb, _azs);

        Serial.printf("\nSaving  Accelerometer Calibration data.\n\n");
        if (_imu.storeAccelCalData(true))
        {
            Serial.printf("EEPROM Storage Success\n");
        }
        else
        {
            Serial.printf("EEPROM Storage FAILED!\n");
        }
    }
    // set the range, bandwidth, and srd back to what they were
    if (_imu.setAccelRange(accelRange) < 0)
    {
        return -4;
    }
    if (_imu.setDlpfBandwidth(bandwidth) < 0)
    {
        return -5;
    }
    if (_imu.setSrd(srd) < 0)
    {
        return -6;
    }
    return 1;
}

char CalImu::get_char()
{
    int ch = 0;
    while (ch <= 0)
    {
        delay(250);
        ch = Serial.read();
    }
    return char(ch);
}
char CalImu::getUserChar()
{
    char ch = Serial.read();
    ;
    if (ch > 0)
    {
        return tolower(ch);
    }
    return 0;
}
void CalImu::displayMenu()
{
    Serial.printf("\n");
    Serial.printf("Options are: \n\n");
    Serial.printf("  m - calibrate magnetometer with min/max\n");
    Serial.printf("  e - calibrate magnetometer with ellipsoid (do min/max first)\n");
    Serial.printf("  a - calibrate accelerometers\n");
    Serial.printf("  x - exit\n\n");
    Serial.printf("Enter option: ");
}

void CalImu::displayMagMinMax()
{
    if (_hxmin != _prev_hxmin || _hxmax != _prev_hxmax ||
        _hymin != _prev_hymin || _hymax != _prev_hymax ||
        _hzmin != _prev_hzmin || _hzmax != _prev_hzmax)
    {
        _prev_hxmin = _hxmin;
        _prev_hxmax = _hxmax;
        _prev_hymin = _hymin;
        _prev_hymax = _hymax;
        _prev_hzmin = _hzmin;
        _prev_hzmax = _hzmax;

        Serial.printf("\n\n");
        Serial.printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\n", _hxmin, _hymin, _hzmin);
        Serial.printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", _hxmax, _hymax, _hzmax);
        Serial.flush();
    }
}

void CalImu::displayAccelMinMax()
{
    Serial.printf("\n\n");

    Serial.printf("Current axis: ");
    if (accelCurrentAxis == 0)
    {
        Serial.printf("x - %s", accelEnables[0] ? "enabled" : "disabled");
        Serial.printf("\nMin x: %6.2f Max x: %6.2f\n", _axmin, _axmax);
        Serial.printf("x: %6.2f\n", _ax);
    }
    else if (accelCurrentAxis == 1)
    {
        Serial.printf("y - %s", accelEnables[1] ? "enabled" : "disabled");
        Serial.printf("\nMin y: %6.2f Max y: %6.2f\n", _aymin, _aymax);
        Serial.printf("y: %6.2f\n", _ay);
    }
    else if (accelCurrentAxis == 2)
    {
        Serial.printf("z - %s", accelEnables[2] ? "enabled" : "disabled");
        Serial.printf("\nMin z: %6.2f Max z: %6.2f\n", _azmin, _azmax);
        Serial.printf("z: %6.2f\n", _az);
    }

    Serial.flush();
}