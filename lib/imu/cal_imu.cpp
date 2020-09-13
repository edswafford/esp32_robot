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
    uint8_t srd = imu_.getSrd();

    // set the srd to 50Hz (sensors read every 20ms)
    if (imu_.setSrd(19) < 0)
    {
        return -1;
    }
    imu_.setMagCalX(hxb_, hxs_);
    imu_.setMagCalY(hyb_, hys_);
    imu_.setMagCalZ(hzb_, hzs_);

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
        imu_.readSensor();
        hxmax_ = imu_.getMagX_uT();
        hxmin_ = imu_.getMagX_uT();
        hymax_ = imu_.getMagY_uT();
        hymin_ = imu_.getMagY_uT();
        hzmax_ = imu_.getMagZ_uT();
        hzmin_ = imu_.getMagZ_uT();

        counter_ = 0;
        while (counter_ < maxCounts_)
        {
            delta_ = 0.0f;
            framedelta_ = 0.0f;
            imu_.readSensor();
            hxfilt_ = (hxfilt_ * ((float)coeff_ - 1) + (imu_.getMagX_uT() / hxs_ + hxb_)) / ((float)coeff_);
            hyfilt_ = (hyfilt_ * ((float)coeff_ - 1) + (imu_.getMagY_uT() / hys_ + hyb_)) / ((float)coeff_);
            hzfilt_ = (hzfilt_ * ((float)coeff_ - 1) + (imu_.getMagZ_uT() / hzs_ + hzb_)) / ((float)coeff_);
            if (hxfilt_ > hxmax_)
            {
                delta_ = hxfilt_ - hxmax_;
                hxmax_ = hxfilt_;
            }
            if (delta_ > framedelta_)
            {
                framedelta_ = delta_;
            }
            if (hyfilt_ > hymax_)
            {
                delta_ = hyfilt_ - hymax_;
                hymax_ = hyfilt_;
            }
            if (delta_ > framedelta_)
            {
                framedelta_ = delta_;
            }
            if (hzfilt_ > hzmax_)
            {
                delta_ = hzfilt_ - hzmax_;
                hzmax_ = hzfilt_;
            }
            if (delta_ > framedelta_)
            {
                framedelta_ = delta_;
            }
            if (hxfilt_ < hxmin_)
            {
                delta_ = abs(hxfilt_ - hxmin_);
                hxmin_ = hxfilt_;
            }
            if (delta_ > framedelta_)
            {
                framedelta_ = delta_;
            }
            if (hyfilt_ < hymin_)
            {
                delta_ = abs(hyfilt_ - hymin_);
                hymin_ = hyfilt_;
            }
            if (delta_ > framedelta_)
            {
                framedelta_ = delta_;
            }
            if (hzfilt_ < hzmin_)
            {
                delta_ = abs(hzfilt_ - hzmin_);
                hzmin_ = hzfilt_;
            }
            if (delta_ > framedelta_)
            {
                framedelta_ = delta_;
            }
            if (framedelta_ > deltaThresh_)
            {
                counter_ = 0;
            }
            else
            {
                counter_++;
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
    hxb_ = (hxmax_ + hxmin_) / 2.0f;
    hyb_ = (hymax_ + hymin_) / 2.0f;
    hzb_ = (hzmax_ + hzmin_) / 2.0f;

    // find the magnetometer scale factor
    hxs_ = (hxmax_ - hxmin_) / 2.0f;
    hys_ = (hymax_ - hymin_) / 2.0f;
    hzs_ = (hzmax_ - hzmin_) / 2.0f;
    avgs_ = (hxs_ + hys_ + hzs_) / 3.0f;
    hxs_ = avgs_ / hxs_;
    hys_ = avgs_ / hys_;
    hzs_ = avgs_ / hzs_;

    imu_.setMagCalX(hxb_, hxs_);
    imu_.setMagCalY(hyb_, hys_);
    imu_.setMagCalZ(hzb_, hzs_);

    Serial.printf("\nSaving Compass Calibration data.\n\n");
    if (imu_.storeMagCalData(true))
    {
        Serial.printf("EEPROM Storage Success\n");
    }
    else
    {
        Serial.printf("EEPROM Storage FAILED!\n");
    }

    // set the srd back to what it was
    if (imu_.setSrd(srd) < 0)
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
    IMU::AccelRange accelRange = imu_.getAccelRange();
    if (imu_.setAccelRange(imu_.ACCEL_RANGE_2G) < 0)
    {
        return -1;
    }
    IMU::DlpfBandwidth bandwidth = imu_.getDlpfBandwidth();
    if (imu_.setDlpfBandwidth(imu_.DLPF_BANDWIDTH_20HZ) < 0)
    {
        imu_.setAccelRange(accelRange);
        return -2;
    }
    uint8_t srd = imu_.getSrd();
    if (imu_.setSrd(19) < 0)
    {
        imu_.setAccelRange(accelRange);
        imu_.setDlpfBandwidth(bandwidth);
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

    resetAccel();

    for (int i = 0; i < 3; i++)
    {
        accelEnables_[i] = false;
    }
    displayTimer = millis();

    imu_.setAccelCalX(axb_, axs_);
    imu_.setAccelCalY(ayb_, ays_);
    imu_.setAccelCalZ(azb_, azs_);

    bool done = false;
    bool save_accelCal = false;
    while (!done)
    {
        int sampleCount = 0;
        axbD_ = 0;
        aybD_ = 0;
        azbD_ = 0;
        previousAccelAxis_ = accelCurrentAxis_;

        while (!done && (previousAccelAxis_ == accelCurrentAxis_))
        {
            imu_.readSensor();

            ax_ = imu_.getAccelX_mss();
            ay_ = imu_.getAccelY_mss();
            az_ = imu_.getAccelZ_mss();

            if (accelCurrentAxis_ == 0)
            {
                if (accelEnables_[0])
                {
                    sampleCount += 1;
                    axbD_ += (ax_ / axs_ + axb_) / ((double)numSamples_);
                    if (sampleCount == numSamples_)
                    {
                        sampleCount = 0;
                        if (axbD_ > 9.0f && axbD_ > axmax_)
                        {
                            axmax_ = (float)axbD_;
                        }
                        if (axbD_ < -9.0f && axbD_ < axmin_)
                        {
                            axmin_ = (float)axbD_;
                        }
                        axbD_ = 0;
                    }
                }
            }
            else if (accelCurrentAxis_ == 1)
            {
                if (accelEnables_[0])
                {
                    sampleCount += 1;
                    aybD_ += (ay_ / ays_ + ayb_) / ((double)numSamples_);
                    if (sampleCount == numSamples_)
                    {
                        sampleCount = 0;
                        if (aybD_ > 9.0f && aybD_ > aymax_)
                        {
                            aymax_ = (float)aybD_;
                        }
                        if (aybD_ < -9.0f && aybD_ < aymin_)
                        {
                            aymin_ = (float)aybD_;
                        }
                        aybD_ = 0.0;
                    }
                }
            }
            else
            {
                if (accelEnables_[0])
                {
                    sampleCount += 1;
                    azbD_ += (az_ / azs_ + azb_) / ((double)numSamples_);
                    if (sampleCount == numSamples_)
                    {
                        sampleCount = 0;
                        if (azbD_ > 9.0f && azbD_ > azmax_)
                        {
                            azmax_ = (float)azbD_;
                        }
                        if (azbD_ < -9.0f && azbD_ < azmin_)
                        {
                            azmin_ = (float)azbD_;
                        }
                        azbD_ = 0;
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
                    accelEnables_[accelCurrentAxis_] = true;
                    break;

                case 'd':
                    accelEnables_[accelCurrentAxis_] = false;
                    break;

                case 'r':
                    sampleCount = 0;
                    resetAccel();
                    break;

                case ' ':
                    if (++accelCurrentAxis_ == 3)
                    {
                        accelCurrentAxis_ = 0;
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
        if ((abs(axmin_) > 9.0f) && (abs(axmax_) > 9.0f))
        {
            axb_ = (axmin_ + axmax_) / 2.0f;
            axs_ = G_ / ((abs(axmin_) + abs(axmax_)) / 2.0f);
        }
        if ((abs(aymin_) > 9.0f) && (abs(aymax_) > 9.0f))
        {
            ayb_ = (aymin_ + aymax_) / 2.0f;
            ays_ = G_ / ((abs(aymin_) + abs(aymax_)) / 2.0f);
        }
        if ((abs(azmin_) > 9.0f) && (abs(azmax_) > 9.0f))
        {
            azb_ = (azmin_ + azmax_) / 2.0f;
            azs_ = G_ / ((abs(azmin_) + abs(azmax_)) / 2.0f);
        }

        imu_.setAccelCalX(axb_, axs_);
        imu_.setAccelCalY(ayb_, ays_);
        imu_.setAccelCalZ(azb_, azs_);

        Serial.printf("\nSaving  Accelerometer Calibration data.\n\n");
        if (imu_.storeAccelCalData(true))
        {
            Serial.printf("EEPROM Storage Success\n");
        }
        else
        {
            Serial.printf("EEPROM Storage FAILED!\n");
        }
    }
    // set the range, bandwidth, and srd back to what they were
    if (imu_.setAccelRange(accelRange) < 0)
    {
        return -4;
    }
    if (imu_.setDlpfBandwidth(bandwidth) < 0)
    {
        return -5;
    }
    if (imu_.setSrd(srd) < 0)
    {
        return -6;
    }
    return 1;
}

void CalImu::resetAccel()
    {
        accelCurrentAxis_ = 0;
        accelEnables_[0] = false;
        accelEnables_[1] = false;
        accelEnables_[2] = false;
        axbD_ = 0;
        aybD_ = 0;
        azbD_ = 0;
        axmax_ = axmin_ = 0.0f;
        aymax_ = aymin_ = 0.0f;
        azmax_ = azmin_ = 0.0f;
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
    if (hxmin_ != prev_hxmin_ || hxmax_ != prev_hxmax_ ||
        hymin_ != prev_hymin_ || hymax_ != prev_hymax_ ||
        hzmin_ != prev_hzmin_ || hzmax_ != prev_hzmax_)
    {
        prev_hxmin_ = hxmin_;
        prev_hxmax_ = hxmax_;
        prev_hymin_ = hymin_;
        prev_hymax_ = hymax_;
        prev_hzmin_ = hzmin_;
        prev_hzmax_ = hzmax_;

        Serial.printf("\n\n");
        Serial.printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\n", hxmin_, hymin_, hzmin_);
        Serial.printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", hxmax_, hymax_, hzmax_);
        Serial.flush();
    }
}

void CalImu::displayAccelMinMax()
{
    Serial.printf("\n\n");

    Serial.printf("Current axis: ");
    if (accelCurrentAxis_ == 0)
    {
        Serial.printf("x - %s", accelEnables_[0] ? "enabled" : "disabled");
        Serial.printf("\nMin x: %6.2f Max x: %6.2f\n", axmin_, axmax_);
        Serial.printf("x: %6.2f\n", ax_);
    }
    else if (accelCurrentAxis_ == 1)
    {
        Serial.printf("y - %s", accelEnables_[1] ? "enabled" : "disabled");
        Serial.printf("\nMin y: %6.2f Max y: %6.2f\n", aymin_, aymax_);
        Serial.printf("y: %6.2f\n", ay_);
    }
    else if (accelCurrentAxis_ == 2)
    {
        Serial.printf("z - %s", accelEnables_[2] ? "enabled" : "disabled");
        Serial.printf("\nMin z: %6.2f Max z: %6.2f\n", azmin_, azmax_);
        Serial.printf("z: %6.2f\n", az_);
    }

    Serial.flush();
}