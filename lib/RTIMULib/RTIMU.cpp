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

#include "RTIMU.h"
#include "RTIMUHal.h"
#include "RTIMUSettings.h"
#include "RTFusion.h"
#include "RTFusionKalman4.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA (RTFLOAT)0.2

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_GYRO_ZERO (RTFLOAT)0.20

#define RTIMU_FUZZY_GYRO_ZERO_SQUARED (RTIMU_FUZZY_GYRO_ZERO * RTIMU_FUZZY_GYRO_ZERO)

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_ACCEL_ZERO (RTFLOAT)0.05

#define RTIMU_FUZZY_ACCEL_ZERO_SQUARED (RTIMU_FUZZY_ACCEL_ZERO * RTIMU_FUZZY_ACCEL_ZERO)

//  Axis rotation array

#ifdef RTIMU_XNORTH_YEAST
RTFLOAT RTIMU::m_axisRotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
#endif

#ifdef RTIMU_XEAST_YSOUTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
#endif

#ifdef RTIMU_XSOUTH_YWEST
RTFLOAT RTIMU::m_axisRotation[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
#endif

#ifdef RTIMU_XWEST_YNORTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
#endif

#ifdef RTIMU_XNORTH_YWEST
RTFLOAT RTIMU::m_axisRotation[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};
#endif

#ifdef RTIMU_XEAST_YNORTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, 1, 0, 1, 0, 0, 0, 0, -1};
#endif

#ifdef RTIMU_XSOUTH_YEAST
RTFLOAT RTIMU::m_axisRotation[9] = {-1, 0, 0, 0, 1, 0, 0, 0, -1};
#endif

#ifdef RTIMU_XWEST_YSOUTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, -1, 0, -1, 0, 0, 0, 0, -1};
#endif

#ifdef RTIMU_XUP_YNORTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, 1, 0, 0, 0, -1, -1, 0, 0};
#endif

#ifdef RTIMU_XUP_YEAST
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, 1, 0, 1, 0, -1, 0, 0};
#endif

#ifdef RTIMU_XUP_YSOUTH
{0, -1, 0, 0, 0, 1, -1, 0, 0};
#endif

#ifdef RTIMU_XUP_YWEST
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, -1, 0, -1, 0, -1, 0, 0};
#endif

#ifdef RTIMU_XDOWN_YNORTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};
#endif

#ifdef RTIMU_XDOWN_YEAST
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, -1, 0, 1, 0, 1, 0, 0};
#endif

#ifdef RTIMU_XDOWN_YSOUTH
RTFLOAT RTIMU::m_axisRotation[9] = {0, -1, 0, 0, 0, -1, 1, 0, 0};
#endif

#ifdef RTIMU_XDOWN_YWEST
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, 1, 0, -1, 0, 1, 0, 0};
#endif

#ifdef RTIMU_XNORTH_YUP
RTFLOAT RTIMU::m_axisRotation[9] = {1, 0, 0, 0, 0, 1, 0, -1, 0};
#endif

#ifdef RTIMU_XEAST_YUP
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, -1, 1, 0, 0, 0, -1, 0};
#endif

#ifdef RTIMU_XSOUTH_YUP
RTFLOAT RTIMU::m_axisRotation[9] = {-1, 0, 0, 0, 0, -1, 0, -1, 0};
#endif

#ifdef RTIMU_XWEST_YUP
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, 1, -1, 0, 0, 0, -1, 0};
#endif

#ifdef RTIMU_XNORTH_YDOWN
RTFLOAT RTIMU::m_axisRotation[9] = {1, 0, 0, 0, 0, -1, 0, 1, 0};
#endif

#ifdef RTIMU_XEAST_YDOWN
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};
#endif

#ifdef RTIMU_XSOUTH_YDOWN
RTFLOAT RTIMU::m_axisRotation[9] = {-1, 0, 0, 0, 0, 1, 0, 1, 0};
#endif

#ifdef RTIMU_XWEST_YDOWN
RTFLOAT RTIMU::m_axisRotation[9] = {0, 0, -1, -1, 0, 0, 0, 1, 0};
#endif

#if defined(MPU9150_68) || defined(MPU9150_69)
#include "RTIMUMPU9150.h"
#endif

#if defined(MPU9250_68) || defined(MPU9250_69)
#include "RTIMUMPU9250.h"
#endif

#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)
#include "RTIMULSM9DS0.h"
#endif

#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)
#include "RTIMUGD20HM303D.h"
#endif

#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)
#include "RTIMUGD20M303DLHC.h"
#endif

#if defined(GD20HM303DLHC_6a) || defined(GD20HM303DLHC_6b)
#include "RTIMUGD20HM303DLHC.h"
#endif

#if defined(BNO055_28) || defined(BNO055_29)
#include "RTIMUBNO055.h"
#endif

RTIMU *RTIMU::createIMU(RTIMUSettings& settings)
{
#if defined(MPU9150_68) || defined(MPU9150_69)
    return new RTIMUMPU9150(settings);
#endif
#if defined(MPU9250_68) || defined(MPU9250_69)
    return new RTIMUMPU9250(settings);
#endif
#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)
    return new RTIMULSM9DS0(settings);
#endif
#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)
    return new RTIMUGD20HM303D(settings);
#endif
#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)
    return new RTIMUGD20M303DLHC(settings);
#endif
#if defined(GD20HM303DLHC_6a) || defined(GD20HM303DLHC_6b)
    return new RTIMUGD20HM303DLHC(settings);
#endif
#if defined(BNO055_28) || defined(BNO055_29)
    return new RTIMUBNO055(settings);
#endif
}

RTIMU::RTIMU(RTIMUSettings& settings) : m_settings(settings), m_calibrationMode(false),
    m_calibrationValid(false),
    m_gyroBiasValid(false)
{
    for (int i = 0; i < 3; i++) {
        m_runtimeMagCalMax[i] = -1000;
        m_runtimeMagCalMin[i] = 1000;
    }

    switch (m_settings.m_fusionType) {
    case RTFUSION_TYPE_KALMANSTATE4:
        m_fusion = new RTFusionKalman4();
        break;

    case RTFUSION_TYPE_RTQF:
        m_fusion = new RTFusionRTQF();
        break;

    default:
        m_fusion = new RTFusion();
        break;
    }
    HAL_INFO1("Using fusion algorithm %s\n", RTFusion::fusionName(m_settings.m_fusionType));
}

RTIMU::~RTIMU()
{
    delete m_fusion;
    m_fusion = NULL;
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;
    if (m_settings.m_compassCalValid)
    {
        //  find biggest range

        for (int i = 0; i < 3; i++)
        {
            if ((m_settings.m_compassCalMax.data(i) - m_settings.m_compassCalMin.data(i)) > maxDelta)
                maxDelta = m_settings.m_compassCalMax.data(i) - m_settings.m_compassCalMin.data(i);
        }
        if (maxDelta < 0)
        {
            HAL_ERROR("Error in compass calibration data\n");
            return;
        }
        maxDelta /= 2.0f; // this is the max +/- range

        for (int i = 0; i < 3; i++)
        {
            delta = (m_settings.m_compassCalMax.data(i) - m_settings.m_compassCalMin.data(i)) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta; // makes everything the same range
            m_compassCalOffset[i] = (m_settings.m_compassCalMax.data(i) + m_settings.m_compassCalMin.data(i)) / 2.0f;
        }
    }

    if (m_settings.m_compassCalValid)
    {
        HAL_INFO("Using min/max compass calibration\n");
    }
    else
    {
        HAL_INFO("min/max compass calibration not in use\n");
    }

    if (m_settings.m_compassCalEllipsoidValid)
    {
        HAL_INFO("Using ellipsoid compass calibration\n");
    }
    else
    {
        HAL_INFO("Ellipsoid compass calibration not in use\n");
    }

    if (m_settings.m_accelCalValid)
    {
        HAL_INFO("Using accel calibration\n");
    }
    else
    {
        HAL_INFO("Accel calibration not in use\n");
    }
}

void RTIMU::gyroBiasInit()
{
    m_gyroAlpha = 2.0f / m_sampleRate;
    m_gyroSampleCount = 0;
}

//  Note - code assumes that this is the first thing called after axis swapping
//  for each specific IMU chip has occurred.

void RTIMU::handleGyroBias()
{
    // do axis rotation if necessary
#ifndef RTIMU_XNORTH_YEAST
    // need to do an axis rotation
    float *matrix = m_axisRotation;
    RTVector3 tempGyro = m_gyro;
    RTVector3 tempAccel = m_accel;
    RTVector3 tempCompass = m_compass;

    // do new x value
    if (matrix[0] != 0)
    {
        m_gyro.setX(tempGyro.x() * matrix[0]);
        m_accel.setX(tempAccel.x() * matrix[0]);
        m_compass.setX(tempCompass.x() * matrix[0]);
    }
    else if (matrix[1] != 0)
    {
        m_gyro.setX(tempGyro.y() * matrix[1]);
        m_accel.setX(tempAccel.y() * matrix[1]);
        m_compass.setX(tempCompass.y() * matrix[1]);
    }
    else if (matrix[2] != 0)
    {
        m_gyro.setX(tempGyro.z() * matrix[2]);
        m_accel.setX(tempAccel.z() * matrix[2]);
        m_compass.setX(tempCompass.z() * matrix[2]);
    }

    // do new y value
    if (matrix[3] != 0)
    {
        m_gyro.setY(tempGyro.x() * matrix[3]);
        m_accel.setY(tempAccel.x() * matrix[3]);
        m_compass.setY(tempCompass.x() * matrix[3]);
    }
    else if (matrix[4] != 0)
    {
        m_gyro.setY(tempGyro.y() * matrix[4]);
        m_accel.setY(tempAccel.y() * matrix[4]);
        m_compass.setY(tempCompass.y() * matrix[4]);
    }
    else if (matrix[5] != 0)
    {
        m_gyro.setY(tempGyro.z() * matrix[5]);
        m_accel.setY(tempAccel.z() * matrix[5]);
        m_compass.setY(tempCompass.z() * matrix[5]);
    }

    // do new z value
    if (matrix[6] != 0)
    {
        m_gyro.setZ(tempGyro.x() * matrix[6]);
        m_accel.setZ(tempAccel.x() * matrix[6]);
        m_compass.setZ(tempCompass.x() * matrix[6]);
    }
    else if (matrix[7] != 0)
    {
        m_gyro.setZ(tempGyro.y() * matrix[7]);
        m_accel.setZ(tempAccel.y() * matrix[7]);
        m_compass.setZ(tempCompass.y() * matrix[7]);
    }
    else if (matrix[8] != 0)
    {
        m_gyro.setZ(tempGyro.z() * matrix[8]);
        m_accel.setZ(tempAccel.z() * matrix[8]);
        m_compass.setZ(tempCompass.z() * matrix[8]);
    }
#endif

    if (!m_gyroBiasValid)
    {
        RTVector3 deltaAccel = m_previousAccel;
        deltaAccel -= m_imuData.accel; // compute difference
        m_previousAccel = m_imuData.accel;

        if ((deltaAccel.squareLength() < RTIMU_FUZZY_ACCEL_ZERO_SQUARED) &&
            (m_imuData.gyro.squareLength() < RTIMU_FUZZY_GYRO_ZERO_SQUARED))
        {
            // what we are seeing on the gyros should be bias only so learn from this
            m_settings.m_gyroBias.setX((1.0 - m_gyroAlpha) * m_settings.m_gyroBias.x() + m_gyroAlpha * m_imuData.gyro.x());
            m_settings.m_gyroBias.setY((1.0 - m_gyroAlpha) * m_settings.m_gyroBias.y() + m_gyroAlpha * m_imuData.gyro.y());
            m_settings.m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_settings.m_gyroBias.z() + m_gyroAlpha * m_imuData.gyro.z());

            if (m_gyroSampleCount < (5 * m_sampleRate))
            {
                m_gyroSampleCount++;

                if (m_gyroSampleCount == (5 * m_sampleRate))
                {
                    m_gyroBiasValid = true;
                }
            }
        }
    }

    m_imuData.gyro -= m_settings.m_gyroBias;
}

void RTIMU::calibrateAverageCompass()
{
    //  calibrate if required

    if (!m_calibrationMode && m_calibrationValid)
    {
        m_imuData.compass.setX((m_imuData.compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_imuData.compass.setY((m_imuData.compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_imuData.compass.setZ((m_imuData.compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);
    }

    //  update running average

    m_compassAverage.setX(m_imuData.compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_imuData.compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_imuData.compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));

    m_imuData.compass = m_compassAverage;
}

void RTIMU::calibrateAccel()
{
    if (!getAccelCalibrationValid())
        return;

    if (m_imuData.accel.x() >= 0)
        m_imuData.accel.setX(m_imuData.accel.x() / m_settings.m_accelCalMax.x());
    else
        m_imuData.accel.setX(m_imuData.accel.x() / -m_settings.m_accelCalMin.x());

    if (m_imuData.accel.y() >= 0)
        m_imuData.accel.setY(m_imuData.accel.y() / m_settings.m_accelCalMax.y());
    else
        m_imuData.accel.setY(m_imuData.accel.y() / -m_settings.m_accelCalMin.y());

    if (m_imuData.accel.z() >= 0)
        m_imuData.accel.setZ(m_imuData.accel.z() / m_settings.m_accelCalMax.z());
    else
        m_imuData.accel.setZ(m_imuData.accel.z() / -m_settings.m_accelCalMin.z());
}

void RTIMU::updateFusion()
{
    m_fusion->newIMUData(m_imuData, m_settings);
}

bool RTIMU::IMUGyroBiasValid()
{
    return m_gyroBiasValid;
}
