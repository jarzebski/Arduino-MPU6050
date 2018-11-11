/*
MPU6050.cpp - Class file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: 1.0.3
(c) 2014-2015 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

https://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers
*/

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <Wire.h>
#include <math.h>

#include <MPU6050.h>

#define GRAVITY_VALUE 9.80665 // https://en.wikipedia.org/wiki/Gravity_of_Earth



bool MPU6050::begin(mpu6050_dps_t gyroScale, mpu6050_range_t accelRange, int mpuAddress)
{
    // Set Address
    mMpuAddress = mpuAddress;

    Wire.begin();

    // Reset threshold values
    mGyroscopeThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (readRegister8(MPU6050_REG_WHO_AM_I) != 0x68 && readRegister8(MPU6050_REG_WHO_AM_I) != 0x72) // For some reason my device returns 0x72
    {
        return false;
    }

    // Set Clock Source
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setGyroscopeScale(gyroScale);
    setAccelerometerRange(accelRange);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}

void MPU6050::setAccelerometerRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
        case MPU6050_RANGE_2G:
            mRangePerDigit = .000061f;
            break;
        case MPU6050_RANGE_4G:
            mRangePerDigit = .000122f;
            break;
        case MPU6050_RANGE_8G:
            mRangePerDigit = .000244f;
            break;
        case MPU6050_RANGE_16G:
            mRangePerDigit = .0004882f;
            break;
        default:
            break;
    }

    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t MPU6050::getAccelerometerRange(void)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void MPU6050::setGyroscopeScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
        case MPU6050_SCALE_250DPS:
            mDegreesPerDigit = .007633f;
            break;
        case MPU6050_SCALE_500DPS:
            mDegreesPerDigit = .015267f;
            break;
        case MPU6050_SCALE_1000DPS:
            mDegreesPerDigit = .030487f;
            break;
        case MPU6050_SCALE_2000DPS:
            mDegreesPerDigit = .060975f;
            break;
        default:
            break;
    }

    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t MPU6050::getGyroscopeScale(void)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void MPU6050::getPitchAndRoll(Vector normAccel, float *pitch, float *roll)
{
    *pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
    *roll  =  (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
}

// By Henrique Bruno Fantauzzi de Almeida (SrBrahma) - Minerva Rockets, UFRJ, Rio de Janeiro - Brazil
// To calibrate, put your hardware in a stable and straight surface until the end of the process.
// https://forum.arduino.cc/index.php?action=dlattach;topic=418665.0;attach=229569
void MPU6050::calibrateAccelerometer(uint16_t samples)
{
    mpu6050_range_t previousAccelerometerRange = getAccelerometerRange();

    int32_t accelVectorSum[3] = {0, 0, 0}; // Make sure it initializes zeroed!
    uint16_t counter;
    Vector rawAccel;

    int16_t newOffset, previousOffset;

    setAccelerometerRange(MPU6050_RANGE_16G); // The versions below 1.2 of the MPU HW Offset registers say the test should happen
    // on +-8g Range. However, after hours getting results that didn't match the theory, found out the 1.2 version, which as I thought
    // (as I was getting the double of the expected results), corrected to the +-16g range.

    for (counter = 0; counter < samples; counter++)
    {
        rawAccel = readRawAccelerometer();

        accelVectorSum[0] += rawAccel.XAxis;
        accelVectorSum[1] += rawAccel.YAxis;
        accelVectorSum[2] += rawAccel.ZAxis;
    }

    // As this https://forum.arduino.cc/index.php?action=dlattach;topic=418665.0;attach=229569 says,
    // we must preserve the bit 0 of the default offset value, for each vector.
    
    previousOffset = getAccelerometerOffsetX();
    newOffset = previousOffset - (int16_t) (accelVectorSum[0] / counter); // The X vector must be 0 when on the normal position
    newOffset = (newOffset & 0xFFFE) | (previousOffset & 0x0001);         // To keep the previous value of the bit 0, as said in manual.
    setAccelerometerOffsetX(newOffset); 

    previousOffset = getAccelerometerOffsetY();
    newOffset = previousOffset - (int16_t) (accelVectorSum[1] / counter); // The Y vector must be 0 when on the normal position
    newOffset = (newOffset & 0xFFFE) | (previousOffset & 0x0001);         // To keep the previous value of the bit 0, as said in manual.
    setAccelerometerOffsetY(newOffset); 

    previousOffset = getAccelerometerOffsetZ();
    newOffset = previousOffset - (int16_t) ((accelVectorSum[2] / counter) - 2048); // The Z vector must be 1 when on the normal position
    newOffset = (newOffset & 0xFFFE) | (previousOffset & 0x0001);         // To keep the previous value of the bit 0, as said in manual.
    setAccelerometerOffsetZ(newOffset); 

    setAccelerometerRange(previousAccelerometerRange); // Restore previous range
}

// https://forum.arduino.cc/index.php?action=dlattach;topic=418665.0;attach=229569
void MPU6050::calibrateGyroscope(uint16_t samples)
{
    mpu6050_dps_t previousGyroscopeScale = getGyroscopeScale();

    int32_t gyroVectorSum[3] = {0, 0, 0}; // Make sure it initializes zeroed!
    uint16_t counter;
    Vector rawGyro;

    int16_t previousOffset;

    setGyroscopeScale(MPU6050_SCALE_1000DPS); //As seen in the link above, we must set the scale to +-1000dps

    for (counter = 0; counter < samples; counter++)
    {
        rawGyro = readRawGyroscope();

        gyroVectorSum[0] += rawGyro.XAxis;
        gyroVectorSum[1] += rawGyro.YAxis;
        gyroVectorSum[2] += rawGyro.ZAxis;
    }

    previousOffset = getGyroscopeOffsetX();
    setGyroscopeOffsetX(previousOffset - (int16_t) (gyroVectorSum[0] / counter));

    previousOffset = getGyroscopeOffsetY();
    setGyroscopeOffsetY(previousOffset - (int16_t) (gyroVectorSum[1] / counter));

    previousOffset = getGyroscopeOffsetZ();
    setGyroscopeOffsetZ(previousOffset - (int16_t) (gyroVectorSum[2] / counter));

    setGyroscopeScale(previousGyroscopeScale); // Restore previous range
}

Vector MPU6050::readRawAccelerometer(void)
{
    Wire.beginTransmission(mMpuAddress);
    #if ARDUINO >= 100
        Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    #else
        Wire.send(MPU6050_REG_ACCEL_XOUT_H);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mMpuAddress);
    Wire.requestFrom(mMpuAddress, 6);

    while (Wire.available() < 6);

    #if ARDUINO >= 100
        uint8_t xha = Wire.read();
        uint8_t xla = Wire.read();
            uint8_t yha = Wire.read();
        uint8_t yla = Wire.read();
        uint8_t zha = Wire.read();
        uint8_t zla = Wire.read();
    #else
        uint8_t xha = Wire.receive();
        uint8_t xla = Wire.receive();
        uint8_t yha = Wire.receive();
        uint8_t yla = Wire.receive();
        uint8_t zha = Wire.receive();
        uint8_t zla = Wire.receive();
    #endif

    mRawAccelerometer.XAxis = (int16_t) (xha << 8 | xla);
    mRawAccelerometer.YAxis = (int16_t) (yha << 8 | yla);
    mRawAccelerometer.ZAxis = (int16_t) (zha << 8 | zla);

    return mRawAccelerometer;
}

Vector MPU6050::readNormalizedAccelerometer(void)
{
    readRawAccelerometer();

    mNormalizedAccelerometer.XAxis = mRawAccelerometer.XAxis * mRangePerDigit * GRAVITY_VALUE;
    mNormalizedAccelerometer.YAxis = mRawAccelerometer.YAxis * mRangePerDigit * GRAVITY_VALUE;
    mNormalizedAccelerometer.ZAxis = mRawAccelerometer.ZAxis * mRangePerDigit * GRAVITY_VALUE;

    return mNormalizedAccelerometer;
}

Vector MPU6050::readScaledAccelerometer(void)
{
    readRawAccelerometer();

    mNormalizedAccelerometer.XAxis = mRawAccelerometer.XAxis * mRangePerDigit;
    mNormalizedAccelerometer.YAxis = mRawAccelerometer.YAxis * mRangePerDigit;
    mNormalizedAccelerometer.ZAxis = mRawAccelerometer.ZAxis * mRangePerDigit;

    return mNormalizedAccelerometer;
}


Vector MPU6050::readRawGyroscope(void)
{
    Wire.beginTransmission(mMpuAddress);
    #if ARDUINO >= 100
    Wire.write(MPU6050_REG_GYRO_XOUT_H);
    #else
    Wire.send(MPU6050_REG_GYRO_XOUT_H);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mMpuAddress);
    Wire.requestFrom(mMpuAddress, 6);

    while (Wire.available() < 6);

    #if ARDUINO >= 100
        uint8_t xha = Wire.read();
        uint8_t xla = Wire.read();
        uint8_t yha = Wire.read();
        uint8_t yla = Wire.read();
        uint8_t zha = Wire.read();
        uint8_t zla = Wire.read();
    #else
        uint8_t xha = Wire.receive();
        uint8_t xla = Wire.receive();
        uint8_t yha = Wire.receive();
        uint8_t yla = Wire.receive();
        uint8_t zha = Wire.receive();
        uint8_t zla = Wire.receive();
    #endif

    mRawGyroscope.XAxis = (int16_t) (xha << 8 | xla);
    mRawGyroscope.YAxis = (int16_t) (yha << 8 | yla);
    mRawGyroscope.ZAxis = (int16_t) (zha << 8 | zla);

    return mRawGyroscope;
}

Vector MPU6050::readNormalizedGyroscope(void)
{
    readRawGyroscope();

    if (mGyroscopeThreshold)
    {
        if (abs(mRawGyroscope.XAxis) < mGyroscopeThreshold)
            mRawGyroscope.XAxis = 0;
        if (abs(mRawGyroscope.YAxis) < mGyroscopeThreshold)
            mRawGyroscope.YAxis = 0;
        if (abs(mRawGyroscope.ZAxis) < mGyroscopeThreshold)
            mRawGyroscope.ZAxis = 0;
    }

    mNormalizedGyroscope.XAxis = mRawGyroscope.XAxis * mDegreesPerDigit;
    mNormalizedGyroscope.YAxis = mRawGyroscope.YAxis * mDegreesPerDigit;
    mNormalizedGyroscope.ZAxis = mRawGyroscope.ZAxis * mDegreesPerDigit;

    

    return mNormalizedGyroscope;
}

void MPU6050::setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

void MPU6050::setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    writeRegister8(MPU6050_REG_CONFIG, value);
}

void MPU6050::setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

mpu6050_clockSource_t MPU6050::getClockSource(void)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t)value;
}

bool MPU6050::getSleepEnabled(void)
{
    return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

void MPU6050::setSleepEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

bool MPU6050::getIntZeroMotionEnabled(void)
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 5);
}

void MPU6050::setIntZeroMotionEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 5, state);
}

bool MPU6050::getIntMotionEnabled(void)
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 6);
}

void MPU6050::setIntMotionEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 6, state);
}

bool MPU6050::getIntFreeFallEnabled(void)
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 7);
}

void MPU6050::setIntFreeFallEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 7, state);
}

uint8_t MPU6050::getMotionDetectionThreshold(void)
{
    return readRegister8(MPU6050_REG_MOT_THRESHOLD);
}

void MPU6050::setMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6050_REG_MOT_THRESHOLD, threshold);
}

uint8_t MPU6050::getMotionDetectionDuration(void)
{
    return readRegister8(MPU6050_REG_MOT_DURATION);
}

void MPU6050::setMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6050_REG_MOT_DURATION, duration);
}

uint8_t MPU6050::getZeroMotionDetectionThreshold(void)
{
    return readRegister8(MPU6050_REG_ZMOT_THRESHOLD);
}

void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6050_REG_ZMOT_THRESHOLD, threshold);
}

uint8_t MPU6050::getZeroMotionDetectionDuration(void)
{
    return readRegister8(MPU6050_REG_ZMOT_DURATION);
}

void MPU6050::setZeroMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6050_REG_ZMOT_DURATION, duration);
}

uint8_t MPU6050::getFreeFallDetectionThreshold(void)
{
    return readRegister8(MPU6050_REG_FF_THRESHOLD);
}

void MPU6050::setFreeFallDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6050_REG_FF_THRESHOLD, threshold);
}

uint8_t MPU6050::getFreeFallDetectionDuration(void)
{
    return readRegister8(MPU6050_REG_FF_DURATION);
}

void MPU6050::setFreeFallDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6050_REG_FF_DURATION, duration);
}

bool MPU6050::getI2CMasterModeEnabled(void)
{
    return readRegisterBit(MPU6050_REG_USER_CTRL, 5);
}

void MPU6050::setI2CMasterModeEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_USER_CTRL, 5, state);
}

void MPU6050::setI2CBypassEnabled(bool state)
{
    return writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

bool MPU6050::getI2CBypassEnabled(void)
{
    return readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

void MPU6050::setAccelerometerPowerOnDelay(mpu6050_onDelay_t delay)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    writeRegister8(MPU6050_REG_MOT_DETECT_CTRL, value);
}

mpu6050_onDelay_t MPU6050::getAccelerometerPowerOnDelay(void)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b00110000;
    return (mpu6050_onDelay_t)(value >> 4);
}

uint8_t MPU6050::getIntStatus(void)
{
    return readRegister8(MPU6050_REG_INT_STATUS);
}

Activities MPU6050::readActivites(void)
{
    uint8_t data = readRegister8(MPU6050_REG_INT_STATUS);

    mActivities.isOverflow =   ((data >> 4) & 1);
    mActivities.isFreeFall =   ((data >> 7) & 1);
    mActivities.isInactivity = ((data >> 5) & 1);
    mActivities.isActivity =   ((data >> 6) & 1);
    mActivities.isDataReady =  ((data >> 0) & 1);

    data = readRegister8(MPU6050_REG_MOT_DETECT_STATUS);

    mActivities.isNegActivityOnX = ((data >> 7) & 1);
    mActivities.isPosActivityOnX = ((data >> 6) & 1);

    mActivities.isNegActivityOnY = ((data >> 5) & 1);
    mActivities.isPosActivityOnY = ((data >> 4) & 1);

    mActivities.isNegActivityOnZ = ((data >> 3) & 1);
    mActivities.isPosActivityOnZ = ((data >> 2) & 1);

    return mActivities;
}



float MPU6050::readTemperature(void)
{
    int16_t T;
    T = readRegister16(MPU6050_REG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

int16_t MPU6050::getGyroscopeOffsetX(void)
{
    return readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t MPU6050::getGyroscopeOffsetY(void)
{
    return readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t MPU6050::getGyroscopeOffsetZ(void)
{
    return readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

void MPU6050::setGyroscopeOffsetX(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void MPU6050::setGyroscopeOffsetY(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void MPU6050::setGyroscopeOffsetZ(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t MPU6050::getAccelerometerOffsetX(void)
{
    return readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t MPU6050::getAccelerometerOffsetY(void)
{
    return readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t MPU6050::getAccelerometerOffsetZ(void)
{
    return readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void MPU6050::setAccelerometerOffsetX(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void MPU6050::setAccelerometerOffsetY(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void MPU6050::setAccelerometerOffsetZ(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Get current threshold value
float MPU6050::getGyroscopeThreshold(void)
{
    return mGyroscopeThreshold;
}

// Set treshold value
void MPU6050::setGyroscopeThreshold(float percentOfMaximumValue) // Argument is in %. So, 1.5 = 1.5%.
{
    if (percentOfMaximumValue > 0)
        mGyroscopeThreshold = (32767 / 100.0) * percentOfMaximumValue;
    else
        mGyroscopeThreshold = 0;
}

// Fast read 8-bit from register
uint8_t MPU6050::fastRegister8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(mMpuAddress);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif

    Wire.endTransmission();

    Wire.beginTransmission(mMpuAddress);
    Wire.requestFrom(mMpuAddress, 1);

    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif

    Wire.endTransmission();

    return value;
}

// Read 8-bit from register
uint8_t MPU6050::readRegister8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(mMpuAddress);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mMpuAddress);
    Wire.requestFrom(mMpuAddress, 1);

    while(!Wire.available()) {};

    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif

    Wire.endTransmission();

    return value;
}

// Write 8-bit to register
void MPU6050::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(mMpuAddress);

    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif

    Wire.endTransmission();
}

int16_t MPU6050::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(mMpuAddress);

    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif

    Wire.endTransmission();

    Wire.beginTransmission(mMpuAddress);
    Wire.requestFrom(mMpuAddress, 2);

    while(!Wire.available()) {};

    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif

    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void MPU6050::writeRegister16(uint8_t reg, int16_t value)
{
    Wire.beginTransmission(mMpuAddress);

    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write((uint8_t)(value >> 8));
        Wire.write((uint8_t)value);
    #else
        Wire.send(reg);
        Wire.send((uint8_t)(value >> 8));
        Wire.send((uint8_t)value);
    #endif

    Wire.endTransmission();
}

// Read register bit
bool MPU6050::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MPU6050::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
        value |= (1 << pos);
    else 
        value &= ~(1 << pos);

    writeRegister8(reg, value);
}

void MPU6050::resetDevice(mpu6050_dps_t gyroScale, mpu6050_range_t accelRange, int mpuAddress)
{
    writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 7, 1); // as seen in the register map
    delay(100);

    // Reset threshold values
    mGyroscopeThreshold = 0;

    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    setGyroscopeScale(gyroScale);
    setAccelerometerRange(accelRange);
    
    // Disable Sleep Mode
    setSleepEnabled(false);
}