/*
MPU6050.cpp - Class file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: W.I.P.
(c) 2014 Korneliusz Jarzebski
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
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <math.h>

#include <MPU6050.h>

bool MPU6050::begin(mpu6050_dps_t scale, mpu6050_range_t range)
{
    Wire.begin();

    // Check MPU6050 Who Am I Register
    if (fastRegister8(MPU6050_REG_WHO_AM_I) != 0x68)
    {
	return false;
    }

    // Set Clock Source
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setScale(scale);
    setRange(range);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}

void MPU6050::setScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
	    dpsPerDigit = .007633f;
	    break;
	case MPU6050_SCALE_500DPS:
	    dpsPerDigit = .015267f;
	    break;
	case MPU6050_SCALE_1000DPS:
	    dpsPerDigit = .030487f;
	    break;
	case MPU6050_SCALE_2000DPS:
	    dpsPerDigit = .060975f;
	    break;
	default:
	    break;
    }

    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t MPU6050::getScale(void)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void MPU6050::setRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
	    rangePerDigit = .000061f;
	    break;
	case MPU6050_RANGE_4G:
	    rangePerDigit = .000122f;
	    break;
	case MPU6050_RANGE_8G:
	    rangePerDigit = .000244f;
	    break;
	case MPU6050_RANGE_16G:
	    rangePerDigit = .0004882f;
	    break;
	default:
	    break;
    }

    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t MPU6050::getRange(void)
{
    uint8_t value;
    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
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

bool MPU6050::getSleepEnabled()
{
    return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

void MPU6050::setSleepEnabled(bool state)
{
    writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

Vector MPU6050::readRawAccel()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    #if ARDUINO >= 100
	Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    #else
	Wire.send(MPU6050_REG_ACCEL_XOUT_H);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.requestFrom(MPU6050_ADDRESS, 6);

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

    ra.XAxis = xha << 8 | xla;
    ra.YAxis = yha << 8 | yla;
    ra.ZAxis = zha << 8 | zla;

    return ra;
}

Vector MPU6050::readNormalizeAccel()
{
    readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}

Vector MPU6050::readRawGyro()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    #if ARDUINO >= 100
	Wire.write(MPU6050_REG_GYRO_XOUT_H);
    #else
	Wire.send(MPU6050_REG_GYRO_XOUT_H);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.requestFrom(MPU6050_ADDRESS, 6);

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

    rg.XAxis = xha << 8 | xla;
    rg.YAxis = yha << 8 | yla;
    rg.ZAxis = zha << 8 | zla;

    return rg;
}

Vector MPU6050::readNormalizeGyro()
{
    readRawGyro();

    ng.XAxis = rg.XAxis * dpsPerDigit;
    ng.YAxis = rg.YAxis * dpsPerDigit;
    ng.ZAxis = rg.ZAxis * dpsPerDigit;

    return ng;
}

int16_t MPU6050::getGyroOffsetX(void)
{
    return readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t MPU6050::getGyroOffsetY(void)
{
    return readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t MPU6050::getGyroOffsetZ(void)
{
    return readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

void MPU6050::setGyroOffsetX(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void MPU6050::setGyroOffsetY(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void MPU6050::setGyroOffsetZ(int16_t offset)
{
    writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t MPU6050::getAccelOffsetX(void)
{
    return readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t MPU6050::getAccelOffsetY(void)
{
    return readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t MPU6050::getAccelOffsetZ(void)
{
    return readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void MPU6050::setAccelOffsetX(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void MPU6050::setAccelOffsetY(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void MPU6050::setAccelOffsetZ(int16_t offset)
{
    writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Fast read 8-bit from register
uint8_t MPU6050::fastRegister8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(MPU6050_ADDRESS);
    #if ARDUINO >= 100
	Wire.write(reg);
    #else
	Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    #if ARDUINO >= 100
	value = Wire.read();
    #else
	value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Read 8-bit from register
uint8_t MPU6050::readRegister8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(MPU6050_ADDRESS);
    #if ARDUINO >= 100
	Wire.write(reg);
    #else
	Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
	value = Wire.read();
    #else
	value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Write 8-bit to register
void MPU6050::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDRESS);

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
    Wire.beginTransmission(MPU6050_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.requestFrom(MPU6050_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif;
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void MPU6050::writeRegister16(uint8_t reg, int16_t value)
{
    Wire.beginTransmission(MPU6050_ADDRESS);

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
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    writeRegister8(reg, value);
}
