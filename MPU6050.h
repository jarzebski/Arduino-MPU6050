/*
MPU6050.h - Header file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: W.I.P
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

#ifndef MPU6050_h
#define MPU6050_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MPU6050_ADDRESS           (0x68) // 0x69 when AD0 pin to Vcc

#define MPU6050_REG_ACCEL_XOFFS_H (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H  (0x13)
#define MPU6050_REG_GYRO_XOFFS_L  (0x14)
#define MPU6050_REG_GYRO_YOFFS_H  (0x15)
#define MPU6050_REG_GYRO_YOFFS_L  (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H  (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L  (0x18)
#define MPU6050_REG_GYRO_CONFIG   (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG  (0x1C) // Accelerometer Configuration
#define MPU6050_REG_INT_PIN_CFG   (0x37) // INT Pin. Bypass Enable Configuration
#define MPU6050_REG_ACCEL_XOUT_H  (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L  (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H  (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L  (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H  (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L  (0x40)
#define MPU6050_REG_TEMP_OUT_H    (0x41)
#define MPU6050_REG_TEMP_OUT_L    (0x42)
#define MPU6050_REG_GYRO_XOUT_H   (0x43)
#define MPU6050_REG_GYRO_XOUT_L   (0x44)
#define MPU6050_REG_GYRO_YOUT_H   (0x45)
#define MPU6050_REG_GYRO_YOUT_L   (0x46)
#define MPU6050_REG_GYRO_ZOUT_H   (0x47)
#define MPU6050_REG_GYRO_ZOUT_L   (0x48)
#define MPU6050_REG_USER_CTRL     (0x6A) // User Control
#define MPU6050_REG_PWR_MGMT_1    (0x6B) // Power Management 1
#define MPU6050_REG_WHO_AM_I      (0x75) // Who Am I

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

class MPU6050
{
    public:

	bool begin(mpu6050_dps_t scale = MPU6050_SCALE_2000DPS, mpu6050_range_t = MPU6050_RANGE_2G);

	void setClockSource(mpu6050_clockSource_t source);
	void setScale(mpu6050_dps_t scale);
	void setRange(mpu6050_range_t range);
	mpu6050_clockSource_t getClockSource(void);
	mpu6050_dps_t getScale(void);
	mpu6050_range_t getRange(void);

	bool getSleepEnabled(void);
	void setSleepEnabled(bool state);
	bool getI2CMasterModeEnabled(void);
	void setI2CMasterModeEnabled(bool state);
	bool getI2CBypassEnabled(void);
	void setI2CBypassEnabled(bool state);

	float readTemperature(void);

	int16_t getGyroOffsetX(void);
	void setGyroOffsetX(int16_t offset);
	int16_t getGyroOffsetY(void);
	void setGyroOffsetY(int16_t offset);
	int16_t getGyroOffsetZ(void);
	void setGyroOffsetZ(int16_t offset);

	int16_t getAccelOffsetX(void);
	void setAccelOffsetX(int16_t offset);
	int16_t getAccelOffsetY(void);
	void setAccelOffsetY(int16_t offset);
	int16_t getAccelOffsetZ(void);
	void setAccelOffsetZ(int16_t offset);

	void calibrateGyro(uint8_t samples = 50);
	void setThreshold(uint8_t multiple = 1);
	uint8_t getThreshold(void);

	Vector readRawGyro(void);
	Vector readNormalizeGyro(void);

	Vector readRawAccel(void);
	Vector readNormalizeAccel(void);

    private:
	Vector ra, rg; // Raw vectors
	Vector na, ng; // Normalized vectors
	Vector tg, dg; // Threshold and Delta for Gyro
	Vector th;     // Threshold
	
	float dpsPerDigit, rangePerDigit;
	float actualThreshold;
	bool useCalibrate;

	uint8_t fastRegister8(uint8_t reg);

	uint8_t readRegister8(uint8_t reg);
	void writeRegister8(uint8_t reg, uint8_t value);

	int16_t readRegister16(uint8_t reg);
	void writeRegister16(uint8_t reg, int16_t value);

	bool readRegisterBit(uint8_t reg, uint8_t pos);
	void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);

};

#endif
