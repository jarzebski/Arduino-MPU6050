/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski

    This version was made by Henrique Bruno Fantauzzi de Almeida (SrBrahma) - Minerva Rockets, UFRJ, Rio de Janeiro - Brazil

    Will first calibrate the accelerometer and the gyroscope, than will get its normalized values and the angles.
    It's much better now.
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define BASIC_FILTER_SAMPLES    10

unsigned long timer;

// Pitch, Roll and Yaw values
float pitchByGyroscope = 0, rollByGyroscope = 0, yawByGyroscope = 0;
float pitchByAccelerometer, rollByAccelerometer;

int counterBasicFilter  = 0, counterPrint = 0;

Vector normAccel, newAccel;
Vector normGyro,  newGyro;

void setup() 
{
    Serial.begin(115200);

    while (!Serial); // Wait the Serial Monitor

    Serial.println("Initialize MPU6050");

    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }
    
    mpu.resetDevice(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G); // remove any previous settings on the register from a previous session

    // Calibrate Accelerometer
    Serial.print("Offsets of the accelerometer previously were (x, y, z): ");
    Serial.print(mpu.getAccelerometerOffsetX());
    Serial.print(", ");
    Serial.print(mpu.getAccelerometerOffsetY());
    Serial.print(", ");
    Serial.print(mpu.getAccelerometerOffsetZ());
    Serial.print("\n\n");

    mpu.calibrateAccelerometer();

    Serial.print("Offsets of the accelerometer now are (x, y, z): ");
    Serial.print(mpu.getAccelerometerOffsetX());
    Serial.print(", ");
    Serial.print(mpu.getAccelerometerOffsetY());
    Serial.print(", ");
    Serial.print(mpu.getAccelerometerOffsetZ());
    Serial.print("\n\n");



    // Calibrate Gyroscope
    Serial.print("Offsets of the gyroscope previously were (x, y, z): ");
    Serial.print(mpu.getGyroscopeOffsetX());
    Serial.print(", ");
    Serial.print(mpu.getGyroscopeOffsetY());
    Serial.print(", ");
    Serial.print(mpu.getGyroscopeOffsetZ());
    Serial.print("\n\n");

    mpu.calibrateGyroscope();

    Serial.print("Offsets of the gyroscope now are (x, y, z): ");
    Serial.print(mpu.getGyroscopeOffsetX());
    Serial.print(", ");
    Serial.print(mpu.getGyroscopeOffsetY());
    Serial.print(", ");
    Serial.print(mpu.getGyroscopeOffsetZ());
    Serial.print("\n\n");

    mpu.setGyroscopeThreshold(0.5);

    Serial.println("Setup ended. Press any key to continue to the main loop.");
    while(!Serial.available());

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    //mpu.setGyroscopeThreshold(0);

    timer = millis();
}

void loop()
{

    normAccel.XAxis = 0;
    normAccel.YAxis = 0;
    normAccel.ZAxis = 0;

    normGyro.XAxis = 0;
    normGyro.YAxis = 0;
    normGyro.ZAxis = 0;

    // A simple filter.
    for(counterBasicFilter = 0; counterBasicFilter < BASIC_FILTER_SAMPLES; counterBasicFilter++)
    {
        newAccel = mpu.readNormalizedAccelerometer();
        normAccel.XAxis += newAccel.XAxis;
        normAccel.YAxis += newAccel.YAxis;
        normAccel.ZAxis += newAccel.ZAxis;

        newGyro = mpu.readNormalizedGyroscope();
        normGyro.XAxis += newGyro.XAxis;
        normGyro.YAxis += newGyro.YAxis;
        normGyro.ZAxis += newGyro.ZAxis;
    }

    normAccel.XAxis /= BASIC_FILTER_SAMPLES;
    normAccel.YAxis /= BASIC_FILTER_SAMPLES;
    normAccel.ZAxis /= BASIC_FILTER_SAMPLES;

    normGyro.XAxis /= BASIC_FILTER_SAMPLES;
    normGyro.YAxis /= BASIC_FILTER_SAMPLES;
    normGyro.ZAxis /= BASIC_FILTER_SAMPLES;

    // Output
    mpu.getPitchAndRoll(normAccel, &pitchByAccelerometer, &rollByAccelerometer);

    // Read normalized values
    normGyro = mpu.readNormalizedGyroscope();

    

    // Calculate Pitch, Roll and Yaw
    pitchByGyroscope = pitchByGyroscope + (normGyro.YAxis * (millis() - timer)) / 1000.0;
    rollByGyroscope  = rollByGyroscope  + (normGyro.XAxis * (millis() - timer)) / 1000.0;
    yawByGyroscope   = yawByGyroscope   + (normGyro.ZAxis * (millis() - timer)) / 1000.0;
    timer = millis();

    counterPrint++;

    if (counterPrint % 3 == 0) // Less time spent writing to the Serial, also, makes the reading easier. Change it to fit your needs.
    {
        Serial.print("AccelX = ");
        Serial.print(normAccel.XAxis);
        Serial.print(" | AccelY = ");
        Serial.print(normAccel.YAxis);
        Serial.print(" | AccelZ = ");
        Serial.println(normAccel.ZAxis);
        Serial.print(" | AccelPitch = ");
        Serial.print(pitchByAccelerometer);
        Serial.print(" | AccelRoll = ");
        Serial.println(rollByAccelerometer);

        Serial.print("GyrosX = ");
        Serial.print(normGyro.XAxis);
        Serial.print(" | GyrosY = ");
        Serial.print(normGyro.YAxis);
        Serial.print(" | GyrosZ = ");
        Serial.print(normGyro.ZAxis);
        Serial.print(" | GyrosPitch = ");
        Serial.print(pitchByGyroscope);
        Serial.print(" | GyrosRoll = ");
        Serial.print(rollByGyroscope);  
        Serial.print(" | GyrosYaw = ");
        Serial.println(yawByGyroscope);
        Serial.println();
        counterPrint = 0;
    }

}