
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#include <Adafruit_GPS.h>
#include <PID_v1.h>

//Hardware setup: This library supports communicating with the
//MPU9150 over I2C. These are the only connections that need to be made:
//MPU9150 --------- Arduino
//SCL ---------- SCL (A5 on older 'Duinos')
//SDA ---------- SDA (A4 on older 'Duinos')
//VDD ------------- 3.3V
//GND ------------- GND
//
//The LSM9DS0 has a maximum voltage of 3.5V. Make sure you power it
//off the 3.3V rail! And either use level shifters between SCL
//and SDA or just use a 3.3V Arduino Pro.


// Declare device MPU6050 class
MPU6050 mpu;

uint16_t mcount;
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading
uint8_t MagRate;     // read rate for magnetometer data

float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void setup()
{
	setupMPU();
}

void loop()
{
	getAngles();
	
}

