//MPU
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include "HardwareSerial.h"

MPU6050 mpu;
MPU6050 accelgyro;
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float gyroX_filt, gyroY_filt, gyroZ_filt;

float last_pitch     = 0;
float last_roll      = 0;
float last_yaw       = 0;

double rad_yaw, rad_pitch, rad_roll;
double roll_kalman, pitch_kalman, yaw_kalman;
double accum_roll  = 0;
double accum_pitch = 0;
double k_acc = 0;
double k_gps = 0;
double yaw_deg;
double pitch_deg;
double roll_deg;
double pitch_deg_previous, roll_deg_previous;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_ACCELGYRO

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
boolean interruptLock = false;

void dmpDataReady(){ mpuInterrupt = true; }


void init_MPU() {
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-5);
  mpu.setYGyroOffset(32);
  mpu.setZGyroOffset(-4);
  mpu.setXAccelOffset(1382);
  mpu.setYAccelOffset(1828);
  mpu.setZAccelOffset(702);
  if (devStatus == 0) {

    mpu.setDMPEnabled(true);
    attachInterrupt(17, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(false);
  mpu.setSleepEnabled(false);
}

void get_YPR() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyroX_filt = gyroX_filt * 0.93 + gx * 0.07;
  gyroY_filt = (gyroY_filt * 0.93 + gy * 0.07) * -1;
  gyroZ_filt = gyroZ_filt * 0.93 + gz * 0.07;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pitch_deg = (ypr[1] * 180 / M_PI);
    roll_deg  = (ypr[2] * 180 / M_PI);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    //      accel_z = aaWorld.z/1000.0;
  }
}
HardwareSerial Serial2(PD6, PD5); //RX TX
void setup() {
    Wire.begin();
    Serial2.begin(115200);
    init_MPU();

}

void loop() {
  get_YPR();
  Serial2.print(roll_deg)       ;Serial2.print("\t");
  Serial2.print(pitch_deg)      ;Serial2.print("\t");
  Serial2.println();
//si
}
