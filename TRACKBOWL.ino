// Jacek Fedorynski <jfedor@jfedor.org>
// http://www.jfedor.org/

// based on https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define LED_PIN 13
#define INTERRUPT_PIN 7

MPU6050 mpu;
bool blinkState = false;
uint16_t packetSize;
uint16_t fifoCount;
uint32_t lastCommandTime = 0;
float prevquat[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float rotationZ = 0.0f;
float x = 0.0f;
float y = 0.0f;

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial1.begin(9600);

  pinMode(LED_PIN, OUTPUT);

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
}

// quaternion multiplication
void quatmul(float* result, float* r, float* q) {
  result[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
  result[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
  result[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
  result[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
}

// sends the command to the Bluefruit EZ-Key
void mouseCommand(uint8_t buttons, int8_t x, int8_t y) {
  Serial1.write(0xFD);
  Serial1.write((byte)0x00);
  Serial1.write((byte)0x03);
  Serial1.write(buttons);
  Serial1.write(x);
  Serial1.write(y);
  Serial1.write((byte)0x00);
  Serial1.write((byte)0x00);
  Serial1.write((byte)0x00);
}

void loop() {
  if (!mpuInterrupt && fifoCount < packetSize) {
    return;
  }

  mpuInterrupt = false;
  uint8_t mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  // check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    fifoCount = 0;
    return;
  }
  
  if (mpuIntStatus & 0x02) {
    // make sure there's enough data in the queue
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    uint8_t fifoBuffer[64];
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // there may be more than one packet in the queue
    fifoCount -= packetSize;

    for (int i = 1; i < 4; i++) {
      prevquat[i] *= -1;
    }

    Quaternion q;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    float quat[4] = { q.w, q.x, q.y, q.z };

    // calculate the difference between previous and current quaternion
    float dquat[4];
    quatmul(dquat, prevquat, quat);

    for (int i = 0; i < 4; i++) {
      prevquat[i] = quat[i];
    }

    // convert quaternion to axis-angle
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

    float s = sqrt(1 - dquat[0] * dquat[0]);

    if (s > 0.00001) {
      float angle = 2.0 * acos(dquat[0]);
      // we want the angle between -180 and 180 degrees
      if (angle > PI) {
        angle = PI - angle;
      }

      float rotationX = angle * dquat[1] / s;
      float rotationY = angle * dquat[2] / s;
      rotationZ += angle * dquat[3] / s;

      // rotate around Z axis using the accumulated angle to let the user set the "up" direction
      float sinz = sin(rotationZ);
      float cosz = cos(rotationZ);
      // rotation around X axis moves the cursor along Y axis
      y += rotationX * cosz - rotationY * sinz;
      x += rotationX * sinz + rotationY * cosz;
    }

    // limit reporting to 50 Hz
    if (millis() - lastCommandTime > 20) {
      int8_t int_x = (int8_t) (255 * x);
      int8_t int_y = (int8_t) (255 * y);
      x = 0.0f;
      y = 0.0f;

      if (int_x != 0 || int_y != 0) {
        mouseCommand(0, int_x, int_y);
        lastCommandTime = millis();
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }
    }
  }
}
