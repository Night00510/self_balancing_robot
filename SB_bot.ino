#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"

// ตั้งค่าขาให้ TB6612FNG
const byte IN1 = 4, IN2 = 5, PWMA = 3;  // มอเตอร์ซ้าย
const byte IN3 = 6, IN4 = 7, PWMB = 11; // มอเตอร์ขว
const byte STBY = 8;

MPU6050 mpu; // set MPU
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// set PID
double setPoint; // มุมที่เราต้องการให้หุ่นตั้งตรง (องศา)
double input, output;
double Kp = 0, Ki = 0, Kd = 0;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

void moveMotors(double speed);

void setup()
{
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // เปิดการทำงาน Driver

  mpu.initialize();
  if (mpu.dmpInitialize() == 0)
  {
    // ---------- calibate offset --------------
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    // ------------------------------------------

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255); //ให้สอดคล้องกับค่า PWM
    myPID.SetSampleTime(10); // ทำงานทุกๆ 10ms (100Hz)
    dmpReady = true;
  }
}

void loop()
{
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180 / M_PI; //ใช้ค่า Pitch (การก้ม/เงย) เป็น Input
    myPID.Compute();

    moveMotors(output); // สั่งมอเตอร์ตามค่าที่ PID คำนวณได้
  }
}

void moveMotors(double speed)
{
  if (speed > 0) // // เดินหน้า
  {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }else
  {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    speed = -speed; // PID อาจให้ค่าเป็น - ได้ ต้องทำให้เป็น + ถ้าใช้กับ PWM
  }

  analogWrite(PWMA, speed); analogWrite(PWMB, speed);
}