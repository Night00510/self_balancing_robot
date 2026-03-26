// โปรเจกต์รถทรงตัว (Self-Balancing Robot) ใช้ Arduino Uno R3
// ควบคุมผ่าน nRF24L01 และเซนเซอร์ MPU6050 (DMP Mode)

#include <I2Cdev.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

// --- โครงสร้างข้อมูลที่รับจาก Remote (ต้องตรงกับตัวส่ง) ---
struct from_Remote_Data {
  byte direction = 6; // ทิศทาง 0=หน้า, 1=ขวา, 2=ซ้าย, 3=หลัง, 6=หยุด
  byte speed = 0;     // ความแรงจากการโยกจอยสติ๊ก (0-255)
};
from_Remote_Data myData;

// --- กำหนดขาเชื่อมต่อ Motor Driver TB6612FNG ---
const int AIN1 = 2;  const int AIN2 = 4;  const int PWMA = 3; // มอเตอร์ A (ซ้าย)
const int BIN1 = 7;  const int BIN2 = 8;  const int PWMB = 6; // มอเตอร์ B (ขวา)
// ต่อ stanby กับ VCC ทิ้งไว้เลย 

// --- ตัวแปรสำหรับระบบ PID Control ---
double setpoint = 0;        // มุมเป้าหมายที่รถต้องรักษาไว้ (จะเปลี่ยนตามการบังคับ)
double originalSetpoint = 0; // จุดสมดุลตอนรถนิ่ง (จูนค่านี้เพื่อให้รถตั้งตรงไม่ไหล)
double Kp = 25, Ki = 120, Kd = 1.2; // ค่าพารามิเตอร์ PID (ต้องจูนตามโครงสร้างรถจริง)
double input, output;       // input = มุมปัจจุบัน, output = ความเร็วมอเตอร์ที่คำนวณได้

// --- การตั้งค่าอุปกรณ์สื่อสารและเซนเซอร์ ---
RF24 radio(9, 10);          // ขา CE=9, CSN=10
const byte address[7] = "REMOTE"; // ชื่อท่อสัญญาณ (ต้องตรงกับตัวส่ง)
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
MPU6050 mpu;

// --- ตัวแปรจัดการค่าจาก MPU6050 ---
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];               // [0]=Yaw, [1]=Pitch, [2]=Roll (เราใช้ Pitch ในการทรงตัว)

// --- ฟังก์ชันสั่งงานมอเตอร์แต่ละข้าง ---
void setMotor(int p1, int p2, int p_pwm, double speed) {
  if (speed > 0) { // เดินหน้า
    digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
  } else if (speed < 0) { // ถอยหลัง
    digitalWrite(p1, LOW);  digitalWrite(p2, HIGH);
    speed = -speed; // เปลี่ยนค่าลบเป็นบวกเพื่อส่งให้ PWM
  } else { // หยุด
    digitalWrite(p1, LOW);  digitalWrite(p2, LOW);
  }

  // Deadband: ป้องกันมอเตอร์ครางแต่ไม่หมุน (ถ้า PWM ต่ำกว่า 30 ให้ปัดเป็น 30)
  int finalSpeed = (int)speed;
  if (finalSpeed > 0 && finalSpeed < 30) finalSpeed = 30;

  analogWrite(p_pwm, constrain(finalSpeed, 0, 255)); // ส่งค่า PWM (0-255)
}

// --- ฟังก์ชันคำนวณการเลี้ยว (บวก/ลบ ความเร็วล้อซ้าย-ขวา) ---
void driveMotors(double pidOutput, int turn) {
  float turnHalf = turn / 2.0;
  // ล้อซ้ายบวกค่าเลี้ยว ล้อขวาลบคค่าเลี้ยว ทำให้รถหมุนตัวได้
  setMotor(AIN1, AIN2, PWMA, pidOutput + turnHalf); 
  setMotor(BIN1, BIN2, PWMB, pidOutput - turnHalf); 
}

// --- ฟังก์ชันหยุดมอเตอร์ทั้งหมดทันที ---
void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // ตั้งค่า Pin Mode
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  stopMotors();

  // เริ่มต้นการทำงานของ nRF24L01 (ตัวรับ)
  if (radio.begin()) {
    radio.openReadingPipe(1, address);
    radio.setDataRate(RF24_2MBPS); // ความเร็วในการส่งข้อมูล
    radio.setPALevel(RF24_PA_LOW); // กำลังส่ง (ใช้ Low ในระยะใกล้เพื่อความเสถียร)
    radio.setPayloadSize(sizeof(myData));
    radio.startListening(); // ตั้งเป็นโหมดรอรับสัญญาณ
  }

  // เริ่มต้นการทำงานของ MPU6050
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    // --- จุดใส่ค่า Offset (ได้จากการรันโค้ด IMU_Zero) ---
    mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);
    
    mpu.setDMPEnabled(true); // เปิดใช้งานระบบคำนวณมุมในตัวชิป (DMP)
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    // ตั้งค่าระบบ PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255); // ให้ Output สอดคล้องกับค่า PWM
    myPID.SetSampleTime(10);          // คำนวณทุกๆ 10ms (100Hz)
    Serial.println(F("System Ready!"));
  }
}

void loop() {
  static int turnValue = 0;

  // --- 1. ตรวจสอบและรับข้อมูลจาก Remote ---
  if (radio.available()) {
    radio.read(&myData, sizeof(myData));
    
    // จัดการเรื่องการเลี้ยว (Turn)
    if (myData.direction == 1)      turnValue = myData.speed / 3;  // เลี้ยวขวา
    else if (myData.direction == 2) turnValue = -(myData.speed / 3); // เลี้ยวซ้าย
    else turnValue = 0;

    // จัดการเรื่องการเคลื่อนที่ (หน้า/หลัง) โดยการเปลี่ยนมุมเป้าหมาย (Setpoint)
    if (myData.direction == 0)      setpoint = originalSetpoint + 3.0; // เอียงหน้าเพื่อวิ่งไปข้างหน้า
    else if (myData.direction == 3) setpoint = originalSetpoint - 3.0; // เอียงหลังเพื่อถอยหลัง
    else setpoint = originalSetpoint; // ทรงตัวอยู่กับที่
  }

  // --- 2. อ่านค่ามุมปัจจุบันจาก MPU6050 ---
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // ดึงค่ามุม Pitch (การเอียงหน้า-หลัง) มาเป็น input ให้ PID
    input = ypr[1] * 180 / M_PI;

    // --- 3. ตรวจสอบเงื่อนไขความปลอดภัยและสั่งงานมอเตอร์ ---
    if (abs(input) > 45) { 
      // ถ้ารถล้มเกิน 45 องศา ให้หยุดมอเตอร์เพื่อป้องกันความเสียหาย
      stopMotors(); 
    } else {
      // คำนวณค่า PID และสั่งมอเตอร์ขับเคลื่อน
      myPID.Compute();
      driveMotors(output, turnValue); 
    }
  }
}