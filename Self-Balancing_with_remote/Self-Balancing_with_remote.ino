#include <I2Cdev.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <IRremote.hpp>
#include <Wire.h>

// --- ขามอเตอร์ (ใช้ขาที่เลี่ยง Interrupt 2, 3) ---
const int IN1 = 7, IN2 = 8, ENA = 6;  
const int IN3 = 10, IN4 = 11, ENB = 9;  
const int IRPIN = 12;// ย้าย IR ไปขา 12 เพื่อคืนขา 2, 3 ให้ Interrupt หลัก

// --- ตัวแปร PID ---
double Kp = 25, Ki = 120, Kd = 1.2; // ค่าเริ่มต้นสำหรับการใช้ Interrupt (มักจะสูงขึ้นได้)
double input = 0, PIDoutput = 0, OG_setpoint = -0.9379, setpoint = -0.9379;
double turnOffset = 0; 
const int turnSpeed = 20;
bool hold_button = false;
long hold_button_timer = 0;

bool debug = 1;

// --- วัตถุควบคุม ---
MPU6050 mpu;
PID pid(&input, &PIDoutput, &setpoint, Kp, Ki, Kd, REVERSE);

// --- ตัวแปรจัดการ DMP & Interrupt ---
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// ตัวแปรสถานะ Interrupt
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

void setDirection(unsigned int direction) {
  hold_button_timer = millis();
  switch (direction) {
    case 24: setpoint = OG_setpoint + 0.5; turnOffset = 0; break;  // หน้า (เอียงไปข้างหน้าเล็กน้อย)
    case 82: setpoint = OG_setpoint -0.5; turnOffset = 0; break; // หลัง
    case 8:  turnOffset = -turnSpeed; break;         // ซ้าย
    case 90: turnOffset = turnSpeed; break;          // ขวา
    case 28: default: setpoint = OG_setpoint; turnOffset = 0; break;    // นิ่ง
  }

  hold_button = true;
}

void driveMotors(double output) {
  int minPWM = 50; 
  double leftSpeed = output + turnOffset;
  double rightSpeed = output - turnOffset;

  // ล้อซ้าย
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    leftSpeed = (leftSpeed + minPWM) * 1.32;
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    leftSpeed = (abs(leftSpeed) + minPWM) * 1.32;
  }
  // ล้อขวา
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    rightSpeed = rightSpeed + minPWM;
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    rightSpeed = abs(rightSpeed) + minPWM;
  }

  analogWrite(ENA, constrain((int)leftSpeed, 85, 255));
  analogWrite(ENB, constrain((int)rightSpeed, 68, 255));
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(2, INPUT); // ขา INT จาก MPU6050
  stopMotors();

  Serial.println(F("Initializing DMP with Interrupt..."));
  mpu.initialize();
  
  if (mpu.dmpInitialize() == 0) {
    // ใส่ค่า Offset ของคุณ
    mpu.setXAccelOffset(-5780); mpu.setYAccelOffset(-6482); mpu.setZAccelOffset(12314);
    mpu.setXGyroOffset(-24);  mpu.setYGyroOffset(61);  mpu.setZGyroOffset(-8);
    
    mpu.setDMPEnabled(true);

    // --- เปิดการทำงาน Interrupt ที่ขา 2 ---
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255, 255);
    pid.SetSampleTime(10);
    
    IrReceiver.begin(IRPIN, DISABLE_LED_FEEDBACK);
    Serial.println(F("DMP Ready! Waiting for Interrupt..."));
  }
}


void loop() {
  // 1. อ่านรีโมท
  if (IrReceiver.decode()) {
    setDirection(IrReceiver.decodedIRData.command);
    IrReceiver.resume();
  }

  if (millis() - hold_button_timer > 200) {
  setpoint = OG_setpoint;
  turnOffset = 0; // คืนค่าการเลี้ยว
  }

  // 2. เช็คว่ามี Interrupt จาก MPU หรือยัง  
  if (mpuInterrupt) {
    mpuInterrupt = false; // Reset สถานะ
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      input = ypr[2] * 180 / M_PI; // ใช้แกน Roll

      // 3. คำนวณและขับเคลื่อน
      if (abs(input) > 45) {
        stopMotors();
      } else {
        pid.Compute();
        PIDoutput = (abs(PIDoutput) < 1) ? 0 : PIDoutput;
        driveMotors(PIDoutput);
      }

      // 4. Debug
      if (debug) {
        static unsigned long lastPrint;
        if (millis() - lastPrint > 100) {
          Serial.print("In:"); Serial.print(input);
          Serial.print(" Out:"); Serial.println(PIDoutput);
          lastPrint = millis();
        }
      }
    }
  }
}
