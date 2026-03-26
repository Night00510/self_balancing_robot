// โปรเจกต์รีโมทคอนโทรลสำหรับรถทรงตัว (Remote Control) ใช้ ESP32 38-pin
// ส่งสัญญาณผ่าน nRF24L01 ไปยังตัวรับ (Arduino Uno)

#include <math.h>
#include <SPI.h>
#include <RF24.h>

// --- กำหนดขาเชื่อมต่อจอยสติ๊ก ---
const byte x_join_stick = 34; // ขา Analog รับค่าแกน X
const byte y_join_stick = 35; // ขา Analog รับค่าแกน Y

// --- กำหนดขาเชื่อมต่อ nRF24L01 กับ ESP32 ---
const byte CE_PIN = 4;
const byte CSN_PIN = 5;
const byte address[7] = "REMOTE"; // ชื่อท่อสัญญาณ (ต้องตรงกับฝั่งรับ)

// --- โครงสร้างข้อมูลที่จะส่งออกไป (ต้องเหมือนกับตัวรับเป๊ะๆ) ---
struct from_Remote_Data {
  byte direction = 6; // เริ่มต้นที่ 6 (Stop)
  byte speed = 0;     // เริ่มต้นความแรงที่ 0
};
from_Remote_Data myData;

RF24 radio(CE_PIN, CSN_PIN);

// --- ฟังก์ชันคำนวณความแรง (Speed) แบบวงกลม ---
byte map_Speed(unsigned int X, unsigned int Y) {
  // 1. แปลงค่า 0-255 (8-bit) ให้เป็นช่วง -1 ถึง 1 (Normalization)
  double xf = (X - 127.5) / 127.5; 
  double yf = (Y - 127.5) / 127.5;

  // 2. Squircle Mapping: สูตรคณิตศาสตร์เปลี่ยนพื้นที่สี่เหลี่ยมของจอยสติ๊กให้เป็นวงกลม
  // ช่วยให้เวลาโยกเฉียงๆ ค่าความแรงไม่ทะลุเกิน 100% (ลดอาการค่ากระโดดที่มุม)
  double u = xf * sqrt(1.0 - (pow(yf, 2) / 2.0));
  double v = yf * sqrt(1.0 - (pow(xf, 2) / 2.0));

  // 3. คำนวณความยาวของเวกเตอร์ (Magnitude)
  double magnitude = sqrt((u * u) + (v * v));
  
  // 4. แปลงกลับเป็นค่า 0-255 เพื่อส่งออกไป
  int s = (int)(magnitude * 255.0);
  return (byte)constrain(s, 0, 255);
}

// --- ฟังก์ชันตัดสินใจทิศทาง (Direction) ---
byte map_Direction(unsigned int X, unsigned int Y) {
  // เช็ค Deadzone: ถ้าจอยสติ๊กคืนตัวไม่ตรงกลางเป๊ะ (ช่วง 117-137) ให้ถือว่าหยุด
  if (X >= 117 && X <= 137 && Y >= 117 && Y <= 137) return 6; // stop

  // หาว่าเราโยกแกนไหนไปไกลจากจุดศูนย์กลางมากกว่ากัน
  int deltaX = abs((int)X - 127);
  int deltaY = abs((int)Y - 127);

  // ถ้าโยกแกน Y (ขึ้น/ลง) มากกว่าแกน X -> สั่งเดินหน้าหรือถอยหลัง
  if (deltaY >= deltaX) {
    if (Y > 137) return 0; // forward
    if (Y < 117) return 3; // backward
  } 
  // ถ้าโยกแกน X (ซ้าย/ขวา) มากกว่าแกน Y -> สั่งเลี้ยวซ้ายหรือขวา
  else {
    if (X > 137) return 1; // right
    if (X < 117) return 2; // left
  }
  return 6; // กรณีอื่นๆ ให้หยุด
}

void setup() {
  // ตั้งค่า Pin
  pinMode(x_join_stick, INPUT);
  pinMode(y_join_stick, INPUT);
  
  Serial.begin(115200);
  
  // ตั้งค่าความละเอียด Analog เป็น 8-bit (0-255) เพื่อให้คำนวณง่ายเหมือน Arduino Uno
  analogReadResolution(8); 

  // เริ่มต้นการทำงาน nRF24L01
  if (!radio.begin()) {
    Serial.println("nRF24L01 not found!"); // ถ้าโมดูลเสียหรือต่อสายผิดจะค้างที่นี่
    while(1);
  }

  radio.openWritingPipe(address);      // เปิดท่อสำหรับการส่ง
  radio.setDataRate(RF24_2MBPS);       // ความเร็วรับส่งสูง (Latency ต่ำ)
  radio.setPALevel(RF24_PA_LOW);       // กำลังส่งต่ำ (ประหยัดพลังงานและเสถียรในระยะใกล้)
  radio.setRetries(0, 0);              // ไม่ต้องรอส่งซ้ำ (เน้นส่งไวๆ แบบ Real-time)
  radio.setPayloadSize(sizeof(myData)); // กำหนดขนาดข้อมูลที่จะส่ง (2 bytes)
  radio.stopListening();               // ตั้งเป็นโหมดตัวส่ง
  
  Serial.println("Remote Ready!");
}

void loop() {
  // --- อ่านค่าจากจอยสติ๊กแบบหาค่าเฉลี่ย 32 ครั้ง เพื่อลดสัญญาณรบกวน (Noise) ---
  unsigned int sumX = 0, sumY = 0;
  for(byte i = 0; i < 32; i++){
    sumX += analogRead(x_join_stick);
    sumY += analogRead(y_join_stick);
  } 
  sumX /= 32; 
  sumY /= 32;

  // นำค่าเฉลี่ยไปเข้าฟังก์ชันคำนวณ
  myData.speed = map_Speed(sumX, sumY);
  myData.direction = map_Direction(sumX, sumY);
  
  // ส่งข้อมูลออกไปแบบ writeFast (ไม่รอตอบกลับ เพื่อความลื่นไหลที่สุด)
  radio.writeFast(&myData, sizeof(myData));
  
  // พัก 20ms (ส่งข้อมูล 50 ครั้งต่อวินาที) เพื่อความเนียนในการคุม
  delay(20); 
}