#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;
uint16_t analog1 = 0, analog2 = 0, servolen1 = 0, servolen2 = 0;

void setup() {
  Serial.begin(9600);

  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);

  delay(100);
}

void bacaPotensio(uint16_t jeda) {
  static unsigned long pmillis;
  if(millis()-pmillis > jeda){
    analog1 = analogRead(A0);
    analog2 = analogRead(A1);
    // servolen1 = map(analog1, 0, 1023, 150, 600);
    // servolen2 = map(analog2, 0, 1023, 150, 600);
    servolen1 = map(analog1, 0, 1023, 800, 2000);
    servolen2 = map(analog2, 0, 1023, 10, 2500);
    Serial.print("hasil baca potensio RAW : ");
    Serial.print(analog1);
    Serial.print(" ");
    Serial.print(analog2);
    Serial.println(" ");
    Serial.print("hasil servolen : ");
    Serial.print(servolen1);
    Serial.print(" ");
    Serial.print(servolen2);
    Serial.println(" ");
    pmillis = millis();
}
}

void loop() {
  //bacaPotensio(50);
  moveAll(50, 100, 70, 50, 50);
  delay(2000);
  moveAll(80, 140, 90, 80, 80);
  delay(2000);
  // servo.writeMicroseconds(0, servolen1);
  // servo.writeMicroseconds(1, servolen2);
  // servo.setPWM(0, 0, servolen1);
  // servo.setPWM(1, 0, servolen2);


}


void moveAll(uint8_t servo1, uint8_t servo2, uint8_t servo3, uint8_t servo4, uint8_t servo5){
  uint16_t servoDegree1 = map(servo1, 0, 180, 800, 2200);
  uint16_t servoDegree2 = map(servo2, 0, 180, 800, 2200);
  uint16_t servoDegree3 = map(servo3, 0, 180, 800, 2200);
  uint16_t servoDegree4 = map(servo4, 0, 180, 800, 2200);
  uint16_t servoDegree5 = map(servo5, 0, 180, 800, 2200);
  Serial.print(servoDegree1);
  servo.writeMicroseconds(0, servoDegree1);
  servo.writeMicroseconds(1, servoDegree2);
  servo.writeMicroseconds(2, servoDegree3);
  servo.writeMicroseconds(3, servoDegree4);
  servo.writeMicroseconds(4, servoDegree5);
}