#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50

uint8_t armDeg[5] = {90, 90, 90, 90, 90};
uint8_t nextDeg[5] = {90, 90, 90, 90, 90};

TaskHandle_t serialCommunicationHandle;
TaskHandle_t armControlHandle;

void serialCommunication(void *pvParameters){
  while(1){
    if (Serial.available() > 0){
      String command = Serial.readStringUntil('\n');
    }
  }
}

void armControl(void *pvParameters){
  while(1){
    moveAll(nextDeg[0], nextDeg[1], nextDeg[2], nextDeg[3], nextDeg[4]);
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void moveDefault(){
  // atur nilai default untuk tiap masing masing servo
  uint16_t sudut1 = 90;
  uint16_t sudut2 = 90;
  uint16_t sudut3 = 90;
  uint16_t sudut4 = 90;
  uint16_t sudut5 = 90;
  Serial.println("memulai pengaturan default");
  // konversi dari sudut menuju microsecond time on pada frekuensi 50hz
  servo.writeMicroseconds(0, map( armDeg[0] = sudut1 ,0, 180, 800, 2200)); // default pos untuk servo 1 (paling bawah)
  servo.writeMicroseconds(1, map( armDeg[1] = sudut2 ,0, 180, 800, 2200)); // default pos untuk servo 2
  servo.writeMicroseconds(2, map( armDeg[2] = sudut3 ,0, 180, 800, 2200)); // default pos untuk servo 3
  servo.writeMicroseconds(3, map( armDeg[3] = sudut4 ,0, 180, 800, 2200)); // default pos untuk servo 4
  servo.writeMicroseconds(4, map( armDeg[4] = sudut5 ,0, 180, 800, 2200)); // default pos untuk servo 5
  Serial.println("posisi default berhasil diatur");
}


void moveAll(uint8_t servoDeg1, uint8_t servoDeg2, uint8_t servoDeg3, uint8_t servoDeg4, uint8_t servoDeg5){
  uint16_t servoMicSec1 = map(armDeg[0] = servoDeg1, 0, 180, 800, 2200);
  uint16_t servoMicSec2 = map(armDeg[1] = servoDeg2, 0, 180, 800, 2200);
  uint16_t servoMicSec3 = map(armDeg[2] = servoDeg3, 0, 180, 800, 2200);
  uint16_t servoMicSec4 = map(armDeg[3] = servoDeg4, 0, 180, 800, 2200);
  uint16_t servoMicSec5 = map(armDeg[4] = servoDeg5, 0, 180, 800, 2200);

  servo.writeMicroseconds(0, servoMicSec1);
  servo.writeMicroseconds(1, servoMicSec2);
  servo.writeMicroseconds(2, servoMicSec3);
  servo.writeMicroseconds(3, servoMicSec4);
  servo.writeMicroseconds(4, servoMicSec5);

}

void setup() {
  Serial.begin(9600);

  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);
  delay(500);
  moveDefault();
  delay(3000); // memberi delay setelah pengaturan posisi default dan memastikan bahwa servo dapat bergerak sesuai
  xTaskCreate(
    serialCommunication, // menjalankan fungsi serialCommunication
    "SerialTask", // menamai tugas sebagai SerialTask
    128, // memberi nilai stack sebanyak 128
    NULL, //
    1, // mengatur prioritas pada tingkat rendah (1 untuk rendah, 3 untuk paling tinggi)
    NULL
  );

  xTaskCreate(
    armControl,
    "ArmControlMove",
    128,
    NULL,
    1,
    NULL
  );

}

void loop() {
  // put your main code here, to run repeatedly:

}
