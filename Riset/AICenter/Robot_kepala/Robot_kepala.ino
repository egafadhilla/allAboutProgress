#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

Servo servo1;
Servo servo2;

SemaphoreHandle_t xSerialMutex;
uint8_t armDeg[2] = {90, 90};
uint8_t nextDeg[2] = {90, 90};

#define INPUT_SIZE 128
char inputBuffer[INPUT_SIZE];
uint8_t bufferIndex = 0;

void serialCommunication(void *pvParameters){
  while(1){
    while(Serial.available() > 0 && bufferIndex < INPUT_SIZE-1) {
      char c = Serial.read();
      
      if(c == '\n') {
        inputBuffer[bufferIndex] = '\0';
        processCommand(inputBuffer);
        bufferIndex = 0;
      } else {
        inputBuffer[bufferIndex++] = c;
      }
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
  }
}

void processCommand(const char* command) {
  // Ambil mutex untuk operasi serial
  if(xSemaphoreTake(xSerialMutex, portMAX_DELAY)) {
    if(strncmp(command, "mov(", 4) == 0 && command[strlen(command)-1] == ')') {
      char params[strlen(command)-5];
      strncpy(params, command+4, strlen(command)-5);
      params[strlen(command)-5] = '\0';

      uint8_t index = 0;
      char* token = strtok(params, ",");
      
      while(token != NULL && index < 2) {
        nextDeg[index++] = atoi(token);
        token = strtok(NULL, ",");
      }

      if(index == 2) {
        moveAll(nextDeg[0], nextDeg[1]);
        Serial.println("OK: Servos moved");
      } else {
        Serial.println("ERROR: Invalid mov() arguments");
      }
    }
    else {
      Serial.println("ERROR: Invalid command");
      String data = Serial.readStringUntil('\n');
       int nilai = data.toInt();
      moveAll(nilai,90);
    }

    xSemaphoreGive(xSerialMutex);
  }
}

void moveAll(uint8_t servoDeg1, uint8_t servoDeg2){
  servo1.write(servoDeg1);
  servo2.write(servoDeg2);
}

void moveDefault() {
  if(xSemaphoreTake(xSerialMutex, portMAX_DELAY)) {
    Serial.println("Memulai pengaturan default");
    xSemaphoreGive(xSerialMutex);
  }

  moveAll(armDeg[1], armDeg[2]);
}

void setup() {
  Serial.begin(9600);

  servo1.attach(9);
  servo2.attach(10);

  xSerialMutex = xSemaphoreCreateMutex();
  xTaskCreate(
    serialCommunication,
    "SerialTask",
    256,  // Reduced stack size
    NULL,
    1,
    NULL
  );

  moveDefault();

}

void loop() {
  // put your main code here, to run repeatedly:

}
