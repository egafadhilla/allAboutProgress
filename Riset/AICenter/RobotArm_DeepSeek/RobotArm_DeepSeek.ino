#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ 50

// Shared resources
SemaphoreHandle_t xSerialMutex;
uint8_t armDeg[5] = {90, 90, 90, 90, 90};
uint8_t nextDeg[5] = {90, 90, 90, 90, 90};

// Serial buffer
#define INPUT_SIZE 128
char inputBuffer[INPUT_SIZE];
uint8_t bufferIndex = 0;

void serialCommunication(void *pvParameters) {
  while(1) {
    // Non-blocking serial read
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
      
      while(token != NULL && index < 5) {
        nextDeg[index++] = atoi(token);
        token = strtok(NULL, ",");
      }

      if(index == 5) {
        moveAll(nextDeg[0], nextDeg[1], nextDeg[2], nextDeg[3], nextDeg[4]);
        Serial.println("OK: Servos moved");
      } else {
        Serial.println("ERROR: Invalid mov() arguments");
      }
    }
    else if(strncmp(command, "grb(", 4) == 0 && command[strlen(command)-1] == ')') {
      char param = command[4];
      
      if(param == '0') {
        grabMove("close");
        Serial.println("OK: Grabber closed");
      }
      else if(param == '1') {
        grabMove("open");
        Serial.println("OK: Grabber opened");
      }
      else {
        Serial.println("ERROR: Invalid grb() arguments");
      }
    }
    else {
      Serial.println("ERROR: Invalid command");
    }
    
    xSemaphoreGive(xSerialMutex);
  }
}

void moveDefault() {
  if(xSemaphoreTake(xSerialMutex, portMAX_DELAY)) {
    Serial.println("Memulai pengaturan default");
    xSemaphoreGive(xSerialMutex);
  }
  
  uint8_t defaultDeg[5] = {90, 90, 90, 90, 90};
  moveAll(defaultDeg[0], defaultDeg[1], defaultDeg[2], defaultDeg[3], defaultDeg[4]);
}

void moveAll(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint8_t s5) {
  servo.writeMicroseconds(0, map(s1, 0, 180, 800, 2200));
  servo.writeMicroseconds(1, map(s2, 0, 180, 800, 2200));
  servo.writeMicroseconds(2, map(s3, 0, 180, 800, 2200));
  servo.writeMicroseconds(3, map(s4, 0, 180, 800, 2200));
  servo.writeMicroseconds(4, map(s5, 0, 180, 800, 2200));
  
  // Update shared state
  armDeg[0] = s1;
  armDeg[1] = s2;
  armDeg[2] = s3;
  armDeg[3] = s4;
  armDeg[4] = s5;
}

void grabMove(const char* state) {
  if(strcmp(state, "open") == 0) {
    servo.writeMicroseconds(5, map(120, 0, 180, 800, 2200));
  }
  else if(strcmp(state, "close") == 0) {
    servo.writeMicroseconds(5, map(50, 0, 180, 800, 2200));
  }
}

void setup() {
  Serial.begin(9600);
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);
  
  // Create mutex for serial access
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
  if(xSemaphoreTake(xSerialMutex, portMAX_DELAY)) {
    Serial.print("Current angles: ");
    for(int i = 0; i < 5; i++) {
      Serial.print(armDeg[i]);
      Serial.print(i < 4 ? "," : "\n");
    }
    xSemaphoreGive(xSerialMutex);
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}