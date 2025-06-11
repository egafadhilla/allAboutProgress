#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Ps3Controller.h>

#define LF_PWM GPIO_NUM_19 //pin right forward ch1
#define LR_PWM GPIO_NUM_18 //pin right reverse ch2
#define RF_PWM GPIO_NUM_32 //pin left forward ch3
#define RR_PWM GPIO_NUM_33 //pin left reverse ch4
#define LED_BUILTIN GPIO_NUM_2 
#define LED_Ps3 GPIO_NUM_12

#define CLKPin_rotary GPIO_NUM_14
#define DTNPin_rotary GPIO_NUM_43
int CLKState_rotary;
int lastCLKState_rotary = LOW;
const int sw_encoder = GPIO_NUM_27;

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define MAX_SPEED 255
#define HALF_SPEED 128

const char *ssid = "egafadhilla";
const char *password = "egaega123";

const int buttonPin = GPIO_NUM_4;
volatile bool otaEnabled = false;
volatile bool wifiRequested = false;
TaskHandle_t otaTaskHandle = NULL, tugasPs3Controll;
TimerHandle_t otaTimeoutTimer = NULL;

void IRAM_ATTR buttonISR() {
  wifiRequested = true;
}

void buttonTask(void *pvParameters) {
  const TickType_t debounceDelay = pdMS_TO_TICKS(50);
  
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(buttonPin, buttonISR, FALLING);

  while (1) {
    if (wifiRequested) {
      vTaskDelay(debounceDelay);
      
      if (digitalRead(buttonPin) == LOW) {
        otaEnabled = !otaEnabled;
        Serial.println(otaEnabled ? "OTA Enabled" : "OTA Disabled");

        if (otaEnabled) {
          // Start OTA timeout timer (5 menit)
          xTimerStart(otaTimeoutTimer, 0);
        } else {
          WiFi.disconnect(true);
          WiFi.mode(WIFI_OFF);
          xTimerStop(otaTimeoutTimer, 0);
        }
      }
      wifiRequested = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void otaTask(void *pvParameters) {
  bool wifiConnected = false;
  
  while (1) {
    if (otaEnabled) {
      if (!wifiConnected) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        
        Serial.println("Menghubungkan ke WiFi...");
        
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && 
               millis() - startAttemptTime < 10000) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        if (WiFi.status() == WL_CONNECTED) {
          wifiConnected = true;
          Serial.println("WiFi Terhubung");
          Serial.print("IP Address: ");
          Serial.println(WiFi.localIP());
          
          ArduinoOTA.setHostname("ESP32_ROBOT_SUMO");
          ArduinoOTA
            .onStart([]() {
              String type;
              if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
              } else {
                type = "filesystem";
              }
              Serial.println("Memulai update " + type);
            })
            .onEnd([]() {
              Serial.println("\nSelesai");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
              Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
              Serial.printf("Error[%u]: ", error);
              if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
              else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
              else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
              else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
              else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });
            
          ArduinoOTA.begin();
        } else {
          Serial.println("Gagal menghubungkan WiFi!");
          otaEnabled = false;
        }
      }
      
      if (wifiConnected) {
        ArduinoOTA.handle();
        blinkLED(200, true);
      }
    } else {
      if (wifiConnected) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        wifiConnected = false;
        Serial.println("WiFi Dimatikan");
      }
      blinkLED(1000, false); // Blink lambat saat idle
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void otaTimeoutCallback(TimerHandle_t xTimer) {
  otaEnabled = false;
  Serial.println("OTA Dinonaktifkan (Timeout 5 menit)");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("Memulai Inisialisasi...");
  
  ledcAttachChannel(RF_PWM, PWM_FREQ, PWM_RESOLUTION, 0);
  ledcAttachChannel(RR_PWM ,PWM_FREQ, PWM_RESOLUTION, 1);
  ledcAttachChannel(LF_PWM, PWM_FREQ, PWM_RESOLUTION, 2);
  ledcAttachChannel(LR_PWM ,PWM_FREQ, PWM_RESOLUTION, 3);
  ledcAttachChannel(LED_BUILTIN, PWM_FREQ, PWM_RESOLUTION, 4);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_Ps3, OUTPUT);
  pinMode(CLKPin_rotary, INPUT);
  pinMode(DTNPin_rotary, INPUT);
  
  Ps3.begin("78:42:1C:6D:77:B6");
  
  otaTimeoutTimer = xTimerCreate(
    "OTATimeout",
    pdMS_TO_TICKS(300000),
    pdFALSE,
    (void*)0,
    otaTimeoutCallback
  );
  
  xTaskCreate(otaTask, "OTATask", 4096, NULL, 1, &otaTaskHandle);
  xTaskCreate(buttonTask, "ButtonTask", 2048, NULL, 2, NULL);
  xTaskCreate(Ps3Controll, "Tugas Ps3 controll", 4096, NULL, 1, &tugasPs3Controll);
  
  Serial.println("Inisialisasi Selesai");
}

void loop() {
}

void Ps3Controll(void* param){
  (void) param;
  bool l1Pressed = false;
  bool r1Pressed = false;
  for(;;){
    if(Ps3.isConnected()){
      int ly = constrain(Ps3.data.analog.stick.ly, -127, 127);  // Arah vertikal: 0 (atas) - 255 (bawah)
      int lx = constrain(Ps3.data.analog.stick.lx, -127, 127); // Arah horizontal: 0 (kiri) - 255 (kanan)
      int ry = constrain(Ps3.data.analog.stick.ry, -127, 127);  // Arah vertikal: 0 (atas) - 255 (bawah)
      int rx = constrain(Ps3.data.analog.stick.rx, -127, 127); // Arah horizontal: 0 (kiri) - 255 (kanan)
      digitalWrite(LED_Ps3, HIGH);
      Serial.print("\nly =");
      Serial.print(ly);
      Serial.print("      lx =");
      Serial.print(lx);

      Serial.print("\nry =");
      Serial.print(ry);
      Serial.print("      rx =");
      Serial.print(rx);

      if(Ps3.data.button.l1) {
                if(!l1Pressed) {
                    l1Pressed = true;
                }
                // Aksi selama L1 ditekan
                kontrolMotorKiri(MAX_SPEED, false);
                kontrolMotorKanan(MAX_SPEED, true);
            } else if(l1Pressed) {
                Serial.println("L1 dilepas");
                l1Pressed = false;
                stopMotor();
            }

      if(Ps3.data.button.r1) {
                if(!r1Pressed) {
                    r1Pressed = true;
                }
                // Aksi selama R1 ditekan
                kontrolMotorKiri(MAX_SPEED, true);
                kontrolMotorKanan(MAX_SPEED, false);
            } else if(r1Pressed) {
                r1Pressed = false;
                stopMotor();
            }

      else if(ly < -10 && lx > -10 && lx < 10){
        kontrolMotorKanan(MAX_SPEED, false);
        kontrolMotorKiri(MAX_SPEED, false);
        //Serial.println("Maju fullspeed");
      } 
      else if(ly > 10 && (lx > -10 && lx < 10)){
        kontrolMotorKanan(MAX_SPEED, true);
        kontrolMotorKiri(MAX_SPEED, true);
        //mundur full speed
      }
      else if((ly < 10 && ly > -10) && lx > 10){
        kontrolMotorKanan(0, true);
        kontrolMotorKiri(MAX_SPEED, true);
        // puter kanan
      }
      else if((ly < 10 && ly > -10) && lx < -10){
        kontrolMotorKanan(MAX_SPEED, true);
        kontrolMotorKiri(0, true);
        // puter kiri
      }
      else if((ly < -10) && lx > 10){
        kontrolMotorKanan(MAX_SPEED, false);
        kontrolMotorKiri(HALF_SPEED, false);
        //maju kanan
      }
      else if(lx < - 10 && ly < -10 ){
        kontrolMotorKanan(HALF_SPEED, false);
        kontrolMotorKiri(MAX_SPEED, false);
        //Serial.println("Maju Kiri");
      }
      else if(lx > 10 && ly > 10 ){
        kontrolMotorKanan(MAX_SPEED, true);
        kontrolMotorKiri(HALF_SPEED, true);
        //Serial.println("Mundur Kanan");
      }
      else if(lx < - 10 && ly > 10 ){
        kontrolMotorKanan(HALF_SPEED, true);
        kontrolMotorKiri(MAX_SPEED, true);
        //Serial.println("Mundur Kiri");
      }
      else if((abs(lx) < 10) && (abs(ly) < 10)){
        stopMotor();
        //Serial.println("Motor Mati");
      }
    }
    else{
       digitalWrite(LED_Ps3, LOW);
    }
}
}

void blinkLED(uint16_t interval, bool fastBlink) {
  static unsigned long prevMillis = 0;
  static bool ledState = false;
  
  if (millis() - prevMillis >= interval) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    prevMillis = millis();
  }
}

// Implementasi fungsi kontrol motor (tetap sama seperti sebelumnya)
void kontrolMotorKanan(int speed, bool maju) {
  // Motor Kanan
  if(maju){
    ledcWrite(RF_PWM, 0);
    ledcWrite(RR_PWM, speed);
  }
  else{
    ledcWrite(RR_PWM, 0);
    ledcWrite(RF_PWM, speed);
  }
}

void kontrolMotorKiri(int speed, bool maju) {
  if(maju){
    ledcWrite(LF_PWM, 0);
    ledcWrite(LR_PWM, speed);
  }
  else{
    ledcWrite(LF_PWM, speed);
    ledcWrite(LR_PWM, 0);
  }
  }

void stopMotor(){
  ledcWrite(RR_PWM, 0);
  ledcWrite(RF_PWM, 0);
  // Motor Kiri
  ledcWrite(LR_PWM, 0);
  ledcWrite(LF_PWM, 0);
}