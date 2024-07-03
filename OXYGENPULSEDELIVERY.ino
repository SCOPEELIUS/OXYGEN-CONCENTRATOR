#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Q2HX711.h>

// Private defines
#define P1Data A3
#define P1Clk  A2
#define P2Data A0
#define P2Clk  A1
#define Relay 2
#define Valve1 3
#define Valve2 4
#define Valve3 5

// Driver initialization
LiquidCrystal_I2C lcd(0x27, 20, 4); 
SoftwareSerial mySerial(10, 9); // RX, TX
Q2HX711 PressureSens1(P1Data, P1Clk);
Q2HX711 PressureSens2(P2Data, P2Clk);

// Tasks definition
void TaskDisplay(void *pvParameters);
void TaskOxygen(void *pvParameters);
void TaskPressure(void *pvParameters);
void TimerCallback(TimerHandle_t xTimer);

// Variables
TimerHandle_t xTimer;
float concentrationPercent = 0;
float flowRate = 0;
float temperatureCelsius = 0;
long int pressure1 = 0;
long int pressure2 = 0;
bool Openvalve1=false;

void setup() {
  mySerial.begin(9600);
pinMode(Relay,OUTPUT);
pinMode(Valve1,OUTPUT);
pinMode(Valve2,OUTPUT);
pinMode(Valve3,OUTPUT);

  // Initialize LCD before creating tasks
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("BIOMEDICAL PROJECT");
  lcd.setCursor(0, 1);
  lcd.print("OXYGEN CONCENTRATOR");
  lcd.setCursor(1, 2);
  lcd.print("MADE BY GROUP");
  lcd.setCursor(5, 3);
  lcd.print("YEAR 2024");

//Timer created
   xTimer = xTimerCreate("MyTimer", pdMS_TO_TICKS(2500), pdTRUE, 0, TimerCallback);
       if (xTimer != NULL) {
        if (xTimerStart(xTimer, 0) != pdPASS) {
            // Handle error if timer start fails
        }
    } else {
        // Handle error if timer creation fails
    }

  // Create the tasks
  BaseType_t task1 = xTaskCreate(TaskDisplay, "Display", 150, NULL, 3, NULL);  
  BaseType_t task2 = xTaskCreate(TaskOxygen, "Consentration", 150, NULL, 2, NULL);  
  BaseType_t task3 = xTaskCreate(TaskPressure, "Pressure", 150, NULL, 1, NULL);  
  
  if (task1 == pdPASS && task2 == pdPASS && task3 == pdPASS ) {
  } else {
    
  }
}

void loop() {
  // Nothing here
}

void TaskDisplay(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("CONCENTRATION " + String(concentrationPercent) + "%");
      lcd.setCursor(0, 1);
      lcd.print("FLOW RATE " + String(flowRate) + "L/m");
      lcd.setCursor(0, 2);
      lcd.print("TEMPERATURE " + String(temperatureCelsius) + char(223) + "C");
      lcd.setCursor(0, 3);
      lcd.print("P1 " + String(pressure1) + " P2 " + String(pressure2));
    
    vTaskDelay(pdMS_TO_TICKS(3500));
  }
}

void TaskOxygen(void *pvParameters) {
  (void) pvParameters;
  byte queryCommand[] = {0x11, 0x01, 0x01, 0xED};
  for (;;) {
  
      mySerial.write(queryCommand, sizeof(queryCommand));
      vTaskDelay(pdMS_TO_TICKS(100));
      if (mySerial.available() >= 12) {
        byte data[12];
        for (int i = 0; i < 12; i++) {
          data[i] = mySerial.read();
        }
        // Calculate checksum
        byte checksum = 0;
        for (int i = 0; i < 11; i++) {
          checksum += data[i];
        }
        checksum = 0x00 - checksum;

        // Validate checksum
        if (checksum != data[11]) {

          // Attempt to recover by resetting serial communication
          mySerial.end();
          vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before re-initialization
          mySerial.begin(9600);
          vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after re-initialization
          continue; // Exit loop to retry sending query
        }

        // Convert Concentration, Flow, and Temperature from hex to decimal
        int concentration = (data[3] << 8) | data[4];
        concentrationPercent = concentration / 10.0;

        int flow = (data[5] << 8) | data[6];
        flowRate = flow / 10.0;

        int temperature = (data[7] << 8) | data[8];
        temperatureCelsius = temperature / 10.0;
      
    }
    vTaskDelay(pdMS_TO_TICKS(2500));
  }
}

void TaskPressure(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
   pressure1=PressureSens1.read()/1000;
   pressure2=PressureSens2.read()/1000;

   if(pressure1>9577){
    digitalWrite(Valve3,HIGH);   
   }else{
       digitalWrite(Valve3,LOW);  
   }
   
    if(pressure2<8400){
      digitalWrite(Relay,LOW);
      }else{
      digitalWrite(Relay,HIGH);
      }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void TimerCallback(TimerHandle_t xTimer) {
      if(pressure2<8400){
      if(Openvalve1){
        digitalWrite(Valve2,LOW);
        digitalWrite(Valve1,HIGH);
        Openvalve1=false;
      }else{
         digitalWrite(Valve1,LOW);
        digitalWrite(Valve2,HIGH);
        Openvalve1=true;
      }
      }else{
             digitalWrite(Valve1,LOW);
             digitalWrite(Valve2,LOW);
      }
}
