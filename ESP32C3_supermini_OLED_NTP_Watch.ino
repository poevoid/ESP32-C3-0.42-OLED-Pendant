#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <WiFi.h>
#include "time.h"
#include "Adafruit_SHTC3.h"
#include "Adafruit_SGP40.h"
#include <esp_task_wdt.h>
#include <driver/i2s.h>

#include "func.h"

void setup() {
  initOLED();
  
  initSensors();
  pinMode(0, OUTPUT);//for FLEDs
  digitalWrite(0, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    display.clearDisplay();
    display.drawBitmap(GX((72/2)-16), GY(4), load[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
    display.display();  // transfer internal memory to the display
    frame = (frame+1)%LOAD_FRAME_COUNT;
    delay(FRAME_DELAY);
  }
    display.clearDisplay();
    display.drawBitmap(GX((72/2)-16), GY(4), checkmark, FRAME_WIDTH, FRAME_HEIGHT, 1);
    display.display();  // transfer internal memory to the display
   

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();


  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  pinMode(4, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  initMicrophone();
  noise_floor = calibrateNoiseFloor();
  pinMode(0, OUTPUT); //for FLEDs
  digitalWrite(0, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
}

void loop(void) {
  pollButtons();

  if (screen != ScreenMode::Twinkles) {
    digitalWrite(0, LOW);
    digitalWrite(3, LOW);
  }
 handleScreens();
}