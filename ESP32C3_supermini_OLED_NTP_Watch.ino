#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include "time.h"
#include "Adafruit_SHTC3.h"
#include "Adafruit_SGP40.h"
#include <driver/i2s.h>

#include "func.h"

void setup(void) {
  //delay(1000);
  Wire.begin(5, 6);  // Critical for ESP32-C3 pin remapping
  initOLED();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    u8g2.setCursor(timeX, timeY);
    u8g2.print("NO");
    u8g2.setCursor(timeX, timeY + 15);
    u8g2.print("SIGNAL");
    u8g2.sendBuffer();  // transfer internal memory to the display
  }
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();


  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  if (!clim.begin()) {
    //matrix.println("Couldn't Find SHTC3");
    return;
  }
  if (!mox.begin()) {
    //matrix.println("couldn't find SGP40 either");
    //matrix.show();
    return;
  }
  // start up the I2S peripheral
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  pinMode(4, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
}

void loop(void) {
  pollButtons();

  if (screen != ScreenMode::Twinkles) {
    digitalWrite(0, LOW);
    digitalWrite(3, LOW);
  }
  handleScreens();
}