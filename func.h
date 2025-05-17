#include "vars.h"

uint8_t buttonsState() {
  uint8_t state = 0;
  // Read GPIO4 (Left Button). Assume active-low wiring (LOW = pressed)
  if (digitalRead(4) == LOW) {
    state |= LEFT_BUTTON;  // Set bit 4 if pressed
  }
  // Read GPIO1 (Right Button). Use caution with GPIO1 (UART TX)!
  if (digitalRead(1) == LOW) {
    state |= RIGHT_BUTTON;  // Set bit 1 if pressed
  }
  return state;
}
bool anyPressed(uint8_t buttons) {
  return (buttonsState() & buttons) != 0;
}
bool pressed(uint8_t buttons) {
  return (buttonsState() & buttons) == buttons;
}
bool notPressed(uint8_t buttons) {
  return (buttonsState() & buttons) == 0;
}

bool justPressed(uint8_t button) {
  return (!(previousButtonState & button) && (currentButtonState & button));
}
bool justReleased(uint8_t button) {
  return (!(previousButtonState & button) && !(currentButtonState & button));
}
void pollButtons() {
  previousButtonState = currentButtonState;
  currentButtonState = buttonsState();
}


void printLocalTime() {
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  u8g2.setCursor(timeX + 6, timeY);
  u8g2.print(&timeinfo, "%I:%M:%S");
  int currentHour = timeinfo.tm_hour;
  if (currentHour >= 12) {
    am = false;
    pm = true;
  } else {
    am = true;
    pm = false;
  }
  u8g2.setCursor(timeX + 24, timeY + 15);
  u8g2.print(pm ? "PM" : "AM");
}


void initOLED() {
  u8g2.begin();
  u8g2.setContrast(255);     // set contrast to maximum
  u8g2.setBusClock(400000);  //400kHz I2C
  u8g2.setFont(u8g2_font_ncenB10_tr);
}

void sensorsUpdate() {
  Wire.endTransmission();           // Force release I2C bus
  clim.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  voc_index = mox.measureVocIndex(temp.temperature, humidity.relative_humidity);
}

void handleScreens() {
  Wire.endTransmission();           // Force release I2C bus
  clim.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  voc_index = mox.measureVocIndex(temp.temperature, humidity.relative_humidity);

  switch (screen) {
    case ScreenMode::Time:

      u8g2.clearBuffer();  // clear the internal memory
      printLocalTime();
      u8g2.sendBuffer();       // transfer internal memory to the display
      Wire.endTransmission();  // Force release I2C bus
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::Twinkles;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::TempHumidity;
      }
      break;

    case ScreenMode::TempHumidity:
      //Wire.endTransmission();  // Force release I2C bus

      // Read SHTC3 sensor (Adafruit library uses Wire)
      //sensors_event_t humidity, temp;
      //clim.getEvent(&humidity, &temp);

      // Update display with sensor data
      u8g2.clearBuffer();
      u8g2.setCursor(xOffset, textyOffset);
      u8g2.print("T: ");
      u8g2.println(temp.temperature * 1.8 + 32);
      u8g2.print("*F");
      u8g2.setCursor(xOffset, textyOffset + 20);
      u8g2.print("H:");
      u8g2.print(humidity.relative_humidity);
      u8g2.sendBuffer();
      Wire.endTransmission();  // Release bus again
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::Time;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::VocIndex;
      }
      break;

    case ScreenMode::VocIndex:

      // Update display with sensor data
      u8g2.clearBuffer();
      u8g2.setCursor(xOffset, textyOffset);
      u8g2.println("VOC: ");
      u8g2.setCursor(xOffset, textyOffset + 20);
      u8g2.println(voc_index);
      u8g2.sendBuffer();
      Wire.endTransmission();
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::TempHumidity;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::Waveform;
      }

      break;

    case ScreenMode::Waveform:
      /*
        // ---- Add opening brace here ----
        bytes_read;
        i2s_read(I2S_NUM_0, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);

        samples_read = bytes_read / sizeof(int32_t);
        if (samples_read == 0) break;  // Prevent division by zero

        downsample_factor = (samples_read / width) + xOffset;

        u8g2.clearBuffer();
        for (int x = 0; x < width; x++) {
          sample_index = x * downsample_factor;
          if (sample_index >= samples_read) sample_index = samples_read - 1;

          raw_value = raw_samples[sample_index];
          audio_sample = raw_value >> 14;
          abs_sample = abs(audio_sample);

          // Fix mapping syntax:
          scaled = map(abs_sample, 0, 262144, 0, 1023);
          newy = map(scaled, 0, 1024, 0, 16) + 8 + yOffset;  // Fixed line

          u8g2.drawLine(x, lastsample, x, newy);
          lastsample = newy;
        }
        u8g2.sendBuffer();
        Wire.endTransmission();
         */
        if (justPressed(LEFT_BUTTON)) screen = ScreenMode::VocIndex;
        if (justPressed(RIGHT_BUTTON)) screen = ScreenMode::Twinkles;
        break;
        // ---- Add closing brace here ----

    case ScreenMode::Twinkles:
      digitalWrite(0, HIGH);
      digitalWrite(3, HIGH);
      u8g2.clearBuffer();  // clear the internal memory

      u8g2.setCursor(timeX + 6, timeY);
      u8g2.print("Twinkle");
      u8g2.sendBuffer();  // transfer internal memory to the display
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::Waveform;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::Time;
      }
      break;
  }
}