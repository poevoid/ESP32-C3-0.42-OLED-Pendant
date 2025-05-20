#include "vars.h"
void enterBootloader() {
  delay(100);

  // Proper watchdog configuration
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(xTaskGetCurrentTaskHandle());

  while (true)
    ;  // Wait for watchdog to trigger
}

void checkUploadTrigger() {
  if (Serial.available() && Serial.read() == 'b') {
    enterBootloader();
  }
}

// Hardware reset function
void hardwareReset() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
}

void drawSprite(uint8_t x, uint8_t y) {
  display.drawBitmap(GX(x), GY(y), twinkle0, 8, 8, SSD1306_WHITE);
}

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
  display.setCursor(timeX, timeY);
  display.print(&timeinfo, "%I:%M");
  int currentHour = timeinfo.tm_hour;
  if (currentHour >= 12) {
    am = false;
    pm = true;
  } else {
    am = true;
    pm = false;
  }
  display.setCursor(timeX + 20, timeY + 18);
  display.print(pm ? "PM" : "AM");
}

void initSensors() {
  if (!clim.begin()) {
    while (1) delay(1);
  }
  if (!mox.begin()) {
    while (1) delay(1);
  }
}
void initOLED() {
  //delay(1000);
  pinMode(9, INPUT_PULLUP);  // GPIO9 = BOOT button

  // Upload rescue delay
  delay(2000);  // 2-second window for uploads

  // Check for serial upload trigger
  Serial.begin(115200);
  for (int i = 0; i < 20; i++) {
    checkUploadTrigger();
    delay(10);
  }
  hardwareReset();

  Wire.begin(5, 6);  // SDA=GPIO5, SCL=GPIO6
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.display();
}


void swapInt16(int16_t& a, int16_t& b) {
  int16_t temp = a;
  a = b;
  b = temp;
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color = WHITE) {
  // bresenham's algorithm - thx wikpedia
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swapInt16(x0, y0);
    swapInt16(x1, y1);
  }

  if (x0 > x1) {
    swapInt16(x0, x1);
    swapInt16(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int8_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      display.drawPixel(y0, x0, color);
    } else {
      display.drawPixel(x0, y0, color);
    }

    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}
float removeDCOffset(int32_t raw) {
  static float dc_offset = 0.0f;
  static float alpha = 0.9999f;  // Very slow DC removal

  dc_offset = alpha * dc_offset + (1 - alpha) * raw;
  return raw - dc_offset;
}
void initMicrophone() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // INMP441 uses 32-bit container
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// Improved envelope detector
float calculateEnvelope(float sample) {
  static float envelope = MIN_AMPLITUDE;
  const float attack = (sample > envelope) ? 0.1f : 0.0001f;

  envelope = (1 - attack) * envelope + attack * fabs(sample);
  return max(envelope, MIN_AMPLITUDE);
}

float calculateGain(int16_t sample) {
  static float gain = 2.0;
  const float attack = 1.0 - exp(-1.0 / (SAMPLE_RATE * ATTACK_US / 1000000.0));
  const float release = exp(-1.0 / (SAMPLE_RATE * RELEASE_US / 1000000.0));

  float amplitude = fabs(sample / 32768.0f);
  static float envelope = 0.0;

  envelope = max(amplitude, envelope * release);

  if (envelope > (NOISE_FLOOR / 32768.0f)) {
    gain = min(1.0f, gain + attack);
  } else {
    gain = max(0.01f, gain * release);
  }

  return gain;
}

// Proper 24-bit sample conversion
float processSample(int32_t raw) {
  // 1. Convert 32-bit container to 24-bit sample
  raw >>= 8;  // Discard lower 8 bits (24-bit in 32-bit container)

  // 2. Convert to float (-1.0 to 1.0)
  float sample = raw / 8388608.0f;  // 2^23 = 8,388,608

  // 3. DC offset removal
  static float dc_offset = 0;
  dc_offset = 0.9999f * dc_offset + 0.0001f * sample;
  return sample - dc_offset;
}

// Debug output
void printLevels() {
  static uint32_t last_print = 0;
  if (millis() - last_print > 1000) {
    Serial.printf("Raw: %6d  Envelope: %.4f  dB: %6.1f\n",
                  raw_samples[0],
                  envelope,
                  20 * log10(envelope));
    last_print = millis();
  }
}

// INMP441-specific noise floor calibration
float calibrateNoiseFloor() {
  const int calibration_samples = 512;
  float sum = 0;

  for (int i = 0; i < calibration_samples; i++) {
    int32_t raw;
    size_t bytes_read;
    i2s_read(I2S_NUM_0, &raw, sizeof(raw), &bytes_read, portMAX_DELAY);
    sum += fabs(processSample(raw));
  }
  return (sum / calibration_samples) * 3.0f;  // 3σ threshold
}
void captureAudio() {
  size_t bytes_read;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);

  // Track peak over 100ms window
  static float peak = 0;
  float current_peak = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    waveform[i] = processSample(raw_samples[i]);
    current_peak = max(current_peak, fabs(waveform[i] / 32768.0f));
  }

  // Smooth peak tracking
  peak = 0.9 * peak + 0.1 * current_peak;
  max_amplitude = 32768 * (peak + 0.01);  // Prevent division by zero
}

// Audio processing pipeline
void processAudio() {
  static float noise_floor = 0.01f;
  static float max_amplitude = 3.0f;

  size_t bytes_read;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);

  // Process samples
  float current_max = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float sample = processSample(raw_samples[i]);

    // Noise gate
    if (fabs(sample) < noise_floor) {
      sample = 0;
    }

    // Adaptive scaling
    current_max = fmax(current_max, fabs(sample));
    processed_samples[i] = sample;
  }

  // Update amplitude tracking
  max_amplitude = 0.9f * max_amplitude + 0.1f * current_max;
}

// OLED Visualization
void drawWaveform() {
  const float vertical_scale = 8.0f;
  const int mid_y = GY(VISIBLE_HEIGHT / 2);
  const int amp_range = (VISIBLE_HEIGHT / 2) * vertical_scale;



  // Draw center line
  display.drawFastHLine(GX(0), mid_y, VISIBLE_WIDTH, SSD1306_WHITE);

  // Draw waveform
  for (int x = 1; x < SAMPLE_COUNT; x++) {
    int y_prev = mid_y + constrain(processed_samples[x - 1] * amp_range, -amp_range, amp_range);
    int y_curr = mid_y + constrain(processed_samples[x] * amp_range, -amp_range, amp_range);

    display.drawLine(GX(x - 1), y_prev, GX(x), y_curr, SSD1306_WHITE);
  }
}


void initTwinkles(const uint8_t (*bitmap)[128], uint8_t w, uint8_t h, int cframe) {
  uint8_t icons[NUMFLAKES][3];
  cframe = 0;
  // initialize
  for (uint8_t f = 0; f < NUMFLAKES; f++) {
    icons[f][XPOS] = random(GX(-16), GX(VISIBLE_WIDTH));
    icons[f][YPOS] = GY(40);
    icons[f][DELTAY] = random(-5, -1) + -1;
  }
  while (screen == ScreenMode::Twinkles) {
    // draw each icon
    digitalWrite(0, HIGH);
    digitalWrite(3, HIGH);
    pollButtons();
    display.clearDisplay();




    for (uint8_t f = 0; f < NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap[cframe], w, h, 1);
      // move it
      icons[f][YPOS] += icons[f][DELTAY];
      // if its gone, reinit
      if (icons[f][YPOS] < GY(-16)) {
        icons[f][XPOS] = random(GX(-16), GY(VISIBLE_WIDTH));
        icons[f][YPOS] = GY(40);
        icons[f][DELTAY] = random(-5, -1) + -1;
      }
    }
    cframe = (cframe + 1) % BAT_FRAME_COUNT;
    delay(FRAME_DELAY);

    if (justPressed(LEFT_BUTTON)) {
      screen = ScreenMode::Waveform;
    }
    if (justPressed(RIGHT_BUTTON)) {
      screen = ScreenMode::Time;
    }
    display.display();
  }
}

void sensorsUpdate() {
  //Wire.endTransmission();           // Force release I2C bus
  clim.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  voc_index = mox.measureVocIndex(temp.temperature, humidity.relative_humidity);
}

void handleScreens() {
  ////Wire.endTransmission();           // Force release I2C bus
  clim.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  voc_index = mox.measureVocIndex(temp.temperature, humidity.relative_humidity);

  switch (screen) {
    case ScreenMode::Time:

      display.clearDisplay();  // clear the internal memory
      printLocalTime();
      display.display();  // transfer internal memory to the display
      //Wire.endTransmission();  // Force release I2C bus
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::Twinkles;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::TempHumidity;
      }
      break;

    case ScreenMode::TempHumidity:
      ////Wire.endTransmission();  // Force release I2C bus

      // Read SHTC3 sensor (Adafruit library uses Wire)
      //sensors_event_t humidity, temp;
      //clim.getEvent(&humidity, &temp);

      // Update display with sensor data
      display.clearDisplay();
      display.setCursor(GX(0), GY(0));
      display.print("T:");
      display.println(temp.temperature * 1.8 + 32);
      display.print("F");
      display.setCursor(GX(0), GY(20));
      display.print("H%");
      display.print(humidity.relative_humidity);
      display.display();
      //Wire.endTransmission();  // Release bus again
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::Time;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::VocIndex;
      }
      break;

    case ScreenMode::VocIndex:

      // Update display with sensor data
      display.clearDisplay();
      display.setCursor(GX(0), GY(0));
      display.println("VOC: ");
      display.setCursor(GX(0), GY(20));
      display.println(voc_index);
      display.display();
      //Wire.endTransmission();
      if (justPressed(LEFT_BUTTON)) {
        screen = ScreenMode::TempHumidity;
      }
      if (justPressed(RIGHT_BUTTON)) {
        screen = ScreenMode::Waveform;
      }

      break;

    case ScreenMode::Waveform:
      while (screen == ScreenMode::Waveform) {
        pollButtons();
        processAudio();
        display.clearDisplay();
        drawWaveform();
        display.display();
        if (pressed(LEFT_BUTTON)) { screen = ScreenMode::VocIndex; }
        if (pressed(RIGHT_BUTTON)) { screen = ScreenMode::Twinkles; }
      }
      break;

    case ScreenMode::Twinkles:

      initTwinkles(bat, 32, 32, frame);

      break;
  }
}