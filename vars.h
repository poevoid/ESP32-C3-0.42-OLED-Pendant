
#pragma once

#include "arrays.h"

////////
///////Please keep in mind that pins 5 and 6 should not be used as buttons, and cannot be grounded for the display to work.
///////
///////
//#define LEFT_BUTTON A4
//#define RIGHT_BUTTON A1
// Define bitmasks for buttons (GPIO4 = Left, GPIO1 = Right)
#define LEFT_BUTTON (1 << 4)   // Bit 4 represents GPIO4
#define RIGHT_BUTTON (1 << 1)  // Bit 1 represents GPIO1
#define NUMFLAKES 5
#define XPOS 0
#define YPOS 1
#define DELTAY 2

// Add these parameters at the top
#define SAMPLE_RATE           16000  // INMP441 supports up to 48kHz
#define BIT_DEPTH             24     // INMP441 is 24-bit

#define NOISE_FLOOR 10       // Adjust based on your environment (0-1000)
#define ATTACK_US         5000  // 10ms attack time
#define RELEASE_US        300000 // 300ms release time
#define DYNAMIC_RANGE 40.0f   // dB of range to display (40dB is natural)
// Add these constants
#define MIN_AMPLITUDE 0.0001f  // -80 dBFS (avoid log(0))
#define CALIBRATION_SAMPLES 512

// OLED Configuration
#define VISIBLE_WIDTH 72
#define VISIBLE_HEIGHT 40
#define X_OFFSET 28
#define Y_OFFSET 24
#define TEXT_Y_OFFSET 36
#define GX(x) (x + X_OFFSET)
#define GY(y) (y + Y_OFFSET)
#define TX(x) (x + X_OFFSET)  // Text X position
#define TY(y) (y + TEXT_Y_OFFSET) // Text Y position
// I2S Microphone Configuration
#define SAMPLE_COUNT VISIBLE_WIDTH
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_10
#define I2S_MIC_LEFT_RIGHT GPIO_NUM_7
#define I2S_MIC_SERIAL_DATA GPIO_NUM_2

#define BUFFER_WIDTH 128
#define BUFFER_HEIGHT 64
#define RESET_PIN 4
#define SCREEN_ADDRESS 0x3C

const char* ssid = "Polaris";
const char* password = "Gaystuffs";
const char* ntpServer = "pool.ntp.org";

const long gmtOffset_sec = -21600;
const long interval = 1000;

const int daylightOffset_sec = 3600;

// Global variables
float current_gain = 1.0;
float envelope = 0.0f;
float processed_samples[SAMPLE_COUNT];
float noise_floor;

uint32_t last_activity = 0;

int32_t raw_samples[SAMPLE_COUNT];
int32_t voc_index;
int16_t waveform[SAMPLE_COUNT];
int16_t max_amplitude = 1000;    // Auto-adjusts to input

int xOffset = 28;      
int textyOffset = 36;  
int yOffset = 24;
int width = xOffset+72;
int height = yOffset+40;
int frame = 0;

uint8_t timeX = (GX(5));
uint8_t timeY = (GY(0));
uint8_t currentButtonState = 0;
uint8_t previousButtonState = 0;


bool am;
bool pm;

unsigned long previousMillis = 0;

sensors_event_t humidity, temp;


Adafruit_SGP40 mox;                      //metal oxide sensor
Adafruit_SHTC3 clim = Adafruit_SHTC3();  //environmental sensor (temp/humidity)

Adafruit_SSD1306 display(BUFFER_WIDTH, BUFFER_HEIGHT, &Wire, -1);



enum class ScreenMode : uint8_t {
  Time,
  Waveform,
  TempHumidity,
  VocIndex,
  Twinkles
};

struct Wave {
  int x;
  int y;
};

Wave wave = { 0, 0 };
ScreenMode screen = { ScreenMode::Time };
