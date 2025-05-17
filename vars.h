
#pragma once
// there is no 72x40 constructor in u8g2 hence the 72x40 screen is mapped in the middle of the 132x64 pixel buffer of the SSD1306 controller
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);


Adafruit_SGP40 mox;                      //metal oxide sensor
Adafruit_SHTC3 clim = Adafruit_SHTC3();  //environmental sensor (temp/humidity)

////////
////////
///////Please keep in mind that pins 5 and 6 should not be used as buttons, and cannot be grounded for the display to work.
///////
///////
//#define LEFT_BUTTON A4
//#define RIGHT_BUTTON A1
// Define bitmasks for buttons (GPIO4 = Left, GPIO1 = Right)
#define LEFT_BUTTON (1 << 4)   // Bit 4 represents GPIO4
#define RIGHT_BUTTON (1 << 1)  // Bit 1 represents GPIO1


const char* ssid = "ATTQ6FviKI";
const char* password = "hgchazeh76iy";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -21600;
const int daylightOffset_sec = 3600;

int width = 72;
int height = 40;
int xOffset = 28;      // = (132-w)/2
int textyOffset = 36;  // = (64-h)/2
uint8_t timeX = (xOffset);
uint8_t timeY = (textyOffset + 10);
int yOffset = 24;
int val = 0;
bool am;
bool pm;
unsigned long previousMillis = 0;
const long interval = 1000;
uint8_t currentButtonState = 0;
uint8_t previousButtonState = 0;
int lastsample = (height / 2) + yOffset;
sensors_event_t humidity, temp;
int32_t voc_index;


int sample_index;


size_t bytes_read;

int samples_read;

int downsample_factor;

int32_t raw_value;
int32_t audio_sample;
int abs_sample;

// Fix mapping syntax:
int scaled;
int newy;  // Fixed line





// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_10
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_7
#define I2S_MIC_SERIAL_DATA GPIO_NUM_2

// don't mess around with this
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 256,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};


int32_t raw_samples[SAMPLE_BUFFER_SIZE];

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
