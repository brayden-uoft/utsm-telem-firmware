struct TelemetryRecord_t;

#include <Wire.h>
#include <LittleFS.h>
#include <math.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

// =========================
// Pin mapping (ESP32-C3 Super Mini)
// =========================
static const int PIN_I2C_SDA = 8;
static const int PIN_I2C_SCL = 9;
static const int BUTTON_PIN = 0;  // GPIO0 / A0
static const int LED_PIN = 1;     // External status LED on GPIO1

// =========================
// Device addresses
// =========================
static const uint8_t ADS1115_ADDR = 0x48;
static const uint8_t MPU6050_ADDR = 0x68;

// =========================
// Sampling constants
// =========================
#define CURRENT_NUM_SAMPLES   20
#define CURRENT_SAMPLE_DELAY  3   // ms

#define MPU_NUM_SAMPLES       20
#define MPU_SAMPLE_DELAY      3   // ms

// =========================
// ADS1115 registers/config
// =========================
#define ADS1115_REG_CONVERSION      0x00
#define ADS1115_REG_CONFIG          0x01

#define ADS1115_CONFIG_OS_SINGLE    0x8000
#define ADS1115_CONFIG_MUX_AIN0_GND 0x4000
#define ADS1115_CONFIG_MUX_AIN1_GND 0x5000
#define ADS1115_CONFIG_PGA_4V096    0x0200
#define ADS1115_CONFIG_MODE_SINGLE  0x0100
#define ADS1115_CONFIG_DR_128SPS    0x0080
#define ADS1115_CONFIG_COMP_DISABLE 0x0003

#define ADS1115_CONFIG_WORD_AIN0 (ADS1115_CONFIG_OS_SINGLE  | \
                                  ADS1115_CONFIG_MUX_AIN0_GND | \
                                  ADS1115_CONFIG_PGA_4V096  | \
                                  ADS1115_CONFIG_MODE_SINGLE | \
                                  ADS1115_CONFIG_DR_128SPS  | \
                                  ADS1115_CONFIG_COMP_DISABLE)

#define ADS1115_CONFIG_WORD_AIN1 (ADS1115_CONFIG_OS_SINGLE  | \
                                  ADS1115_CONFIG_MUX_AIN1_GND | \
                                  ADS1115_CONFIG_PGA_4V096  | \
                                  ADS1115_CONFIG_MODE_SINGLE | \
                                  ADS1115_CONFIG_DR_128SPS  | \
                                  ADS1115_CONFIG_COMP_DISABLE)

#define ADS1115_LSB_VOLTS (4.096f / 32768.0f)

// =========================
// Calibration
// =========================
static const float ADC_DIVIDER_RATIO   = 1.0f;
static const float ADC_ZERO_CURRENT_V  = 2.55f;
static const float ADC_SENSITIVITY     = 0.066f;
static const float VOLTAGE_INPUT_SCALE = 5.93f;

// =========================
// Current offset system
// =========================
static const uint32_t CURRENT_ZERO_CAL_DELAY_MS = 15000;   // wait 15 s before zeroing
static const uint32_t CURRENT_ZERO_CAL_TIME_MS  = 3000;    // then calibrate for 3 s
float g_currentOffsetA = 0.0f;

// =========================
// MPU6050 registers
// =========================
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

#define MPU6050_ACCEL_SENS_2G     16384.0f
#define GRAVITY_MPS2              9.80665f

// SD Card Button Stuff:
int lastButtonState = HIGH;
volatile bool buttonEvent = false;
volatile uint32_t lastInterruptTimeMs = 0;

// =========================
// SD card config
// =========================
static const int SD_CS   = 7;
static const int SD_MOSI = 6;
static const int SD_MISO = 5;
static const int SD_SCK  = 4;

static const uint16_t SD_MAX_LOG_FILES = 999;
char g_sdLogFile[32] = "/telemetry_001.csv";
uint16_t g_sdLogIndex = 0;

bool g_sdReady = false;

uint32_t lastButtonHandledMs = 0;

// Forward declarations for Arduino/C++ builds
bool selectNextSDLogFile();
void startLoggingNewFile(const char *reason);
void updateLoggingLed();

// =========================
// Logged record
// =========================
struct TelemetryRecord_t
{
  uint32_t timestamp_ms;
  int16_t current_mA;
  int32_t voltage_mV;
  int16_t accel_x_mps2_x100;
  int16_t accel_y_mps2_x100;
  int16_t accel_z_mps2_x100;
  uint16_t accel_mag_mps2_x100;
} __attribute__((packed));

// SD Card interrupt
void IRAM_ATTR buttonISR() {
  // Keep the ISR tiny: just record that a falling edge happened.
  // Debouncing is handled in the main loop.
  buttonEvent = true;
}

// =========================
// Logging control
// =========================
bool g_loggingEnabled = false;      // Set true at end of setup for AUTO-START on boot
static const char *LOG_FILE = "/telemetry.bin";

// =========================
// LED status helpers
// =========================
void setLed(bool on)
{
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void updateLoggingLed()
{
  setLed(g_loggingEnabled);
}

void blinkLedFor(uint32_t durationMs, uint32_t intervalMs = 250)
{
  uint32_t start = millis();
  bool ledState = false;
  uint32_t lastToggle = 0;

  while (millis() - start < durationMs) {
    uint32_t now = millis();
    if (now - lastToggle >= intervalMs) {
      lastToggle = now;
      ledState = !ledState;
      setLed(ledState);
    }
    delay(10);
  }

  setLed(false);
}

// =========================
// I2C helpers
// =========================
bool i2cWrite8(uint8_t devAddr, uint8_t regAddr, uint8_t value)
{
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool i2cWrite16BE(uint8_t devAddr, uint8_t regAddr, uint16_t value)
{
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write((uint8_t)((value >> 8) & 0xFF));
  Wire.write((uint8_t)(value & 0xFF));
  return (Wire.endTransmission() == 0);
}

bool i2cReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, size_t len)
{
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  if (Wire.endTransmission(false) != 0) return false;

  size_t n = Wire.requestFrom((int)devAddr, (int)len);
  if (n != len) return false;

  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// =========================
// ADS1115 functions
// =========================
bool adc16Init()
{
  Wire.beginTransmission(ADS1115_ADDR);
  return (Wire.endTransmission() == 0);
}

bool adc16ReadRawAIN0(int16_t &raw)
{
  if (!i2cWrite16BE(ADS1115_ADDR, ADS1115_REG_CONFIG, ADS1115_CONFIG_WORD_AIN0)) return false;
  delay(10);

  uint8_t rx[2];
  if (!i2cReadBytes(ADS1115_ADDR, ADS1115_REG_CONVERSION, rx, 2)) return false;

  raw = (int16_t)((rx[0] << 8) | rx[1]);
  return true;
}

bool adc16ReadCurrent(float &currentA, int16_t &rawOut)
{
  int16_t raw;
  if (!adc16ReadRawAIN0(raw)) return false;

  float v_adc = ((float)raw) * ADS1115_LSB_VOLTS;
  float v_sensor = v_adc / ADC_DIVIDER_RATIO;
  currentA = (v_sensor - ADC_ZERO_CURRENT_V) / ADC_SENSITIVITY;

  rawOut = raw;
  return true;
}

bool adc16ReadVoltage(float &voltageV)
{
  if (!i2cWrite16BE(ADS1115_ADDR, ADS1115_REG_CONFIG, ADS1115_CONFIG_WORD_AIN1)) return false;
  delay(10);

  uint8_t rx[2];
  if (!i2cReadBytes(ADS1115_ADDR, ADS1115_REG_CONVERSION, rx, 2)) return false;

  int16_t raw = (int16_t)((rx[0] << 8) | rx[1]);
  float valueV = ((float)raw) * ADS1115_LSB_VOLTS;
  voltageV = valueV * VOLTAGE_INPUT_SCALE;
  return true;
}

bool adc16ReadAverageCurrent(float &avgCurrentA, float &avgVoltageV, int16_t &avgRaw)
{
  float currentSum = 0.0f;
  float voltageSum = 0.0f;
  int32_t rawSum = 0;
  uint16_t validSamples = 0;

  for (uint16_t i = 0; i < CURRENT_NUM_SAMPLES; i++) {
    float currentA, voltageV;
    int16_t raw;

    if (adc16ReadCurrent(currentA, raw) && adc16ReadVoltage(voltageV)) {
      currentSum += currentA;
      voltageSum += voltageV;
      rawSum += raw;
      validSamples++;
    }

    delay(CURRENT_SAMPLE_DELAY);
  }

  if (validSamples == 0) return false;

  avgCurrentA = currentSum / validSamples;
  avgVoltageV = voltageSum / validSamples;
  avgRaw = (int16_t)(rawSum / validSamples);
  return true;
}

// =========================
// MPU6050 functions
// =========================
bool mpuInit()
{
  if (!i2cWrite8(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);

  if (!i2cWrite8(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00)) return false;
  return true;
}

bool mpuReadAccel(float &ax, float &ay, float &az)
{
  uint8_t rx[6];
  if (!i2cReadBytes(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, rx, 6)) return false;

  int16_t rawX = (int16_t)((rx[0] << 8) | rx[1]);
  int16_t rawY = (int16_t)((rx[2] << 8) | rx[3]);
  int16_t rawZ = (int16_t)((rx[4] << 8) | rx[5]);

  ax = ((float)rawX / MPU6050_ACCEL_SENS_2G) * GRAVITY_MPS2;
  ay = ((float)rawY / MPU6050_ACCEL_SENS_2G) * GRAVITY_MPS2;
  az = ((float)rawZ / MPU6050_ACCEL_SENS_2G) * GRAVITY_MPS2;
  return true;
}

// =========================
// I2C scan helper
// =========================
void scanI2C()
{
  Serial.println("I2C probe start");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found I2C device at 0x%02X\n", addr);
    }
  }
  Serial.println("I2C probe done");
}

// =========================
// Current offset calibration
// =========================
bool calibrateCurrentOffset()
{
  Serial.printf("Waiting %lu ms before zero-current calibration...\n",
                (unsigned long)CURRENT_ZERO_CAL_DELAY_MS);
  blinkLedFor(CURRENT_ZERO_CAL_DELAY_MS, 250);

  Serial.println("Calibrating current offset for 3 seconds...");
  Serial.println("Keep the current sensor at resting current now.");

  uint32_t start = millis();
  float sumA = 0.0f;
  int32_t rawSum = 0;
  uint32_t count = 0;

  bool ledState = false;
  uint32_t lastToggle = 0;

  while (millis() - start < CURRENT_ZERO_CAL_TIME_MS) {
    uint32_t now = millis();
    if (now - lastToggle >= 250) {
      lastToggle = now;
      ledState = !ledState;
      setLed(ledState);
    }

    int16_t raw;
    float currentA;

    if (adc16ReadCurrent(currentA, raw)) {
      sumA += currentA;
      rawSum += raw;
      count++;
    }

    delay(10);
  }

  setLed(false);

  if (count == 0) {
    Serial.println("Current offset calibration failed: no valid samples");
    return false;
  }

  g_currentOffsetA = sumA / count;

  Serial.printf(
    "Current offset capture done. avg offset = %.6f A (%ld mA), avg raw = %ld, samples = %lu\n",
    g_currentOffsetA,
    (long)(g_currentOffsetA * 1000.0f),
    (long)(rawSum / (int32_t)count),
    (unsigned long)count
  );

  return true;
}

// =========================
// Flash logging helpers
// =========================
bool appendRecord(const TelemetryRecord_t &rec)
{
  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return false;

  size_t n = f.write((const uint8_t *)&rec, sizeof(rec));
  f.close();   // close every record so random power cut loses at most one record
  return (n == sizeof(rec));
}

void dumpLogCsv()
{
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) {
    Serial.println("No log file found");
    return;
  }

  Serial.println("timestamp_ms,current_mA,voltage_mV,ax_x100,ay_x100,az_x100,amag_x100");

  TelemetryRecord_t rec;
  uint32_t count = 0;

  while (f.available() >= (int)sizeof(TelemetryRecord_t)) {
    size_t n = f.read((uint8_t *)&rec, sizeof(rec));
    if (n != sizeof(rec)) {
      Serial.println("Partial/corrupt record encountered");
      break;
    }

    Serial.printf("%lu,%d,%ld,%d,%d,%d,%u\n",
      (unsigned long)rec.timestamp_ms,
      rec.current_mA,
      (long)rec.voltage_mV,
      rec.accel_x_mps2_x100,
      rec.accel_y_mps2_x100,
      rec.accel_z_mps2_x100,
      rec.accel_mag_mps2_x100
    );
    count++;
  }

  f.close();
  Serial.printf("Dump complete. %lu records.\n", (unsigned long)count);
}

void eraseLog()
{
  if (LittleFS.exists(LOG_FILE)) {
    if (LittleFS.remove(LOG_FILE)) Serial.println("Log erased");
    else Serial.println("Failed to erase log");
  } else {
    Serial.println("No log file to erase");
  }
}

void printHelp()
{
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  s = start logging with a new CSV file");
  Serial.println("  x = stop logging");
  Serial.println("  d = dump log as CSV");
  Serial.println("  e = erase log");
  Serial.println("  h = help");
  Serial.println();
}

void handleSerialCommands()
{
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 's') {
      if (!g_loggingEnabled) {
        startLoggingNewFile("Serial");
      } else {
        Serial.printf("Already logging to %s\n", g_sdReady ? g_sdLogFile : "NO_SD");
      }
    } else if (c == 'x') {
      g_loggingEnabled = false;
      updateLoggingLed();
      Serial.println("Logging STOPPED");
    } else if (c == 'd') {
      g_loggingEnabled = false;
      updateLoggingLed();
      Serial.println("Logging STOPPED for dump");
      dumpLogCsv();
    } else if (c == 'e') {
      g_loggingEnabled = false;
      updateLoggingLed();
      Serial.println("Logging STOPPED for erase");
      eraseLog();
    } else if (c == 'h') {
      printHelp();
    }
  }
}


bool writeSDCsvHeader(const char *path)
{
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("Failed to create %s\n", path);
    return false;
  }

  f.println("timestamp_ms,current_mA,voltage_mV,ax_x100,ay_x100,az_x100,amag_x100");
  f.close();
  return true;
}

bool selectNextSDLogFile()
{
  if (!g_sdReady) return false;

  for (uint16_t i = 1; i <= SD_MAX_LOG_FILES; i++) {
    snprintf(g_sdLogFile, sizeof(g_sdLogFile), "/telemetry_%03u.csv", i);

    if (!SD.exists(g_sdLogFile)) {
      g_sdLogIndex = i;

      if (!writeSDCsvHeader(g_sdLogFile)) {
        return false;
      }

      Serial.printf("New SD log file: %s\n", g_sdLogFile);
      return true;
    }
  }

  Serial.println("No free SD log filename left. Delete old telemetry_###.csv files or raise SD_MAX_LOG_FILES.");
  return false;
}

void startLoggingNewFile(const char *reason)
{
  if (g_sdReady) {
    if (!selectNextSDLogFile()) {
      g_loggingEnabled = false;
      updateLoggingLed();
      Serial.println("Logging NOT started because no SD log file could be created");
      return;
    }
  } else {
    Serial.println("WARNING: SD not ready. Logging enabled, but SD writes will fail.");
  }

  g_loggingEnabled = true;
  updateLoggingLed();

  if (reason) {
    Serial.printf("%s logging STARTED", reason);
  } else {
    Serial.print("Logging STARTED");
  }

  if (g_sdReady) {
    Serial.printf(" -> %s\n", g_sdLogFile);
  } else {
    Serial.println();
  }
}

bool initSDCard()
{
  Serial.println("Initializing SD card...");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD card mount failed");
    return false;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }

  Serial.print("SD card type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  Serial.printf("SD card size: %llu MB\n", SD.cardSize() / (1024 * 1024));

  return true;
}

bool appendRecordSD(const TelemetryRecord_t &rec)
{
  if (!g_sdReady) return false;

  File f = SD.open(g_sdLogFile, FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open SD log file");
    return false;
  }

  f.printf("%lu,%d,%ld,%d,%d,%d,%u\n",
    (unsigned long)rec.timestamp_ms,
    rec.current_mA,
    (long)rec.voltage_mV,
    rec.accel_x_mps2_x100,
    rec.accel_y_mps2_x100,
    rec.accel_z_mps2_x100,
    rec.accel_mag_mps2_x100
  );

  f.close();
  return true;
}

// =========================
// Setup
// =========================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  // Blink while initialization starts. Calibration also blinks later.
  blinkLedFor(1000, 150);

  // SD Card Push Button
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    buttonISR,
    FALLING   // button press pulls pin from HIGH to LOW
  );

  Serial.println();
  Serial.println("ESP32-C3 Telemetry Logger");

  g_sdReady = initSDCard();

  if (!g_sdReady) {
    Serial.println("Continuing without SD card");
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  scanI2C();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
    while (1) delay(1000);
  }
  Serial.println("LittleFS mounted");

  if (!adc16Init()) {
    Serial.println("ADC16 init failed!");
    while (1) delay(1000);
  }
  Serial.println("ADC16 initialized");

  if (!mpuInit()) {
    Serial.println("MPU init failed!");
    while (1) delay(1000);
  }
  Serial.println("MPU initialized");

  if (!calibrateCurrentOffset()) {
    while (1) delay(1000);
  }

  printHelp();

  // Auto-start logging after setup/calibration is complete.
  startLoggingNewFile("AUTO");

  Serial.println("LED ON = logging, LED OFF = not logging, LED blinking = startup/calibration");
  Serial.println("Use 'x' to stop, 's' to start a new CSV, 'd' to dump LittleFS, 'e' to erase LittleFS");
}

// =========================
// Main loop
// =========================
void loop()
{
  handleSerialCommands();

  // SD Card Push Button
  if (buttonEvent) {
    buttonEvent = false;

    uint32_t now = millis();

    // Debounce. Do NOT check digitalRead(BUTTON_PIN) here.
    // The button may already be released by the time loop() gets here.
    if (now - lastButtonHandledMs > 250) {
      lastButtonHandledMs = now;

      if (g_loggingEnabled) {
        g_loggingEnabled = false;
        updateLoggingLed();
        Serial.println("Button: SD logging STOPPED");
      } else {
        startLoggingNewFile("Button");
      }
    }
  }


  if (!g_loggingEnabled) {
    delay(20);
    return;
  }

  float currentA = 0.0f;
  float avgVoltageV = 0.0f;
  int16_t adcRaw = 0;

  float axSum = 0.0f;
  float aySum = 0.0f;
  float azSum = 0.0f;
  uint16_t accelValid = 0;
  
  if (!adc16ReadAverageCurrent(currentA, avgVoltageV, adcRaw)) {
    Serial.println("ADC16 read failed");
    delay(50);
    return;
  }

  for (uint16_t i = 0; i < MPU_NUM_SAMPLES; i++) {
    float ax, ay, az;
    if (mpuReadAccel(ax, ay, az)) {
      axSum += ax;
      aySum += ay;
      azSum += az;
      accelValid++;
    }
    delay(MPU_SAMPLE_DELAY);
  }

  if (accelValid == 0) {
    Serial.println("MPU accel read failed");
    delay(50);
    return;
  }

  float axAvg = axSum / accelValid;
  float ayAvg = aySum / accelValid;
  float azAvg = azSum / accelValid;
  float amag = sqrtf(axAvg * axAvg + ayAvg * ayAvg + azAvg * azAvg);

  float correctedCurrentA = currentA - g_currentOffsetA;

  TelemetryRecord_t rec;
  rec.timestamp_ms = millis();
  rec.current_mA = (int16_t)(correctedCurrentA * 1000.0f);
  rec.voltage_mV = (int32_t)(avgVoltageV * 1000.0f);
  rec.accel_x_mps2_x100 = (int16_t)(axAvg * 100.0f);
  rec.accel_y_mps2_x100 = (int16_t)(ayAvg * 100.0f);
  rec.accel_z_mps2_x100 = (int16_t)(azAvg * 100.0f);
  rec.accel_mag_mps2_x100 = (uint16_t)(amag * 100.0f);

  
  if (appendRecordSD(rec)) {
    Serial.printf(
      "SD LOG %s t=%lu ms I=%d mA V=%ld mV Ax=%d Ay=%d Az=%d Mag=%u raw=%d\n",
      g_sdLogFile,
      (unsigned long)rec.timestamp_ms,
      rec.current_mA,
      (long)rec.voltage_mV,
      rec.accel_x_mps2_x100,
      rec.accel_y_mps2_x100,
      rec.accel_z_mps2_x100,
      rec.accel_mag_mps2_x100,
      adcRaw
    );
  } else {
    Serial.println("SD write failed");
  }

  delay(20);
}