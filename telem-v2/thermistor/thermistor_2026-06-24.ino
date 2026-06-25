/*
  Thermistor via ADS1115 (ESP32-C3 Super Mini)
  ----------------------------------------------------------------
  Reads a 100k NTC thermistor on a pull-up divider through the ADS1115 (AIN2)
 
  Divider topology assumed (pull-up):
      3.3V --[ R1 = 33k ]-- node(AIN2) --[ thermistor ]-- GND
*/

#include <Wire.h>
#include <math.h>

// I2C pins (ESP32-C3 Super Mini)
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

// ADS1115
#define ADS1115_ADDR 0x48

#define ADS1115_REG_CONVERSION      0x00
#define ADS1115_REG_CONFIG          0x01

#define ADS1115_CONFIG_OS_SINGLE    0x8000
#define ADS1115_CONFIG_MUX_AIN2_GND 0x6000   // single-ended AIN2 vs GND (thermistor channel)
#define ADS1115_CONFIG_PGA_4V096    0x0200   // ±4.096 V full scale
#define ADS1115_CONFIG_MODE_SINGLE  0x0100
#define ADS1115_CONFIG_DR_128SPS    0x0080
#define ADS1115_CONFIG_COMP_DISABLE 0x0003

#define ADS1115_CONFIG_WORD_AIN2 (ADS1115_CONFIG_OS_SINGLE   | \
                                  ADS1115_CONFIG_MUX_AIN2_GND | \
                                  ADS1115_CONFIG_PGA_4V096    | \
                                  ADS1115_CONFIG_MODE_SINGLE  | \
                                  ADS1115_CONFIG_DR_128SPS    | \
                                  ADS1115_CONFIG_COMP_DISABLE)

// ±4.096 V over 15 bits -> volts per LSB (= 125 uV)
static const float ADS1115_LSB_VOLTS = 4.096f / 32768.0f;

// Hardware Setup
float R1 = 33500.0;      // Series resistor for thermistors (33kΩ)

// Thermistor curve via the Beta (B-parameter) equation, anchored to a measured point.
float nominalResistance = 95000.0;   // measured thermistor resistance...
float nominalTemp       = 23.5;      // at this measured temperature (°C)
float bCoefficient      = 3950.0;    // Beta value (cheap 100k 3D-printer NTC ≈ 3950 K)

// ADC zero-offset calibration (optional)
// The ADS1115 has a tiny intrinsic offset. Measure it ONCE: ground AIN2 (0 V),
// send 'c' in the Serial Monitor, read the printed value, and paste it into
// ADC_OFFSET_V below. It defaults to 0, so it does nothing until you calibrate.
static const float ADC_OFFSET_V = 0.0f;   // <-- paste your measured offset here
float g_adcOffsetV = ADC_OFFSET_V;        // active value ('c' updates it for the session)

#define OFFSET_CAL_SAMPLES 64

// I2C helpers
bool i2cWrite16BE(uint8_t devAddr, uint8_t regAddr, uint16_t value) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write((uint8_t)((value >> 8) & 0xFF));
  Wire.write((uint8_t)(value & 0xFF));
  return (Wire.endTransmission() == 0);
}

bool i2cReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, size_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  if (Wire.endTransmission(false) != 0) return false;

  size_t n = Wire.requestFrom((int)devAddr, (int)len);
  if (n != len) return false;

  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ADS1115 read
// Trigger one single-shot conversion for the given config word and return the raw count.
bool adsReadRaw(uint16_t configWord, int16_t &raw) {
  if (!i2cWrite16BE(ADS1115_ADDR, ADS1115_REG_CONFIG, configWord)) return false;
  delay(10);   // 128 SPS -> ~8 ms conversion; 10 ms is safe

  uint8_t rx[2];
  if (!i2cReadBytes(ADS1115_ADDR, ADS1115_REG_CONVERSION, rx, 2)) return false;

  raw = (int16_t)((rx[0] << 8) | rx[1]);
  return true;
}

// Read the AIN2 node voltage in volts, offset-corrected. Returns false on I2C error.
bool readThermistorVoltage(float &voltage) {
  int16_t raw;
  if (!adsReadRaw(ADS1115_CONFIG_WORD_AIN2, raw)) return false;
  voltage = ((float)raw * ADS1115_LSB_VOLTS) - g_adcOffsetV;
  return true;
}

// Ground AIN2 (0 V), then send 'c' to measure the ADC zero offset.
// Prints the value so you can paste it into ADC_OFFSET_V above.
void calibrateAdcOffset() {
  Serial.println("Offset cal: make sure AIN2 is at 0 V (grounded). Sampling...");

  double sumV = 0.0;
  uint16_t valid = 0;
  for (uint16_t i = 0; i < OFFSET_CAL_SAMPLES; i++) {
    int16_t raw;
    if (adsReadRaw(ADS1115_CONFIG_WORD_AIN2, raw)) {
      sumV += (double)raw * ADS1115_LSB_VOLTS;
      valid++;
    }
    delay(5);
  }

  if (valid == 0) {
    Serial.println("Offset cal FAILED: no valid samples");
    return;
  }

  g_adcOffsetV = (float)(sumV / valid);
  Serial.print("Offset cal done. measured offset = ");
  Serial.print(g_adcOffsetV, 6);
  Serial.println(" V  -> paste this into ADC_OFFSET_V to make it permanent");
}

float readThermistorTemperature() {
  // Read voltage from the ADS1115 (AIN2)
  float voltage;
  if (!readThermistorVoltage(voltage)) {
    return 99.9;   // ADS1115 / I2C read error
  }

  Serial.print(" Voltage = "); Serial.print(voltage);

  // HARDWARE SAFETY CHECK FIRST
  // Raised to 3.25V so it doesn't fail when the room gets cold!
  if (voltage >= 3.25 || voltage <= 0.1) {
    return 99.9;
  }

  // Calculate Thermistor Resistance (Assuming Pull-Up Divider)
  float R2 = (voltage * R1) / (3.28 - voltage);

  // Beta (B-parameter) equation, anchored to the measured nominal point
  float steinhart;
  steinhart  = R2 / nominalResistance;          // R/Ro
  steinhart  = log(steinhart);                   // ln(R/Ro)
  steinhart /= bCoefficient;                     // (1/B) * ln(R/Ro)
  steinhart += 1.0 / (nominalTemp + 273.15);     // + (1/To)
  steinhart  = 1.0 / steinhart;                  // invert -> Kelvin

  // Return Celsius
  return (steinhart - 273.15);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Wait for serial port to connect. Needed for native USB chips.
  while (!Serial) {
    delay(10);
  }

  Serial.println("\n--- Serial Monitor Initialized ---");

  // Init I2C and probe the ADS1115
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  Wire.beginTransmission(ADS1115_ADDR);
  if (Wire.endTransmission() == 0) Serial.println("ADS1115 found at 0x48");
  else                             Serial.println("WARNING: ADS1115 not responding at 0x48");

  Serial.printf("Thermistor via ADS1115 (AIN2). Using ADC offset = %.6f V.\n", g_adcOffsetV);
  Serial.println("Ground AIN2 and send 'c' to (re)measure the offset.");
}

void loop() {
  // Send 'c' (with AIN2 grounded) to re-measure the ADC offset
  while (Serial.available()) {
    if (Serial.read() == 'c') calibrateAdcOffset();
  }

  float temperature = readThermistorTemperature();

  Serial.print("  Temperature = ");
  Serial.print(temperature);
  Serial.println(" C");

  delay(2000);
}
