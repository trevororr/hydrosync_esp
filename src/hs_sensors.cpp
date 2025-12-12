#include <Arduino.h>
#include "Wire.h"
#include "Adafruit_INA219.h"
#include "hs_sensors.h"
#include <ArduinoJson.h>

Adafruit_INA219 ina219;
JsonDocument current_vals;
bool ina219_ok = false;

//I2C using DEFAULT SDA on GPIO 21 and SCL on GPIO 22
/*I2C Sensors Addresses/Configurations:
  - INA219 Current Sensor
    - 0x40 (default)
    - 0x41 (A0 Bridge)
    - 0x44 (A1 Bridge)
    - 0x45 (A0 & A1 Bridge)
  - Flow Sensor (YF-S201)
    - Analog Output on GPIO 4 with Interrupts
  - Seeed Water Level Sensor
    - ATTINY1 (High) - 0x78
    - ATTINY2 (Low)  - 0x77
*/

#define FLOW_SENSOR_PIN 4 // GPIO pin connected to the flow sensor
void IRAM_ATTR flow_interrupt(); // Interrupt Function Prototype (iram_attr for ISR on ESP32)
void try_init_ina219(); // INA219 init retry function prototype

void sensor_init() {
  Wire.begin(); // Initialize I2C with DEFAULT SDA on GPIO 21 and SCL on GPIO 22

  const int INA_RETRY_MS = 2000;
  int begin_ina_try = millis();

  while (!ina219.begin()) {
    int now = millis();
    // Timeout for INA219 init retries
    if(now - begin_ina_try > INA_RETRY_MS) {
      break;
    }
    try_init_ina219();
    delay(100);
  }

  pinMode(FLOW_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flow_interrupt, RISING);

  Serial.println("Sensors initialized!");
}

JsonDocument get_current() {
  if (!ina219_ok) {
    // INA219 still not available: return safe placeholders
    current_vals["connected"]   = false;
    current_vals["current_mA"]  = 0.0f;
    current_vals["power_mW"]    = 0.0f;
    current_vals["load_V"]      = 0.0f;
    return current_vals;
  }

  float shunt_mV = ina219.getShuntVoltage_mV();
  float bus_V = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float load_V = bus_V + (shunt_mV / 1000.0f);

  current_vals["current_mA"] = current_mA;
  current_vals["power_mW"] = power_mW;
  current_vals["load_V"] = load_V;

  return current_vals;
}

void try_init_ina219() {
  if (ina219_ok) return; // already good

  Serial.println("Attempting INA219 init...");

  if (ina219.begin()) {
    ina219.setCalibration_16V_400mA();
    ina219_ok = true;
    Serial.println("INA219 initialized successfully.");
  } else {
    ina219_ok = false;
    Serial.println("INA219 not found (non-fatal). Will retry later.");
  }
}

/*
Flow Sensor Functions
*/
volatile unsigned long flow_pulse_count = 0;
unsigned long last_pulse_check = 0;
double flow_lpm = 0.0;  // Liters per minute

// ---- ISR ----
void IRAM_ATTR flow_interrupt() {
  flow_pulse_count++;
}

// ---- Efficient flow computation ----
double get_flow() {
  const double pulses_per_liter = 450.0; // typical for YF-S201 sensor
  const unsigned long now = millis();
  const unsigned long elapsed = now - last_pulse_check;

  noInterrupts();
  unsigned long pulses = flow_pulse_count;
  flow_pulse_count = 0;
  interrupts();

  last_pulse_check = now;

  // pulses per ms → L/min
  double liters = (double)pulses / pulses_per_liter;
  double seconds = (double)elapsed / 1000.0;
  double flow = liters / seconds;

  return flow;  // Liters per second
}
/*
End of Flow Sensor Functions
*/

/*
Seeed Water Level Sensor Functions
*/
// --- Tunables ---
constexpr uint8_t ATTINY1_HIGH_ADDR = 0x78;
constexpr uint8_t ATTINY2_LOW_ADDR  = 0x77;
constexpr uint8_t LOW_N  = 8;
constexpr uint8_t HIGH_N = 12;
constexpr uint8_t TOTAL_SECTIONS = LOW_N + HIGH_N; // 20
constexpr uint8_t THRESHOLD = 100;
constexpr uint8_t SENSOR_MIN = 250, SENSOR_MAX = 255; // for optional counts
constexpr uint16_t I2C_TIMEOUT_MS = 5;

// --- Buffers ---
uint8_t low_data[LOW_N];
uint8_t high_data[HIGH_N];

static bool readI2C(uint8_t addr, uint8_t* buf, size_t n, uint16_t timeout_ms = I2C_TIMEOUT_MS) {
  size_t got = Wire.requestFrom(addr, (uint8_t)n, (uint8_t)true);
  unsigned long t0 = millis();
  while (got < n && (millis() - t0) < timeout_ms) {
    // give hardware time
    yield();
    got = Wire.available(); // how many actually buffered
  }
  if (Wire.available() < (int)n) {
    // drain any partial data to keep bus clean
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}

// Count trailing 1s efficiently (0..TOTAL_SECTIONS)
static uint8_t countTrailingOnes(uint32_t x) {
  const uint32_t mask = (TOTAL_SECTIONS == 32) ? 0xFFFFFFFFu : ((1u << TOTAL_SECTIONS) - 1u);
  x &= mask;
  uint32_t inv = (~x) & mask;        // 1s→0s, so trailing 1s become trailing 0s
  if (inv == 0) return TOTAL_SECTIONS; // all ones
  return (uint8_t)__builtin_ctz(inv);  // trailing zeros of ~x == trailing ones of x
}

uint8_t get_water_level() {
  if (!readI2C(ATTINY2_LOW_ADDR,  low_data,  LOW_N))  return 0;   // or last-known
  if (!readI2C(ATTINY1_HIGH_ADDR, high_data, HIGH_N)) return 0;

  // Optional: counts in valid range (kept in case you use them)
  uint8_t low_count = 0, high_count = 0;

  // Build bitmask & compute counts in a single pass
  uint32_t touch_val = 0;

  for (uint8_t i = 0; i < LOW_N; ++i) {
    uint8_t v = low_data[i];
    if (v >= SENSOR_MIN && v <= SENSOR_MAX) ++low_count;
    if (v > THRESHOLD) touch_val |= (1u << i);
  }
  for (uint8_t i = 0; i < HIGH_N; ++i) {
    uint8_t v = high_data[i];
    if (v >= SENSOR_MIN && v <= SENSOR_MAX) ++high_count;
    if (v > THRESHOLD) touch_val |= (1u << (LOW_N + i));
  }

  const uint8_t wet_sections = countTrailingOnes(touch_val);
  return wet_sections * 5;  // mm
}
/*
End of Seeed Water Level Sensor Functions
*/
