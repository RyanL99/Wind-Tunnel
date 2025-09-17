#include "DHT.h"
#include "HX711.h"

// =====================
// === Pin Assignments ===
// Adjust to match your wiring
// HX711 modules (each load cell has its own HX711)
const int DOUT1 = 2;  const int SCK1 = 3;   // Front lift
const int DOUT2 = 4;  const int SCK2 = 5;   // Front drag
const int DOUT3 = 6;  const int SCK3 = 7;   // Rear lift
const int DOUT4 = 8;  const int SCK4 = 9;   // Rear drag

// Analog pressure transducer
const int PT_PIN = A0;  // change if needed

// DHT sensor
const int DHTPIN = 10;          // digital pin for DHT
const uint8_t DHTTYPE = DHT22;  // DHT11, DHT22, DHT21

// =====================
// === Objects ===
HX711 scale1, scale2, scale3, scale4;
DHT dht(DHTPIN, DHTTYPE);

// Track which HX711s are alive (avoid blocking if one is missing)
bool hx1_ok = false, hx2_ok = false, hx3_ok = false, hx4_ok = false;

// =====================
// === Calibration & constants ===
// Supply voltage used by the pressure transducer (Volts)
const float VCC = 5.0;   // set to 3.3 if using a 3V3 board

// ADC resolution (counts)
#if defined(ESP32)
  const float ADC_COUNTS = 4095.0f;  // 12-bit by default
#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
  const float ADC_COUNTS = 4095.0f;  // many are 12-bit; adjust if needed
#else
  const float ADC_COUNTS = 1023.0f;  // AVR UNO/Nano are 10-bit
#endif

// Pressure sensor model (typical ratiometric 0.5–4.5 V for 0–P_MAX)
// If your sensor differs, adjust these two values
const float P_MAX_PA = 7000.0f;      // full-scale pressure in Pa (example: 7 kPa)
const float V_OFFSET_FRAC = 0.10f;   // 10% of VCC at zero pressure (0.5 V on 5 V)

// Gas constant for dry air (J/(kg·K))
const float R_AIR = 287.05f;

// Ambient static pressure (Pa). Adjust for local elevation if you wish.
float ambient_pressure_Pa = 101325.0f;

// HX711 calibration factors (counts per unit); set via your calibration routine
// Positive numbers; sign is handled by wiring orientation.
float SCALE1 = 1.0f, SCALE2 = 1.0f, SCALE3 = 1.0f, SCALE4 = 1.0f;  // placeholder – set properly!

// State
float f1 = 0, f2 = 0, f3 = 0, f4 = 0;    // load readings (your chosen units)
float temperatureC = NAN;                 // latest DHT temperature
unsigned long lastDHT = 0;                // DHT reads no more than every 2s

// Baseline for zeroing pressure transducer at startup
bool baselineSet = false;
float pressureBaseline_Pa = 0.0f;

// Simple moving-average for pressure (reduce noise)
const int P_FILT_N = 8;
float pBuf[P_FILT_N];
int pIdx = 0;
int pCount = 0;

// =====================
// === Helpers ===
float filteredPressure(float p) {
  pBuf[pIdx] = p; pIdx = (pIdx + 1) % P_FILT_N; if (pCount < P_FILT_N) pCount++;
  float s = 0; for (int i = 0; i < pCount; ++i) s += pBuf[i];
  return s / pCount;
}

float readPressurePa() {
  int adc = analogRead(PT_PIN);
  float v = (adc * (VCC / ADC_COUNTS));
  const float V0 = V_OFFSET_FRAC * VCC;          // zero-pressure volts
  const float span = VCC * (1.0f - 2.0f * V_OFFSET_FRAC);  // e.g., 0.8*VCC
  float p = (v - V0) * (P_MAX_PA / span);        // linear map to Pa
  if (!baselineSet) { pressureBaseline_Pa = p; baselineSet = true; }
  p -= pressureBaseline_Pa;                      // zeroed dynamic pressure estimate
  if (p < 0) p = 0;                              // clamp small negative noise
  return filteredPressure(p);
}

void maybeReadDHT() {
  const unsigned long NOW = millis();
  if (NOW - lastDHT >= 2000UL || isnan(temperatureC)) { // DHT library caches result <2s
    float t = dht.readTemperature(false /*Celsius*/);
    if (!isnan(t)) temperatureC = t;
    lastDHT = NOW;
  }
}

void maybeReadScale(HX711 &s, float cal, float &out) {
  if (s.is_ready()) {
    s.set_scale(cal);
    out = s.get_units(3); // average 3 samples
  }
}

bool safeTare(HX711 &s, byte times = 10) {
  // Try to avoid the library's infinite wait by using timeout
  if (!s.wait_ready_timeout(2000, 1)) {
    return false;
  }
  s.tare(times);
  return true;
}


// =====================
// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println(F("booting WinTunnel..."));

  // HX711s (begin first, then try a non-blocking tare)
  scale1.begin(DOUT1, SCK1);  hx1_ok = safeTare(scale1, 10);
  scale2.begin(DOUT2, SCK2);  hx2_ok = safeTare(scale2, 10);
  scale3.begin(DOUT3, SCK3);  hx3_ok = safeTare(scale3, 10);
  scale4.begin(DOUT4, SCK4);  hx4_ok = safeTare(scale4, 10);

  if (!hx1_ok || !hx2_ok || !hx3_ok || !hx4_ok) {
    Serial.print(F("HX711 status: "));
    Serial.print(hx1_ok); Serial.print(',');
    Serial.print(hx2_ok); Serial.print(',');
    Serial.print(hx3_ok); Serial.print(',');
    Serial.println(hx4_ok);
  }

  // DHT
  dht.begin();

  // Prime pressure filter & baseline
  for (int i = 0; i < P_FILT_N; ++i) pBuf[i] = 0;
  (void)readPressurePa();

  // Header for logging (tab-separated)
  Serial.println(F("wind_mps\tdensity_kgm3\tfrontLift\tfrontDrag\trearLift\trearDrag\ttC\tPdyn_Pa"));
  Serial.flush();
}

// =====================
// === Main Loop ===
void loop() {
  // Non-blocking reads – only update when data is ready
  maybeReadScale(scale1, SCALE1, f1);
  maybeReadScale(scale2, SCALE2, f2);
  maybeReadScale(scale3, SCALE3, f3);
  maybeReadScale(scale4, SCALE4, f4);

  // Pressure & temperature
  float Pdyn = readPressurePa();                 // Pa
  maybeReadDHT();                                // updates temperatureC every ≥2 s
  float T_K = (isnan(temperatureC) ? 20.0f : temperatureC) + 273.15f;

  // Air density from ideal gas (using ambient P minus dynamic pressure)
  float density = (ambient_pressure_Pa - Pdyn) / (R_AIR * T_K);
  if (density < 0.2f) density = 0.2f;            // guard against pathological values

  // Wind speed from dynamic pressure: q = 0.5 * rho * V^2
  float wind_mps = sqrtf((2.0f * Pdyn) / density);

  // Output row (TSV)
  Serial.print(wind_mps); Serial.print('\t');
  Serial.print(density);  Serial.print('\t');
  Serial.print(f1);       Serial.print('\t');
  Serial.print(f2);       Serial.print('\t');
  Serial.print(f3);       Serial.print('\t');
  Serial.print(f4);       Serial.print('\t');
  Serial.print(temperatureC); Serial.print('\t');
  Serial.println(Pdyn);

  delay(50);  // ~20 Hz
}
