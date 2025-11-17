#include <Wire.h>
#include <BlynkSimpleEsp32.h>
#include <MAX30105.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define BLYNK_TEMPLATE_ID "TMPL3--ueCpv_"
#define BLYNK_TEMPLATE_NAME "Dog health Monitor"
#define BLYNK_AUTH_TOKEN "qY5wwiV5vuwM_PKTiHFtPcwDojhM0vJ7"
char ssid[] = "TIS";
char pass[] = "Wecanwewill";

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// Sensor objects
MAX30105 particleSensor;
Adafruit_BME280 bme;

// Analog & digital pins
const int FsAnalogPin       = 27; // FSR -> ADC1_CH7 (input only)
const int AmmAnalogPin      = 26; // Ammonia NH3 analog -> ADC1_CH6 (input only)
const int MoistureDigitalPin = 23; // LM393 digital output (HIGH/LOW)

// Blynk Virtual Pins
#define V_SPO2     V1
#define V_TEMP     V2
#define V_FSR      V3
#define V_MOISTURE V4
#define V_AMM_RAW  V5
#define V_AMM_VOLT V6

// Timers / windows
const unsigned long WINDOW_MS = 4000;     // SpO2 sampling window (ms)
const unsigned long SEND_INTERVAL = 7000; // send interval (ms)
unsigned long lastSend = 0;

// ---------- Helpers ----------
bool isFiniteFloat(float x) {
  return !(isnan(x) || isinf(x));
}

float computeSpO2(const std::vector<uint32_t>& red, const std::vector<uint32_t>& ir) {
  if (red.size() < 20 || red.size() != ir.size()) return -1;
  size_t n = red.size();
  double meanR = 0, meanIR = 0;
  for (size_t i = 0; i < n; ++i) { meanR += red[i]; meanIR += ir[i]; }
  meanR /= (double)n;
  meanIR /= (double)n;

  double maxR = -1e12, minR = 1e12, maxIR = -1e12, minIR = 1e12;
  for (size_t i = 0; i < n; ++i) {
    double r = (double)red[i] - meanR;
    double irv = (double)ir[i] - meanIR;
    if (r > maxR) maxR = r;
    if (r < minR) minR = r;
    if (irv > maxIR) maxIR = irv;
    if (irv < minIR) minIR = irv;
  }

  double acR = (maxR - minR) / 2.0;
  double acIR = (maxIR - minIR) / 2.0;
  if (acR <= 1 || acIR <= 1) return -1;

  double ratio = (acR / meanR) / (acIR / meanIR);
  if (!isFiniteFloat((float)ratio) || ratio <= 0.0) return -1;

  double spo2 = 110.0 - 25.0 * ratio; // empirical approximation
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 0) spo2 = 0;
  return (float)spo2;
}

// ---------- Setup sensor functions ----------
void setupMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    return;
  }
  Serial.println("MAX30102 found.");
  // Default setup - tune if needed
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
}

void setupBME280() {
  if (!bme.begin(0x76)) {
    // try alternate address
    if (!bme.begin(0x77)) {
      Serial.println("BME280 not found. Check wiring.");
      return;
    }
  }
  Serial.println("BME280 OK.");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // ADC resolution and attenuation
  analogReadResolution(12); // 0 - 4095
  // Set attenuation for analog pins so they can read up to ~3.3V
  analogSetPinAttenuation(FsAnalogPin, ADC_11db);
  analogSetPinAttenuation(AmmAnalogPin, ADC_11db);

  // pin modes
  pinMode(MoistureDigitalPin, INPUT);

  // initialize sensors
  setupMAX30102();
  setupBME280();

  // Connect to Blynk (blocks until connected)
  Blynk.begin(auth, ssid, pass);

  Serial.println("Setup complete.");
}

// ---------- Main loop ----------
void loop() {
  Blynk.run();

  unsigned long now = millis();
  if (now - lastSend < SEND_INTERVAL) return;
  lastSend = now;

  // ---------- SpO2 sampling ----------
  std::vector<uint32_t> redSamples;
  std::vector<uint32_t> irSamples;
  redSamples.reserve(500);
  irSamples.reserve(500);

  unsigned long start = millis();
  while (millis() - start < WINDOW_MS) {
    if (particleSensor.available()) {
      uint32_t r = particleSensor.getRed();
      uint32_t ir = particleSensor.getIR();
      redSamples.push_back(r);
      irSamples.push_back(ir);
      particleSensor.nextSample();
    } else {
      delay(2);
    }
  }

  float spo2 = computeSpO2(redSamples, irSamples);
  if (spo2 < 0.0f) {
    Serial.println("SpO2: bad signal / no finger");
    Blynk.virtualWrite(V_SPO2, "-");
  } else {
    Serial.printf("SpO2 = %.1f %%\n", spo2);
    Blynk.virtualWrite(V_SPO2, spo2);
  }

  // ---------- BME280 Temperature ----------
  float tempC = bme.readTemperature(); // Celsius
  if (!isFiniteFloat(tempC)) {
    Serial.println("BME Temp read failed");
    Blynk.virtualWrite(V_TEMP, "-");
  } else {
    Serial.printf("Temp = %.2f C\n", tempC);
    Blynk.virtualWrite(V_TEMP, tempC);
  }

  // ---------- FSR analog ----------
  uint16_t fsrRaw = analogRead(FsAnalogPin); // 0 - 4095
  Serial.printf("FSR raw = %u\n", fsrRaw);
  Blynk.virtualWrite(V_FSR, fsrRaw);

  // ---------- Moisture (LM393 digital) ----------
  int moistureDigital = digitalRead(MoistureDigitalPin); // HIGH or LOW
  Serial.printf("Moisture digital = %d\n", moistureDigital);
  Blynk.virtualWrite(V_MOISTURE, moistureDigital);

  // ---------- Ammonia (NH3) analog ----------
  uint16_t ammRaw = analogRead(AmmAnalogPin); // 0 - 4095
  float ammVoltage = (ammRaw / 4095.0f) * 3.3f; // approximate voltage in volts
  Serial.printf("Ammonia raw = %u  Voltage = %.3f V\n", ammRaw, ammVoltage);
  Blynk.virtualWrite(V_AMM_RAW, ammRaw);
  Blynk.virtualWrite(V_AMM_VOLT, ammVoltage);
}