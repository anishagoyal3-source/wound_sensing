#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/* ================= BLYNK ================= */
#define BLYNK_TEMPLATE_ID "TMPL3g5t-kpp7"
#define BLYNK_TEMPLATE_NAME "Wound"
#define BLYNK_AUTH_TOKEN "SNMuF5z_KSQOrO017_eROfmy_LeOcbb4"

char ssid[] = "a";
char pass[] = "12345678";

/* ================= ANALOG SENSORS ================= */
#define SOIL_PIN     D0
#define AMMONIA_PIN  D1

#define ADC_MIN  0
#define ADC_MAX  4095

/* ================= MAX30105 ================= */
MAX30105 particleSensor;

#if defined(__AVR_ATmega328P__)
uint16_t irBuffer[100];
uint16_t redBuffer[100];
#else
uint32_t irBuffer[100];
uint32_t redBuffer[100];
#endif

int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

/* ================= BME280 ================= */
Adafruit_BME280 bme;

/* ================= TIMERS ================= */
BlynkTimer timer;

/* ================= SENSOR TASK ================= */
void readAndSendData()
{
  /* ---------- Analog Sensors ---------- */
  int soilRaw    = analogRead(SOIL_PIN);
  int ammoniaRaw = analogRead(AMMONIA_PIN);

  int soilPercent = constrain(map(soilRaw, ADC_MIN, ADC_MAX, 100, 0), 0, 100);
  int ammoniaPercent = constrain(map(ammoniaRaw, ADC_MIN, ADC_MAX, 0, 100), 0, 100);

  /* ---------- MAX30105 ---------- */
  const int bufferLength = 100;
  for (int i = 0; i < bufferLength; i++)
  {
    while (!particleSensor.available())
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    bufferLength,
    redBuffer,
    &spo2,
    &validSPO2,
    &heartRate,
    &validHeartRate
  );

  float maxTemp = particleSensor.readTemperature();
  float bmeTemp = bme.readTemperature();

  /* ---------- Serial Debug ---------- */
  Serial.println("===== Sending to Blynk =====");
  Serial.print("MAX Temp: "); Serial.println(maxTemp);
  Serial.print("SpO2: "); Serial.println(validSPO2 ? spo2 : -1);
  Serial.print("Ammonia: "); Serial.println(ammoniaPercent);
  Serial.print("Soil: "); Serial.println(soilPercent);
  Serial.print("BME Temp: "); Serial.println(bmeTemp);

  /* ---------- Blynk Virtual Pins ---------- */
  Blynk.virtualWrite(V0, maxTemp);
  Blynk.virtualWrite(V1, validSPO2 ? spo2 : 0);
  Blynk.virtualWrite(V2, ammoniaPercent);
  Blynk.virtualWrite(V3, soilPercent);
  Blynk.virtualWrite(V4, bmeTemp);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  analogReadResolution(12);

  /* ---------- WiFi & Blynk ---------- */
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  /* ---------- MAX30105 ---------- */
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57))
  {
    Serial.println(" MAX30105 not found");
    while (1);
  }

  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  Serial.println(" MAX30105 ready");

  /* ---------- BME280 ---------- */
  if (!bme.begin(0x76))
  {
    Serial.println(" BME280 not found");
    while (1);
  }

  Serial.println(" BME280 ready");

  /* ---------- Timer ---------- */
  timer.setInterval(2000L, readAndSendData); // every 2 seconds
}

void loop()
{
  Blynk.run();
  timer.run();
}