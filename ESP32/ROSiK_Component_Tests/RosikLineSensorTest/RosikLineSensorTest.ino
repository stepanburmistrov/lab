/*
  ESP32  │  2-выходной датчик «чёрной линии»
  AIN    │  GPIO39 / SVN  (ADC1_CHANNEL_3)   — аналог 0…4095
  DOUT   │  GPIO12        (вход)             — готовый TTL-уровень
  Печать │  RAW, U, ALG  |  DOUT, LOGIC
*/

#include <Arduino.h>

/* ----------- настройки выводов ----------- */
const uint8_t  AIN_PIN   = 39;         // аналог
const uint8_t  DOUT_PIN  = 12;         // цифровой (GPIO12)
const uint16_t THRESHOLD = 2000;       // порог «чёрное» для аналога

/* ----------- инициализация ----------- */
void setup() {
  Serial.begin(115200);
  delay(100);

  analogReadResolution(12);                       // 0 … 4095
  analogSetPinAttenuation(AIN_PIN, ADC_11db);     // 0–3.3 В

  pinMode(DOUT_PIN, INPUT);                       // цифровой вход

  Serial.println(F("Dual-output line-sensor test"));
  Serial.println(F("RAW  U[V]   ALG  |  DOUT  LOGIC"));
}

/* ----------- основной цикл ----------- */
void loop() {
  /* --- аналог --- */
  uint16_t raw = analogRead(AIN_PIN);
  float    u   = raw * (3.3f / 4095.0f);
  bool     algBlack = raw < THRESHOLD;

  /* --- цифровой --- */
  int  dLevel = digitalRead(DOUT_PIN);            // 0 / 1
  bool digBlack = (dLevel == LOW);                // большинство датчиков дают LOW на чёрном

  /* вывод */
  Serial.printf("%4u  %.2f  %s |  %d     %s\n",
                raw, u,
                algBlack ? "WHITE" : "BLACK",
                dLevel,
                digBlack ? "WHITE" : "BLACK");

  delay(50);
}