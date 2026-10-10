/*******************************************************************
 *  ESP32 + VL53L0X  (I²C 400 kГц)  ──>     Точные расстояния
 *  • Поддерживает линейный коэффициент   G   и   офсет   O
 *  • Выводит  Draw  и  Dcorr  (мм)
 *  • Коррекция ≈ ±3 мм на диапазоне 50–250 мм
 *******************************************************************/
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;

/* ------------ 1. ВНЕСИТЕ свои калибровочные константы ------------- */
/*   Измерьте raw-значения при двух точках (REF1, REF2) и посчитайте   */
/*     G = (REF2-REF1)/(RAW2-RAW1);      O = REF1 – G·RAW1             */
/*   Ниже пример для:  60 мм REF →  90 мм raw,   200 мм REF → 250 мм raw*/
const float CAL_GAIN   = 0.969f;     // G
const float CAL_OFFSET = -37.21f;    // O   (мм)
/* ------------------------------------------------------------------ */

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);        // SDA, SCL, 400 kHz

  if (!tof.init()) {
    Serial.println(F("VL53L0X not found!")); while (1) delay(100);
  }
  tof.setMeasurementTimingBudget(200000);  // 200 мс
  Serial.println(F("VL53L0X with linear calibration ready"));
}

static inline uint16_t corr(uint16_t raw)
{
  float d = CAL_GAIN * raw + CAL_OFFSET;    // линейная коррекция
  return (d < 0) ? 0 : (uint16_t)(d + 0.5f);
}

void loop() {
  uint16_t dRaw = tof.readRangeSingleMillimeters();
  uint16_t dCor = corr(dRaw);

  if (tof.timeoutOccurred()) {
    Serial.println(F("TIMEOUT"));
  } else {
    Serial.printf("RAW=%4u mm   →   CORR=%4u mm\n", dRaw, dCor);
  }
  delay(100);                          // 10 Гц
}