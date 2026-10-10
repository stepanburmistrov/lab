/*
 * VL53L0X: RAW и скорректированное расстояние, мм.
 * Для КАЛИБРОВКИ снимите усреднённые RAW при ДВУХ известных расстояниях,
 * рассчитайте G=(REF2-REF1)/(RAW2-RAW1), O=REF1-G*RAW1.
 * Пример REF1=60, RAW1=90; REF2=200, RAW2=250:
 * G=0.875, O=-18.75 (а НЕ 0.969/-37.21).
 * Ниже 0.969/-37.21 — отдельные опытные константы ROSiK, не из примера.
 */
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;
constexpr float CAL_GAIN = 0.969f;   // ЗАМЕНИТЕ после своей калибровки
constexpr float CAL_OFFSET = -37.21f;
constexpr uint8_t SDA_PIN = 21, SCL_PIN = 22;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("VL53L0X not found (0x29)!");
    while (true) delay(1000);
  }
  tof.setMeasurementTimingBudget(200000); // 200 мс
  Serial.println("VL53L0X ready: RAW and CORR (mm)");
}

void loop() {
  uint16_t raw = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred() || raw == 0 || raw >= 8190) {
    Serial.println("TIMEOUT / INVALID");
  } else {
    float corrected = CAL_GAIN * raw + CAL_OFFSET;
    uint16_t corr = (corrected <= 0) ? 0 : (uint16_t)(corrected + 0.5f);
    Serial.printf("RAW=%4u mm  |  CORR=%4u mm\n", raw, corr);
  }
  delay(100);
}
