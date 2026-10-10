// ESP32 + один VL53L0X (библиотека Pololu VL53L0X)
// SDA GPIO21, SCL GPIO22; питание и уровни по спецификации модуля.
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  sensor.setTimeout(300);
  if (!sensor.init()) {
    Serial.println("ERROR: VL53L0X not found (0x29)");
    while (true) delay(1000);
  }
  sensor.setMeasurementTimingBudget(50000); // 50 мс на замер
  sensor.startContinuous(0);
  Serial.println("VL53L0X ready. Distances in mm:");
}

void loop() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || mm >= 8190) {
    Serial.println("Invalid measurement / timeout");
  } else {
    Serial.printf("Distance: %u mm\n", mm);
  }
  delay(100);
}
