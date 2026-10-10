// ESP32 + ДВА VL53L0X на одной I2C-шине.
// SDA=21, SCL=22, XSHUT левый=19, правый=18.
// На старте оба отключаются через XSHUT, затем левый получает 0x30,
// правый остаётся на заводском 0x29. Адреса НЕ постоянные: при перезапуске
// последовательность назначения повторяется.
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint8_t SDA_PIN = 21, SCL_PIN = 22;
constexpr uint8_t XSHUT_LEFT = 19, XSHUT_RIGHT = 18;
constexpr uint8_t ADDRESS_LEFT = 0x30;
VL53L0X leftSensor, rightSensor;

void setup() {
  Serial.begin(115200);
  // XSHUT на большинстве модулей рассчитан на подтяжку к питанию сенсора.
  // GPIO ESP32 нельзя подавать 5 В. Убедитесь, что XSHUT не подтянут к 5 В.
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  // Для пробуждения отпускаем линию XSHUT (вход с внешней подтяжкой),
  // не выдаём 5 В или неподходящее напряжение на XSHUT.
  pinMode(XSHUT_LEFT, INPUT);
  delay(50);
  leftSensor.setTimeout(300);
  if (!leftSensor.init()) {
    Serial.println("ERROR: left sensor init failed");
    while (true) delay(1000);
  }
  leftSensor.setAddress(ADDRESS_LEFT);
  leftSensor.setMeasurementTimingBudget(50000);
  leftSensor.startContinuous(0);
  Serial.println("LEFT: 0x30 ready");

  pinMode(XSHUT_RIGHT, INPUT);
  delay(50);
  rightSensor.setTimeout(300);
  if (!rightSensor.init()) {
    Serial.println("ERROR: right sensor init failed");
    while (true) delay(1000);
  }
  rightSensor.setMeasurementTimingBudget(50000);
  rightSensor.startContinuous(0);
  Serial.println("RIGHT: 0x29 ready");
}

void loop() {
  uint16_t left = leftSensor.readRangeContinuousMillimeters();
  bool leftValid = !leftSensor.timeoutOccurred() && left < 8190 && left > 0;
  uint16_t right = rightSensor.readRangeContinuousMillimeters();
  bool rightValid = !rightSensor.timeoutOccurred() && right < 8190 && right > 0;
  Serial.print("LEFT (0x30): ");
  if (leftValid) Serial.print(left); else Serial.print("ERROR");
  Serial.print(" mm  |  RIGHT (0x29): ");
  if (rightValid) Serial.print(right); else Serial.print("ERROR");
  Serial.println(" mm");
  delay(100);
}
