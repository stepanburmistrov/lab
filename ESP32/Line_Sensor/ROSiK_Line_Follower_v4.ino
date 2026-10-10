/*
  ROSiK — следование по КРАЮ чёрной линии одним аналоговым датчиком.
  Основано на электрических подключениях full_firmware(2).ino.

  Arduino IDE: ESP32 (классический ESP32), FastLED, драйвер PCNT (ESP32 core 2.x).
  Serial Monitor 115200, окончания строк не важны.
  w — измерить белое, b — измерить чёрное, g — старт, s — стоп.
  После калибровки по двум точкам значения сохраняются в NVS.
  GPIO25: кнопка на GND, INPUT_PULLUP; короткое нажатие — старт/стоп.
  СНАЧАЛА калибруйте на своей трассе, затем ставьте датчик на край линии.
  Чёрная линия должна быть слева от датчика (ведение по правому краю).
*/
#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>
#include "driver/pcnt.h"
#include <math.h>

#define BUTTON_PIN 25       // кнопка запуска: GPIO25 -> GND
#define LINE_A_PIN 39       // аналоговый сигнал нижнего датчика (0..4095)
#define LINE_D_PIN 12       // цифровой сигнал (LOW = чёрное) для диагностики
#define L_A 33
#define L_B 32
#define R_A 27
#define R_B 26
#define ENC_R_A 18
#define ENC_R_B 19
#define ENC_L_A 34
#define ENC_L_B 35
#define LED_PIN 14
#define NUM_LEDS 8
#define LED_TYPE WS2812
#define COLOR_ORDER GRB

constexpr float WHEEL_D_MM = 44.0f;
constexpr float TICKS_REV = 2930.0f;
constexpr float MM_PER_TICK = WHEEL_D_MM * PI / TICKS_REV;

// Параметры, подбираемые на трассе (v4: увеличен допуск потери линии и усилен поворот)
constexpr float BASE_SPEED = 48.0f;     // мм/с
constexpr float STEER_KP = 75.0f;       // изменение скорости колёс (мм/с) на ошибку ±0.5
constexpr float MAX_STEER = 30.0f;      // мм/с
constexpr float MAX_SPEED = 100.0f;     // мм/с
constexpr uint32_t LOST_TIMEOUT = 3500; // стоп при длительном полном белом ИЛИ чёрном
constexpr float MIN_WHEEL_SPEED = 18.0f; // не даём колесу замирать на коррекции
constexpr float MAX_STEER_RATE = 155.0f; // плавное изменение коррекции, мм/с за секунду
constexpr uint32_t ENCODER_FAULT_MS = 850; // защита: цель есть, энкодер не считает
constexpr float MAX_ENCODER_MM_S = 350.0f; // отсечение явно ошибочного измерения
constexpr uint16_t MIN_CONTRAST = 140;   // порог различия белого и чёрного (ADC)

Preferences prefs;
CRGB leds[NUM_LEDS];
// Для предотвращения записи случайно смешанной калибровки с прошлым запуском
bool whiteMeasuredThisSession=false, blackMeasuredThisSession=false;
bool buttonStable=HIGH, buttonLastRead=HIGH;
uint32_t buttonLastChange=0;
constexpr uint32_t BUTTON_DEBOUNCE_MS=45;
int whiteRaw = -1, blackRaw = -1;
bool enabled = false;
float filtered = 0;
bool filterReady = false;
uint32_t whiteSince = 0;
uint32_t blackSince = 0;
uint32_t badEncoderSinceL = 0, badEncoderSinceR = 0;
float appliedSteer = 0;
uint32_t lastLog = 0;
uint32_t lastControl = 0;
float targetL = 0, targetR = 0;
float iL = 0, iR = 0;
int32_t totalL = 0, totalR = 0;

void motorsOff() {
  targetL = targetR = 0;
  iL = iR = 0;
  analogWrite(L_A, 0); analogWrite(L_B, 0);
  analogWrite(R_A, 0); analogWrite(R_B, 0);
}
void drivePwm(int a, int b, float command) {
  int pwm = constrain((int)roundf(fabsf(command)), 0, 255);
  analogWrite(a, command >= 0 ? pwm : 0);
  analogWrite(b, command < 0 ? pwm : 0);
}
void pcntInit(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t c = {};
  c.unit = unit;
  c.channel = PCNT_CHANNEL_0;
  c.pulse_gpio_num = pinA;
  c.ctrl_gpio_num = pinB;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = 32767;
  c.counter_l_lim = -32768;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
int16_t popTicks(pcnt_unit_t unit) {
  int16_t n=0;
  pcnt_get_counter_value(unit, &n);
  pcnt_counter_clear(unit);
  return n;
}
void ring(CRGB c) {
  fill_solid(leds, NUM_LEDS, c);
  FastLED.show();
}
int sampleAnalog() {
  uint32_t sum=0;
  for(int i=0;i<32;i++) { sum+=analogRead(LINE_A_PIN); delay(2); }
  return (int)(sum/32);
}
bool calibrationValid() {
  return whiteRaw>=0 && whiteRaw<=4095 && blackRaw>=0 && blackRaw<=4095 &&
         abs(blackRaw-whiteRaw)>=MIN_CONTRAST;
}
void stopFollower(const char* reason="STOP") {
  enabled=false;
  motorsOff();
  ring(calibrationValid()?CRGB::Red:CRGB::Blue);
  Serial.println(reason);
}
void startFollower() {
  if(!calibrationValid()) {
    Serial.println("CALIBRATION REQUIRED: w (white), b (black); check contrast");
    return;
  }
  enabled=true;
  filterReady=false;
  whiteSince=blackSince=0;
  badEncoderSinceL=badEncoderSinceR=0;
  appliedSteer=0;
  iL=iR=0;
  lastControl=millis();
  Serial.printf("GO | WHITE=%d BLACK=%d | black line on LEFT\n",whiteRaw,blackRaw);
}
void toggleFollower() {
  if(enabled) stopFollower("BUTTON -> STOP");
  else startFollower();
}
void handleButton() {
  bool readNow=digitalRead(BUTTON_PIN);
  uint32_t now=millis();
  if(readNow!=buttonLastRead) {
    buttonLastRead=readNow;
    buttonLastChange=now;
  }
  if((uint32_t)(now-buttonLastChange)>=BUTTON_DEBOUNCE_MS && readNow!=buttonStable) {
    buttonStable=readNow;
    if(buttonStable==LOW) toggleFollower();  // один старт на одно нажатие
  }
}
void saveCalibrationIfComplete() {
  if(!whiteMeasuredThisSession || !blackMeasuredThisSession) return;
  if(!calibrationValid()) {
    Serial.println("NOT SAVED: insufficient contrast; recalibrate both surfaces");
    return;
  }
  // Одна транзакция сохранения пары, чтение разрешаем только по признаку valid.
  prefs.begin("line-cal",false);
  prefs.putBool("valid",false);
  prefs.putInt("white",whiteRaw);
  prefs.putInt("black",blackRaw);
  prefs.putBool("valid",true);
  prefs.end();
  Serial.printf("NVS SAVED: WHITE=%d BLACK=%d\n",whiteRaw,blackRaw);
  whiteMeasuredThisSession=blackMeasuredThisSession=false;
}
void loadCalibration() {
  prefs.begin("line-cal",true);
  bool stored=prefs.getBool("valid",false);
  int w=prefs.getInt("white",-1);
  int b=prefs.getInt("black",-1);
  prefs.end();
  if(stored && w>=0 && w<=4095 && b>=0 && b<=4095 && abs(b-w)>=MIN_CONTRAST) {
    whiteRaw=w;
    blackRaw=b;
    Serial.printf("NVS LOADED: WHITE=%d BLACK=%d | press button to start\n",w,b);
  } else {
    Serial.println("NO VALID NVS CALIBRATION: use w and b first");
  }
}
void handleSerial() {
  while (Serial.available()) {
    char c=tolower(Serial.read());
    if(c=='s') stopFollower("SERIAL -> STOP");
    if(c=='w' || c=='b') {
      stopFollower("CALIBRATION MODE");
      int val=sampleAnalog();
      if(c=='w') {
        whiteRaw=val;
        whiteMeasuredThisSession=true;
        Serial.printf("WHITE = %d\n",val);
      } else {
        blackRaw=val;
        blackMeasuredThisSession=true;
        Serial.printf("BLACK = %d\n",val);
      }
      if(whiteRaw>=0 && blackRaw>=0)
        Serial.printf("CONTRAST = %d\n",abs(blackRaw-whiteRaw));
      ring(CRGB::Blue);
      saveCalibrationIfComplete();
    }
    if(c=='g') startFollower();
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(L_A,OUTPUT); pinMode(L_B,OUTPUT);
  pinMode(R_A,OUTPUT); pinMode(R_B,OUTPUT);
  pinMode(LINE_D_PIN,INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonLastRead=buttonStable=digitalRead(BUTTON_PIN);
  analogReadResolution(12);
  analogSetPinAttenuation(LINE_A_PIN, ADC_11db);
  FastLED.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,NUM_LEDS);
  FastLED.setBrightness(70);
  motorsOff(); ring(CRGB::Blue);
  pcntInit(PCNT_UNIT_0, ENC_R_A, ENC_R_B);
  pcntInit(PCNT_UNIT_1, ENC_L_A, ENC_L_B);
  loadCalibration();
  ring(calibrationValid()?CRGB::Red:CRGB::Blue);
  Serial.println("ROSiK LINE FOLLOWER: GPIO25 button=start/stop; w=white, b=black, g=go, s=stop");
  Serial.println("Place robot on right EDGE of black line (black to its LEFT)");
}
void loop() {
  handleSerial();
  handleButton();
  uint32_t now=millis();
  if(now-lastControl<20) return;
  float dt=(now-lastControl)/1000.0f;
  lastControl=now;
  int16_t ticksR=popTicks(PCNT_UNIT_0), ticksL=popTicks(PCNT_UNIT_1);
  totalR+=ticksR; totalL+=ticksL;
  float vR=ticksR*MM_PER_TICK/dt;
  float vL=ticksL*MM_PER_TICK/dt;
  if (fabsf(vR) > MAX_ENCODER_MM_S || fabsf(vL) > MAX_ENCODER_MM_S) {
    if (enabled) { enabled=false; motorsOff(); ring(CRGB::Red); Serial.println("ENCODER SPIKE -> STOP"); }
  }
  uint16_t raw=analogRead(LINE_A_PIN);
  if(!filterReady) {filtered=raw; filterReady=true;}
  filtered += 0.28f*((float)raw-filtered);
  float blackFraction=0;
  if(whiteRaw>=0 && blackRaw>=0 && whiteRaw!=blackRaw)
    blackFraction=constrain((filtered-whiteRaw)/(blackRaw-whiteRaw),0.0f,1.0f);

  if(enabled) {
    // Удерживаем край линии: половина сигнала белого и чёрного = целевая точка.
    float error=blackFraction-0.5f;
    float desiredSteer=constrain(STEER_KP*error,-MAX_STEER,MAX_STEER);
    float maxStep=MAX_STEER_RATE*dt;
    appliedSteer += constrain(desiredSteer-appliedSteer, -maxStep, maxStep);
    // Чёрное слева: на чёрном — вправо, на белом — влево.
    targetL=constrain(BASE_SPEED+appliedSteer,MIN_WHEEL_SPEED,MAX_SPEED);
    targetR=constrain(BASE_SPEED-appliedSteer,MIN_WHEEL_SPEED,MAX_SPEED);

    // Если датчик над чисто белым или чисто чёрным слишком долго,
    // это уже не слежение за границей. Останавливаемся, не закручиваемся.
    if(blackFraction<0.10f) {
      if(!whiteSince) whiteSince=now;
    } else whiteSince=0;
    if(blackFraction>0.90f) {
      if(!blackSince) blackSince=now;
    } else blackSince=0;
    if ((whiteSince && now-whiteSince>LOST_TIMEOUT) ||
        (blackSince && now-blackSince>LOST_TIMEOUT)) {
      enabled=false; motorsOff(); ring(CRGB::Red);
      Serial.println("LINE EDGE LOST -> STOP (white or black for too long)");
    }

    // При требовании ехать вперёд оба энкодера должны давать прямое движение.
    // Это обнаруживает застревание и неверный знак скорости.
    if (enabled) {
      if(vL < 2.0f) { if(!badEncoderSinceL) badEncoderSinceL=now; }
      else badEncoderSinceL=0;
      if(vR < 2.0f) { if(!badEncoderSinceR) badEncoderSinceR=now; }
      else badEncoderSinceR=0;
      if((badEncoderSinceL && now-badEncoderSinceL>ENCODER_FAULT_MS) ||
         (badEncoderSinceR && now-badEncoderSinceR>ENCODER_FAULT_MS)) {
        enabled=false; motorsOff(); ring(CRGB::Red);
        Serial.println("ENCODER/MOTOR FAULT -> STOP. Check signs and wiring");
      }
    }
    if(enabled) ring(blackFraction>0.65f?CRGB::Yellow:CRGB::Green);
  }

  if(enabled) {
    // PID скорости с feedforward: используются штатные энкодеры и H-мосты ROSiK.
    float eL=targetL-vL, eR=targetR-vR;
    // Anti-windup: интегратор не разгоняется при отрицательной обратной связи
    // и при сильных отклонениях энкодера. PWM никогда не уходит в реверс.
    if (vL>=0 && fabsf(eL)<70.0f) iL=constrain(iL+eL*dt,-25.0f,25.0f);
    if (vR>=0 && fabsf(eR)<70.0f) iR=constrain(iR+eR*dt,-25.0f,25.0f);
    float uL=0.65f*eL+0.25f*iL+0.65f*targetL;
    float uR=0.65f*eR+0.25f*iR+0.65f*targetR;
    uL=constrain(uL,0.0f,170.0f);
    uR=constrain(uR,0.0f,170.0f);
    drivePwm(L_A,L_B,uL);
    drivePwm(R_A,R_B,uR);
  }
  if(now-lastLog>=200) {
    lastLog=now;
    Serial.printf("RAW=%u D=%d black=%.2f steer=%.1f targetL=%.1f targetR=%.1f vL=%.1f vR=%.1f %s\n",
       raw,digitalRead(LINE_D_PIN),blackFraction,appliedSteer,targetL,targetR,vL,vR, enabled?"RUN":"STOP");
  }
}
