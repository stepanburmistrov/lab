/*
  ROSiK — следование по КРАЮ чёрной линии одним аналоговым датчиком.
  Основано на электрических подключениях full_firmware(2).ino.

  Arduino IDE: ESP32 (классический ESP32), FastLED, драйвер PCNT (ESP32 core 2.x).
  Serial Monitor 115200, окончания строк не важны.
  w — измерить белое, b — измерить чёрное, g — старт, s — стоп.
  СНАЧАЛА калибруйте на своей трассе, затем ставьте датчик на край линии.
  Чёрная линия должна быть слева от датчика (ведение по правому краю).
*/
#include <Arduino.h>
#include <FastLED.h>
#include "driver/pcnt.h"
#include <math.h>

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

// Параметры, подбираемые на трассе
constexpr float BASE_SPEED = 58.0f;     // мм/с
constexpr float STEER_KP = 80.0f;       // изменение скорости колёс (мм/с) на ошибку ±0.5
constexpr float MAX_STEER = 40.0f;      // мм/с
constexpr float MAX_SPEED = 100.0f;     // мм/с
constexpr uint32_t LOST_TIMEOUT = 2500; // стоп при непрерывном белом поле
constexpr uint16_t MIN_CONTRAST = 140;   // порог различия белого и чёрного (ADC)

CRGB leds[NUM_LEDS];
int whiteRaw = -1, blackRaw = -1;
bool enabled = false;
float filtered = 0;
bool filterReady = false;
uint32_t whiteSince = 0;
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
void handleSerial() {
  while (Serial.available()) {
    char c=tolower(Serial.read());
    if(c=='s') { enabled=false; motorsOff(); ring(CRGB::Red); Serial.println("STOP"); }
    if(c=='w' || c=='b') {
      enabled=false; motorsOff();
      int val=sampleAnalog();
      if(c=='w') {whiteRaw=val; Serial.printf("WHITE = %d\n",val);}
      else {blackRaw=val; Serial.printf("BLACK = %d\n",val);}
      Serial.printf("CONTRAST = %d\n",abs(blackRaw-whiteRaw));
      ring(CRGB::Blue);
    }
    if(c=='g') {
      if(whiteRaw<0 || blackRaw<0 || abs(blackRaw-whiteRaw)<MIN_CONTRAST) {
        Serial.println("CALIBRATION REQUIRED: w (white), b (black); check contrast");
      } else {
        enabled=true; filterReady=false; whiteSince=0;
        iL=iR=0;
        lastControl=millis();
        Serial.printf("GO | WHITE=%d BLACK=%d | black line on LEFT\n",whiteRaw,blackRaw);
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(L_A,OUTPUT); pinMode(L_B,OUTPUT);
  pinMode(R_A,OUTPUT); pinMode(R_B,OUTPUT);
  pinMode(LINE_D_PIN,INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(LINE_A_PIN, ADC_11db);
  FastLED.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,NUM_LEDS);
  FastLED.setBrightness(70);
  motorsOff(); ring(CRGB::Blue);
  pcntInit(PCNT_UNIT_0, ENC_R_A, ENC_R_B);
  pcntInit(PCNT_UNIT_1, ENC_L_A, ENC_L_B);
  Serial.println("ROSiK LINE FOLLOWER: w=white, b=black, g=go, s=stop");
  Serial.println("Place robot on right EDGE of black line (black to its LEFT)");
}
void loop() {
  handleSerial();
  uint32_t now=millis();
  if(now-lastControl<20) return;
  float dt=(now-lastControl)/1000.0f;
  lastControl=now;
  int16_t ticksR=popTicks(PCNT_UNIT_0), ticksL=popTicks(PCNT_UNIT_1);
  totalR+=ticksR; totalL+=ticksL;
  float vR=ticksR*MM_PER_TICK/dt;
  float vL=ticksL*MM_PER_TICK/dt;
  uint16_t raw=analogRead(LINE_A_PIN);
  if(!filterReady) {filtered=raw; filterReady=true;}
  filtered += 0.28f*((float)raw-filtered);
  float blackFraction=0;
  if(whiteRaw>=0 && blackRaw>=0 && whiteRaw!=blackRaw)
    blackFraction=constrain((filtered-whiteRaw)/(blackRaw-whiteRaw),0.0f,1.0f);

  if(enabled) {
    // Удерживаем край линии: половина сигнала белого и чёрного = целевая точка.
    float error=blackFraction-0.5f;
    float steer=constrain(STEER_KP*error,-MAX_STEER,MAX_STEER);
    // Чёрное слева: на чёрном поворачиваем вправо, на белом влево.
    targetL=constrain(BASE_SPEED+steer,0.0f,MAX_SPEED);
    targetR=constrain(BASE_SPEED-steer,0.0f,MAX_SPEED);

    // Потеря: если долго видим только белое, прекращаем движение.
    if(blackFraction<0.12f) {
      if(!whiteSince) whiteSince=now;
      if(now-whiteSince>LOST_TIMEOUT) {
        enabled=false; motorsOff(); ring(CRGB::Red);
        Serial.println("LINE LOST -> STOP (recalibrate/reposition, then g)");
      }
    } else whiteSince=0;
    if(enabled) ring(blackFraction>0.65f?CRGB::Yellow:CRGB::Green);
  }

  if(enabled) {
    // PID скорости с feedforward: используются штатные энкодеры и H-мосты ROSiK.
    float eL=targetL-vL, eR=targetR-vR;
    iL=constrain(iL+eL*dt,-100.0f,100.0f);
    iR=constrain(iR+eR*dt,-100.0f,100.0f);
    float uL=1.0f*eL+0.8f*iL+0.25f*targetL;
    float uR=1.0f*eR+0.8f*iR+0.25f*targetR;
    drivePwm(L_A,L_B,uL);
    drivePwm(R_A,R_B,uR);
  }
  if(now-lastLog>=200) {
    lastLog=now;
    Serial.printf("RAW=%u D=%d black=%.2f targetL=%.1f targetR=%.1f vL=%.1f vR=%.1f %s\n",
       raw,digitalRead(LINE_D_PIN),blackFraction,targetL,targetR,vL,vR, enabled?"RUN":"STOP");
  }
}
