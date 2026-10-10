#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"
#include "driver/pcnt.h"
#include <math.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>

#define LED_PIN 14
#define NUM_LEDS 8
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define BRIGHTNESS 160

CRGB leds[NUM_LEDS];

enum LedMode { BOOT_ANIM,
               CONNECT_FLASH,
               AP_BREATH,
               LED_OFF };
static LedMode ledMode = BOOT_ANIM;

static uint32_t ledT0 = 0;    // таймер текущего режима
static uint8_t flashCnt = 0;  // счётчик вспышек
static uint8_t baseHue = 0;   // для rainbow и breathing


/* ─── VL53L0X ─── */
#define TOF_SDA 21  // если у вас другие пины – меняйте
#define TOF_SCL 22
VL53L0X tof;

RTC_DATA_ATTR bool tofOk = false;  // инициализация прошла?
volatile uint16_t tofRaw = 0;      // мм, нативное
volatile uint16_t tofCorr = 0;     // мм, после калибровки


// Последняя успешная выборка фронтального дальномера.
uint32_t tofLastValidMs = 0;

/* ─── калибровка (замените своими G и O) ─── */
const float TOF_GAIN = 0.969f;     // G
const float TOF_OFFSET = -37.21f;  // O  (мм)
static inline uint16_t tof_correct(uint16_t r) {
  float d = TOF_GAIN * r + TOF_OFFSET;
  return (d < 0) ? 0 : (uint16_t)(d + 0.5f);
}

#define BTN_PIN 25                 // кнопка на GPIO 25 → GND
volatile bool btnPressed = false;  // 1 — нажата, 0 — отпущена
/* ─── dual‑line sensor ─── */
#define LINE_A_PIN 39         // ADC1‑CH3, аналоговый RAW 0…4095
#define LINE_D_PIN 12         // цифровой, LOW = «чёрное»
volatile uint16_t lineA = 0;  // последнее аналоговое чтение
volatile bool lineD = false;

/* ─── battery monitor ─── */
#define BAT_PIN 36  // ADC1‑CH0
constexpr float R1 = 3400.0f;
constexpr float R2 = 900.0f;
constexpr float DIV_K = (R1 + R2) / R2;  // множитель делителя
volatile float vBat = 0.0f;              // V



/* ---------- Camera UART ---------- */
#define CAM_RX 4
#define CAM_TX 23
#define CAM_BAUD 115200
HardwareSerial Cam(1);  // используем UART1 (UART0 – USB, UART2 – лидар)

String camIP;               // последний IP камеры
uint32_t tLastCamStat = 0;  // таймер «камера не отвечает»

/* ---------- Camera helpers ---------- */
inline void sendToCam(const String &cmd)  // отправка команды в камеру
{
  Cam.println(cmd);
  Serial.print(">>>CAM ");
  Serial.println(cmd);
}
inline void parseCamLine(const String &s)  // разбор строк камеры
{
  if (s.startsWith("STATUS|IP=")) {
    camIP = s.substring(10);
    camIP.trim();
    Serial.print("CAMERA IP = ");
    Serial.println(camIP);
  } else if (s.startsWith("STATUS|ERR=")) {
    Serial.println("CAM ERROR: " + s.substring(11));
    camIP = "";
  }
  tLastCamStat = millis();
}


DNSServer dns;
enum NetMode { NM_STA,
               NM_AP };
NetMode netMode = NM_STA;

const char *AP_PASS = "12345678";
IPAddress apIP(192, 168, 4, 1);  // дефолт ESP



Preferences prefs;
/* ---------- Wi-Fi ---------- */
enum WStat { ST_EMPTY,
             ST_CONNECTING,
             ST_IP,
             ST_ERR_WIFI };
WStat wifiState = ST_EMPTY;
String ssid, pass;
static uint32_t wifiStartMs = 0;
bool httpStarted = false;
String hostName;

/* ---------- Пины и параметры робота ---------- */
// Двигатели (H-мосты)
#define L_A 33
#define L_B 32
#define R_A 27
#define R_B 26
// Энкодеры (PCNT)
#define ENC_R_A 18
#define ENC_R_B 19
#define ENC_L_A 34
#define ENC_L_B 35
// Описание механики
constexpr float WHEEL_D = 0.044f;                                    // диаметр колеса, м
constexpr float BASE_L = 0.100f;                                     // база (расстояние между колёсами), м
constexpr int TICKS_REV = 2930;                                      // количество тиков энкодера на оборот колеса
constexpr float MM_PER_TICK = WHEEL_D * M_PI * 1000.0f / TICKS_REV;  // мм за один тик

/* ---------- Глобальные переменные состояния ---------- */
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;  // текущие ШИМ для H-мостов
volatile int32_t encTotL = 0, encTotR = 0;                        // накопленные тики энкодера
volatile int32_t prevEncL = 0, prevEncR = 0;
volatile float speedL = 0.0f, speedR = 0.0f;                   // измеренные скорости, мм/с
volatile float tgtL = 0.0f, tgtR = 0.0f;                       // целевые скорости, мм/с
volatile float odomX = 0.0f, odomY = 0.0f, odomTh = 0.0f;      // одометрия: положение робота (м, м, рад)
volatile float kp = 1.0f, ki = 0.8f, kd = 0.02f, kff = 0.25f;  // PID коэффициенты
float iTermL = 0, iTermR = 0;
float prevErrorL = 0, prevErrorR = 0;
volatile uint32_t lastCmdMs = 0;

// Режим выравнивания (для прямолинейного движения)
bool alignMode = false;
float alignSign = 1.0f;
int32_t alignRefL = 0, alignRefR = 0;
constexpr float kAlign = 1.0f;  // коэффициент P-контроллера выравнивания (мм -> мм/с)

/* ---------- Настройки лидара ---------- */
#define LIDAR_RX_PIN 16  // lidar TX -> ESP RX
#define LIDAR_TX_PIN 17  // lidar RX (не обязательно использовать)
#define LIDAR_BAUD 115200

static const uint8_t HDR[4] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t BODY_LEN = 32;      // байт в теле пакета лидара (8 точек по 4 байта)
static const uint8_t INTENSITY_MIN = 2;  // минимальное значение интенсивности для учёта точки
static const float MAX_SPREAD_DEG = 20.0;
#define FRAME_LEN 20   // длина упакованных данных на каждую порцию (2 байта нач.угол, 2 байта кон.угол, 8*2 байта дистанции)
#define MAX_FRAMES 64  // макс. количество порций на один полный оборот (64*20 ≈ 1280 байт)
static uint8_t scanBuf[MAX_FRAMES * FRAME_LEN];
static uint8_t *wr = scanBuf;
static uint8_t frameCount = 0;
static float prevStartAngle = -1;

/* ---------- Статистика ---------- */
volatile uint32_t stat_rx = 0;  // принятых 20-байтных пакетов от LDS
volatile uint32_t stat_tx = 0;  // переданных полных сканов по WS

/* ---------- Вспомогательные функции ---------- */
inline float decodeAngle(uint16_t raw) {
  // Декодирует угол (двухбайтное значение) из формата LDS
  float a = (raw - 0xA000) / 64.0f;
  if (a < 0) a += 360.0f;
  else if (a >= 360) a -= 360.0f;
  return a;
}
bool readBytes(HardwareSerial &serial, uint8_t *dst, size_t n, uint32_t timeout = 300) {
  uint32_t t0 = millis();
  for (size_t i = 0; i < n; ++i) {
    while (!serial.available()) {
      if (millis() - t0 > timeout) return false;
      vTaskDelay(1);
      esp_task_wdt_reset();
    }
    dst[i] = serial.read();
  }
  return true;
}
bool waitLidarHeader(HardwareSerial &serial) {
  uint8_t pos = 0;
  uint32_t t0 = millis();
  while (true) {
    if (serial.available()) {
      if (uint8_t(serial.read()) == HDR[pos]) {
        if (++pos == 4) return true;
      } else {
        pos = 0;
      }
    }
    if (millis() - t0 > 200) return false;
    esp_task_wdt_reset();
  }
}
inline uint16_t crc16(uint16_t crc, uint8_t v) {
  crc ^= v;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}
inline void setPixelScale(uint8_t i, uint8_t r, uint8_t g,
                          uint8_t b, uint8_t v) {
  // v 0…255: 0=off, 255=полная яркость
  leds[i].setRGB(r, g, b);
  leds[i].nscale8_video(v);
}
bool sendCamAndWait(const String &cmd, uint32_t timeout = 300) {
  sendToCam(cmd);
  uint32_t t0 = millis();
  while (millis() - t0 < timeout) {
    if (Cam.available()) return true;
    vTaskDelay(1);
  }
  return false;  // тишина
}
/* ---------- Настройка WebSocket и HTTP ---------- */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient = nullptr;

// Обработчик событий WebSocket
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    // Новый клиент подключился
    if (wsClient && wsClient->status() == WS_CONNECTED) {
      // Закрываем предыдущего клиента, если был
      wsClient->close();
    }
    wsClient = client;
    //wsClient->printf("[WS] Connected (id=%u)\n", client->id());
    wsClient->client()->setNoDelay(true);  // отключаем алгоритм Нэгла для минимальной задержки
    Serial.printf("[WS] Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    if (client == wsClient) {
      wsClient = nullptr;
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  } else if (type == WS_EVT_DATA) {
    // NEW: входящее сообщение от клиента (ROS2), может содержать команду на движение
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == 4) {
      // Ожидаем 4-байтовое бинарное сообщение: [int16_t left_mm_s, int16_t right_mm_s]
      int16_t left = data[0] | (data[1] << 8);
      int16_t right = data[2] | (data[3] << 8);
      // Устанавливаем новые целевые скорости колес
      tgtL = (float)left;
      tgtR = (float)right;
      lastCmdMs = millis();  // ← обновили «пинг»
      // Включаем режим выравнивания, если |vL|≈|vR| и не ноль (робот едет прямо или крутится на месте)
      if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
        alignMode = true;
        alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
        alignRefL = encTotL;
        alignRefR = encTotR;
      } else {
        alignMode = false;
      }
      // Можно отправить подтверждение или лог (не обязательно)
      Serial.printf("[WS] Cmd: left=%d, right=%d\n", left, right);
    }
    // Если нужно обработать текстовые сообщения или другие бинарные команды, добавить тут
  }
}

// Настройка HTTP-роутов
void setupRoutes() {
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Формируем JSON с текущим состоянием робота
    char json[512];
    snprintf(json, sizeof(json),
             "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
             "\"enc\":{\"left\":%ld,\"right\":%ld},"
             "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
             "\"target\":{\"left\":%.1f,\"right\":%.1f},"
             "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f},"
             "\"btn\":%u,"
             "\"lineA\":%u,\"lineD\":%u,"
             "\"battery\":%.2f,"
             "\"range\":%u}",
             dutyLA, dutyLB, dutyRA, dutyRB,
             encTotL, encTotR,
             speedL, speedR, tgtL, tgtR,
             odomX, odomY, odomTh,
             btnPressed ? 1 : 0,
             lineA, lineD ? 1 : 0,
             vBat,
             tofCorr);

    request->send(200, "application/json", json);
  });
  server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("l")) tgtL = request->getParam("l")->value().toFloat();
    if (request->hasParam("r")) tgtR = request->getParam("r")->value().toFloat();
    lastCmdMs = millis();  // ← обновили «пинг»
    // Управление alignMode аналогично, как выше
    if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
      alignMode = true;
      alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
      alignRefL = encTotL;
      alignRefR = encTotR;
    } else {
      alignMode = false;
    }
    request->send(200, "text/plain", "ok");
  });
  server.on("/setCoeff", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp")) kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) kd = request->getParam("kd")->value().toFloat();
    if (request->hasParam("kff")) kff = request->getParam("kff")->value().toFloat();
    request->send(200, "text/plain", "ok");
  });
  server.on("/resetEnc", HTTP_GET, [](AsyncWebServerRequest *request) {
    encTotL = encTotR = 0;
    prevEncL = prevEncR = 0;
    speedL = speedR = 0;
    alignMode = false;
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    request->send(200, "text/plain", "enc reset");
  });
  server.on("/resetOdom", HTTP_GET, [](AsyncWebServerRequest *request) {
    odomX = odomY = odomTh = 0;
    request->send(200, "text/plain", "odom reset");
  });
  server.on("/factoryReset", HTTP_POST, [](AsyncWebServerRequest *req) {
    prefs.clear();
    req->send(200, "text/plain", "Settings wiped — rebooting…");
    delay(300);
    ESP.restart();
  });
  server.on("/camIP", HTTP_GET, [](AsyncWebServerRequest *req) {
    String json = "{\"camIP\":\"" + camIP + "\"}";
    req->send(200, "application/json", json);
  });
  server.on("/camSend", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("cmd")) {
      r->send(400, "text/plain", "no cmd");
      return;
    }
    String c = r->getParam("cmd")->value();
    sendToCam(c);
    r->send(200, "text/plain", "ok");
  });
  server.on("/setLed", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("d")) {  // нет данных – bad request
      r->send(400, "text/plain", "no data");
      return;
    }

    String s = r->getParam("d")->value();
    const int nLed = s.length() / 8;  // 8 hex / led
    char buf[9] = { 0 };              // под один uint32 hex

    for (int i = 0; i < nLed && i < NUM_LEDS; ++i) {
      s.substring(i * 8, i * 8 + 8).toCharArray(buf, 9);
      uint32_t v = strtoul(buf, nullptr, 16);  // RR GG BB VV
      uint8_t R = v >> 24;
      uint8_t G = v >> 16;
      uint8_t B = v >> 8;
      uint8_t V = v;
      setPixelScale(i, R, G, B, V);
    }
    FastLED.show();
    ledMode = LED_OFF;
    r->send(200, "text/plain", "ok");
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (netMode == NM_AP) {
      // делаем редирект на /cfg, чтобы браузер сразу открыл форму
      request->redirect("/cfg");
      return;
    }
    String html;
    html = R"rawlite(<!DOCTYPE html><html lang="en"><head>
    <meta charset="utf-8"><title>ROSik info</title>
    <style>
      body{font-family:sans-serif;background:#fafafa;margin:0;padding:1rem;color:#222}
      .card{max-width:460px;margin:auto;background:#fff;border-radius:12px;
            padding:1.2rem 1.6rem;box-shadow:0 2px 6px #0002}
      h1{margin-top:0;font-size:1.4rem}
      ul{padding-left:1.1rem}
      code{background:#eee;border-radius:4px;padding:1px 4px}
      a{color:#036;text-decoration:none}
    </style></head><body><div class="card">
  )rawlite";

    html += "<h1>ROSik&nbsp;robot</h1>";
    html += "<p><b>IP:</b> " + WiFi.localIP().toString() + "<br>";
    html += "<b>Name:</b> " + hostName + ".local</p>";

    html += R"rawlite(
    <h3>REST / WebSocket endpoints</h3>
    <ul>
      <li><code>/state</code> – JSON телеметрия</li>
      <li><code>/setSpeed?l=…&amp;r=…</code></li>
      <li><code>/setCoeff?kp=…&amp;ki=…&amp;kd=…&amp;kff=…</code></li>
      <li><code>/resetEnc</code></li>
      <li><code>/resetOdom</code></li>
      <li>WebSocket <code>/ws</code> – бинарная команда/скан лидaра</li>
    </ul>
    <p style="font-size:.85rem;color:#666">
      Serial CLI: <code>SETWIFI|ssid|pass</code>,
      <code>SETNAME|newname</code>, <code>RESETCFG</code>
    </p>
    )rawlite";


    html += R"rawlite(
      <form action="/factoryReset" method="POST"
            onsubmit="return confirm('Сбросить ВСЕ настройки и перезагрузить робота?');">
        <button style="background:#c33;color:#fff;border:none;padding:8px 14px;
                      border-radius:6px;cursor:pointer">
          ⟲ Сбросить настройки
        </button>
      </form>
        )rawlite";
    html += R"rawlite(</div></body></html>)rawlite";

    request->send(200, "text/html", html);
  });
  server.on("/cfg", HTTP_GET, [](AsyncWebServerRequest *req) {
    /* буфер с небольшим запасом */
    char page[1200];

    snprintf(page, sizeof(page),
             R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<title>ROSiK Wi‑Fi setup</title>
<style>
  body{font-family:sans-serif;background:#f2f2f2;margin:0;padding:2rem}
  form{max-width:320px;margin:auto;background:#fff;
       padding:1.2rem 1.4rem;border-radius:8px;box-shadow:0 2px 6px #0003}
  label{display:block;margin-top:.9rem;font-weight:600}
  input{width:100%%;padding:6px;margin-top:4px;border:1px solid #bbb;
        border-radius:4px}
  button{margin-top:1.2rem;padding:8px 12px;border:none;border-radius:6px;
         background:#0a83ff;color:#fff;cursor:pointer}
</style></head><body>
<form method='POST' action='/save'>
  <h2 style="margin-top:0">Configure Wi‑Fi</h2>

  <label>SSID
    <input name='s' placeholder='MyHomeWiFi' required>
  </label>

  <label>Password
    <input name='p' type='password' placeholder='••••••••'>
  </label>

  <label>ROSiK&nbsp;name
    <input name='n' value='%s'>
  </label>

  <button>Save &amp; Reboot</button>
</form></body></html>
)HTML",
             hostName.c_str());  // ← подставили текущее имя

    req->send(200, "text/html", page);
  });
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("s", true)) {
      ssid = req->getParam("s", true)->value();
      pass = req->hasParam("p", true) ? req->getParam("p", true)->value() : "";
      prefs.putString("ssid", ssid);
      prefs.putString("pass", pass);
      if (req->hasParam("n", true)) {
        hostName = req->getParam("n", true)->value();
        prefs.putString("name", hostName);  // сохраняем новое имя
      }
      bool ok = sendCamAndWait("SETWIFI|" + ssid + "|" + pass);

      String msg = "<html><body>Saved. Rebooting…</body></html>";
      req->send(200, "text/html", msg);
      delay(500);
      ESP.restart();
    } else {
      req->send(400, "text/plain", "Bad request");
    }
  });
  static const char *captivePaths[] = {
    "/generate_204", "/gen_204",     // Android (новый и старый)
    "/hotspot-detect.html",          // Apple
    "/connecttest.txt", "/ncsi.txt"  // Windows
  };

  for (auto p : captivePaths) {
    server.on(p, HTTP_ANY, [](AsyncWebServerRequest *r) {
      /* отправляем 302 → /cfg (работает и на Android 10+) */
      r->redirect("/cfg");
    });
  }

  // 2. Любой другой URL, если мы в AP-режиме, тоже отдаём cfg-страницу
  server.onNotFound([](AsyncWebServerRequest *r) {
    if (netMode == NM_AP) r->redirect("/cfg");
    else r->send(404, "text/plain", "Not found");
  });
}

/* ---------- Управление моторами (PWM) ---------- */
inline void setPWM(uint8_t pin, uint8_t value) {
  analogWrite(pin, value);
  // Обновляем глобальные duty-переменные для мониторинга
  switch (pin) {
    case L_A: dutyLA = value; break;
    case L_B: dutyLB = value; break;
    case R_A: dutyRA = value; break;
    case R_B: dutyRB = value; break;
  }
}
inline void stopMotors() {
  setPWM(L_A, 0);
  setPWM(L_B, 0);
  setPWM(R_A, 0);
  setPWM(R_B, 0);
  iTermR = 0;
  prevErrorR = 0;
  iTermL = 0;
  prevErrorL = 0;
}

/* ---------- Инициализация счетчиков PCNT ---------- */
void pcntInit(pcnt_unit_t unit, gpio_num_t pulse_pin, gpio_num_t ctrl_pin) {
  pcnt_config_t cfg = {};
  cfg.unit = unit;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.pulse_gpio_num = pulse_pin;
  cfg.ctrl_gpio_num = ctrl_pin;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = -32768;
  pcnt_unit_config(&cfg);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
inline int16_t readEncoder(pcnt_unit_t unit) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit);
  return count;
}

/* ---------- PID-регулятор скорости ---------- */
void updatePID() {
  static uint32_t prevMillis = millis();
  static float lastTgtL = 0, lastTgtR = 0;

  uint32_t now = millis();
  float dt = (now - prevMillis) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  prevMillis = now;

  // Сброс интегральной части при изменении целевой скорости или переходе через 0
  if (fabs(tgtL) < 1.0f) {
    iTermL = 0;
    prevErrorL = 0;
  }
  if (fabs(tgtR) < 1.0f) {
    iTermR = 0;
    prevErrorR = 0;
  }
  lastTgtL = tgtL;
  lastTgtR = tgtR;

  // Выравнивание (P-регулятор) – корректирует цель при езде строго прямо или повороте на месте
  float corr = 0.0f;
  if (alignMode) {
    int32_t dL = encTotL - alignRefL;
    int32_t dR = encTotR - alignRefR;
    float diff_mm = (float)(dL - alignSign * dR) * MM_PER_TICK;  // разница пройденного пути (мм)
    corr = kAlign * diff_mm;                                     // мм/с коррекция
  }
  float tgtCorrL = tgtL - corr;
  float tgtCorrR = tgtR + alignSign * corr;

  // PID для левого и правого колеса
  float errorL = tgtCorrL - speedL;
  float errorR = tgtCorrR - speedR;
  iTermL += errorL * dt;
  iTermR += errorR * dt;
  // ограничиваем накопление интегральной ошибки
  const float I_LIMIT = 300.0f;
  if (iTermL > I_LIMIT) iTermL = I_LIMIT;
  if (iTermL < -I_LIMIT) iTermL = -I_LIMIT;
  if (iTermR > I_LIMIT) iTermR = I_LIMIT;
  if (iTermR < -I_LIMIT) iTermR = -I_LIMIT;
  float dTermL = (errorL - prevErrorL) / dt;
  float dTermR = (errorR - prevErrorR) / dt;
  prevErrorL = errorL;
  prevErrorR = errorR;
  // Управляющее воздействие
  float outputL = kp * errorL + ki * iTermL + kd * dTermL + kff * tgtCorrL;
  float outputR = kp * errorR + ki * iTermR + kd * dTermR + kff * tgtCorrR;
  // Ограничиваем PWM
  if (outputL > 255) outputL = 255;
  if (outputL < -255) outputL = -255;
  if (outputR > 255) outputR = 255;
  if (outputR < -255) outputR = -255;
  // Задаём PWM на моторах (A-вперёд, B-назад)
  uint8_t pwmLA, pwmLB, pwmRA, pwmRB;
  if (outputL >= 0) {
    pwmLA = (uint8_t)outputL;
    pwmLB = 0;
  } else {
    pwmLA = 0;
    pwmLB = (uint8_t)(-outputL);
  }
  if (outputR >= 0) {
    pwmRA = (uint8_t)outputR;
    pwmRB = 0;
  } else {
    pwmRA = 0;
    pwmRB = (uint8_t)(-outputR);
  }
  setPWM(L_A, pwmLA);
  setPWM(L_B, pwmLB);
  setPWM(R_A, pwmRA);
  setPWM(R_B, pwmRB);
}

/* ---------- Задача чтения Лидара (поток на Core 1) ---------- */
void lidarTask(void *param) {
  esp_task_wdt_add(NULL);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint8_t body[BODY_LEN];
  while (true) {
    esp_task_wdt_reset();
    vTaskDelay(1);
    if (!waitLidarHeader(Serial2)) {
      continue;
    }
    if (!readBytes(Serial2, body, BODY_LEN)) {
      continue;
    }
    // Распарсить порцию точек лидара
    float startDeg = decodeAngle(body[2] | (body[3] << 8));
    uint8_t offset = 4;
    uint16_t dist[8];
    uint8_t quality[8];
    for (int i = 0; i < 8; ++i) {
      dist[i] = body[offset] | (body[offset + 1] << 8);
      quality[i] = body[offset + 2];
      offset += 3;
    }
    float endDeg = decodeAngle(body[offset] | (body[offset + 1] << 8));
    if (endDeg < startDeg) endDeg += 360.0f;
    if (endDeg - startDeg > MAX_SPREAD_DEG) {
      // Пропускаем пакет, если слишком большой разрыв (ошибка)
      continue;
    }
    // Упаковываем 20 байт в общий буфер скана
    uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
    uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
    *wr++ = s & 0xFF;
    *wr++ = s >> 8;
    *wr++ = e & 0xFF;
    *wr++ = e >> 8;
    for (int i = 0; i < 8; ++i) {
      uint16_t d = (quality[i] >= INTENSITY_MIN) ? dist[i] : 0;
      *wr++ = d & 0xFF;
      *wr++ = d >> 8;
    }
    frameCount++;
    stat_rx++;
    if (frameCount >= MAX_FRAMES) {
      frameCount = 0;
      wr = scanBuf;
      prevStartAngle = -1;  // сброс для корректного «перескока» угла
      continue;             // переходим к следующему пакету
    }
    // Проверяем перескок через 0° (начало нового круга)
    if (prevStartAngle >= 0.0f && startDeg < prevStartAngle && frameCount >= 30) {
      size_t scanSize = frameCount * FRAME_LEN;
      // Вычисляем CRC для всего скана
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < scanSize; ++i) {
        crc = crc16(crc, scanBuf[i]);
      }
      scanBuf[scanSize] = crc & 0xFF;
      scanBuf[scanSize + 1] = crc >> 8;
      // Отправляем по WebSocket, если клиент подключен
      if (wsClient && wsClient->canSend()) {
        wsClient->binary(scanBuf, scanSize + 2);
        stat_tx++;
      }
      // Сбрасываем буфер для следующего оборота
      wr = scanBuf;
      frameCount = 0;
    }
    prevStartAngle = startDeg;
  }
}

/* =================================================================
   Wi‑Fi helpers (асинхронная версия)
   =================================================================*/

void startApMode() {
  ledMode = AP_BREATH;
  ledT0 = millis();
  baseHue = 160;  // голубоватый
  if (netMode == NM_AP) return;
  Serial.println("» Wi-Fi STA fail → switching to AP mode");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(hostName.c_str(), AP_PASS);

  dns.start(53, "*", apIP);  // все DNS → на нас
  server.begin();            // HTTP уже настроен
  netMode = NM_AP;

  Serial.printf("AP SSID: %s  pass: %s  ip: %s\n",
                hostName.c_str(), AP_PASS, apIP.toString().c_str());
}
void sendStatus() {
  if (wifiState == ST_IP) Serial.printf("STATUS|IP=%s NAME=%s\r\n", WiFi.localIP().toString().c_str(), hostName.c_str());
  else if (wifiState == ST_ERR_WIFI) Serial.println("STATUS|ERR=NO_WIFI");
  else Serial.println("STATUS|ERR=NO_CFG");
}

/*  Асинхронное подключение: только запускаем Wi‑Fi и помечаем state   */
void connectWifi(const String &s, const String &p) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  wifiStartMs = millis();
  WiFi.begin(s.c_str(), p.c_str());
  wifiState = ST_CONNECTING;
}

void handleWifiSM() {
  if (wifiState == ST_CONNECTING) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
      wifiState = ST_IP;
      if (!httpStarted) {
        server.begin();
        httpStarted = true;
      }
      MDNS.begin(hostName.c_str());
      sendStatus();
      ledMode = CONNECT_FLASH;
      flashCnt = 0;
      ledT0 = millis();
    } else if (millis() - wifiStartMs > 15000) {
      wifiState = ST_ERR_WIFI;
      if (netMode == NM_STA) startApMode();
      if (!httpStarted) {
        server.begin();
        httpStarted = true;
      }
      sendStatus();
    }
  }
}



void handleUart() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {  // копим символы
      buf += c;
      continue;
    }

    buf.trim();  // ← дошли до конца строки — обрабатываем
    if (buf.startsWith("CAM_")) {
      sendToCam(buf.substring(4));  // убираем префикс
      buf = "";
      continue;
    }
    if (buf.startsWith("SETWIFI|")) {
      int p = buf.indexOf('|', 8);
      if (p > 8) {
        ssid = buf.substring(8, p);
        pass = buf.substring(p + 1);
        if (ssid.length()) {  // защита от пустого SSID
          prefs.putString("ssid", ssid);
          prefs.putString("pass", pass);
          connectWifi(ssid, pass);
          Serial.println(wifiState == ST_IP ? "OK" : "WAIT...");
        } else {
          Serial.println("ERR:SSID_EMPTY");
        }
      }
    } else if (buf.startsWith("SETNAME|")) {
      hostName = buf.substring(8);
      prefs.putString("name", hostName);
      if (wifiState == ST_IP) {
        MDNS.end();
        MDNS.begin(hostName.c_str());
      }
      Serial.println("OK");
    } else if (buf == "RESETCFG") {
      prefs.clear();
      Serial.println("OK");
      delay(50);
      ESP.restart();
    }

    buf.clear();
  }
}
void updateLeds() {
  const uint32_t now = millis();
  switch (ledMode) {

    case BOOT_ANIM:
      {
        // rainbow каждые 30 мс
        static uint32_t tStep = 0;
        if (now - tStep >= 30) {
          tStep = now;
          fill_rainbow(leds, NUM_LEDS, baseHue, 256 / NUM_LEDS);
          baseHue++;
          FastLED.show();
        }
        // через 4 с автоматически выключаемся, если режим не сменился
        if (now - ledT0 > 4000) {
          ledMode = LED_OFF;
          FastLED.clear();
          FastLED.show();
        }
        break;
      }

    case CONNECT_FLASH:
      {
        // 3 вспышки по 150 мс, пауза 150 мс
        const uint32_t PERIOD = 150;
        if (now - ledT0 >= PERIOD) {
          ledT0 = now;
          bool on = flashCnt % 2 == 0;
          fill_solid(leds, NUM_LEDS, on ? CRGB::Green : CRGB::Black);
          FastLED.show();
          flashCnt++;
          if (flashCnt >= 6) {  // 3 вкл + 3 выкл
            ledMode = LED_OFF;
            FastLED.clear();
            FastLED.show();
          }
        }
        break;
      }

    case AP_BREATH:
      {
        // синее «дыхание» ~2 с цикл
        float phase = fmodf((now - ledT0) / 2000.0f, 1.0f);  // 0…1
        uint8_t br = (sin(phase * 2 * M_PI) + 1) * (BRIGHTNESS / 2);
        FastLED.setBrightness(br);
        fill_solid(leds, NUM_LEDS, CHSV(baseHue, 200, 255));  // голубой
        FastLED.show();
        break;
      }

    case LED_OFF:
    default:
      break;
  }
}

/* ================================================================
   АВТОНОМНЫЙ РЕЖИМ: следование за целью на 70 мм
   Используется фронтальный лазерный дальномер VL53L0X.
   При неподвижном препятствии в зоне остановки > 3 с — направо 90°.
   ================================================================ */
constexpr uint16_t FOLLOW_DIST_MM = 70;
constexpr uint16_t FOLLOW_HYST_MM = 8;
constexpr uint16_t MAX_VALID_DIST_MM = 1200;
constexpr uint32_t SENSOR_STALE_MS = 250;
constexpr uint32_t BLOCKED_DELAY_MS = 3000;
constexpr uint32_t TURN_TIMEOUT_MS = 6000;
constexpr float FOLLOW_MAX_SPEED = 75.0f;  // мм/с
constexpr float FOLLOW_KP = 1.1f;          // мм/с на мм ошибки
constexpr float TURN_WHEEL_SPEED = 65.0f;  // мм/с
constexpr float RIGHT_TURN_RAD = 1.5707963f;
constexpr float ANGLE_TOLERANCE = 0.07f;

// По результатам тестирования на ROSiK: правая половина — LED 4..7.
// При другой ориентации монтажа перепроверьте индексы.
const uint8_t RIGHT_LED_INDEX[4] = {4, 5, 6, 7};

enum AutoState { AUTO_STOP, AUTO_FORWARD, AUTO_TURN_RIGHT };
AutoState autoState = AUTO_STOP;
uint32_t blockedSince = 0;
uint32_t turnStarted = 0;
float turnStartHeading = 0;

float wrapRad(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

void setAutonomousWheels(float left, float right) {
  // В отличие от сетевых команд не сбрасываем опорные энкодеры
  // при каждом обновлении скорости.
  bool straight = (left > 1.0f && right > 1.0f && fabsf(left - right) < 1.0f);
  if (straight && !alignMode) {
    alignRefL = encTotL;
    alignRefR = encTotR;
    alignSign = 1.0f;
  }
  alignMode = straight;
  tgtL = left;
  tgtR = right;
  lastCmdMs = millis(); // не дать штатному watchdog остановить движение
}

void autonomousControl(uint32_t now) {
  if (autoState == AUTO_TURN_RIGHT) {
    // odomTh в штатной прошивке возрастает при повороте налево.
    const float turnedRight = -wrapRad(odomTh - turnStartHeading);
    if (turnedRight >= RIGHT_TURN_RAD - ANGLE_TOLERANCE ||
        now - turnStarted > TURN_TIMEOUT_MS) {
      autoState = AUTO_STOP;
      blockedSince = 0;
      setAutonomousWheels(0, 0);
    } else {
      setAutonomousWheels(TURN_WHEEL_SPEED, -TURN_WHEEL_SPEED);
    }
    return;
  }

  bool valid = tofOk && tofLastValidMs != 0 &&
               (now - tofLastValidMs <= SENSOR_STALE_MS) &&
               tofRaw > 0 && tofRaw < 8190 &&
               tofCorr > 0 && tofCorr <= MAX_VALID_DIST_MM;
  if (!valid) {
    autoState = AUTO_STOP;
    blockedSince = 0;
    setAutonomousWheels(0, 0);
    return;
  }

  int error = (int)tofCorr - (int)FOLLOW_DIST_MM;
  if (error > (int)FOLLOW_HYST_MM) {
    // Цель впереди: следуем за ней. Если цель приближается — снижаем скорость.
    autoState = AUTO_FORWARD;
    blockedSince = 0;
    float speed = constrain(error * FOLLOW_KP, 0.0f, FOLLOW_MAX_SPEED);
    setAutonomousWheels(speed, speed);
  } else {
    // На расстоянии 70 мм (либо ближе) останавливаемся.
    // Отсчёт 3 секунд ведём только при непрерывном препятствии.
    autoState = AUTO_STOP;
    setAutonomousWheels(0, 0);
    if (blockedSince == 0) blockedSince = now;
    if (now - blockedSince >= BLOCKED_DELAY_MS) {
      autoState = AUTO_TURN_RIGHT;
      turnStarted = now;
      turnStartHeading = odomTh;
      blockedSince = 0;
      setAutonomousWheels(TURN_WHEEL_SPEED, -TURN_WHEEL_SPEED);
    }
  }
}

void autonomousLeds(uint32_t now) {
  static uint32_t ledRefresh = 0;
  if (now - ledRefresh < 50) return;
  ledRefresh = now;
  if (autoState == AUTO_FORWARD) {
    fill_solid(leds, NUM_LEDS, CRGB::Green);
  } else if (autoState == AUTO_STOP) {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    if ((now / 300) % 2 == 0) {
      for (uint8_t i = 0; i < 4; ++i)
        leds[RIGHT_LED_INDEX[i]] = CRGB::Yellow;
    }
  }
  FastLED.show();
}

/* ---------- SETUP ---------- */
void setup() {

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  ledMode = BOOT_ANIM;
  ledT0 = millis();

  Wire.begin(TOF_SDA, TOF_SCL, 400000);  // 400 кГц

  tofOk = tof.init();  // ver ≥ 1.3.0
  if (tofOk) {
    tof.setMeasurementTimingBudget(33000);  // 33 мс – ~30 Гц, хватит роботу
    tof.startContinuous(0);                 // 0 = как можно чаще
    Serial.println("VL53L0X ready");
  } else {
    Serial.println("VL53L0X NOT found");
  }
  Cam.begin(CAM_BAUD, SERIAL_8N1, CAM_RX, CAM_TX);

  prefs.begin("cfg", false);
  Serial.begin(115200);
  Serial.println("=== ESP32 Lidar+Motor Bridge ===");
  // Wi-Fi подключение
  hostName = prefs.getString("name", "");
  if (hostName.isEmpty()) {
    uint16_t rnd = (esp_random() % 9000) + 1000;  // 1000-9999
    hostName = "rosik" + String(rnd);
    prefs.putString("name", hostName);  // сохраняем
  }
  Serial.printf("Robot name: %s.local\n", hostName.c_str());


  WiFi.setSleep(false);
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  if (ssid.length()) {
    connectWifi(ssid, pass);  // как раньше
  } else {
    wifiState = ST_ERR_WIFI;  // чтобы sendStatus() не писал NO_CFG
    startApMode();            // ← поднимаем AP немедленно
  }

  // Настройка GPIO
  pinMode(L_A, OUTPUT);
  pinMode(L_B, OUTPUT);
  pinMode(R_A, OUTPUT);
  pinMode(R_B, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  analogReadResolution(12);                       // 0‑4095
  analogSetPinAttenuation(LINE_A_PIN, ADC_11db);  // 0‑3.3 V
  analogSetPinAttenuation(BAT_PIN, ADC_11db);     // то же для VBAT
  pinMode(LINE_D_PIN, INPUT);



  stopMotors();
  // PCNT для энкодеров
  pcntInit(PCNT_UNIT_0, (gpio_num_t)ENC_R_A, (gpio_num_t)ENC_R_B);
  pcntInit(PCNT_UNIT_1, (gpio_num_t)ENC_L_A, (gpio_num_t)ENC_L_B);
  // Запуск веб-сервера и WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  setupRoutes();
  // Запуск задачи лидара на ядре 0
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 8192, nullptr, 2, nullptr, 0);
}

/* ---------- LOOP ---------- */
void loop() {
  handleUart();
  handleWifiSM();
  if (netMode == NM_AP) dns.processNextRequest();
  static uint32_t t10 = 0, t20 = 0, t2000 = 0;
  uint32_t now = millis();
  // Каждые 10 мс: считываем энкодеры, обновляем одометрию
  if (now - t10 >= 10) {
    t10 = now;
    int16_t dR = readEncoder(PCNT_UNIT_0);
    int16_t dL = readEncoder(PCNT_UNIT_1);
    encTotR += dR;
    encTotL += dL;
    // Расстояние, пройденное каждым колесом за 10 мс (в метрах)
    float sR = dR * MM_PER_TICK / 1000.0f;
    float sL = dL * MM_PER_TICK / 1000.0f;
    // Обновляем одометрические координаты (в глобальной системе odom)
    float ds = 0.5f * (sR + sL);
    float dth = (sR - sL) / BASE_L;
    float midTh = odomTh + 0.5f * dth;
    odomX += ds * cosf(midTh);
    odomY += ds * sinf(midTh);
    odomTh += dth;
    // Нормализуем угол odomTh в [-pi, pi]
    if (odomTh > M_PI) odomTh -= 2 * M_PI;
    if (odomTh < -M_PI) odomTh += 2 * M_PI;
    /* ---------- кнопка debounce (≈30 мс на срабатывание) ---------- */
    constexpr uint8_t CNT_MAX = 5;      // глубина фильтра
    constexpr uint8_t PRESS_THRES = 3;  // ≥3  → «нажата»
    constexpr uint8_t REL_THRES = 2;    // ≤2  → «отпущена»

    static uint8_t pressCnt = 0;  // 0…5

    bool raw = digitalRead(BTN_PIN) == LOW;  // кнопка к GND → LOW = нажата

    if (raw) {
      if (pressCnt < CNT_MAX) pressCnt++;  // инкремент до 5
    } else {
      if (pressCnt > 0) pressCnt--;  // декремент до 0
    }

    if (!btnPressed && pressCnt >= PRESS_THRES)    // было «0»
      btnPressed = true;                           // стало «1»
    else if (btnPressed && pressCnt <= REL_THRES)  // было «1»
      btnPressed = false;                          // стало «0»

    lineD = digitalRead(LINE_D_PIN);  // 0 / 1
    lineA = analogRead(LINE_A_PIN);   // RAW 0…4095
  }
  // Каждые 20 мс: вычисляем текущие скорости колес (мм/с)
  if (now - t20 >= 20) {
    float dt = (now - t20) * 0.001f;
    t20 = now;
    speedL = (encTotL - prevEncL) * MM_PER_TICK / dt;
    speedR = (encTotR - prevEncR) * MM_PER_TICK / dt;
    prevEncL = encTotL;
    prevEncR = encTotR;
  }

  // Каждые 20 мс: автономный регулятор, затем штатный PID моторов
  static uint32_t tPID = 0;
  if (now - tPID >= 20) {
    tPID = now;
    autonomousControl(now);
    updatePID();
    if (now - lastCmdMs > 3000) {  // больше 3с сек без команд - стоп
      tgtL = tgtR = 0;
      alignMode = false;
    }

    static uint8_t batCnt = 0;
    if (++batCnt >= 5) {  // каждые 5×20 мс = 100 мс
      batCnt = 0;
      uint32_t sum = 0;
      const uint8_t N = 8;
      for (uint8_t i = 0; i < N; ++i) {
        sum += analogRead(BAT_PIN);
      }
      float raw = sum / (float)N;
      float uNode = raw * (3.3f / 4095.0f);
      vBat = uNode * DIV_K;
    }
  }
  static uint32_t tTof = 0;
  if (tofOk && now - tTof >= 33) {
    tTof = now;
    uint16_t r = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred()) {
      tofRaw = r;
      tofCorr = tof_correct(r);
      tofLastValidMs = now;
    }
  }
  while (Cam.available()) {
    String line = Cam.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      Serial.print("<CAM> ");
      Serial.println(line);
      parseCamLine(line);
    }
  }
  /* камера молчит >8 с → забываем IP */
  if (millis() - tLastCamStat > 8000) camIP = "";

  autonomousLeds(now);

  // Обслуживание клиентов WebSocket (освобождение памяти для отключившихся)
  ws.cleanupClients();
  // Каждые 2 с: выводим IP (для мониторинга, не обязательно)
  if (now - t2000 >= 2000) {
    t2000 = now;
    sendStatus();
  }
}