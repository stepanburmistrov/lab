/*********************************************************************
 *  ESP32-CAM  UNIVERSAL FW  v1.5
 *  ────────────────────────────────────────────────────────────────
 *  • Неблокирующий MJPEG-стример (один кадр / loop).
 *  • Все настройки храним в NVS («cfg»): Wi-Fi + Res/FPS/Flash.
 *  • Управление одной UART-линией 115 200 8-N-1 (IO1 TX → к ПК /
 *    роботу, IO3 RX ← команды).
 *
 *  Список UART-команд (каждая заканчивается CR/LF):
 *  ----------------------------------------------------------------
 *  SETWIFI|<ssid>|<pass>      сохранить сеть, подключиться
 *  RESETCFG                   стереть ВСЕ ключи NVS, ребут
 *
 *  SETRES |QQVGA|QVGA|HVGA|VGA|SVGA|XGA|SXGA|UXGA
 *                              изменить разрешение (сохраняется)
 *  SETFPS |<1-30>             частота кадров, сохраняется
 *  FLASH  |<0-100>            постоянная яркость вспышки LED, %
 *  BLINK  |<ms>               единичная вспышка на <ms> мс
 *
 *  Камера каждые 2 с пишет статус:
 *     STATUS|IP=<ip>          есть соединение
 *     STATUS|ERR=NO_WIFI      точка недоступна / пароль неверен
 *     STATUS|ERR=NO_IP        DHCP не выдал адрес
 *     STATUS|ERR=NO_CFG       SSID/PASS ещё не заданы
 *     STATUS|ERR=CAM          ошибка инициализации OV2640
 *
 *  Поток MJPEG:  http://<ip>/stream
 *
 *  Платформа  : AI-Thinker ESP32-CAM (OV2640)  @ 240 МГц
 *  Flash-LED  : GPIO 4 (PWM, 0-4095 duty; FLASH|… и BLINK|…)
 *********************************************************************/

#include "esp_camera.h"
#include <WiFi.h>
#include <Preferences.h>

/* ───── compile-time switches ───── */
#define DEBUG_UART 0     // 1 = вывод FPS/heap каждые 5 c
#define BLINK_PERIOD 0   // 10 000 мс «сердцебиение»; 0 отключить
#define BLINK_ON_MS 100  // длительность импульса heartbeat

/* ───── дефолтные значения ───── */
#define LED_PIN 4
#define DEF_RES FRAMESIZE_QVGA  // 320×240
#define DEF_FPS 10
#define DEF_FLASH_PCT 0

#define JPEG_QUALITY 15
#define XCLK_HZ 10000000  // 10 МГц: надёжно

#define UART_BAUD 115200

/* ───── AI-Thinker pin-map (OV2640) ───── */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

/* ───── глобальные переменные/объекты ───── */
Preferences prefs;  // NVS «cfg»
WiFiServer* server = nullptr;
WiFiClient camClient;


framesize_t frameSize;
uint8_t fps;
uint16_t frameInterval;  // мс между кадрами
uint8_t flashPct;        // 0-100 %
uint8_t ledDuty;         // 0-255 (12-bit /16)

enum WStat { ST_EMPTY,
             ST_CONNECTING,
             ST_IP,
             ST_ERR_WIFI,
             ST_ERR_IP };
WStat wifiState = ST_EMPTY;

String ssid, pass;
uint32_t tStatus = 0, tBlink = 0, tBlinkHold = 0;
bool ledTempOn = false;  // идёт BLINK-вспышка

#if DEBUG_UART
uint32_t frameCnt = 0, tDiag = 0;
#define D(...) Serial.printf(__VA_ARGS__)
#else
#define D(...)
#endif

/* =================================================================
   LED PWM (flash-LED GPIO4)
   =================================================================*/

#define LEDC_LED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_LED_TIMER LEDC_TIMER_2  // камера использует TIMER_0
#define LEDC_LED_CH LEDC_CHANNEL_4   // любой свободный канал
#define LEDC_LED_FREQ 5000
#define LEDC_LED_BITS LEDC_TIMER_12_BIT  // 0..4095

void ledcInit() {
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_LED_MODE;
  t.timer_num = LEDC_LED_TIMER;
  t.duty_resolution = LEDC_LED_BITS;
  t.freq_hz = LEDC_LED_FREQ;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t ch = {};
  ch.speed_mode = LEDC_LED_MODE;
  ch.channel = LEDC_LED_CH;
  ch.gpio_num = LED_PIN;
  ch.timer_sel = LEDC_LED_TIMER;
  ch.intr_type = LEDC_INTR_DISABLE;
  ch.duty = 0;
  ch.hpoint = 0;
  ledc_channel_config(&ch);
}
inline void setLedDuty(uint8_t duty) {
  ledDuty = duty;
  uint32_t val = (uint32_t)duty * 4095 / 255;  // 8-бит → 12-бит
  ledc_set_duty(LEDC_LED_MODE, LEDC_LED_CH, val);
  ledc_update_duty(LEDC_LED_MODE, LEDC_LED_CH);
}

/* =================================================================
   Camera init / re-init (OV2640)
   =================================================================*/
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_1;  // отдельный канал ШИМ для XCLK
  c.ledc_timer = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM;
  c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = XCLK_HZ;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = frameSize;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = psramFound() ? 2 : 1;  // 2 буфера при PSRAM

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("STATUS|ERR=CAM");
    return false;
  }
  return true;
}
bool reInitCamera() {
  esp_camera_deinit();
  return initCamera();
}

/* =================================================================
   Wi-Fi helpers
   =================================================================*/
void sendStatus() {
  if (wifiState == ST_IP) Serial.printf("STATUS|IP=%s\r\n", WiFi.localIP().toString().c_str());
  else if (wifiState == ST_ERR_WIFI) Serial.println("STATUS|ERR=NO_WIFI");
  else if (wifiState == ST_ERR_IP) Serial.println("STATUS|ERR=NO_IP");
  else Serial.println("STATUS|ERR=NO_CFG");
}
void connectWifi(const String& s, const String& p) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(s.c_str(), p.c_str());
  wifiState = ST_CONNECTING;

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(100);

  if (WiFi.status() != WL_CONNECTED) {
    wifiState = ST_ERR_WIFI;
    return;
  }
  if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    wifiState = ST_ERR_IP;
    return;
  }
  wifiState = ST_IP;
}

/* =================================================================
   Utils: строка → framesize_t
   =================================================================*/
framesize_t str2size(const String& s) {
  if (s == "QQVGA") return FRAMESIZE_QQVGA;
  if (s == "QVGA") return FRAMESIZE_QVGA;
  if (s == "HVGA") return FRAMESIZE_HVGA;
  if (s == "VGA") return FRAMESIZE_VGA;
  if (s == "SVGA") return FRAMESIZE_SVGA;
  if (s == "XGA") return FRAMESIZE_XGA;
  if (s == "SXGA") return FRAMESIZE_SXGA;
  if (s == "UXGA") return FRAMESIZE_UXGA;
  return (framesize_t)255;
}

/* =================================================================
   UART parser
   =================================================================*/
void handleUart() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      buf += c;
      continue;
    }

    buf.trim();
    /* —— SETWIFI —— */
    if (buf.startsWith("SETWIFI|")) {
      int p = buf.indexOf('|', 8);
      if (p > 8) {
        ssid = buf.substring(8, p);
        pass = buf.substring(p + 1);
        prefs.putString("ssid", ssid);
        prefs.putString("pass", pass);
        connectWifi(ssid, pass);
        Serial.println(wifiState == ST_IP ? "OK" : "ERR");
      }
    }
    /* —— RESETCFG —— */
    else if (buf == "RESETCFG") {
      prefs.clear();
      Serial.println("OK");
      delay(50);
      ESP.restart();
    }
    /* —— SETRES —— */
    else if (buf.startsWith("SETRES|")) {
      framesize_t ns = str2size(buf.substring(7));
      if (ns != 255) {
        frameSize = ns;
        prefs.putUChar("res", ns);
        Serial.println(reInitCamera() ? "OK" : "ERR");
      } else Serial.println("ERR");
    }
    /* —— SETFPS —— */
    else if (buf.startsWith("SETFPS|")) {
      int nf = buf.substring(7).toInt();
      if (nf >= 1 && nf <= 30) {
        fps = nf;
        frameInterval = 1000 / fps;
        prefs.putUChar("fps", fps);
        Serial.println("OK");
      } else Serial.println("ERR");
    }
    /* —— FLASH —— */
    else if (buf.startsWith("FLASH|")) {
      int pct = buf.substring(6).toInt();
      if (pct >= 0 && pct <= 100) {
        flashPct = pct;
        setLedDuty(pct * 2.55);
        prefs.putUChar("flash", pct);
        Serial.println("OK");
      } else Serial.println("ERR");
    }
    /* —— BLINK —— */
    else if (buf.startsWith("BLINK|")) {
      uint32_t d = buf.substring(6).toInt();
      if (d > 0) {
        setLedDuty(255);
        ledTempOn = true;
        tBlinkHold = millis() + d;
        Serial.println("OK");
      } else Serial.println("ERR");
    } else Serial.println("ERR");
    buf = "";
  }
}

/* =================================================================
   LED heartbeat / blink service
   =================================================================*/
void ledService() {
  uint32_t now = millis();

  /* авто-выключение BLINK|… */
  if (ledTempOn && now >= tBlinkHold) {
    setLedDuty(flashPct * 2.55);
    ledTempOn = false;
  }

  /* «сердцебиение» при BLINK_PERIOD>0   и   FLASH 0 % */
  if (BLINK_PERIOD > 0 && !ledTempOn && flashPct == 0) {
    if (now - tBlink >= BLINK_PERIOD) {
      setLedDuty(255);
      tBlink = now;
    } else if (now - tBlink >= BLINK_ON_MS) setLedDuty(0);
  }
}

/* =================================================================
   Non-blocking MJPEG: один кадр за итерацию loop()
   =================================================================*/
void serviceHttp() {
  /* 1. при отсутствии клиента — ждём новый */
  if (!camClient || !camClient.connected()) {
    camClient = server->available();
    if (camClient) {
      camClient.printf(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
    }
    return;
  }

  /* 2. интервал по FPS */
  static uint32_t tLast = 0;
  if (millis() - tLast < frameInterval) return;
  tLast = millis();

  /* 3. берём кадр и шлём */
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    camClient.stop();
    return;
  }

  camClient.printf(
    "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
    fb->len);
  camClient.write(fb->buf, fb->len);
  camClient.write("\r\n", 2);
  esp_camera_fb_return(fb);

#if DEBUG_UART
  frameCnt++;
#endif
}

/* =================================================================
   SETUP
   =================================================================*/
void setup() {
  Serial.begin(UART_BAUD);
  ledcInit();

  prefs.begin("cfg", false);

  frameSize = (framesize_t)prefs.getUChar("res", DEF_RES);
  fps = prefs.getUChar("fps", DEF_FPS);
  if (fps < 1 || fps > 30) fps = DEF_FPS;
  frameInterval = 1000 / fps;

  flashPct = prefs.getUChar("flash", DEF_FLASH_PCT);
  setLedDuty(flashPct * 2.55);

  /* 1) камера */
  if (!initCamera()) Serial.println("STATUS|ERR=CAM");

  /* 2) запускаем стек и HTTP-сервер */
  WiFi.mode(WIFI_STA);
  server = new WiFiServer(80);
  server->begin();

  /* 3) пробуем подключиться к сохранённой сети */
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  if (ssid.length()) connectWifi(ssid, pass);
}


/* =================================================================
   LOOP
   =================================================================*/
void loop() {
  handleUart();   // приём команд
  ledService();   // управление вспышкой
  serviceHttp();  // кадр MJPEG (если есть клиент)

  /* статус IP/ошибок */
  if (millis() - tStatus >= 2000) {
    tStatus = millis();
    sendStatus();
  }

#if DEBUG_UART
  if (millis() - tDiag >= 5000) {
    tDiag = millis();
    D("[diag] fps=%u heap=%u psram=%u\n",
      frameCnt / 5, ESP.getFreeHeap(), ESP.getFreePsram());
    frameCnt = 0;
  }
#endif
}