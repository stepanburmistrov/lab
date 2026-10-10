#include <FastLED.h>

#define LED_PIN     14
#define NUM_LEDS     16
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define BRIGHTNESS 160

CRGB leds[NUM_LEDS];

const uint8_t frameMs = 5;

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  static uint8_t baseHue = 0;
  fill_rainbow(leds, NUM_LEDS, baseHue,
               256 / NUM_LEDS);
  FastLED.show();

  baseHue++;
  delay(frameMs);
}
