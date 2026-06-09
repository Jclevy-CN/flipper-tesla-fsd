#include "led.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

// M5Stack ATOM Lite: single SK6812 (GRB order) on PIN_LED (GPIO27)
static Adafruit_NeoPixel g_strip(1, PIN_LED, NEO_GRB + NEO_KHZ800);
static LedColor g_last_color = LED_OFF;
static uint8_t  g_last_brightness = 0;

void led_init() {
    g_strip.begin();
    g_strip.setBrightness(25);  // keep dim — the ATOM LED is very bright
    g_strip.clear();
    g_strip.show();
    g_last_color = LED_OFF;
    g_last_brightness = 25;
}

static void led_set_uncached(LedColor color, bool force) {
    uint32_t c;
    uint8_t brightness = 25;
    if (color == LED_SLEEP) {
        brightness = 5;
        c = g_strip.Color(255, 255, 255);  // dim white
    } else if (color == LED_WHITE) {
        brightness = 255;
        c = g_strip.Color(255, 255, 255);
    } else {
        switch (color) {
            case LED_BLUE:   c = g_strip.Color(  0,   0, 255); break;
            case LED_GREEN:  c = g_strip.Color(  0, 255,   0); break;
            case LED_YELLOW: c = g_strip.Color(255, 200,   0); break;
            case LED_RED:    c = g_strip.Color(255,   0,   0); break;
            default:         c = 0;                             break;
        }
    }

    if (!force && color == g_last_color && brightness == g_last_brightness)
        return;

    g_strip.setBrightness(brightness);
    g_strip.setPixelColor(0, c);
    g_strip.show();
    g_strip.setBrightness(25);
    g_last_color = color;
    g_last_brightness = brightness;
}

void led_set(LedColor color) {
    led_set_uncached(color, false);
}

void led_factory_blink() {
    for (int i = 0; i < 3; i++) {
        led_set_uncached(LED_WHITE, true);
        delay(300);
        led_set_uncached(LED_OFF, true);
        delay(300);
    }
    led_set_uncached(LED_WHITE, true);  // stay lit to signal armed
}
