#pragma once

#include <esp_err.h>

typedef enum
{
    LED_CONN_CONTROL_ANODE,
    LED_CONN_CONTROL_CATHODE
} led_connection_t;

typedef enum
{
    LED_EFFECT_DISABLED = 0,
    LED_EFFECT_OFF,
    LED_EFFECT_ON,
    LED_EFFECT_BLINK,
    LED_EFFECT_UP,
    LED_EFFECT_DOWN,
    LED_EFFECT_BREATH,

    LED_EFFECT_LAST
} led_effect_t;

#pragma pack(1)
typedef struct
{
    uint8_t gpio;
    led_connection_t conn;
    uint8_t brightness;
    led_effect_t effect;
    uint8_t stage;
    uint8_t frame;
    int repeat;
} led_descriptor_t;

esp_err_t led_effects_init(led_descriptor_t *leds, uint8_t count);

esp_err_t led_effects_set(uint8_t led, led_effect_t effect, int duration, int repeat);

esp_err_t led_effects_brightness(uint8_t led, uint8_t brightness);

esp_err_t led_effects_reset(uint8_t led);