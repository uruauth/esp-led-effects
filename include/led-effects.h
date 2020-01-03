#pragma once

#include <esp_err.h>

typedef enum
{
    LED_EFFECT_DISABLED = 0,
    LED_EFFECT_ON,
    LED_EFFECT_BLINK,

    LED_EFFECT_LAST
} led_effect_t;

typedef struct
{
    uint8_t gpio;
    led_effect_t effect;
} led_descriptor_t;

esp_err_t led_effects_init(led_descriptor_t *leds, uint8_t count);

esp_err_t led_effects_set(uint8_t led, led_effect_t effect, int duration);

esp_err_t led_effects_reset(uint8_t led);