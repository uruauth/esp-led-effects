#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/ledc.h>
#include <esp_log.h>

#include "led-effects.h"

#define LOG_TAG "led-effects"

static led_descriptor_t *led_descriptors = NULL;
static uint8_t led_count = 0;

esp_err_t led_effects_init(led_descriptor_t *leds, uint8_t count)
{
    ESP_LOGI(LOG_TAG, "%s", __FUNCTION__);

    led_descriptors = leds;
    led_count = count;

    for (uint8_t i = 0; i < count; i++)
    {
        led_effects_reset(i);
    }

    //
    ledc_timer_config_t timer_config = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK};

    ledc_timer_config(&timer_config);

    for (uint8_t i = 0; i < count; i++)
    {
        ledc_channel_config_t channel_config = {
            .channel = i,
            .duty = 0,
            .gpio_num = led_descriptors[i].gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0};

        ledc_channel_config(&channel_config);
    }

    return ESP_OK;
}

esp_err_t led_effects_set(uint8_t led, led_effect_t effect, int duration)
{
    ESP_LOGI(LOG_TAG, "Setting LED %d to effect %d", led, effect);

    led_descriptors[led].effect = effect;

    switch (effect)
    {
    case LED_EFFECT_DISABLED:
        break;
    case LED_EFFECT_ON:
        break;
    case LED_EFFECT_BLINK:
        break;
    default:
        break;
    }

    return ESP_OK;
}

esp_err_t led_effects_reset(uint8_t led)
{
    led_descriptors[led].effect = LED_EFFECT_DISABLED;

    return ESP_OK;
}