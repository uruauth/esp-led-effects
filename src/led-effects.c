#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/ledc.h>
#include <esp_log.h>

#include "led-effects.h"

#define LOG_TAG "led-effects"

static led_descriptor_t *led_descriptors = NULL;
static uint8_t led_count = 0;

const uint32_t timer_frequency = 5000;
const uint32_t duty_min = 0;
const uint32_t duty_max = 256;

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
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = timer_frequency,
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
        ledc_set_duty(LEDC_LOW_SPEED_MODE, led, duty_max);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, led);
        break;
    case LED_EFFECT_ON:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, led, duty_min);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, led);
        break;
    case LED_EFFECT_BLINK:
        ledc_set_duty(LEDC_LOW_SPEED_MODE, led, duty_max / 2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, led);
        break;
    default:
        break;
    }

    return ESP_OK;
}

esp_err_t led_effects_reset(uint8_t led)
{

    return ESP_OK;
}