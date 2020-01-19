#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <driver/ledc.h>
#include <esp_log.h>

#include "led-effects.h"

#define LOG_TAG "led-effects"

static led_descriptor_t *led_descriptors = NULL;
static uint8_t led_count = 0;

const uint32_t timer_frequency = 5000;
const uint32_t duty_min = 0;
const uint32_t duty_max = 256;

TimerHandle_t xTimer;

uint8_t led_frame_brightness(led_effect_t effect, uint8_t stage, uint8_t frame)
{
    switch (effect)
    {
    case LED_EFFECT_OFF:
        return 0;
    case LED_EFFECT_ON:
        return 255;
    case LED_EFFECT_BLINK:
        return frame < 12 ? 0 : 255;
    case LED_EFFECT_UP:
        return frame * 10;
    case LED_EFFECT_DOWN:
        return 256 - frame * 10;
    case LED_EFFECT_BREATH:
        return (stage % 2) ? frame * 10 : 256 - frame * 10;
    default:
        return 0;
    }
    return 0;
}

void led_effects_timer_callback(TimerHandle_t pxTimer)
{
    for (uint8_t i = 0; i < led_count; i++)
    {
        led_descriptor_t *led = &led_descriptors[i];
        if (led->effect == LED_EFFECT_DISABLED)
        {
            continue;
        }

        led->frame += 1;

        if (led->frame > 25)
        {
            led->frame = 0;
            led->stage += 1;
        }

        uint8_t brightness = led_frame_brightness(led->effect, led->stage, led->frame);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, i, 256 - brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
    }
}

esp_err_t led_effects_init(led_descriptor_t *leds, uint8_t count)
{
    ESP_LOGD(LOG_TAG, "%s", __FUNCTION__);

    ESP_LOGD(LOG_TAG, "Configuring GPIO");

    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_PIN_INTR_DISABLE,
    };

    for (uint8_t i = 0; i < count; i++)
    {
        io_conf.pin_bit_mask |= 1ULL << leds[i].gpio;

        io_conf.pull_up_en = leds[i].conn == LED_CONN_CONTROL_ANODE ? 1 : 0;
        io_conf.pull_down_en = leds[i].conn == LED_CONN_CONTROL_CATHODE ? 1 : 0;
    }

    gpio_config(&io_conf);

    //

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

    xTimer = xTimerCreate("LED_EFFECTS_TIMER",       // Just a text name, not used by the kernel.
                          40 / portTICK_PERIOD_MS,   // The timer period in ticks.
                          pdTRUE,                    // The timers will auto-reload themselves when they expire.
                          NULL,                      // Assign each timer a unique id equal to its array index.
                          led_effects_timer_callback // Each timer calls the same callback when it expires.
    );

    if (xTimer == NULL)
    {
        return ESP_FAIL;
    }

    if (xTimerStart(xTimer, 0) != pdPASS)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t led_effects_set(uint8_t led, led_effect_t effect, int duration)
{
    ESP_LOGI(LOG_TAG, "Setting LED %d to effect %d", led, effect);

    led_descriptors[led].effect = effect;
    led_descriptors[led].frame = 0;
    led_descriptors[led].stage = 0;

    if (effect == LED_EFFECT_DISABLED)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, led, duty_max);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, led);
    }

    return ESP_OK;
}

esp_err_t led_effects_reset(uint8_t led)
{

    return ESP_OK;
}