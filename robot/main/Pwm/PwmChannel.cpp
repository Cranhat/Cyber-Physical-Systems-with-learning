#include "PwmChannel.hpp"

PwmChannel::PwmChannel(gpio_num_t portNumber, uint8_t duty)
{
    channel = reserveNextChannel();
    ledc_channel_config_t config = {
        .gpio_num = portNumber,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = duty,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {
            .output_invert = false,
        },
        .deconfigure = false,
    };
    esp_err_t esp_ret = ledc_channel_config(&config);
    if (esp_ret != ESP_OK)
    {
        printf("Error: Failed to initialize pwm, ERROR: 0x%x\r\n", esp_ret);
        abort();
    }
}

PwmChannel::~PwmChannel() = default;

ledc_channel_t PwmChannel::reserveNextChannel()
{
    static int nextChannel = static_cast<int>(LEDC_CHANNEL_0);
    if (nextChannel > static_cast<int>(LEDC_CHANNEL_7))
    {
        printf("Error: No LEDC channels left\r\n");
        abort();
    }

    ledc_channel_t reserved = static_cast<ledc_channel_t>(nextChannel);
    nextChannel++;
    return reserved;
}

void PwmChannel::changeDuty(uint8_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void PwmChannel::changeFrequency(uint32_t frequency)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency);
}
