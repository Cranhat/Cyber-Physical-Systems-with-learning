#include "Pwm.hpp"

Pwm::Pwm(uint32_t pwmFrequency)
    : pwmFrequency(pwmFrequency)
{
    ledc_timer_config_t config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = pwmFrequency,
        .clk_cfg = LEDC_USE_RC_FAST_CLK,
        .deconfigure = false,
    };
    esp_err_t esp_ret = ledc_timer_config(&config);
    if (esp_ret != ESP_OK)
    {
        printf("Error: Failed to initialize pwm, ERROR: 0x%x\r\n", esp_ret);
        abort();
    }
}

Pwm::~Pwm() = default;

void Pwm::addChannel(gpio_num_t portNumber, uint8_t duty)
{
    if (pwmChannels.find(portNumber) != pwmChannels.end())
    {
        return;
    }
    pwmChannels.insert({portNumber, PwmChannel(portNumber, duty)});
};
