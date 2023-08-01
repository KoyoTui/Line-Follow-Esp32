#include <stdio.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#include "Buzzer.h"
#include "common.h"

static void buzzer_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = BUZZER_TIMER_MODE,
        .timer_num        = BUZZER_TIMER_MODE,
        .duty_resolution  = BUZZER_TIMER_RES, 
        .freq_hz          = BUZZER_TIMER_FREQ,  // Set output frequency at 5 kHz
        .clk_cfg          = BUZZER_TIMER_CLK
    };
    ledc_timer_config(&buzzer_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel = {
        .speed_mode     = BUZZER_TIMER_MODE,
        .channel        = BUZZER_TIMER_CH,
        .timer_sel      = BUZZER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&channel);
}

void buzzer_play(int gpio_num,uint32_t freq,uint32_t duration) {
    buzzer_init();


	// start
    ledc_set_duty(BUZZER_TIMER_MODE, BUZZER_TIMER_CH, 0x200); // 12% duty - play here for your speaker or buzzer
    ledc_update_duty(BUZZER_TIMER_MODE, BUZZER_TIMER_CH);
	vTaskDelay(duration/portTICK_PERIOD_MS);
	// stop
    ledc_set_duty(BUZZER_TIMER_MODE, BUZZER_TIMER_CH, 0);
    ledc_update_duty(BUZZER_TIMER_MODE, BUZZER_TIMER_CH);
}
