#pragma once
#include "driver/ledc.h"

// ledc_channel_config_t ledc_channel = {
//     .channel = LEDC_CHANNEL_0,
//     .duty = 0,
//     .gpio_num = 2,
//     .speed_mode = qca_pwm_speed_mode, /*esp32s3 only support low speed mode, can not handle LEDC_HIGH_SPEED_MODE*/
//     .hpoint = 0,
//     .timer_sel = LEDC_TIMER_0,
// };

// ledc_timer_config_t ledc_timer = {
//     .duty_resolution = LEDC_TIMER_10_BIT,
//     .freq_hz = 50,
//     .speed_mode = LEDC_LOW_SPEED_MODE,
//     .timer_num = LEDC_TIMER_0,
//     .clk_cfg = LEDC_AUTO_CLK,
// };

ledc_channel_config_t generate_ledc_channel_config_full(int gpio, ledc_mode_t speed_mode, ledc_channel_t channel, ledc_timer_t timer_src, uint32_t duty, int hpoint, unsigned int output_invert);
ledc_timer_config_t generate_ledc_timer_config_full(ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, ledc_timer_t  timer_src, uint32_t frequency, ledc_clk_cfg_t clk_cfg, bool deconfigure);

uint32_t get_timer_resolution(ledc_timer_config_t* timer_cfg);
uint32_t calculate_duty_for_servo_degree(ledc_timer_config_t* timer_cfg, uint32_t target_degree, uint32_t max_degree, uint32_t low_time_us, uint32_t high_time_us, uint32_t freq_hz);
uint32_t calculate_duty_for_motor_level(ledc_timer_config_t* timer_cfg, uint32_t target_level, uint32_t max_level, uint32_t initialize_duty);