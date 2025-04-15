#include "qca_motor_controller/qca_pwm_helper.h"
#include "math.h"
#include "esp_log.h"
#include "qca_motor_controller/qca_global.h"

#define LOG_TAG MAIN_LOG_TAG "/pwm_helper"
// private function declaration
uint32_t get_timer_resolution_internal(uint8_t resolution_bit);
// 




// ledc_channel_config_t ledc_channel = {
//     .channel = LEDC_CHANNEL_0,
//     .duty = 0,
//     .gpio_num = 2,
//     .speed_mode = qca_pwm_speed_mode, /*esp32s3 only support low speed mode, can not handle LEDC_HIGH_SPEED_MODE*/
//     .hpoint = 0,
//     .timer_sel = LEDC_TIMER_0,
// };
/*
// ledc_timer_config_t ledc_timer = {
//     .duty_resolution = LEDC_TIMER_10_BIT,
//     .freq_hz = 50,
//     .speed_mode = LEDC_LOW_SPEED_MODE,
//     .timer_num = LEDC_TIMER_0,
//     .clk_cfg = LEDC_AUTO_CLK,
// };
*/


uint32_t get_timer_resolution(ledc_timer_config_t* timer_cfg){
    uint8_t res_bit = timer_cfg->duty_resolution;
    return get_timer_resolution_internal(res_bit);
}

ledc_channel_config_t generate_ledc_channel_config_full(int gpio, ledc_mode_t speed_mode, ledc_channel_t channel, 
                                                            ledc_timer_t timer_src, uint32_t duty, int hpoint, unsigned int output_invert){
    ledc_channel_config_t channel_cfg = {0};
    channel_cfg.gpio_num = gpio;
    channel_cfg.speed_mode = speed_mode;
    channel_cfg.channel = channel;
    channel_cfg.timer_sel = timer_src;
    channel_cfg.duty = duty;
    channel_cfg.hpoint = hpoint;
    channel_cfg.flags.output_invert = output_invert;
    
    return channel_cfg;
}

ledc_timer_config_t generate_ledc_timer_config_full(ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, 
                                                        ledc_timer_t timer_src, uint32_t frequency, ledc_clk_cfg_t clk_cfg, bool deconfigure){
    ledc_timer_config_t timer_cfg = {
        .speed_mode = speed_mode,
        .duty_resolution = duty_resolution,
        .timer_num = timer_src,
        .freq_hz = frequency,
        .clk_cfg = clk_cfg,
        .deconfigure = deconfigure
    };
    return timer_cfg;
}

uint32_t calculate_duty_for_servo_degree(ledc_timer_config_t* timer_cfg, uint32_t target_degree, uint32_t max_degree, uint32_t low_time_us, uint32_t high_time_us, uint32_t freq_hz){
    float single_run_us = (float) 1000000 / freq_hz;
    float low_duty_cycle = low_time_us / single_run_us; // 保证duty的下边界
    float high_duty_cycle = high_time_us / single_run_us;
    uint32_t resolution_max = get_timer_resolution(timer_cfg);
    float degree_scale = target_degree / max_degree;
    uint32_t duty = (low_duty_cycle + degree_scale * (high_duty_cycle - low_duty_cycle)) * resolution_max;
    ESP_LOGI(LOG_TAG, "%s - single_run_us: %.2f, low_cycle: %.2f, high_cycle: %.2f, res_max: %lu, degree_scale: %.2f, duty: %lu", __func__, single_run_us, low_duty_cycle, high_duty_cycle, resolution_max, degree_scale, duty);
    return duty;
}

uint32_t calculate_duty_for_motor_level(ledc_timer_config_t* timer_cfg, uint32_t target_level, uint32_t max_level, uint32_t initialize_duty){
    uint32_t resolution_max = get_timer_resolution(timer_cfg);
    double scale = (double) target_level/max_level;
    uint32_t duty = (double)(resolution_max - initialize_duty) * scale + initialize_duty;
    return duty;
}

uint32_t get_timer_resolution_internal(uint8_t resolution_bit){
    uint32_t res = pow(2, resolution_bit) - 1;
    return res;
}