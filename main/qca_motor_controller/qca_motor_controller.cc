#include <stdio.h>
#include "qca_motor_controller/qca_global.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "qca_motor_controller/qca_motor_controller.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "math.h"

#include "qca_motor_controller/qca_preset_action.h"

#include "qca_pwm_helper.h"

#define LOG_TAG MAIN_LOG_TAG "/motor_ctl"

#define MOTOR_A_IN1 17
#define MOTOR_A_IN2 18
#define MOTOR_B_IN1 19
#define MOTOR_B_IN2 20
#define SERVO_L_PIN 2
#define SERVO_R_PIN 3

int loop_interval_mills = 50;
const int qca_motor_process_tick = 5; // 50ms
const int qca_motor_process_tick_ms = qca_motor_process_tick * portTICK_PERIOD_MS;
const int servo_suspend_time_mills = 1750;
const ledc_mode_t qca_pwm_speed_mode = LEDC_LOW_SPEED_MODE;

int64_t left_arm_last_control_time = 0;
int64_t left_arm_last_set_degree = 0;
bool left_arm_is_running_preset = false;
//original servo gpio

ledc_timer_config_t qca_timer_motor_cfg;
ledc_timer_config_t qca_timer_servo_cfg;
ledc_channel_config_t qca_channel_motor_l_0;
ledc_channel_config_t qca_channel_motor_l_1;
ledc_channel_config_t qca_channel_motor_r_0;
ledc_channel_config_t qca_channel_motor_r_1;
ledc_channel_config_t qca_channel_servo_l;
ledc_channel_config_t qca_channel_servo_r;

typedef struct PresetActionQueueNode {
    struct PresetActionConfig* config;
    struct PresetActionQueueNode* next;
}PresetActionQueueNode_t;

PresetActionQueueNode_t* qca_motor_preset_action_queue = NULL;

PresetActionQueueNode_t* create_preset_action_queue(){
    PresetActionQueueNode_t* queue = (PresetActionQueueNode_t*) malloc(sizeof(PresetActionQueueNode_t));
    queue->next = NULL;
    return queue;
}

bool add_preset_to_tail(PresetActionQueueNode_t** queue, struct PresetActionConfig* config);
bool add_preset_to_head(PresetActionQueueNode_t** queue, struct PresetActionConfig* config);
bool remove_queue_head(PresetActionQueueNode_t** queue_ptr);

// function declaration start
void qca_motor_main();
uint32_t calc_sg90_semi_duty(int degree, uint32_t precision);
void stop_left_arm_if_needed(int64_t current_time);
void init_gpio_channel_timer_configs();
void register_channel_and_timer_configs();
void ctl_motor_native(int left_level, int right_level);
void stop_arm_if_needed(int64_t current_time);
void init_preset_action_queue();
// function declaration end


void qca_motor_handler(void* args){
    ESP_LOGI(LOG_TAG, "%s - start motor handler main process", __func__);
    qca_motor_main();
}

void qca_motor_main(){
    // initialize 
    unsigned long long count = 0;
    init_gpio_channel_timer_configs();
    register_channel_and_timer_configs();

    // initialize preset action
    init_preset_action();
    init_preset_action_queue();

    // ledc_fade_func_install(0);
    int interval = 10;
    int duty = 0;
    int64_t test_last_preset_time = qca_timer_get_mills();
    while(1){
        int64_t curr_mills = qca_timer_get_mills();
        // if(curr_mills - test_last_preset_time > 10000){
        //     ESP_LOGI(LOG_TAG, "10s, trigger new preset action");
        //     proceed_preset_action(2, 2, false);
        //     test_last_preset_time = curr_mills;
        // }

        if(count % 1000 == 0){
            ESP_LOGI(LOG_TAG, "%s: count +10, i:%d, d: %d, current_duty: %lu", __func__, interval, duty, ledc_get_duty(LEDC_LOW_SPEED_MODE, qca_channel_motor_l_0.channel));
        }

        // check if need to stop arm on sg90 full
        // stop_arm_if_needed(curr_mills);
        
        // check if need to run action;
        // ESP_LOGI(LOG_TAG, "check pa queue");
        if( qca_motor_preset_action_queue != NULL && qca_motor_preset_action_queue->config != NULL ){
            // ESP_LOGI(LOG_TAG, "pa queue not null, check need proceed");
            struct PresetActionConfig* config = qca_motor_preset_action_queue->config;
            if(check_need_proceed_preset(config, curr_mills)){
                // ESP_LOGI(LOG_TAG, "need procceed preset");
                proceed_action_for_config(config, curr_mills);
            }

            if(check_need_clean_up_config(config)){
                ESP_LOGI(LOG_TAG, "need recycle preset");
                struct PresetActionConfig* free_config = config;
                struct PresetActionQueueNode* free_node = qca_motor_preset_action_queue;

                if(free_node != NULL){
                    qca_motor_preset_action_queue = free_node->next;
                    recycle_preset_config(free_config);
                    free(free_node);
                }else{
                    ESP_LOGI(LOG_TAG, "need recycle preset but free node is null");
                }
            }
        }

        

        // if(duty < 0){
        //     interval *= -1;
        //     duty = 0;
        // }

        // if(duty >= 1024){
        //     interval *= -1;
        //     duty = 1023;
        // }
        
        // // // ESP_LOGI(LOG_TAG, "%s: duty to bright", __func__);
        // ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel.channel, duty);
        // ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel.channel);
        // vTaskDelay(20);
        // ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel.channel, 26);
        // ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel.channel);
        // duty+=interval;
        vTaskDelay(qca_motor_process_tick);
        count++;
    }
}

uint32_t calc_sg90_semi_duty(int degree, uint32_t precision){
    double time_percentage = (double)((double)degree / 180 * 10 + 2.5)/100;
    double duty = time_percentage * precision;

    return (uint32_t)duty;
}

uint32_t calc_sg90_full_duty(int degree, uint32_t precision){
    double time_percentage = (double)((double)degree / 360 * 10 + 2.5)/100;
    double duty = time_percentage * precision;

    return (uint32_t)duty;
}

void ctl_left_arm_degree(int degree){
    uint32_t duty = calc_sg90_semi_duty(degree, 1024);
    ESP_LOGI(LOG_TAG, "%s get duty: %lu for degree: %d, to be configured", __func__, duty, degree);
    // ledc_set_duty(qca_pwm_speed_mode, ledc_channel.channel, duty);
    // ledc_update_duty(qca_pwm_speed_mode, ledc_channel.channel);
    // left_arm_last_set_degree = degree;
    // left_arm_last_control_time = qca_timer_get_mills();
    // int64_t time_mills = qca_timer_get_mills();
}

void ctl_arm_degree(int left, int right){
    uint32_t left_duty = calc_sg90_semi_duty(left, 1024);
    uint32_t right_duty = calc_sg90_semi_duty(right, 1024);
    

    ESP_LOGI(LOG_TAG, "ctl_arm_degree l-%d,r-%d, dutyl: %lu, dutyr: %lu", left, right, left_duty, right_duty);
    ledc_set_duty(qca_pwm_speed_mode, qca_channel_servo_l.channel, left_duty);
    ledc_update_duty(qca_pwm_speed_mode, qca_channel_servo_l.channel);

    ledc_set_duty(qca_pwm_speed_mode, qca_channel_servo_r.channel, right_duty);
    ledc_update_duty(qca_pwm_speed_mode, qca_channel_servo_r.channel);

    left_arm_last_control_time = qca_timer_get_mills();

}

void stop_arm_if_needed(int64_t current_time){
    if(!left_arm_is_running_preset && (current_time - left_arm_last_control_time > servo_suspend_time_mills)) {
        // ledc_channel.duty = calc_sg90_semi_duty(left_arm_last_set_degree, 1024);/*还是会漂移，大于90？？*/
        ledc_stop(qca_pwm_speed_mode, qca_channel_servo_l.channel, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_servo_r.channel, 0);
    }
}

void ctl_motor_native(int left_level, int right_level){
    ESP_LOGI(LOG_TAG, "%s - enter, left_level %d, right_level %d", __func__, left_level, right_level);
    if(left_level == 0){
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_l_0.channel, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_l_1.channel, 0);
        ESP_LOGI(LOG_TAG, "%s - left motor stop", __func__);
    }else if(left_level > 0){
        uint32_t duty = calculate_duty_for_motor_level(&qca_timer_motor_cfg, left_level, 10, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_l_1.channel, 0);
        ledc_set_duty(qca_pwm_speed_mode, qca_channel_motor_l_0.channel, duty);
        ledc_update_duty(qca_pwm_speed_mode, qca_channel_motor_l_0.channel);
        ESP_LOGI(LOG_TAG, "%s - left motor pwm 0 updated to %ld", __func__, duty);
    }else{
        uint32_t duty = calculate_duty_for_motor_level(&qca_timer_motor_cfg, -left_level, 10, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_l_0.channel, 0);
        ledc_set_duty(qca_pwm_speed_mode, qca_channel_motor_l_1.channel, duty);
        ledc_update_duty(qca_pwm_speed_mode, qca_channel_motor_l_1.channel);
        ESP_LOGI(LOG_TAG, "%s - left motor pwm 1 updated to %ld", __func__, duty);
    }

    if(right_level == 0){
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_r_0.channel, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_r_1.channel, 0);
        ESP_LOGI(LOG_TAG, "%s - right motor stop", __func__);
    }else if(right_level > 0){
        uint32_t duty = calculate_duty_for_motor_level(&qca_timer_motor_cfg, right_level, 10, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_r_1.channel, 0);
        ledc_set_duty(qca_pwm_speed_mode, qca_channel_motor_r_0.channel, duty);
        ledc_update_duty(qca_pwm_speed_mode, qca_channel_motor_r_0.channel);
         ESP_LOGI(LOG_TAG, "%s - right motor pwm 0 updated to %ld", __func__, duty);
    }else{
        uint32_t duty = calculate_duty_for_motor_level(&qca_timer_motor_cfg, -right_level, 10, 0);
        ledc_stop(qca_pwm_speed_mode, qca_channel_motor_r_0.channel, 0);
        ledc_set_duty(qca_pwm_speed_mode, qca_channel_motor_r_1.channel, duty);
        ledc_update_duty(qca_pwm_speed_mode, qca_channel_motor_r_1.channel);
         ESP_LOGI(LOG_TAG, "%s - right motor pwm 1 updated to %ld", __func__, duty);
    }
}

void ctl_motor_manual(int left_level, int right_level){
    ESP_LOGI(LOG_TAG, "%s - left_level: %d, right_level: %d", __func__, left_level, right_level);
    ctl_motor_native(left_level, right_level);
}

void ctl_motor_hmi_v1(int power, int direction){
    int main_level = 0;
    int sub_level = 0;
    bool isForward = true;
    ESP_LOGI(LOG_TAG, "%s - power: %d, direction: %d", __func__, power, direction);

    if(direction == 0){
        //单纯前进或后退
        ctl_motor_native(power, power);
        return;
    }

    if(power < 0){
        isForward = false;
    }

    int direction_level_abs = abs(direction);
    int power_abs = abs(power);
    main_level = power_abs > direction_level_abs ? power_abs:direction_level_abs;
    sub_level = main_level - 2*direction_level_abs;

    if(!isForward){
        main_level = -main_level;
        sub_level = -sub_level;
    }

    if(direction < 0){
        ctl_motor_native(sub_level, main_level);
    }else{
        ctl_motor_native(main_level, sub_level);
    }
}

void init_gpio_channel_timer_configs(){
    ESP_LOGI(LOG_TAG, "%s %d", __func__, __LINE__);
    #if 0//def CONFIG_IDF_TARGET_ESP32C2
    qca_timer_motor_cfg = generate_ledc_timer_config_full(qca_pwm_speed_mode, LEDC_TIMER_8_BIT, LEDC_TIMER_0, 100, LEDC_AUTO_CLK, false);
    qca_timer_servo_cfg = generate_ledc_timer_config_full(qca_pwm_speed_mode, LEDC_TIMER_8_BIT, LEDC_TIMER_1, 100, LEDC_AUTO_CLK, false);
    #else
    qca_timer_motor_cfg = generate_ledc_timer_config_full(qca_pwm_speed_mode, LEDC_TIMER_10_BIT, LEDC_TIMER_0, 50, LEDC_AUTO_CLK, false);
    qca_timer_servo_cfg = generate_ledc_timer_config_full(qca_pwm_speed_mode, LEDC_TIMER_10_BIT, LEDC_TIMER_1, 50, LEDC_AUTO_CLK, false);
    #endif

    qca_channel_motor_l_0 = generate_ledc_channel_config_full(MOTOR_A_IN1, qca_pwm_speed_mode, LEDC_CHANNEL_0, LEDC_TIMER_0, 0, 0, 0);
    qca_channel_motor_l_1 = generate_ledc_channel_config_full(MOTOR_A_IN2, qca_pwm_speed_mode, LEDC_CHANNEL_1, LEDC_TIMER_0, 0, 0, 0);
    qca_channel_motor_r_0 = generate_ledc_channel_config_full(MOTOR_B_IN1, qca_pwm_speed_mode, LEDC_CHANNEL_2, LEDC_TIMER_0, 0, 0, 0);
    qca_channel_motor_r_1 = generate_ledc_channel_config_full(MOTOR_B_IN2, qca_pwm_speed_mode, LEDC_CHANNEL_3, LEDC_TIMER_0, 0, 0, 0);
    qca_channel_servo_l = generate_ledc_channel_config_full(SERVO_L_PIN, qca_pwm_speed_mode, LEDC_CHANNEL_4, LEDC_TIMER_1, 0, 0, 0);
    qca_channel_servo_r = generate_ledc_channel_config_full(SERVO_R_PIN, qca_pwm_speed_mode, LEDC_CHANNEL_5, LEDC_TIMER_1, 0, 0, 0);
    ESP_LOGI(LOG_TAG, "%s %d", __func__, __LINE__);
}

void register_channel_and_timer_configs(){
    ledc_timer_config(&qca_timer_motor_cfg);
    ledc_channel_config(&qca_channel_motor_l_0);
    ledc_channel_config(&qca_channel_motor_l_1);
    ledc_channel_config(&qca_channel_motor_r_0);
    ledc_channel_config(&qca_channel_motor_r_1);

    ledc_timer_config(&qca_timer_servo_cfg);
    ledc_channel_config(&qca_channel_servo_l);
    ledc_channel_config(&qca_channel_servo_r);
}

void proceed_preset_action(int preset_code, int repeat, bool force){
    ESP_LOGI(LOG_TAG, "%s enter code: %d, repeat: %d, %d", __func__, preset_code, repeat, force);
    
    // 1. 通过 preset_action 获取一个 preset_action对象,  // 2. 设置重复次数
    struct PresetActionConfig* config = genertate_preset_config_for_id(preset_code, repeat);
    
    // 3. 添加到队列中
    bool add_success = false;
    if(force){
        ESP_LOGI(LOG_TAG, "%s, force add to front preset queue", __func__);
        add_success = add_preset_to_head(&qca_motor_preset_action_queue, config);
    }else{
        ESP_LOGI(LOG_TAG, "%s, add to tail preset queue", __func__);
        add_success = add_preset_to_tail(&qca_motor_preset_action_queue, config);
    }

    ESP_LOGI(LOG_TAG, "%s, add status: %d", __func__, add_success);

}


bool add_preset_to_tail(PresetActionQueueNode_t** queue, struct PresetActionConfig* config){
    if(queue == NULL){
        ESP_LOGI(LOG_TAG, "%s: try add with a [null] queue_ptr", __func__);
        return false;
    }

    if(*queue == NULL){
        ESP_LOGI(LOG_TAG, "%s: try add with a [null] queue", __func__);
        *queue = (PresetActionQueueNode_t*)malloc(sizeof(PresetActionQueueNode_t));
        if(*queue == NULL){
            ESP_LOGI(LOG_TAG, "%s: failed to malloc for queue", __func__);
            return false;
        }
        (*queue)->next = NULL;
        (*queue)->config = NULL;        
    }

    PresetActionQueueNode_t *q = *queue;

    while(q->next != NULL){
        q = q->next;
    }

    if(q->config == NULL){
        ESP_LOGI(LOG_TAG, "%s: queue is empty, replacing first", __func__);
        (*queue)->config = config;
        (*queue)->next = NULL;
        return true;
    }

    PresetActionQueueNode_t* new_node = (PresetActionQueueNode_t*) malloc(sizeof(PresetActionQueueNode_t));
    if(new_node == NULL){
        ESP_LOGI(LOG_TAG, "%s: failed to allocated for new node", __func__);
        return false;
    }

    new_node->next = NULL;
    new_node->config = config;

    q->next = new_node;
    return true;
}

bool add_preset_to_head(PresetActionQueueNode_t** queue_ptr, struct PresetActionConfig* config){
    if(queue_ptr == NULL){
        ESP_LOGI(LOG_TAG, "%s: try add with a [null] queue_ptr", __func__);
        return false;
    }

    if(*queue_ptr == NULL){
        ESP_LOGI(LOG_TAG, "%s: try add with a [null] queue", __func__);
        *queue_ptr = (PresetActionQueueNode_t*)malloc(sizeof(PresetActionQueueNode_t));
        if(*queue_ptr == NULL){
            ESP_LOGI(LOG_TAG, "%s: failed to malloc for queue", __func__);
            return false;
        }
        (*queue_ptr)->next = NULL;
        (*queue_ptr)->config = NULL;        
    }

    if((*queue_ptr)->config == NULL){
        ESP_LOGI(LOG_TAG, "%s: queue is empty, replacing first", __func__);
        (*queue_ptr)->config = config;
        (*queue_ptr)->next = NULL;
        return true;
    }

    PresetActionQueueNode_t* new_node = (PresetActionQueueNode_t*) malloc(sizeof(PresetActionQueueNode_t));
    if(new_node == NULL){
        ESP_LOGI(LOG_TAG, "%s: failed to allocated for new node", __func__);
        return false;
    }

    new_node->config = config;
    new_node->next = *queue_ptr;
    *queue_ptr = new_node;
    return true;
}

bool remove_queue_head(PresetActionQueueNode_t** queue_ptr){
    if(queue_ptr == NULL){
        ESP_LOGI(LOG_TAG, "%s: try add with a [null] queue_ptr", __func__);
        return false;
    }

    if(*queue_ptr == NULL){
        ESP_LOGI(LOG_TAG, "%s: queue_ptr point to a [null] queue", __func__);
        return false;
    }

    PresetActionQueueNode_t* queue = *queue_ptr;
    if(queue->next == NULL){
        queue->config = NULL;
    }else{
        //free PresetActionConfig
        // queue->config;

        *queue_ptr = queue->next;
        free(queue);
    }

    return true;
}

void init_preset_action_queue(){
    qca_motor_preset_action_queue = (PresetActionQueueNode_t*)malloc(sizeof(PresetActionQueueNode_t));
    qca_motor_preset_action_queue->config = NULL;
    qca_motor_preset_action_queue->next = NULL;
}

void stop_all_arm(){
    ledc_stop(qca_pwm_speed_mode, qca_channel_servo_l.channel, 0);
    ledc_stop(qca_pwm_speed_mode, qca_channel_servo_r.channel, 0);
}

int64_t qca_timer_get_mills(){
    return esp_timer_get_time() / 1000;
}