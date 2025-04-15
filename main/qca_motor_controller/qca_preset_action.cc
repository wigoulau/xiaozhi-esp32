#include "qca_motor_controller/qca_preset_action.h"
#include "qca_motor_controller/qca_motor_controller.h"
#include <string.h>
#include "qca_motor_controller/qca_global.h"
#include <esp_log.h>

#define LOG_TAG MAIN_LOG_TAG "/pre_act"
#define MAX_ACTION 999

AtomicPresetAction_t* create_preset_action(int groupId, int wait_time, void(*action)(int, int), int arg1, int arg2);
AtomicPresetAction_t* create_preset_action_list(int wait_time, void(*action)(int, int), int arg1, int arg2);
void add_action_after_last(AtomicPresetAction_t** last, AtomicPresetAction_t* action);
void add_action_after_last_param(AtomicPresetAction_t** last, int groupId, int wait_time, void(*action)(int, int), int arg1, int arg2);
AtomicPresetAction_t* test_init_preset_0();
AtomicPresetAction_t* test_init_preset_2();
AtomicPresetAction_t* test_init_preset_100();
AtomicPresetAction_t* test_init_preset_101();
AtomicPresetAction_t* test_init_preset_102();
AtomicPresetAction_t* test_init_preset_103();
AtomicPresetAction_t* test_init_preset_104();
AtomicPresetAction_t* test_init_preset_105();
AtomicPresetAction_t* test_init_preset_106();
AtomicPresetAction_t* test_init_preset_107();
AtomicPresetAction_t* test_init_preset_108();
AtomicPresetAction_t* test_init_preset_109();
AtomicPresetAction_t* test_init_preset_110();
void clone_static_atomic_value(struct AtomicPresetAction* src, struct AtomicPresetAction* dst);
bool deep_free_atomic_list(struct AtomicPresetAction* action_list);
void load_preset_from_fs();
uint32_t assign_new_serial_id();
struct PresetActionConfig* create_empty_action_config();
struct PresetActionConfig* create_empty_action_config(int repeat);
struct AtomicPresetAction* deep_clone_atomic_list(struct AtomicPresetAction* action_list);
void dump_atomic_preset_action(struct AtomicPresetAction* action);
void dump_preset_action_config(struct PresetActionConfig* config);

AtomicPresetAction_t* preset_list[MAX_ACTION] = {NULL};
AtomicPresetAction_t* preset_2;
AtomicPresetAction_t* preset_100;

// const static bool DBG = true;
const static bool VDBG = false;

uint32_t global_serial_id = 0;


void init_preset_action(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    preset_2 = test_init_preset_2();
    preset_list[2] = preset_2;
    preset_list[100] = test_init_preset_100();
    preset_list[101] = test_init_preset_101();
    preset_list[102] = test_init_preset_102();
    preset_list[103] = test_init_preset_103();
    preset_list[104] = test_init_preset_104();
    preset_list[105] = test_init_preset_105();
    preset_list[106] = test_init_preset_106();
    preset_list[107] = test_init_preset_107();
    preset_list[108] = test_init_preset_108();
    preset_list[109] = test_init_preset_109();
    preset_list[110] = test_init_preset_110();
    preset_list[0] = test_init_preset_0();
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
}

AtomicPresetAction_t* test_init_preset_0(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 0, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 1, 1200, &ctl_motor_hmi_v1, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_2(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 200, &ctl_arm_degree, 0, 180);
    add_action_after_last_param(&preset_last, 2, 200, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 3, 200, &ctl_arm_degree, 0, 180);
    add_action_after_last_param(&preset_last, 4, 1200, &ctl_arm_degree, 90, 90);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_100(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 0, &ctl_arm_degree, 45, 135);
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_motor_hmi_v1, 5, 0);
    add_action_after_last_param(&preset_last, 2, 0, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 2, 1000, &ctl_motor_hmi_v1, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}
// 后退
AtomicPresetAction_t* test_init_preset_101(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 0, &ctl_arm_degree, 135, 45);
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_motor_hmi_v1, -5, 0);
    add_action_after_last_param(&preset_last, 2, 0, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 2, 1000, &ctl_motor_hmi_v1, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

//原地左转90度，先设置为1秒 （1秒270度了
AtomicPresetAction_t* test_init_preset_102(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    // ctl_motor_manual
    add_action_after_last_param(&preset_last, 1, 333, &ctl_motor_hmi_v1, 0, -8);
    add_action_after_last_param(&preset_last, 2, 100, &ctl_motor_hmi_v1, 0, 0);
    
    // add_action_after_last_param(&preset_last, 1, 333, &ctl_motor_manual, 8, 0);
    // add_action_after_last_param(&preset_last, 2, 100, &ctl_motor_manual, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

// 右转
AtomicPresetAction_t* test_init_preset_103(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 333, &ctl_motor_hmi_v1, 0, 8);
    add_action_after_last_param(&preset_last, 2, 100, &ctl_motor_hmi_v1, 0, 0);
    // add_action_after_last_param(&preset_last, 1, 333, &ctl_motor_manual, 0, 8);
    // add_action_after_last_param(&preset_last, 2, 100, &ctl_motor_manual, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_104(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 666, &ctl_motor_hmi_v1, 0, -5);
    add_action_after_last_param(&preset_last, 2, 100, &ctl_motor_hmi_v1, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_105(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 0, &ctl_arm_degree, 0, 180);
    add_action_after_last_param(&preset_last, 1, 1333, &ctl_motor_hmi_v1, 0, -5);
    add_action_after_last_param(&preset_last, 2, 0, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 2, 100, &ctl_motor_hmi_v1, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_106(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_arm_degree, 0, 90);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_107(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_arm_degree, 90, 180);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_108(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_arm_degree, 0, 180);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_109(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_arm_degree, 135, 135);
    add_action_after_last_param(&preset_last, 1, 333, &ctl_motor_hmi_v1, 0, -5);
    add_action_after_last_param(&preset_last, 2, 1000, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 2, 400, &ctl_motor_hmi_v1, 0, 0);
    add_action_after_last_param(&preset_last, 3, 1000, &ctl_arm_degree, 45, 45);
    add_action_after_last_param(&preset_last, 3, 666, &ctl_motor_hmi_v1, 0, 5);
    add_action_after_last_param(&preset_last, 4, 1000, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 4, 400, &ctl_motor_hmi_v1, 0, 0);
    add_action_after_last_param(&preset_last, 5, 1000, &ctl_arm_degree, 135, 135);
    add_action_after_last_param(&preset_last, 5, 333, &ctl_motor_hmi_v1, 0, -5);
    add_action_after_last_param(&preset_last, 6, 1000, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 6, 333, &ctl_motor_hmi_v1, 0, -0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}

AtomicPresetAction_t* test_init_preset_110(){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset_head = create_preset_action_list(200, &ctl_arm_degree, 90, 90);
    AtomicPresetAction_t* preset_last = preset_head;
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_arm_degree, 45, 135);
    add_action_after_last_param(&preset_last, 1, 1000, &ctl_motor_hmi_v1, 5, 0);
    add_action_after_last_param(&preset_last, 2, 1000, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 2, 1000, &ctl_motor_hmi_v1, 0, 0);
    add_action_after_last_param(&preset_last, 3, 1000, &ctl_arm_degree, 135, 45);
    add_action_after_last_param(&preset_last, 3, 1000, &ctl_motor_hmi_v1, -5, 0);
    add_action_after_last_param(&preset_last, 4, 1000, &ctl_arm_degree, 90, 90);
    add_action_after_last_param(&preset_last, 4, 1000, &ctl_motor_hmi_v1, 0, 0);
    preset_last->next = NULL;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset_head;
}



// @private
AtomicPresetAction_t* create_preset_action(int groupId, int wait_time, void(*action)(int, int), int arg1, int arg2){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset = NULL;
    preset = (AtomicPresetAction_t*)malloc(sizeof(AtomicPresetAction_t));
    if(preset == NULL){
        return preset;
    }
    preset->group = groupId;
    preset->wait_time = wait_time;
    preset->preset_func = action;
    preset->arg1 = arg1;
    preset->arg2 = arg2;
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return preset;
}

// @private
AtomicPresetAction_t* create_preset_action_list(int wait_time, void(*action)(int, int), int arg1, int arg2){
    ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* list = NULL;
    list = create_preset_action(0, wait_time, action, arg1, arg2);
    ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return list;
}

// @private
void add_action_after_last(AtomicPresetAction_t** last, AtomicPresetAction_t* action){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    (*last)->next = action;
    *last = action;
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
}

// @private
void add_action_after_last_param(AtomicPresetAction_t** last, int groupId, int wait_time, void(*action)(int, int), int arg1, int arg2){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    AtomicPresetAction_t* preset = create_preset_action(groupId, wait_time, action, arg1, arg2);
    if(preset == NULL){
        ESP_LOGI(LOG_TAG, "%s create preset failed, return NULL", __func__);
        return;
    }
    add_action_after_last(last, preset);
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
}

// @public
bool check_need_proceed_preset(struct PresetActionConfig* config, int64_t curr_time){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);

    if(config->repeat_time < 1){
        ESP_LOGI(LOG_TAG, "%s preset action no more repeat, exit", __func__);
        return false;
    }

    if(curr_time - config->last_group_proceed_time > config->last_action_wait_interval){
        return true;
    }
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return false;
}

// @public
bool recycle_preset_config(struct PresetActionConfig* config){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s deep free atomic list in config: 0x%p", __func__, config);
    deep_free_atomic_list(config->act_head);
    if(VDBG) ESP_LOGI(LOG_TAG, "%s recycle config: 0x%p", __func__, config);
    free(config);
    return true;
}

// @private
bool deep_free_atomic_list(struct AtomicPresetAction* action_list){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    struct AtomicPresetAction* curr = action_list;
    struct AtomicPresetAction* next = action_list;
    if(VDBG) ESP_LOGI(LOG_TAG, "%s try deep free for addr: 0x%p", __func__, action_list);
    while(curr->next != NULL){
        next = curr->next;
        if(VDBG) ESP_LOGI(LOG_TAG, "%s free curr addr: 0x%p, next addr: 0x%p", __func__, curr, next);
        free(curr);
        curr = next;
        next = NULL;
    }
    free(curr);
    if(VDBG) ESP_LOGI(LOG_TAG, "%s try deep free finished", __func__);
    return true;
}

// @private
void clone_static_atomic_value(struct AtomicPresetAction* src, struct AtomicPresetAction* dst){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    if(src == NULL){
        ESP_LOGI(LOG_TAG, "%s clone from null source" , __func__);
    }
    dst->group = src->group;
    dst->wait_time = src->wait_time;
    dst->preset_func = src->preset_func;
    dst->arg1 = src->arg1;
    dst->arg2 = src->arg2;
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit, dump: group: %lu, wait: %llu, func: %pF, arg1: %d, arg2: %d", __func__, dst->group, dst->wait_time, dst->preset_func, dst->arg1, dst->arg2);
}

// @private
struct AtomicPresetAction* deep_clone_atomic_list(struct AtomicPresetAction* action_list){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    struct AtomicPresetAction* new_list = NULL;
    struct AtomicPresetAction* origin = action_list;
    
    if(action_list == NULL){
        //exit NULL;
        ESP_LOGI(LOG_TAG, "%s pass in NULL list, return", __func__);
        return new_list;
    }

    new_list = (struct AtomicPresetAction*)malloc(sizeof(struct AtomicPresetAction));
    if(new_list == NULL){
        //log
        ESP_LOGI(LOG_TAG, "%s failed to malloc mem for new list", __func__);
        return new_list;
    }
    new_list->next = NULL;

    clone_static_atomic_value(origin, new_list);

    struct AtomicPresetAction* new_list_iterator = new_list;

    origin = origin->next;
    
    while(origin != NULL){
        struct AtomicPresetAction* new_node = (struct AtomicPresetAction*)malloc(sizeof(struct AtomicPresetAction));
        new_node->next = NULL;
        if(new_node == NULL){
            //error mem failed, recycle?
            break;
        }
        // ESP_LOGI(LOG_TAG, "%s call clone_static_value", __func__);
        clone_static_atomic_value(origin, new_node);
        // ESP_LOGI(LOG_TAG, "%s call add_action_after_last", __func__);
        add_action_after_last(&new_list_iterator, new_node);
        origin = origin->next;
    }
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return new_list;
}

// @private
struct AtomicPresetAction* get_action_list_for_preset(int preset_code){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    if(preset_code >= MAX_ACTION){
        ESP_LOGI(LOG_TAG, "preset action code not defined");
        return NULL;
    }
    struct AtomicPresetAction* action = preset_list[preset_code];
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return deep_clone_atomic_list(action);
}

// @private
uint32_t assign_new_serial_id(){
    // TODO: mutex lock
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    uint32_t tmp = global_serial_id;
    global_serial_id += 1;
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return tmp;
}


// @private
struct PresetActionConfig* create_empty_action_config(int repeat){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    struct PresetActionConfig* new_conf = (struct PresetActionConfig*)malloc(sizeof(struct PresetActionConfig));
    if(new_conf == NULL){
        ESP_LOGI(LOG_TAG, "%s failed to allocate mem for PresetActionConfig, return null", __func__);
        return NULL;
    }
    new_conf->action_serial = assign_new_serial_id();
    new_conf->repeat_time = repeat;
    new_conf->last_group_proceed_time = 0;
    new_conf->last_action_wait_interval = 0;
    new_conf->act_head = NULL;
    new_conf->act_curr = NULL;
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return new_conf;
}

// @public
struct PresetActionConfig* genertate_preset_config_for_id(int preset_code, int repeat){
    if(VDBG) ESP_LOGI(LOG_TAG, "%s enter", __func__);
    struct PresetActionConfig* new_config = create_empty_action_config(repeat);
    if(new_config == NULL){
        ESP_LOGI(LOG_TAG, "%s failed to get config", __func__);
        return NULL;
    }
    struct AtomicPresetAction* action_list = get_action_list_for_preset(preset_code);
    new_config->act_head = action_list;
    new_config->act_curr = action_list;
    if(VDBG) ESP_LOGI(LOG_TAG, "%s exit", __func__);
    return new_config;
}

// @public
bool proceed_action_for_config(struct PresetActionConfig* config, int64_t start_time){
    struct AtomicPresetAction* action = config->act_curr;
    if(action == NULL){
        ESP_LOGI(LOG_TAG, "%s failed to proceed as action is NULL, maybe end of action", __func__);
        stop_all_arm();
        config->last_group_proceed_time = 0;
        config->last_action_wait_interval = 0;
        return false;
    }
    uint32_t curr_group_id = config->act_curr->group;
    int64_t time = start_time;
    while(action->group == curr_group_id){
        ESP_LOGI(LOG_TAG, "%s groupId: %lu, call action[%pS] arg1: %d, arg2: %d", __func__, action->group ,action->preset_func, action->arg1, action->arg2);
        if(config->act_curr != NULL){
            action->preset_func(action->arg1, action->arg2);
            if(action->next != NULL){
                if(action->next->group != curr_group_id){
                    config->last_group_proceed_time = time;
                    config->last_action_wait_interval = action->wait_time;
                    config->act_curr = action->next;
                    break;
                }
                action = action->next;
                continue;
            }else{
                config->act_curr = NULL;
                config->last_group_proceed_time = time;
                config->last_action_wait_interval = action->wait_time;
                break;
            }
        }else{
            stop_all_arm();
            config->last_group_proceed_time = 0;
            config->last_action_wait_interval = 0;
        }
        
    }

    return true;
}

// @public
bool check_need_clean_up_config(struct PresetActionConfig* config){
    if(config->repeat_time < 1){
        ESP_LOGI(LOG_TAG, "%s act no repeat time, need cleanup", __func__);
        return true;
    }
    if(config->act_curr == NULL && config->last_action_wait_interval == 0){
        ESP_LOGI(LOG_TAG, "%s act null, maybe renew", __func__);
        config->repeat_time -= 1;
        // 重新指向头结点
        config->act_curr = config->act_head;
    }

    return false;
}


void dump_atomic_preset_action(struct AtomicPresetAction* action){

}

void dump_preset_action_config(struct PresetActionConfig* config){

}