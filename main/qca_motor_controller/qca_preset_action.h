#pragma once
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void init_preset_action();




typedef struct AtomicPresetAction;


struct PresetActionConfig {
    uint32_t action_serial;
    uint32_t repeat_time; // cleanup or repeat action
    int64_t last_group_proceed_time;
    uint32_t last_action_wait_interval;
    struct AtomicPresetAction* act_head;
    struct AtomicPresetAction* act_curr;
};

typedef struct AtomicPresetAction {
    uint32_t group;
    uint64_t wait_time; // 只针对不同group生效
    void (*preset_func)(int, int);
    int arg1;
    int arg2;
    struct AtomicPresetAction* next;
}AtomicPresetAction_t;


bool check_need_proceed_preset(struct PresetActionConfig* config, int64_t curr_time);

// recycle 时释放 act_head和act_curr，因此act_head要使用深拷贝
bool recycle_preset_config(struct PresetActionConfig* config);

struct PresetActionConfig* genertate_preset_config_for_id(int preset_code, int repeat);

bool proceed_action_for_config(struct PresetActionConfig* config, int64_t start_time);

bool check_need_clean_up_config(struct PresetActionConfig* config);