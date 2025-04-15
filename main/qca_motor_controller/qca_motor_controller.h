#pragma once
#include <stdbool.h>

void qca_motor_handler(void* args);
void ctl_left_arm_degree(int degree);
void ctl_arm_degree(int left, int right);
void ctl_motor_manual(int left_level, int right_level);
void ctl_motor_hmi_v1(int power, int direction);

void proceed_preset_action(int preset_code, int repeat, bool force);
void stop_all_arm();