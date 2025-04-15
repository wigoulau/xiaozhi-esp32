#pragma once
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#ifndef MAIN_LOG_TAG
#define MAIN_LOG_TAG "QCA"
#endif

int64_t qca_timer_get_mills();

//全局变量在qca_global声明，并在main中定义，通过extern导出
extern QueueHandle_t qca_ble_recv_queue;
extern QueueHandle_t qca_ble_send_queue;

extern char tx_buffer[256]; //TODO set to global

extern const int qca_ble_mtu_size;
extern const int qca_ble_recv_queue_size;
extern const int qca_ble_send_queue_size;
extern const int qca_ble_recv_queue_bf_size;
extern const int qca_ble_send_queue_bf_size;