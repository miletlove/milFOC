#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "main.h"
#include "fdcan.h"


void bsp_can_init(void);// BSP 初始化

void can_filter_init(void);// 滤波器配置

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);// 发送函数

uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);// 接收函数

void fdcan1_rx_callback(void);// 回调函数声明

#endif /* __BSP_FDCAN_H__ */