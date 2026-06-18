/**
 ******************************************************************************
 * @file    can_test.h
 * @author  milFOC Team
 * @brief   CAN communication test module.
 *          Simple echo test: sends a heartbeat frame and echoes any received
 *          frame with incremented ID. LED toggles on activity.
 *
 * @note    Call CAN_Test_Init() once, then CAN_Test_Task() in main loop.
 *          Uses direct BSP API (fdcanx_send_data / fdcanx_receive).
 ******************************************************************************
 */

#ifndef CAN_TEST_H
#define CAN_TEST_H

#include "stdint.h"

void CAN_Test_Init(void);
void CAN_Test_Task(void);

#endif /* CAN_TEST_H */
