/**
 * @brief  VOFA+ FireWater protocol interface.
 *          Sends FOC transform results (v_a/v_b/v_c/i_alpha/i_beta/i_q)
 *          to VOFA+上位机 via USB CDC at ~50Hz.
 *
 *          VOFA+ config: 协议 = FireWater, 端口 = STM32 Virtual COM.
 */

#ifndef VOFA_H
#define VOFA_H

#include "general_def.h"
#include "usart.h"
#include "usbd_cdc_if.h"

/* Forward declaration (full struct in calc_test_task.h) */
typedef struct CALC_TEST CALC_TEST;

#define VOFA_FW_BUF_SIZE   256

void vofa_firewater_send(float *data, uint8_t count);
void Vofa_Packet(void);
void vofa_Receive(uint8_t *buf, uint16_t len);

/**
 * @brief  Send calc_test data to VOFA+ (FireWater protocol)
 * @param  ct  Pointer to calc test instance
 *
 *         Channel mapping (10 channels):
 *         CH1: theta          — electrical angle [rad]
 *         CH2: v_a            — FOC-computed phase A voltage [V]
 *         CH3: v_b            — FOC-computed phase B voltage [V]
 *         CH4: v_c            — FOC-computed phase C voltage [V]
 *         CH5: i_a_raw        — REAL ADC phase A raw [0-4095] (JDR3)
 *         CH6: i_b_raw        — REAL ADC phase B raw [0-4095] (JDR2)
 *         CH7: i_c_raw        — REAL ADC phase C raw [0-4095] (JDR1)
 *         CH8: vbus_raw       — REAL ADC bus voltage raw [0-4095] (JDR4)
 *         CH9: i_a            — ADC raw as float (for bias check, ≈2048 idle)
 *         CH10: vbus          — Bus voltage [V] from ADC reading
 */
void Vofa_CalcTest_Send(CALC_TEST *ct);

#endif /* VOFA_H */
