/**
 ******************************************************************************
 * @file    vofa_test.h
 * @author  milFOC Team
 * @brief   VOFA+ JustFloat test sender — sends 4-channel sine waves via USB CDC
 *          to verify VOFA+上位机 connectivity without FOC running.
 ******************************************************************************
 */

#ifndef VOFA_TEST_H
#define VOFA_TEST_H

void VOFA_Test_Init(void);
void VOFA_Test_Task(void);

#endif /* VOFA_TEST_H */
