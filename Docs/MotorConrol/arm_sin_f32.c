/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_sin_f32.c
 * Description:  Fast sine calculation for floating-point values
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "main.h"
#include "cordic.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include <stdint.h>

#define OUR_CORDIC_Q31_SCALE_F      (2147483648.0f)     /* 2^31 */
#define OUR_CORDIC_Q31_TO_FLOAT_F   (4.65661287308e-10f)/* 1/2^31 */
#define OUR_INV_TWO_PI_F            (0.159154943092f)   /* 1/(2*pi) */

static float32_t our_arm_sin_f32_lut(float32_t x) {
  float32_t sinVal, fract, in;
  uint16_t index;
  float32_t a, b;
  int32_t n;
  float32_t findex;

  in = x * OUR_INV_TWO_PI_F;
  n = (int32_t) in;
  if (x < 0.0f) {
    n--;
  }
  in = in - (float32_t) n;

  findex = (float32_t)FAST_MATH_TABLE_SIZE * in;
  index = (uint16_t)findex;
  if (index >= FAST_MATH_TABLE_SIZE) {
    index = 0;
    findex -= (float32_t)FAST_MATH_TABLE_SIZE;
  }

  fract = findex - (float32_t) index;
  a = sinTable_f32[index];
  b = sinTable_f32[index + 1];
  sinVal = (1.0f - fract) * a + fract * b;
  return sinVal;
}

static int32_t cordic_angle_q31_from_radians(float32_t x) {
  float32_t in = x * OUR_INV_TWO_PI_F;
  int32_t n = (int32_t)in;
  if (in < 0.0f) {
    n--;
  }
  in -= (float32_t)n;
  if (in >= 0.5f) {
    in -= 1.0f;
  }
  return (int32_t)((in * 2.0f) * OUR_CORDIC_Q31_SCALE_F);
}

/**
 * @ingroup groupFastMath
 */

/**
 * @defgroup sin Sine
 *
 * Computes the trigonometric sine function using a combination of table lookup
 * and linear interpolation.  There are separate functions for
 * Q15, Q31, and floating-point data types.
 * The input to the floating-point version is in radians and in the range [0 2*pi) while the
 * fixed-point Q15 and Q31 have a scaled input with the range
 * [0 +0.9999] mapping to [0 2*pi).  The fixed-point range is chosen so that a
 * value of 2*pi wraps around to 0.
 *
 * The implementation is based on table lookup using 256 values together with linear interpolation.
 * The steps used are:
 *  -# Calculation of the nearest integer table index
 *  -# Compute the fractional portion (fract) of the table index.
 *  -# The final result equals <code>(1.0f-fract)*a + fract*b;</code>
 *
 * where
 * <pre>
 *    b=Table[index+0];
 *    c=Table[index+1];
 * </pre>
 */

/**
 * @addtogroup sin
 * @{
 */

/**
 * @brief  Fast approximation to the trigonometric sine function for floating-point data.
 * @param[in] x input value in radians.
 * @return  sin(x).
 */

float32_t our_arm_sin_f32(
  float32_t x)
{
  CORDIC_ConfigTypeDef cfg = {
    .Function = CORDIC_FUNCTION_SINE,
    .Scale = CORDIC_SCALE_0,
    .InSize = CORDIC_INSIZE_32BITS,
    .OutSize = CORDIC_OUTSIZE_32BITS,
    .NbWrite = CORDIC_NBWRITE_1,
    .NbRead = CORDIC_NBREAD_1,
    .Precision = CORDIC_PRECISION_6CYCLES
  };

  int32_t in_q31 = cordic_angle_q31_from_radians(x);
  int32_t out_q31 = 0;

  if ((HAL_CORDIC_Configure(&hcordic, &cfg) == HAL_OK) &&
      (HAL_CORDIC_Calculate(&hcordic, &in_q31, &out_q31, 1U, 1U) == HAL_OK)) {
    return (float32_t)out_q31 * OUR_CORDIC_Q31_TO_FLOAT_F;
  }

  /* Fallback to LUT if CORDIC is unavailable/busy */
  return our_arm_sin_f32_lut(x);
}

/**
 * @} end of sin group
 */
