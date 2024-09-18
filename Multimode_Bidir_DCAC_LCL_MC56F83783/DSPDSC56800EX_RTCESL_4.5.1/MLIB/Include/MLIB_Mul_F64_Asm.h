/*******************************************************************************
*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*
****************************************************************************//*!
*
* @brief  Multiply functions with 64-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MUL_F64_ASM_H_
#define _MLIB_MUL_F64_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Mul_F64ll_Asm(f32Mult1, f32Mult2) MLIB_Mul_F64ll_FAsm(f32Mult1, f32Mult2)
#define MLIB_MulSat_F64ll_Asm(f32Mult1, f32Mult2) MLIB_MulSat_F64ll_FAsm(f32Mult1, f32Mult2)
#define MLIB_MulNeg_F64ll_Asm(f32Mult1, f32Mult2) MLIB_MulNeg_F64ll_FAsm(f32Mult1, f32Mult2)

#define MLIB_Mul_F64_Asm(f64Mult1, f64Mult2) MLIB_Mul_F64_FAsm(f64Mult1, f64Mult2)
#define MLIB_MulSat_F64_Asm(f64Mult1, f64Mult2) MLIB_MulSat_F64_FAsm(f64Mult1, f64Mult2)
#define MLIB_MulNeg_F64_Asm(f64Mult1, f64Mult2) MLIB_MulNeg_F64_FAsm(f64Mult1, f64Mult2)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern frac64_t MLIB_Mul_F64ll_FAsm(frac32_t f32Mult1, frac32_t f32Mult2);
extern frac64_t MLIB_MulSat_F64ll_FAsm(frac32_t f32Mult1, frac32_t f32Mult2);
extern frac64_t MLIB_MulNeg_F64ll_FAsm(frac32_t f32Mult1, frac32_t f32Mult2);

extern frac64_t MLIB_Mul_F64_FAsm(frac64_t f64Mult1, frac64_t f64Mult2);
extern frac64_t MLIB_MulSat_F64_FAsm(frac64_t f64Mult1, frac64_t f64Mult2);
extern frac64_t MLIB_MulNeg_F64_FAsm(frac64_t f64Mult1, frac64_t f64Mult2);

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MUL_F64_ASM_H_ */
