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
* @brief  Main PCLIB header file for devices without FPU.
* 
*******************************************************************************/

#ifndef _PCLIB_H_
#define _PCLIB_H_

#if defined(__cplusplus)
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/
/*******************************************************************************
* Macros
*******************************************************************************/
#define PCLIB_Ctrl2P2ZInit_F16(psParam)          PCLIB_Ctrl2P2ZInit_F16_Asmi(psParam)
#define PCLIB_Ctrl2P2Z_F16(f16InErr, psParam)    PCLIB_Ctrl2P2Z_F16_Asm(f16InErr, psParam)
#define PCLIB_Ctrl3P3ZInit_F16(psParam)          PCLIB_Ctrl3P3ZInit_F16_Asmi(psParam)
#define PCLIB_Ctrl3P3Z_F16(f16InErr, psParam)    PCLIB_Ctrl3P3Z_F16_Asm(f16InErr, psParam)
#define PCLIB_CtrlPIDInit_F16(psParam)           PCLIB_CtrlPIDInit_F16_Asmi(psParam)
#define PCLIB_CtrlPID_F16(f16InErr, psParam)     PCLIB_CtrlPID_F16_Asm(f16InErr, psParam)
#define PCLIB_CtrlPIInit_F16(psParam)            PCLIB_CtrlPIInit_F16_Asmi(psParam)
#define PCLIB_CtrlPI_F16(f16InErr, psParam)      PCLIB_CtrlPI_F16_Asm(f16InErr, psParam)
#define PCLIB_CtrlPIandLPInit_F16(psParam)       PCLIB_CtrlPIandLPInit_F16_Asmi(psParam)
#define PCLIB_CtrlPIandLP_F16(f16InErr, psParam) PCLIB_CtrlPIandLP_F16_Asm(f16InErr, psParam) 

/******************************************************************************
* Includes
******************************************************************************/
#include "PCLIB_CtrlPI_F16_Asm.h"
#include "PCLIB_CtrlPIandLPFilter_F16_Asm.h"
#include "PCLIB_CtrlPID_F16_Asm.h"
#include "PCLIB_Ctrl2P2Z_F16_Asm.h"
#include "PCLIB_Ctrl3P3Z_F16_Asm.h"

#if defined(__cplusplus)
}
#endif  
  
#endif /* _PCLIB_H_ */
