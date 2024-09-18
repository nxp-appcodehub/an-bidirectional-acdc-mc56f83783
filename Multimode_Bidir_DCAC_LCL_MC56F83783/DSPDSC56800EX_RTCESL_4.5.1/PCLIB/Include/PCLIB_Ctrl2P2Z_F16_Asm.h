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
****************************************************************************/
/*!
*
* @file       PCLIB_Ctrl2P2Z_F16_Asm.h
*
* @author    B37570
* 
* @version   2.0.0.0
* 
* @date      November 2014
* 
* @brief     2P2Z Control algorithm implemented in assembler
*
*******************************************************************************
*
* 2P2Z  algorithm implemented in assembler.
*
******************************************************************************/

#ifndef _PCLIB_CTRL2P2Z_F16_ASM_H_
#define _PCLIB_CTRL2P2Z_F16_ASM_H_

#if defined(__cplusplus)
extern "C" {
#endif

/******************************************************************************
* Includes
*******************************************************************************/  
#include "mlib.h"
#include "pclib_types.h"

/****************************************************************************
* Defines and macros            (scope: module-local)
****************************************************************************/
#define PCLIB_Ctrl2P2ZInit_F16_Asmi(psParam) PCLIB_Ctrl2P2ZInit_F16_FAsmi(psParam)
#define PCLIB_Ctrl2P2Z_F16_Asm(f16InErr, psParam) PCLIB_Ctrl2P2Z_F16_FAsm(f16InErr, psParam)

/****************************************************************************
* Implementation variant: 16-bit fractional
****************************************************************************/
 
  #define PCLIB_CTRL_2P2Z_DEFAULT_F16 {(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0}     /*!< Default value for PCLIB_CTRL_2P2Z_T_F16. */

  /************************************************************************/
  /*!
  @struct PCLIB_CTRL_2P2Z_T_F16 "\PCLIB_Ctrl2P2Z_F16.h"

  @brief  Structure containing parameters and states of the 2P 2Z controller.
  *//**********************************************************************/

  typedef struct{
    frac16_t f16CoeffB0;         /*!< Controller coefficient for present error. */
    frac16_t f16CoeffB1;         /*!< Controller coefficient for past error. */
    frac16_t f16CoeffB2;         /*!< Controller coefficient for past to past error. */
    frac16_t f16CoeffA1;         /*!< Controller coefficient for past result. */
    frac16_t f16CoeffA2;         /*!< Controller coefficient for past to past result. */
    frac16_t f16DelayX1;         /*!< Controller delay parameter, past error. */
    frac16_t f16DelayX2;         /*!< Controller delay parameter, past to past error. */
    frac16_t f16DelayY1;         /*!< Controller delay parameter, past result. */
    frac16_t f16DelayY2;         /*!< Controller delay parameter, past to past result. */
  }PCLIB_CTRL_2P2Z_T_F16;


/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************/
/*!
@brief        This function initialize the 2 pole 2 zero control algorithm structure.

@param[in,out]     psParam    Pointer to the structure of 2P 2Z controller parameters.

@return       Function returns no value.

@details      This function clears the internal buffers of a 2P 2Z controller. It
              shall be called before the filter parameter initialization and whenever
              the filter initialization is required.

@note         This function shall not be called together with 2P2Z controller
              unless periodic clearing of buffers is required.

@par Re-entrancy
              The function is re-entrant.
****************************************************************************/
extern inline void PCLIB_Ctrl2P2ZInit_F16_FAsmi(register PCLIB_CTRL_2P2Z_T_F16 *psParam)
{
    psParam->f16DelayX1 = (frac16_t)0;
    psParam->f16DelayY1 = (frac16_t)0;
    psParam->f16DelayX2 = (frac16_t)0;
    psParam->f16DelayY2 = (frac16_t)0;
}

/****************************************************************************
* Exported function prototypes
****************************************************************************/
   extern asm frac16_t PCLIB_Ctrl2P2Z_F16_FAsm(frac16_t f16InErr, PCLIB_CTRL_2P2Z_T_F16 *psParam);



#if defined(__cplusplus)
}
#endif

#endif /* _PCLIB_CTRL2P2Z_F16_ASM_H_ */
