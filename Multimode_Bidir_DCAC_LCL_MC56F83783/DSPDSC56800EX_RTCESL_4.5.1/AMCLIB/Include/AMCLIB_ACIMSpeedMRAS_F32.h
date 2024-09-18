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
* @brief ACIM Speed Estimator based on MRAS   
* 
*******************************************************************************/
#ifndef _AMCLIB_ACIMSPEEDMRAS_F32_H_
#define _AMCLIB_ACIMSPEEDMRAS_F32_H_

#if defined(__cplusplus) 
extern "C" {
#endif
  
/****************************************************************************
* Includes
****************************************************************************/   
#include "amclib_types.h" 
#include "gdflib.h"  
#include "gflib.h"     
#include "gmclib.h"     
#include "mlib.h"

/******************************************************************************
* Macros 
******************************************************************************/      
#define AMCLIB_ACIMSpeedMRAS_F16_C(psIsAlBe, psPsiRAlBe, f16RotPos,  psCtrl) \
        AMCLIB_ACIMSpeedMRAS_F16_FC(psIsAlBe, psPsiRAlBe, f16RotPos,  psCtrl) 
#define AMCLIB_ACIMSpeedMRASInit_F16_Ci(psCtrl)                              \
        AMCLIB_ACIMSpeedMRASInit_F16_FCi(psCtrl)                                              
            
/******************************************************************************
* Types 
******************************************************************************/     
typedef struct
{   
    GDFLIB_FILTER_IIR1_T_F32 sSpeedElIIR1Param; /* IIR1 filter structure for estimated speed */ 
    
    /* function output variables */
    GMCLIB_2COOR_DQ_T_F32 sPsiRotRDQ; /* Rotor flux estimated structure from rotor (current) model, in/out structure */
    GFLIB_INTEGRATOR_T_A32 sSpeedInteg; /* Structure contains the integrator parameters */
    /* function parameters */    
    frac32_t f32KPsiRA1Gain; /* Constant determined by: Tau_r / (Tau_r + Ts); must be < 1 */                   
    frac32_t f32KPsiRB1Gain; /* Constant determined by: Lm * Ts / (Tau_r) * i_max / u_max; must be < 1 */                   
    frac32_t f32KImaxGain; /* Constant determined by: 1/i_max */
    frac32_t f32Error;
    
    frac32_t f32Ts; 
    frac16_t f16RotPos; /* Rotor estimated electric position, output variable  */ 

    frac16_t f16SpeedEl; /* Rotor estimated electric speed, output variable  */
    frac16_t f16SpeedElIIR1; /* Rotor estimated electric speed filtered,  output variable  */
    
    /* function state variables */
    struct                                       
    {                                            
        frac32_t f32SpeedElInteg_1; /* Integral part state variable for rotor electrical estimated speed controller  */  
        frac32_t f32SpeedElErr_1; /* Error_1 state variable for rotor electrical estimated speed controller */        
        acc32_t a32PGain; /* Proportional gain Kp for MRAS PI controller */                                              
        acc32_t a32IGain; /* Integration gain Ki for MRAS PI controller */ 
    } sCtrl;    

} AMCLIB_ACIM_SPEED_MRAS_T_F32;   
   
/****************************************************************************
* Exported function prototypes
****************************************************************************/   
extern void AMCLIB_ACIMSpeedMRAS_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psISAlBe,    
											const GMCLIB_2COOR_ALBE_T_F32 *psPsiRAlBe,  
											frac16_t f16RotPos,
											AMCLIB_ACIM_SPEED_MRAS_T_F32 *psCtrl); 
                                        
/****************************************************************************
* Inline functions 
****************************************************************************/                                 

/***************************************************************************//*!
* @brief  The function initializes the actual values of float AMCLIB_ACIMSpeedMRAS function.
*
* @params:      ptr  AMCLIB_ACIM_SPEED_MRAS_T_F16 *psParam - Pointer to MRAS structure                
*
* @return       N/A
* 
*******************************************************************************/
inline void AMCLIB_ACIMSpeedMRASInit_F16_FCi(AMCLIB_ACIM_SPEED_MRAS_T_F32 *psCtrl)
{
    GDFLIB_FilterIIR1Init_F16(&psCtrl->sSpeedElIIR1Param);
    GFLIB_IntegratorInit_F16(0, &psCtrl->sSpeedInteg);
    psCtrl->sPsiRotRDQ.f32D = 0;
    psCtrl->sPsiRotRDQ.f32Q = 0;    
    psCtrl->f16RotPos       = 0;
    psCtrl->sCtrl.f32SpeedElErr_1   = 0;
    psCtrl->sCtrl.f32SpeedElInteg_1 = 0;
}
   
#if defined(__cplusplus) 
}
#endif

#endif /* _AMCLIB_ACIMSPEEDMRAS_F32_H_ */
