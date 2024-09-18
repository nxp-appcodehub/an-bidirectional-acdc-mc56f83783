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
* @brief  ACIM Flux Observer  
* 
*******************************************************************************/
#ifndef _AMCLIB_ACIMROTFLUXOBSERVER_A32_H_
#define _AMCLIB_ACIMROTFLUXOBSERVER_A32_H_

#if defined(__cplusplus) 
extern "C" {
#endif
  
/****************************************************************************
* Includes
****************************************************************************/   
#include "amclib_types.h"  
#include "gflib.h"     
#include "gmclib.h"     
#include "mlib.h"

/******************************************************************************
* Macros 
******************************************************************************/   
#define AMCLIB_ACIMRotFluxObsrv_F16_C(psIsAlBe, psUSAlBe, psCtrl)             \
        AMCLIB_ACIMRotFluxObsrv_F16_FC(psIsAlBe, psUSAlBe, psCtrl)                                                    
#define AMCLIB_ACIMRotFluxObsrvInit_F16_Ci(psCtrl)                           \
        AMCLIB_ACIMRotFluxObsrvInit_F16_FCi(psCtrl)              
            
/******************************************************************************
* Types 
******************************************************************************/     
typedef struct
{   /* function output variables */     
    GMCLIB_2COOR_DQ_T_F32 sPsiRotRDQ; /* Rotor flux estimated structure from rotor (current) model, in/out structure */   
    GMCLIB_2COOR_ALBE_T_F32 sPsiRotSAlBe; /* Rotor flux estimated structure from stator (voltage) model, in/out structure */   
    GMCLIB_2COOR_ALBE_T_F32 sPsiStatSAlBe; /* Stator flux estimated structure from stator (voltage) model, in/out structure */ 
    
    /* function state variables */
    struct  
    {   
        frac32_t f32CompAlphaInteg_1; /* Integral part state variable for alpha coefficient */                               
        frac32_t f32CompBetaInteg_1; /* Integral part state variable for beta coefficient */ 
        acc32_t a32PGain; /* Proportional gain Kp for PI controller */                              
        acc32_t a32IGain; /* Integration gain Ki for PI controller */       
    } sCtrl;       
    
    /* function parameters */ /* Constant Tau_r = Lr / Rr, CoeffRFO    */
    frac32_t f32KPsiRA1Gain; /* Constant determined by: Tau_r / (Tau_r + Ts); must be < 1 */                   
    frac32_t f32KPsiRB1Gain; /* Constant determined by: Lm * Ts / (Tau_r) * i_max / u_max; must be < 1 */                   
    frac32_t f32KPsiSA1Gain; /* Constant determined by: (1 / (2 * PI * f1)) / ((1 / (2 * PI * f1)) + Ts); must be < 1 */                   
    frac32_t f32KPsiSA2Gain; /* Constant determined by: (Ts / (2 * PI * f1)) / ((1 / (2 * PI * f1)) + Ts); must be < 1 */                   
    acc32_t a32KrInvGain; /* Constant determined by: Lr / Lm */                              
    acc32_t a32KrLsTotLeakGain; /* Constant determined by: (Lr / Lm) * Ls * (1 - ((Lm * Lm) / (Ls * Lr))) * i_max / u_max*/                           
    acc32_t a32TorqueGain; /* Torque constant given by: 3 * Pp * Lm / (2 * Lr) / i_max */ 
    frac16_t f16Torque; /* Motor torque calculated from currents and fluxes */
    frac16_t f16KRsEst; /* Estimated stator resistance = Rs [ohm] / (u_max / i_max); u_max and i_max must be properly scale so Rs_sc < 1 */  
    frac16_t f16RotFluxPos; /* Rotor flux estimated position (angle), in/out variable */

} AMCLIB_ACIM_ROT_FLUX_OBSRV_T_A32;     
   
/****************************************************************************
* Exported function prototypes
****************************************************************************/
extern void AMCLIB_ACIMRotFluxObsrv_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psISAlBe,
                                    	   const GMCLIB_2COOR_ALBE_T_F16 *psUSAlBe,                                               
                                    	   AMCLIB_ACIM_ROT_FLUX_OBSRV_T_A32 *psCtrl); 
                                     
/****************************************************************************
* Inline functions 
****************************************************************************/
/***************************************************************************//*!
* @brief  The function initializes the actual values of float AMCLIB_ACIMRotFluxObsrv function.
*
* @params:      ptr  AMCLIB_ACIM_ROT_FLUX_OBSRV_T_A32 *psParam - Pointer to flux observer structure                
*
* @return       N/A
* 
*******************************************************************************/
inline void AMCLIB_ACIMRotFluxObsrvInit_F16_FCi(AMCLIB_ACIM_ROT_FLUX_OBSRV_T_A32 *psCtrl)
{
    psCtrl->sPsiRotRDQ.f32D           = 0;
    psCtrl->sPsiRotRDQ.f32Q           = 0;    
    psCtrl->sPsiRotSAlBe.f32Alpha     = 0;
    psCtrl->sPsiRotSAlBe.f32Beta      = 0;
    psCtrl->sPsiStatSAlBe.f32Alpha    = 0;
    psCtrl->sPsiStatSAlBe.f32Beta     = 0;
    psCtrl->f16RotFluxPos             = 0;
    psCtrl->sCtrl.f32CompAlphaInteg_1 = 0;
    psCtrl->sCtrl.f32CompBetaInteg_1  = 0;

}

#if defined(__cplusplus) 
}
#endif

#endif /* _AMCLIB_ACIMROTFLUXOBSERVER_A32_H_ */
