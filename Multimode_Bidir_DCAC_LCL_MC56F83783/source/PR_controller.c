/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "PR_controller.h"
#include "cpu.h"
/***************************************************************************//*!
*
* @brief   discrete resonant controller parameters calculation
*
* @param   INVSTRUC_PR_PARAMS_T *concoef
*			- structure of continuous PR controller parameters
*
*		   INVSTRUC_PR_2P2Z_T *discoef
*			- structure of discrete PR controller parameters
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
void ComputePRCoeff(ACDCSTRUC_PR_PARAMS_T *concoef, FILTER_2P2Z_TRANS_T *discoef)
{
	acc32_t a32temp1;
	frac32_t f32temp, f32temp1, f32temp2;
	frac32_t  f32wcDivfs, f32w0Divfs, f32w0DivfsSqu;
	
	f32wcDivfs = MLIB_Div1Q_A32ll(concoef->wc, concoef->fs)<<1;
	f32w0Divfs = MLIB_Div1Q_A32ll(concoef->w0, concoef->fs)<<1;
	f32w0DivfsSqu = MLIB_Mul_F32(f32w0Divfs, f32w0Divfs);
	
	f32temp = MLIB_Add_F32(FRAC32(0.5), f32wcDivfs>>1);
	f32temp = MLIB_Add_F32(f32temp, f32w0DivfsSqu>>3);
	f32temp = MLIB_Div1Q_F32(FRAC32(0.125), f32temp);
	
	a32temp1 = MLIB_Sub_A32as(ACC32(8.0), f32w0DivfsSqu>>15);
	discoef->sCoeff.i32A1 = MLIB_Mul_F32(a32temp1, f32temp);
	
	f32temp1 = MLIB_Mul_F32(f32wcDivfs<<3, f32temp);
	discoef->sCoeff.i32A2 = MLIB_Sub_F32(f32temp1, FRAC32(1.0));
	
	f32temp2 = MLIB_Mul_F32(f32wcDivfs<<2, f32temp);
	discoef->sCoeff.i32B0 = MLIB_Mul_F32(concoef->ki, f32temp2);
	discoef->sCoeff.i32B2 = -discoef->sCoeff.i32B0;
	discoef->sCoeff.i32B1 = 0;
	
	if(concoef->kp != 0)
	{
		discoef->sCoeff.i32B0 = MLIB_Add_F32(concoef->kp, discoef->sCoeff.i32B0);
		discoef->sCoeff.i32B1 = MLIB_Add_F32(discoef->sCoeff.i32B1, MLIB_Mul_A32(-concoef->kp, discoef->sCoeff.i32A1));
		discoef->sCoeff.i32B2 = MLIB_Sub_F32(discoef->sCoeff.i32B2, MLIB_Mul_F32(concoef->kp, discoef->sCoeff.i32A2));
	}
	
	discoef->sCoeff.i32A1 = discoef->sCoeff.i32A1<<12;  /* Q5.27 is used in PR calculation for calculation accuracy  */
	discoef->sCoeff.i32A2 = discoef->sCoeff.i32A2>>4;
	discoef->sCoeff.i32B0 = discoef->sCoeff.i32B0<<12;
	discoef->sCoeff.i32B1 = discoef->sCoeff.i32B1<<12;
	discoef->sCoeff.i32B2 = discoef->sCoeff.i32B2<<12;
}
#pragma section CODES_IN_RAM end

