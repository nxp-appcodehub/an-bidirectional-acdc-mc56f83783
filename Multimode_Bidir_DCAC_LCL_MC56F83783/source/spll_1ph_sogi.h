/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/


#ifndef SPLL_1PH_SOGI_H_
#define SPLL_1PH_SOGI_H_

#include "mlib.h"
#include "gflib.h"


/******************************************************************************
* Types
******************************************************************************/
/*  */
typedef struct
{
	acc32_t  k;
	frac32_t b0;
	frac32_t b2;
	frac32_t a1;
	frac32_t a2;
	frac32_t qb0;
	frac32_t qb1;
	frac32_t qb2;
}SPLL_1PH_SOGI_PD_COEFF_T;

/* coefficient of loop PI controller, discrete using bi-linear method
   b0=kp+ki*Ts/2, b1=ki*Ts/2 */ 
typedef struct
{
	acc32_t b0;
	acc32_t b1;
}SPLL_1PH_SOGI_LCTRL_COEFF_T;

typedef struct
{
	frac32_t f32Uin[3];
	frac32_t f32PDout[3];
	frac32_t f32PDQout[3];
	frac32_t f32ParkoutQ[2];
	frac32_t f32ParkoutD[2];	
	frac32_t a32LCtrlout[2];
	acc32_t  a32NominalFreq; 
	acc32_t  a32PLLoutFreq;
	frac16_t f16PLLoutsine;
	frac16_t f16PLLoutcosine;
	acc32_t  a32PLLCtrlFreq;
	frac16_t f16Angle;
	
	SPLL_1PH_SOGI_PD_COEFF_T    sSogiPDCoeff;
	SPLL_1PH_SOGI_LCTRL_COEFF_T sSogiLCtrlCoeff;	
}SPLL_1PH_SOGI_T;
/******************************************************************************
* Functions
******************************************************************************/
static inline void SPLL_1PH_SOGI_PDCoeffConfig(SPLL_1PH_SOGI_T *spllobj,acc32_t a32acFreq,acc32_t a32ctrlFreq,acc32_t a32sogik)
{
	frac32_t f32osgix, f32osgiy, f32wnDivfs;
	acc32_t a32wn;
	frac32_t f32temp1,f32temp2,f32temp3;
	
	spllobj->a32NominalFreq = a32acFreq;
	spllobj->a32PLLCtrlFreq = a32ctrlFreq;
	spllobj->sSogiPDCoeff.k = a32sogik;
	
	//a32wn = MLIB_Mul_A32(ACC32(6.2831853),spllobj->a32NominalFreq);
	a32wn = MLIB_Mul_F32(ACC32(6.2831853)<<8,spllobj->a32NominalFreq<<8); // A32 lib is preferred, but there is some problem with it
	f32wnDivfs = MLIB_Div1Q_F32(a32wn,spllobj->a32PLLCtrlFreq)>>1; //PLL control frequency/2 is used because it's out of Q16.15 range
	f32osgix = MLIB_Mul_F32(spllobj->sSogiPDCoeff.k<<12,f32wnDivfs<<4)<<1; //x=2k*wn*Ts
	f32osgiy = MLIB_Mul_F32(f32wnDivfs,f32wnDivfs); //y=(wn*Ts)^2
	f32temp1 = MLIB_AddSat_F32(ACC32(4.0)<<13, MLIB_AddSat_F32(f32osgix,f32osgiy)>>3); //Q4.28, f32temp1=4+x+y
	f32temp2 = MLIB_SubSat_F32(MLIB_SubSat_F32(f32osgix,f32osgiy)>>3, ACC32(4.0)<<13); //Q4.28, f32temp2=x-y-4;
	f32temp3 = MLIB_Div1Q_F32(FRAC32(0.125),f32temp1); //Q1.31, f32temp3=1/(4+x+y)
	
	spllobj->sSogiPDCoeff.a1 = MLIB_Mul_F32(MLIB_Sub_F32(ACC32(4.0)<<13,f32osgiy>>3),f32temp3)<<1; //Q4.28, a1=2*(4-y)/(x+y+4)
	spllobj->sSogiPDCoeff.a2 = MLIB_Mul_F32(f32temp2,f32temp3)<<3; //Q1.31=(Q4.28*Q1.31)<<3, a2=(x-y-4)/(x+y+4)
	spllobj->sSogiPDCoeff.b0 = MLIB_Mul_F32(f32osgix,f32temp3); //Q1.31, b0=x/(x+y+4)
	spllobj->sSogiPDCoeff.b2 = MLIB_Neg_F32(spllobj->sSogiPDCoeff.b0); //Q1.31, b2=-b0
	spllobj->sSogiPDCoeff.qb0 = MLIB_Mul_F32(MLIB_Mul_F32(spllobj->sSogiPDCoeff.k<<12,f32osgiy<<4),f32temp3); //Q1.31, qb0=k*y/(x+y+4)
	spllobj->sSogiPDCoeff.qb1 = spllobj->sSogiPDCoeff.qb0<<1; //Q1.31, qb1=2*qb0
	spllobj->sSogiPDCoeff.qb2 = spllobj->sSogiPDCoeff.qb0;
}



static inline void SPLL_1PH_SOGI_LFConfig(SPLL_1PH_SOGI_T *spllobj,acc32_t a32b0,acc32_t a32b1)
{
	spllobj->sSogiLCtrlCoeff.b0 = a32b0;
	spllobj->sSogiLCtrlCoeff.b1 = a32b1;
}

static inline void SPLL_1PH_SOGI_Reset(SPLL_1PH_SOGI_T *spllobj)
{
	spllobj->f32Uin[0] = 0;
	spllobj->f32Uin[1] = 0;
	spllobj->f32Uin[2] = 0;
	
	spllobj->f32PDout[0] = 0;
	spllobj->f32PDout[1] = 0;
	spllobj->f32PDout[2] = 0;
	
	spllobj->f32PDQout[0] = 0;
	spllobj->f32PDQout[0] = 0;
	spllobj->f32PDQout[0] = 0;
	
	spllobj->f32ParkoutD[0] = 0;
	spllobj->f32ParkoutD[1] = 0;
	
	spllobj->f32ParkoutQ[0] = 0;
	spllobj->f32ParkoutQ[1] = 0;
	
	spllobj->a32LCtrlout[0] = 0;
	spllobj->a32LCtrlout[1] = 0;
	
	spllobj->f16Angle = 0;
}

//#pragma section CODES_IN_RAM begin
//#pragma interrupt called
void inline static SPLL_1PH_SOGI_Run(SPLL_1PH_SOGI_T *spllobj, frac32_t f32in)
{
	frac32_t f32temp1, f32temp2;
	frac16_t f16temp1;
	
	spllobj->f32Uin[0] = f32in;
	//sogi PD, orthogoanl signal generator
	f32temp1 = MLIB_Add_F32(MLIB_Mul_F32(spllobj->sSogiPDCoeff.b0,MLIB_SubSat_F32(spllobj->f32Uin[0],spllobj->f32Uin[2]))>>3, \
			                MLIB_Mul_F32(spllobj->sSogiPDCoeff.a2,spllobj->f32PDout[2])>>3); //Q4.28
	f32temp2 = MLIB_Add_F32(f32temp1,MLIB_Mul_F32(spllobj->sSogiPDCoeff.a1,spllobj->f32PDout[1])); //Q4.28
	spllobj->f32PDout[0] = MLIB_ShLSat_F32(f32temp2,3); //Q1.31
	spllobj->f32PDout[2] = spllobj->f32PDout[1];
	spllobj->f32PDout[1] = spllobj->f32PDout[0];
	
	//f32temp1 = MLIB_Add4_F32(MLIB_Mul_F32(spllobj->sSogiPDCoeff.qb0,spllobj->f32Uin[0]), \
			                 MLIB_Mul_F32(spllobj->sSogiPDCoeff.qb1,spllobj->f32Uin[1]), \
			                 MLIB_Mul_F32(spllobj->sSogiPDCoeff.qb2,spllobj->f32Uin[2]), \
			                 MLIB_Mul_F32(spllobj->sSogiPDCoeff.a2,spllobj->f32PDQout[2]))>>3; //Q4.28	
	f32temp1 = MLIB_Add_F32(MLIB_Mul_F32(spllobj->sSogiPDCoeff.qb0,spllobj->f32Uin[0]), \
			                MLIB_Mul_F32(spllobj->sSogiPDCoeff.qb1,spllobj->f32Uin[1]));
	f32temp1 = MLIB_Add_F32(f32temp1,MLIB_Mul_F32(spllobj->sSogiPDCoeff.qb2,spllobj->f32Uin[2]));
	f32temp1 = MLIB_Add_F32(f32temp1,MLIB_Mul_F32(spllobj->sSogiPDCoeff.a2,spllobj->f32PDQout[2]))>>3; //Q4.28
	
	f32temp2 = MLIB_Add_F32(f32temp1,MLIB_Mul_F32(spllobj->sSogiPDCoeff.a1,spllobj->f32PDQout[1])); //Q4.28
	spllobj->f32PDQout[0] = MLIB_ShLSat_F32(f32temp2,3); //Q1.31	
	spllobj->f32PDQout[2] = spllobj->f32PDQout[1];
	spllobj->f32PDQout[1] = spllobj->f32PDQout[0];
	
	spllobj->f32Uin[2] = spllobj->f32Uin[1];
	spllobj->f32Uin[1] = spllobj->f32Uin[0];
	
	//park transform 
	spllobj->f32ParkoutD[0] = MLIB_SubSat_F32(MLIB_Mul_F32((frac32_t)spllobj->f16PLLoutcosine<<16,spllobj->f32PDQout[0]), \
			                               MLIB_Mul_F32((frac32_t)spllobj->f16PLLoutsine<<16,spllobj->f32PDout[0]));
	spllobj->f32ParkoutQ[0] = MLIB_AddSat_F32(MLIB_Mul_F32((frac32_t)spllobj->f16PLLoutcosine<<16,spllobj->f32PDout[0]), \
			                               MLIB_Mul_F32((frac32_t)spllobj->f16PLLoutsine<<16,spllobj->f32PDQout[0]));

	//low pass controller, y[0]=y[1]+b0*x[0]+b1*x[1]
	f32temp1 = MLIB_Add_F32(MLIB_Mul_F32(spllobj->sSogiLCtrlCoeff.b0,spllobj->f32ParkoutQ[0]), \
			                   MLIB_Mul_F32(spllobj->sSogiLCtrlCoeff.b1,spllobj->f32ParkoutQ[1])); //MLIB_Mul_F32(q17.15*q1.31)=q17.15
	spllobj->a32LCtrlout[0] = MLIB_Add_F32(f32temp1,spllobj->a32LCtrlout[1]);
	spllobj->a32LCtrlout[1] = spllobj->a32LCtrlout[0];
	
	//spllobj->f32ParkoutD[1] = spllobj->f32ParkoutD[0];
	spllobj->f32ParkoutQ[1] = spllobj->f32ParkoutQ[0];
	
	//locked angle calculation
	spllobj->a32PLLoutFreq = MLIB_Add_F32(spllobj->a32NominalFreq,spllobj->a32LCtrlout[0]);
	spllobj->f16Angle = MLIB_Add_F16(spllobj->f16Angle,(frac16_t)(MLIB_Div_F32(spllobj->a32PLLoutFreq,spllobj->a32PLLCtrlFreq)>>16));
	
	spllobj->f16PLLoutsine = GFLIB_Sin_F16(spllobj->f16Angle);
	spllobj->f16PLLoutcosine = GFLIB_Cos_F16(spllobj->f16Angle);
}
//#pragma interrupt off
//#pragma section CODES_IN_RAM end*/

#endif /* SPLL_1PH_SOGI_H_ */
