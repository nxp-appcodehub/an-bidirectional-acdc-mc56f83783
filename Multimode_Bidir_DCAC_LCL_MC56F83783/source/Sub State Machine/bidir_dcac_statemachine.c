/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/******************************************************************************
* Includes
******************************************************************************/
#include "bidir_dcac_statemachine.h"
#include "hwcontrol.h"
#include "spll_1ph_sogi.h"

/******************************************************************************
* Global variables
******************************************************************************/
uint32_t   guw32StartCnt; /* time interval measurement start count */
uint16_t   gu16TriacOnCnt=0;

SM_APP_CTRL_T            gsACDC_Ctrl;
ACDCSTRUC_BI_DIR_T       gsACDC_Drive;

/****** AC_TO_DC structure ******/
PFC_RUN_SUBSTATE_T   gsPFC_Runsub;
SPLL_1PH_SOGI_T      spll_obj;

/******************************************************************************
* Local variables
******************************************************************************/
static bool_t     bACDC_Run; /* ACDC run/stop command */

/* AC_TO_DC */
bool_t     bPFC_Precharge; /* ACDC pre-charge command */
uint16_t   u16PWMOffDurCnt; /* a counter to record how many half cycles of PWM turn off at light load */ 

/******************************************************************************
* functions
******************************************************************************/
void ACDC_FaultDetection(void);
bool_t ACDC_TimeDelay(uint32_t uw32timerstartval, uint32_t uw32delaytime);

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void ACDC_StateFault(void);
static void ACDC_StateInit(void);
static void ACDC_StateStop(void);
static void ACDC_StateRun(void);
/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void ACDC_TransFaultInit(void);
static void ACDC_TransInitFault(void);
static void ACDC_TransInitStop(void);
static void ACDC_TransStopFault(void);
static void ACDC_TransStopInit(void);
static void ACDC_TransStopRun(void);
static void ACDC_TransRunFault(void);
static void ACDC_TransRunStop(void);

/* State machine functions field (in pmem) */
static const SM_APP_STATE_FCN_T msSTATE = {ACDC_StateFault, ACDC_StateInit, ACDC_StateStop, ACDC_StateRun};

/* State-transition functions field (in pmem) */
static const SM_APP_TRANS_FCN_T msTRANS = {ACDC_TransFaultInit, ACDC_TransInitFault, ACDC_TransInitStop, ACDC_TransStopFault, ACDC_TransStopInit, ACDC_TransStopRun, ACDC_TransRunFault, ACDC_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsACDC_Ctrl = 
{
	/* gsMC_Ctrl.psState, User state functions  */
	&msSTATE,
 	
 	/* gsMC_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsMC_Ctrl.uiCtrl, Deafult no control command */
  	SM_CTRL_NONE,
  	
  	/* gsMC_Ctrl.eState, Default state after reset */
  	INIT 	
};

#pragma section CODES_IN_RAM begin
/***************************************************************************//*!
*
* @brief   Fault Detection function
*
* @param   void
*
* @return  none
*
******************************************************************************/
void ACDC_FaultDetection(void)
{
	/* Clearing actual faults before detecting them again  */	
	gsACDC_Drive.sFaultIdPending.R = 0;
	
	/* software over-current protection */
	if(MLIB_Abs_F16(gsACDC_Drive.sIlCtrl.f16IlFilt) > gsACDC_Drive.sFaultThresholds.f16IlOver)
	{
		gsACDC_Drive.sFaultIdPending.B.IlOver = 1;
	}
	if(MLIB_Abs_F16(gsACDC_Drive.sIGridCtrl.f16IGridFilt) > gsACDC_Drive.sFaultThresholds.f16IGridOver)
	{
		gsACDC_Drive.sFaultIdPending.B.IGridOver = 1;
	}
	/* IGBT over-temperature protection */
	if(gsACDC_Drive.f16TempSamp < gsACDC_Drive.sFaultThresholds.f16TempOver)  
	{
		gsACDC_Drive.sFaultIdPending.B.TempOver = 1;
	}
		
	/* DC bus protection */
	if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > gsACDC_Drive.sFaultThresholds.f16VdcOver)
	{
		gsACDC_Drive.sFaultIdPending.B.VdcOver = 1;
	}
	else if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt < gsACDC_Drive.sFaultThresholds.f16VdcUnder)
	{
		gsACDC_Drive.sFaultIdPending.B.VdcUnder = 1;
	}		
	
	if(gsACDC_Drive.sFlag.VacFaultEn)	/* AC side voltage protection */
	{
		if((gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)||(gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV))
		{
		    if(gsACDC_Drive.sPowerMetering.f16VGridRms < gsACDC_Drive.sFaultThresholds.f16VacRMSUnder)
		    {
			    gsACDC_Drive.sFaultIdPending.B.VacRMSUnder = 1;
		    }
		    else if(gsACDC_Drive.sPowerMetering.f16VGridRms > gsACDC_Drive.sFaultThresholds.f16VacRMSOver)
		    {
		    		gsACDC_Drive.sFaultIdPending.B.VacRMSOver = 1;
		    }
		    if(gsACDC_Drive.sPowerMetering.a32VGridFreq > gsACDC_Drive.sFaultThresholds.a32VGridFreqOver)
		    {
			    gsACDC_Drive.sFaultIdPending.B.VacFreqOver = 1;
		    }
		    else if(gsACDC_Drive.sPowerMetering.a32VGridFreq < gsACDC_Drive.sFaultThresholds.a32VGridFreqUnder)
		    {
			    gsACDC_Drive.sFaultIdPending.B.VacFreqUnder = 1;
		    }
		    if(gsACDC_Drive.sVacDetect.f16VGridFiltAbs > gsACDC_Drive.sFaultThresholds.f16VacOver)
		    {
		    	gsACDC_Drive.sFaultIdPending.B.VacOver = 1;
		    }		    
		}
		else if(gsACDC_Drive.u16CurrentInvMode == OFFGRID_INV)
		{
			if(gsACDC_Drive.sPowerMetering.f16VInvRms < gsACDC_Drive.sFaultThresholds.f16VacRMSUnder)
			{
				gsACDC_Drive.sFaultIdPending.B.VacRMSUnder = 1;
			}
			else if(gsACDC_Drive.sPowerMetering.f16VInvRms > gsACDC_Drive.sFaultThresholds.f16VacRMSOver)
			{
				gsACDC_Drive.sFaultIdPending.B.VacRMSOver = 1;
			}
			if(gsACDC_Drive.sPowerMetering.a32VInvFreq > gsACDC_Drive.sFaultThresholds.a32VInvFreqOver)
			{
				gsACDC_Drive.sFaultIdPending.B.VacFreqOver = 1;
			}
			else if(gsACDC_Drive.sPowerMetering.a32VInvFreq < gsACDC_Drive.sFaultThresholds.a32VInvFreqUnder)
			{
				gsACDC_Drive.sFaultIdPending.B.VacFreqUnder = 1;
			}
			if(MLIB_Abs_F16(gsACDC_Drive.sVacCtrl.f16VInvFilt) > gsACDC_Drive.sFaultThresholds.f16VacOver)
			{
				gsACDC_Drive.sFaultIdPending.B.VacOver = 1;
			}
		}		
	}

	if(ACDC_HW_OVERVOLT()) /* hw fault flag is not cleared here, so the system won't restart */
	{
		gsACDC_Drive.sFaultIdPending.B.HW_VdcOver = 1;
	}
	if(gsACDC_Drive.u16WorkModeUsed==AC_TO_DC || gsACDC_Drive.u16CurrentInvMode==GRIDCONNECTED_INV)
	{
		if(ACDC_HW_OVERCUR())  gsACDC_Drive.sFaultIdPending.B.HW_IlOver = 1;
	}
	
	/* pass fault to FaultId for recording */
	gsACDC_Drive.sFaultId.R |= gsACDC_Drive.sFaultIdPending.R;
	
	if(gsACDC_Drive.sFaultIdPending.R)
	{
		gsACDC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
	}
}

/***************************************************************************//*!
*
* @brief   time delay function
*
* @param   uw16timerstartval - the counter value recorded at the start point of the delay
*          uw16delaytime - required delay time, counter difference 
*
* @return  bool_t - delay ready flag
*
******************************************************************************/
bool_t ACDC_TimeDelay(uint32_t uw32timerstartval, uint32_t uw32delaytime)
{
	uint32_t uw32delta_t;
	
	if(uw32timerstartval > gu32TimerCnt)
	{
	 	uw32delta_t = gu32TimerCnt + 0xFFFFFFFF - uw32timerstartval;
	}
	else
	{
	 	uw32delta_t = gu32TimerCnt - uw32timerstartval;
	}
	if(uw32delta_t >= uw32delaytime)   return 1;
	else                               return 0;
}
#pragma section CODES_IN_RAM end

/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_StateFault()
{
	ACDC_FASTBRIDGE_PWM_DIS();
	ACDC_SLOWBRIDGE_PWM_DIS();
	ACDC_FASTBRIDGE_PWM_NOMASK();
	OPEN_SW_ALL();
	
	if(gu32IOTogTimeCnt>=2000) /* rapid LED flash indicate fault status */
	{
		HVP_LED_TOGGLE();
		gu32IOTogTimeCnt=0;
	}
	
	gsACDC_Drive.u16VacRecoveryOkCnt = 0;
	
	if(!gsACDC_Drive.sFaultIdPending.R) // no fault detected for a certain time interval, recovery from INIT state
	{
		if(ACDC_TimeDelay(guw32StartCnt, FAULT_RELEASE_DURATION))
		{
			//gsACDC_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR; /* fault recovery is not enabled here */
		}
	}
	else guw32StartCnt = gu32TimerCnt;	
}

/***************************************************************************//*!
*
* @brief   Init state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_StateInit()
{
#if BOARD_TEST
	gsACDC_Drive.u16WorkModeUsed = DC_TO_AC;
	gsACDC_Drive.u16CurrentInvMode = OFFGRID_INV;
	
#else
	if((gsACDC_Drive.u16WorkModeCmd == AC_TO_DC)||(gsACDC_Drive.u16WorkModeCmd == DC_TO_AC))
	{
	    gsACDC_Drive.u16WorkModeUsed = gsACDC_Drive.u16WorkModeCmd;
	    if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)
	    {
	    	gsACDC_Drive.u16CurrentInvMode = gsACDC_Drive.u16InvModeCmd;
	    }
	}
#endif
	
	if(!gsACDC_Drive.sFlag.VarInitReady)
	{
		bACDC_Run = 0;		
		
		/*====== PWM reset, ensure start from a small duty cycle to avoid current spike ======*/
		RESETOUTPUTPOLARITY;	
		PWMA->MCTRL |= PWM_MCTRL_CLDOK(1);  
		PWMA->SM[0].VAL2 = 0;
		PWMA->SM[0].VAL3 = 0;
		PWMA->MCTRL |= PWM_MCTRL_LDOK(1);
				
		/*================== fault state initialisation ===================*/
		gsACDC_Drive.sFaultId.R = 0;
		gsACDC_Drive.sFaultIdPending.R = 0;
		ACDC_CLEAR_HW_FAULT();
		
		if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)
		{
			gsACDC_Drive.sFaultThresholds.f16VacRMSOver = ACDC_VAC_RMS_UP_LIMIT;
			gsACDC_Drive.sFaultThresholds.f16VacRMSUnder = ACDC_VAC_RMS_LOW_LIMIT;		
			gsACDC_Drive.sFaultThresholds.a32VGridFreqOver = ACDC_VAC_OVERFREQ_LIMIT;
			gsACDC_Drive.sFaultThresholds.a32VGridFreqUnder = ACDC_VAC_UNDERFREQ_LIMIT;
		}
		else if(gsACDC_Drive.u16CurrentInvMode!=GRIDCONNECTED_INV) 
		{
			/* It only takes effect when the inverter  is entered for the first time after reset and the mode is off-grid mode. At this time, the inverter
			 * output is determined by the macro definition rather than the current grid. */
			gsACDC_Drive.sVacDetect.f16VacrmsNominal = FRAC16(INV_AC_REF_RMS/VAC_SCALE);
			gsACDC_Drive.sVacDetect.a32VacFreqNominal = ACC32(INV_AC_FREQ);
			gsACDC_Drive.sFaultThresholds.f16VacRMSOver = MLIB_Add_F16(gsACDC_Drive.sVacDetect.f16VacrmsNominal, FRAC16(40.0/VAC_SCALE));
			gsACDC_Drive.sFaultThresholds.f16VacRMSUnder = MLIB_Sub_F16(gsACDC_Drive.sVacDetect.f16VacrmsNominal, FRAC16(30.0/VAC_SCALE));
			gsACDC_Drive.sFaultThresholds.a32VInvFreqOver = MLIB_Add_F32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(3.0));
			gsACDC_Drive.sFaultThresholds.a32VInvFreqUnder = MLIB_Sub_F32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(3.0));
		}

	    gsACDC_Drive.sFaultThresholds.f16VacOver = FRAC16(ACDC_VAC_OVERVOLT_LIMIT/VAC_SCALE);
	    gsACDC_Drive.sFaultThresholds.f16IlOver = FRAC16(ACDC_ILOVER_LIMIT/ISNS_SCALE);
	    gsACDC_Drive.sFaultThresholds.f16IGridOver = FRAC16(ACDC_IGRIDOVER_LIMIT/ISNS_SCALE);
	    gsACDC_Drive.sFaultThresholds.f16TempOver = FRAC16(ACDC_TEMPERATURE_LIMIT);
	    gsACDC_Drive.sFaultThresholds.f16VdcOver = FRAC16(ACDC_VDC_OVERVOLT_LIMIT/VDC_SCALE);
	    gsACDC_Drive.sFaultThresholds.f16VdcUnder = 0;
	    
	    /*================== flag initialization ==========================*/
	    gsACDC_Drive.sFlag.AcDrop = 0; 
	    gsACDC_Drive.sFlag.ZeroCross = 0;
	    gsACDC_Drive.sFlag.Ensoftzero = 0;
	    gsACDC_Drive.sFlag.VGridMeteringDone = 1;
	    gsACDC_Drive.sFlag.VInvMeteringDone = 1;
	    gsACDC_Drive.sFlag.VacFaultEn = 0;
	    gsACDC_Drive.sFlag.VInvReadyforMetering = 0;
	    gsACDC_Drive.sFlag.VGridReadyforMetering = 0;
	    gsACDC_Drive.sFlag.GridOK = 0;
	    gsACDC_Drive.sFlag.GridCheckDone = 0;
	    gsACDC_Drive.sFlag.ReadyforINVModeChange = 0;
	    gsACDC_Drive.sFlag.AcFirstCycleDetect = 1;
	   	    
	    /*================== reset metering parameters ===========================*/	    
	    gsACDC_Drive.sPowerMetering.a32VGridSqrSum = 0;
	    gsACDC_Drive.sPowerMetering.a32VInvSqrSum = 0;
	    gsACDC_Drive.sPowerMetering.a32IacSqrSum = 0;
	    gsACDC_Drive.sPowerMetering.a32VIinstSum = 0;
	    gsACDC_Drive.sPowerMetering.a32VASum = 0;
	    gsACDC_Drive.sPowerMetering.a32PSum = 0;
	    gsACDC_Drive.sPowerMetering.a32VGridFreqSum = 0;
	    gsACDC_Drive.sVacDetect.u16VGridPolConfirmCnt = 0;
	    gsACDC_Drive.sVacDetect.u16VInvPolConfirmCnt = 0;
	    gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt = 0;
	    
	    /*================== vol&cur filter parameters ===========================*/
	    /* initialize AC voltage IIR1 filter */
	    gsACDC_Drive.sVacDetect.sVGridFilter.sFltCoeff.f32A1 = ACDC_VAC_IIR_A1;
	    gsACDC_Drive.sVacDetect.sVGridFilter.sFltCoeff.f32B0 = ACDC_VAC_IIR_B0;
	    gsACDC_Drive.sVacDetect.sVGridFilter.sFltCoeff.f32B1 = ACDC_VAC_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsACDC_Drive.sVacDetect.sVGridFilter);
	    gsACDC_Drive.sVacCtrl.sVInvFilter.sFltCoeff.f32A1 = ACDC_VAC_IIR_A1;
	    gsACDC_Drive.sVacCtrl.sVInvFilter.sFltCoeff.f32B0 = ACDC_VAC_IIR_B0;
	    gsACDC_Drive.sVacCtrl.sVInvFilter.sFltCoeff.f32B1 = ACDC_VAC_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsACDC_Drive.sVacCtrl.sVInvFilter);
	    /* initialize inductor current IIR1 filter */
	    gsACDC_Drive.sIlCtrl.sIlFilter.sFltCoeff.f32A1 = ACDC_IL_IIR_A1;
	    gsACDC_Drive.sIlCtrl.sIlFilter.sFltCoeff.f32B0 = ACDC_IL_IIR_B0;
	    gsACDC_Drive.sIlCtrl.sIlFilter.sFltCoeff.f32B1 = ACDC_IL_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsACDC_Drive.sIlCtrl.sIlFilter);
	    /* initialize grid current IIR1 filter */
	    gsACDC_Drive.sIGridCtrl.sIGridFilter.sFltCoeff.f32A1 = INV_IGRID_IIR_A1;
	    gsACDC_Drive.sIGridCtrl.sIGridFilter.sFltCoeff.f32B0 = INV_IGRID_IIR_B0;
	    gsACDC_Drive.sIGridCtrl.sIGridFilter.sFltCoeff.f32B1 = INV_IGRID_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsACDC_Drive.sIGridCtrl.sIGridFilter);
	    /* initialize DC bus voltage IIR1 filter */
	    gsACDC_Drive.sVdcCtrl.sVdcBusFilter.sFltCoeff.f32A1 = ACDC_U_DCB_IIR_A1;
	    gsACDC_Drive.sVdcCtrl.sVdcBusFilter.sFltCoeff.f32B0 = ACDC_U_DCB_IIR_B0;
	    gsACDC_Drive.sVdcCtrl.sVdcBusFilter.sFltCoeff.f32B1 = ACDC_U_DCB_IIR_B1;
	    GDFLIB_FilterIIR1Init_F16(&gsACDC_Drive.sVdcCtrl.sVdcBusFilter);	 	    	    
	    
	    /*================== mode related parameter initialization =======================*/
	    if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)                   /* DC to AC mode */
	    {	    	
	    	gsACDC_Drive.sVacDetect.u16FirstCycle = 1;
	    	
	    	/*============================= OFFGRID MODE ==========================*/
	    	/*====== INV output voltage parameters =====*/
	    	gsACDC_Drive.sVacCtrl.f16VInv = 0;
	    	gsACDC_Drive.sVacCtrl.f16VacRefAmpReq = FRAC16(1.414*INV_AC_REF_RMS/VAC_SCALE); 
		    gsACDC_Drive.sVacCtrl.a32VInvRefFreq = ACC32(INV_AC_FREQ);	    
	    	
	        gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams.fs = INV_SAMPLING_FREQ;
	        gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams.kp = ACC32(INV_VOL_1H_P_GAIN);
	        gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams.ki = ACC32(INV_VOL_1H_I_GAIN);
	        gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(6.283185));
	        gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams.wc = ACC32(INV_VOL_1H_WC);		    
	        ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams,&gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams);		    		     
	        PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams);
	        				
	        gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams.fs = INV_SAMPLING_FREQ;
	        gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams.kp = 0;
	        gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams.ki = ACC32(INV_VOL_3H_I_GAIN);
	        gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(18.849556));
	        gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams.wc = ACC32(INV_VOL_3H_WC);		    
	        ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams,&gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams);		    		     
	        PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams);
	        				    
	        gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams.fs = INV_SAMPLING_FREQ;
	        gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams.kp = 0;
	        gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams.ki = ACC32(INV_VOL_5H_I_GAIN);
	        gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(31.415927));
	        gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams.wc = ACC32(INV_VOL_5H_WC);		    
	        ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams,&gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams);		    		     
	        PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams);
	        						    	    
	        gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams.fs = INV_SAMPLING_FREQ;
	        gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams.kp = 0;
	        gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams.ki = ACC32(INV_VOL_7H_I_GAIN);
	        gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(43.982297));
	        gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams.wc = ACC32(INV_VOL_7H_WC);		    
	        ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams,&gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams);		    		     
	        PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams);							    
	        
	    	gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_VOL_1H_PR_LOWER_LIMIT);
	    	gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_VOL_1H_PR_UPPER_LIMIT);
	    		    		    	    
	    	gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_VOL_1H_PR_LOWER_LIMIT);
	    	gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_VOL_1H_PR_UPPER_LIMIT);
	    		    		    	   
	    	gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_VOL_1H_PR_LOWER_LIMIT);
	    	gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_VOL_1H_PR_UPPER_LIMIT);
	    		    		    	    
	    	gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_VOL_1H_PR_LOWER_LIMIT);
	    	gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_VOL_1H_PR_UPPER_LIMIT);		    			    
	    			    		    
		    /* sine reference generation for AC voltage reference */
		    gsACDC_Drive.sVacCtrl.sSinGen.f32Angle = 0;
		    gsACDC_Drive.sVacCtrl.sSinGen.f32AnglePrev = 0;
		    gsACDC_Drive.sVacCtrl.sSinGen.f32AngleStep = INV_SIN_STEP;
		    gsACDC_Drive.sVacCtrl.sSinGen.uw16SinCnt = 0;
		    
		    /*===== INV inductor current parameters =====*/
		    gsACDC_Drive.sIlCtrl.f16Il = 0;  		    
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.a32PGain = INV_CUR_P_GAIN;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.a32IGain = INV_CUR_I_GAIN;		    
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16UpperLim = INV_CUR_PI_UPPER_LIMIT;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16LowerLim = INV_CUR_PI_LOWER_LIMIT;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16InErrK_1 = 0;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
		    gsACDC_Drive.sIlCtrl.bStopIntegFlag = 0;
		    
		    /*============================= GRIDCONNECTED MODE ==========================*/
		    /*===== Grid current parameters =====*/
		    gsACDC_Drive.sIGridCtrl.f16IGridRefPeak = FRAC16(GRIDCONNECTED_CURRENT_PEAK/ISNS_SCALE);
		    
		    /* grid current PR parameter calculation */ 
		    gsACDC_Drive.sIGridCtrl.sIGridFundamentalPRParams.fs = INV_SAMPLING_FREQ;
		    gsACDC_Drive.sIGridCtrl.sIGridFundamentalPRParams.kp = ACC32(INV_GRIDCUR_1H_P_GAIN);
		    gsACDC_Drive.sIGridCtrl.sIGridFundamentalPRParams.ki = ACC32(INV_GRIDCUR_1H_I_GAIN);
		    gsACDC_Drive.sIGridCtrl.sIGridFundamentalPRParams.wc = ACC32(INV_GRIDCUR_1H_WC);		    
		    PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams);			
		    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonicPRParams.fs = INV_SAMPLING_FREQ;
		    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonicPRParams.kp = 0;
		    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonicPRParams.ki = ACC32(INV_GRIDCUR_3H_I_GAIN);
		    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonicPRParams.wc = ACC32(INV_GRIDCUR_3H_WC);		    
		    PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams);					    
		    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonicPRParams.fs = INV_SAMPLING_FREQ;
		    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonicPRParams.kp = 0;
		    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonicPRParams.ki = ACC32(INV_GRIDCUR_5H_I_GAIN);
		    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonicPRParams.wc = ACC32(INV_GRIDCUR_5H_WC);		    
		    PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams);				    
		    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonicPRParams.fs = INV_SAMPLING_FREQ;
		    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonicPRParams.kp = 0;
		    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonicPRParams.ki = ACC32(INV_GRIDCUR_7H_I_GAIN);
		    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonicPRParams.wc = ACC32(INV_GRIDCUR_7H_WC);		    
		    PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams);			
		    
		    gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_LOWER_LIMIT);
		    gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_UPPER_LIMIT);
		    	    		    		    	    
		    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_LOWER_LIMIT);
		    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_UPPER_LIMIT);
		    	    		    		    	   
		    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_LOWER_LIMIT);
		    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_UPPER_LIMIT);
		    	    		    		    	    
		    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams.sLim.i32LowerLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_LOWER_LIMIT);
		    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams.sLim.i32UpperLim = FRAC_DYN(IFD,INV_GRIDCUR_PR_UPPER_LIMIT);
		    		    
		    gsACDC_Drive.sIGridCtrl.sPIpAWParams.a32PGain = INV_GRIDCUR_P_GAIN;
		    gsACDC_Drive.sIGridCtrl.sPIpAWParams.a32IGain = INV_GRIDCUR_I_GAIN;
		    gsACDC_Drive.sIGridCtrl.sPIpAWParams.f16UpperLim = FRAC16(0.2);
		    gsACDC_Drive.sIGridCtrl.sPIpAWParams.f16LowerLim = FRAC16(-0.2);
		    gsACDC_Drive.sIGridCtrl.sPIpAWParams.f16InErrK_1 = 0;
		    gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1 = 0;
		    gsACDC_Drive.sIGridCtrl.bStopIntegFlag = 0;
		    
		    /* capacitor current feedback coefficient */
		    gsACDC_Drive.sCurDetect.f16ICapFeedbackcoef = FRAC16(INV_HIC*ICAP_SCALE);
		    
		    SPLL_1PH_SOGI_Reset(&spll_obj);
		    SPLL_1PH_SOGI_LFConfig(&spll_obj,SPLL_SOGI_LF_B0,SPLL_SOGI_LF_B1);
		    
		    /*============================= transition mode control parameters =============================*/ 
		    gsACDC_Drive.sVacCtrl.sSyncPIpAWParams.a32PGain = INV_SYNC_P_GAIN;
		    gsACDC_Drive.sVacCtrl.sSyncPIpAWParams.a32IGain = INV_SYNC_I_GAIN;
		    gsACDC_Drive.sVacCtrl.sSyncPIpAWParams.f16LowerLim = INV_SYNC_PI_LOWER_LIMIT;
		    gsACDC_Drive.sVacCtrl.sSyncPIpAWParams.f16UpperLim = INV_SYNC_PI_UPPER_LIMIT;
		    gsACDC_Drive.sVacCtrl.sSyncPIpAWParams.f16InErrK_1 = 0;
		    gsACDC_Drive.sVacCtrl.sSyncPIpAWParams.f32IAccK_1 = 0;
		    gsACDC_Drive.sVacCtrl.bSyncPIStopIntegFlag = 0;
	    }
	    else if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC) 						/* AC to DC mode */
	    {
	    	PIT0_RUN();		/* begin to run the volatege loop for PFC */
		    gsPFC_Runsub = SOFTSTART; /* converter always start from soft-start mode */		    			    
		    /*=========== VDC control parameters =======*/
		    /* pre-charge parameters */
			bPFC_Precharge = 0;
			gsACDC_Drive.sFlag.PreChargeDone = 0;
			gsACDC_Drive.sVdcCtrl.bVacRise = 0;
			gsACDC_Drive.sVdcCtrl.uw16VacFallTrendCnt = 0;
			gsACDC_Drive.sVdcCtrl.uw16VacRiseTrendCnt = 0;
		    /* soft start vout ramp parameters */
		    gsACDC_Drive.sVdcCtrl.sVdcRamp.sRamp.f16RampUp = FRAC16(RAMPUP_VOL_VAL/VDC_SCALE);
		    gsACDC_Drive.sVdcCtrl.sVdcRamp.f16Target = FRAC16(PFC_VDC_REF/VDC_SCALE);
		    gu16SoftStartStepCnt = 0;			
		    /* VDC control parameters */
		    gsACDC_Drive.sVdcCtrl.sPIpAWParams.a32PGain = PFC_VOL_P_GAIN;
		    gsACDC_Drive.sVdcCtrl.sPIpAWParams.a32IGain = PFC_VOL_I_GAIN;
		    gsACDC_Drive.sVdcCtrl.StopIntegFlag = 0;
		    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 = 0;
		    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16InErrK_1 = 0;
		    gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut = 0;
		    
		    u16PWMOffDurCnt = 0;
		    gsACDC_Drive.sVdcCtrl.f16VBurston = FRAC16(PFC_VDC_BURSTON/VDC_SCALE);
		    gsACDC_Drive.sVdcCtrl.f16VBurstoff = FRAC16(PFC_VDC_BURSTOFF/VDC_SCALE); 
		
		    /*======== current control parameters ========*/	
		    gsACDC_Drive.sIlCtrl.a32IrefReq = 0;		    
		    gsACDC_Drive.sIlCtrl.f16RefHlim = PFC_IREF_HLIMIT;
		
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.a32PGain = PFC_CUR_P_GAIN_LVRMS;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.a32IGain = PFC_CUR_I_GAIN_LVRMS;
		    gsACDC_Drive.sIlCtrl.bStopIntegFlag = 0;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16InErrK_1 = 0;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16LowerLim = PFC_CUR_PI_LOW_LIMIT;
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16UpperLim = PFC_CUR_PI_UP_LIMIT;
		    gsACDC_Drive.sCurDetect.f16ICapFeedbackcoef = FRAC16(PFC_HIC);	    	        

	        SPLL_1PH_SOGI_Reset(&spll_obj);
	        SPLL_1PH_SOGI_LFConfig(&spll_obj,SPLL_SOGI_LF_B0,SPLL_SOGI_LF_B1);
		    /* ====*********** duty params **********==== */
		    gsACDC_Drive.f16Duty = 0; 
		    gsACDC_Drive.f16DutyComp = 0;
		    
		    /* ====******** vac drop/recovery detection******==== */
		    gsACDC_Drive.f16VacLackTh = VAC_FAIL_THRESHOLD;
		    gsACDC_Drive.f16VacRecoveryTh = VAC_RECOVERY_THRESHOLD;
		    
		    PWMA->FAULT[0].FCTRL &= ~PWM_FCTRL_FAUTO(3); //disable cycle by cycle protection
	    }
	  
	    gsACDC_Drive.sFlag.VarInitReady = 1;	    
	}
	
	/*======================== offset detection ===========================*/
	if(gsACDC_Drive.sFlag.SampOffsetReady == 1)
	{		
		gsACDC_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
	}
}

/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_StateStop(void)
{
#if BOARD_TEST
	bACDC_Run = 1;
	gsACDC_Drive.sVdcCtrl.f16VdcBusLowLimit = FRAC16(60.0/VDC_SCALE); /* dc bus limit that ensure the stable operation of the auxiliary power*/
	if((bACDC_Run ==1) && (gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > gsACDC_Drive.sVdcCtrl.f16VdcBusLowLimit)) 
	{
		if(gsACDC_Drive.sVacCtrl.sSinGen.uw16SinCnt <= 3)
		{
		    gsACDC_Ctrl.uiCtrl |= SM_CTRL_START;
		}
	}
#else	
	if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)                   /* DC to AC mode */
	{
		gsACDC_Drive.u16CurrentInvMode = gsACDC_Drive.u16InvModeCmd;
		gsACDC_Drive.sVdcCtrl.f16VdcBusLowLimit = MLIB_Mul_F16as(ACC32(1.626),MLIB_Mul_F16as(VAC_TO_VDC,gsACDC_Drive.sVacDetect.f16VacrmsNominal)); /* dc bus limit(1.15*vmax) that prevent current backfeed */
		/* inverter starts at a reference position close to 0 */
		if(gsACDC_Drive.u16CurrentInvMode==GRIDCONNECTED_INV && gsACDC_Drive.sFlag.GridCheckDone==1)
		{
			if((bACDC_Run ==1) && (gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > gsACDC_Drive.sVdcCtrl.f16VdcBusLowLimit))
		    {	
			    if(gu16VGridCycleCnt <= 3 && gsACDC_Drive.sFlag.VGridPol==1)
			    {
				    gsACDC_Ctrl.uiCtrl |= SM_CTRL_START;
			    }
		    }
		}
		else if(gsACDC_Drive.u16CurrentInvMode==OFFGRID_INV)
		{
			if((bACDC_Run ==1) && (gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > gsACDC_Drive.sVdcCtrl.f16VdcBusLowLimit))
			{
		        if(gsACDC_Drive.sVacCtrl.sSinGen.uw16SinCnt <= 3)
				{
					gsACDC_Ctrl.uiCtrl |= SM_CTRL_START;
				}
			}
		}
	}
	else if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)				/* AC to DC */
	{	    	    		
		Check_AC_drop(&gsACDC_Drive); /* avoid wrong power metering and control out when ac drop */
		
		if(gsACDC_Drive.sFlag.AcDrop)
		{
			OPEN_SW_ALL();
			gsACDC_Drive.sFlag.PreChargeDone = 0; //pre-charge again, avoid large current from damaging the board after the output voltage drops.
			gsACDC_Drive.sFlag.GridOK = 0;
			bPFC_Precharge = 0;
			gsACDC_Drive.sVdcCtrl.uw16VacRiseTrendCnt = 0;
			gsACDC_Drive.sVdcCtrl.uw16VacFallTrendCnt = 0;			
		}
		
		if(bACDC_Run &&(gsACDC_Drive.sFlag.GridCheckDone)&&(gsACDC_Drive.sFlag.GridOK) &&(!gsACDC_Drive.sFlag.AcDrop))  
		{
		/*========================= pre-charge logic ==============================*/
	        if(!gsACDC_Drive.sFlag.PreChargeDone)
	        {
		    if(!bPFC_Precharge)
	        {
	    	    gsACDC_Drive.sVdcCtrl.f16VdcUnctrlRec = MLIB_MulSat_F16as(VAC_TO_VDC,gsACDC_Drive.sVacDetect.f16VGridPeak);
	    	    if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt< MLIB_Sub_F16(gsACDC_Drive.sVdcCtrl.f16VdcUnctrlRec,FRAC16(VAC_PRECHARGE_STEP/VAC_SCALE)))
	    	    {
	    	        gsACDC_Drive.sVdcCtrl.bVacRise = gsACDC_Drive.sFlag.VGridPol?1:0; //avoid triac closing in wrong position during the first cycle
	    		    gsACDC_Drive.sVdcCtrl.f16VacThreshold = MLIB_AddSat_F16(MLIB_Mul_F16(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt,VDC_TO_VAC),FRAC16(VAC_PRECHARGE_STEP/VAC_SCALE));
	    		    gsACDC_Drive.sFlag.PreChargeDone =0;
	    	    }
	    	    else 
	    	    {
	    	    	CLOSE_SW_ALL();
	    		    gsACDC_Drive.sFlag.PreChargeDone =1;
	    		    gsACDC_Drive.sFlag.ZeroCross = 0;
	    	    }
	    	    bPFC_Precharge = 1;
	        }
	    
	        if((!gsACDC_Drive.sFlag.PreChargeDone)&& bPFC_Precharge)
	        {
	    	    /*======= detect the position of the AC voltage ======*/
	    	    if((gsACDC_Drive.sVacDetect.f16VGridFilt>gsACDC_Drive.sVacDetect.f16VGridFiltlast1)&&(gsACDC_Drive.sVdcCtrl.bVacRise==0))
	    	    {
	    		    gsACDC_Drive.sVdcCtrl.uw16VacRiseTrendCnt++;
	    		    if(gsACDC_Drive.sVdcCtrl.uw16VacRiseTrendCnt>=VAC_TREND_CONFIRM_CNT)
	    		    {
	    		        gsACDC_Drive.sVdcCtrl.bVacRise = 1;
	    		        gsACDC_Drive.sVdcCtrl.uw16VacRiseTrendCnt=0;
	    		    }
	    	    }
	    	    else    gsACDC_Drive.sVdcCtrl.uw16VacRiseTrendCnt=0;
	    		    
	    	    if((gsACDC_Drive.sVacDetect.f16VGridFilt<gsACDC_Drive.sVacDetect.f16VGridFiltlast1)&&(gsACDC_Drive.sVdcCtrl.bVacRise==1))
	    	    {
	    		    gsACDC_Drive.sVdcCtrl.uw16VacFallTrendCnt++;
	    		    if(gsACDC_Drive.sVdcCtrl.uw16VacFallTrendCnt>=VAC_TREND_CONFIRM_CNT)
	    		    {
	    		        gsACDC_Drive.sVdcCtrl.bVacRise = 0;
	    		        gsACDC_Drive.sVdcCtrl.uw16VacFallTrendCnt=0;
	    		    }
	    	    }
	    	    else    gsACDC_Drive.sVdcCtrl.uw16VacFallTrendCnt=0;	    		    
	    	    gsACDC_Drive.sVacDetect.f16VGridFiltlast1 = gsACDC_Drive.sVacDetect.f16VGridFilt;
	    	
	    	    /*====== control the on/off of triac ======*/
	    	    if((gsACDC_Drive.sFlag.VGridPol)&&(!gsACDC_Drive.sVdcCtrl.bVacRise))
	            {
	    	    	if(gu16TriacOnCnt>0)
	    	    	{ 
	    	    		gu16TriacOnCnt++;
	    	    		if(gu16TriacOnCnt>TRIAC_ON_TIME_CNT)
	    	    		{
	    	    		   OPEN_SW_ALL();
	    	    		}
	    	    	}
	    	    	else if((gsACDC_Drive.sVacDetect.f16VGridFiltAbs<gsACDC_Drive.sVdcCtrl.f16VacThreshold)&&(gsACDC_Drive.sVacDetect.f16VGridFiltAbs>FRAC16(VAC_TRIAC_THRESHOLD/VAC_SCALE))) 
	    		    {
	        	    	 CLOSE_SW_ALL();
	        	    	 gu16TriacOnCnt++;
	        	    } 
	            }
	    	
	    	    if(gsACDC_Drive.sFlag.ZeroCross)
	    	    {
	    	    	if((gsACDC_Drive.sFlag.VGridPol)&&((gsACDC_Drive.sVdcCtrl.f16VacThreshold>=MLIB_Add_F16(gsACDC_Drive.sVacDetect.f16VGridPeak,FRAC16(10.0/VAC_SCALE)))||(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt> MLIB_Sub_F16(MLIB_MulSat_F16as(VAC_TO_VDC,gsACDC_Drive.sVacDetect.f16VGridPeak),FRAC16(2.0/VDC_SCALE)))))
	    	    	{
	    	    		CLOSE_SW_ALL();
	    	    		gsACDC_Drive.sFlag.PreChargeDone=1;
	    	    		gsACDC_Ctrl.uiCtrl |= SM_CTRL_START;
	    	    	}
	    	    	else if(gsACDC_Drive.sVdcCtrl.f16VacThreshold<MLIB_AddSat_F16(MLIB_Mul_F16(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt,VDC_TO_VAC),FRAC16(45.0/VAC_SCALE)))
	    	    	{
	    	    		gsACDC_Drive.sVdcCtrl.f16VacThreshold = MLIB_AddSat_F16(gsACDC_Drive.sVdcCtrl.f16VacThreshold,FRAC16(VAC_PRECHARGE_STEP/VAC_SCALE));
	    	    	}
	    	        gsACDC_Drive.sFlag.ZeroCross=0;
	    	    }	    	    	        	    	    	            
	        } 
	        }	    
	        /* PFC start-up at zero-crossing point after pre-charge is done */
	        else if(gsACDC_Drive.sFlag.ZeroCross)
	        {
	    	    gsACDC_Ctrl.uiCtrl |= SM_CTRL_START;	    	
	        }
	    }
	    else gsACDC_Drive.sFlag.ZeroCross = 0;
	}
#endif	
}
#pragma section CODES_IN_RAM begin
/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_StateRun(void)
{	
	if(bACDC_Run == 0)
	{
		gsACDC_Ctrl.uiCtrl |= SM_CTRL_STOP;
	}	
		
	if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)  /* AC to DC */
	{			
	    if(gsPFC_Runsub  == NORMAL)
	    {
		/*================= normal to light-load mode switch control ========================*/
		    /* Enter light-load mode (burst off) when output voltage is larger than low overshoot command and voltage controller
		      outputs lower limit or enter burst mode when DC bus voltage is larger than high overshoot command */
		    if(((gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut == gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim) && \
		      (gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > FRAC16(VDC_NORMAL_OVERSHOOT_LOW/VDC_SCALE))) \
			  ||(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > FRAC16(VDC_NORMAL_OVERSHOOT_HIGH/VDC_SCALE)))
		    {
			    gsACDC_Drive.bCloseLoopOnOff = 0;     		// stop the loop calculation when in burst off mode
			    ACDC_FASTBRIDGE_PWM_MASK();

			    u16PWMOffDurCnt = 0;
			    gsPFC_Runsub  = LIGHTLOAD;
			    gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0; /* avoid current spike when tube is turned on in light-load mode */
		    }
		    
		    if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > FRAC16(VDC_FEEDFORWARD_COMP_UP/VDC_SCALE))		/* DC voltage feed-forward, only enabled when the DC voltage varies greatly */
		    {
			    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 -= (((frac32_t)1)<<16); /* approximate output voltage feed forward to restrain the bus voltage overshoot */
		    }
		    else if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt < FRAC16(VDC_FEEDFORWARD_COMP_LOW/VDC_SCALE))		/* DC voltage feed-forward, only enabled when the DC voltage varies greatly */
		    {
		    	gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 += (((frac32_t)1)<<16); /* approximate output voltage feed forward to restrain the bus voltage overshoot */
		    }
	    }
	    else if(gsPFC_Runsub  == LIGHTLOAD)
	    {
		    /* if reach the peak-valley controller peak value, close all tubes */ 
		    if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt > gsACDC_Drive.sVdcCtrl.f16VBurstoff)
		    {
			    if(gsACDC_Drive.bCloseLoopOnOff)  u16PWMOffDurCnt = 0;
			    gsACDC_Drive.bCloseLoopOnOff = 0;
			    ACDC_FASTBRIDGE_PWM_MASK();
		    }
			
		    /* burst on from zero crossing point */
		    if(gsACDC_Drive.sFlag.ZeroCross)
		    {
			    if(u16PWMOffDurCnt < PFC_BURST_OFF_MIN_DURATION)  u16PWMOffDurCnt++;		  
			     gsACDC_Drive.sFlag.ZeroCross = 0;
			
			    /* if reach the peak-valley controller valley value, start converter again */
			    if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt < gsACDC_Drive.sVdcCtrl.f16VBurston)
			    {
				    /* burst off time is less than the minimum duration, voltage drop too fast, directly return to normal mode */
				    if(u16PWMOffDurCnt < PFC_BURST_OFF_MIN_DURATION)
				    { 
					    /* fixed current reference is used in light-load mode, reset voltage PI integrator before back to normal mode to avoid current surge */
					    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 = MLIB_Conv_F32s(gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim);
					    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16InErrK_1 = 0;
					
					    gsPFC_Runsub  = NORMAL;
					    u16PWMOffDurCnt = 0;
				    }
				    gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
				    gsACDC_Drive.bCloseLoopOnOff = 1;
			    }
		    }		
		    /* output voltage drop too low, directly return to normal mode */
		    if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt < FRAC16(VDC_BURST_UNDER/VDC_SCALE))
		    {
			    u16PWMOffDurCnt = 0;
			    /* fixed current reference is used in light-load mode, reset voltage PI integrator before back to normal mode to avoid current surge */
			    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 = MLIB_Conv_F32s(gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim);
			    gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16InErrK_1 = 0;
			    gsPFC_Runsub = NORMAL;
			    if(!gsACDC_Drive.bCloseLoopOnOff)
			    {
				    gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
				    gsACDC_Drive.bCloseLoopOnOff = 1;
			    }
		    }
	    }
#if !BOARD_TEST
		Check_AC_drop(&gsACDC_Drive);
		
		/* Vac drop too long, switch to DC-TO-AC mode,
		 * test condition do not allow mode change mechanism, turn off the converter for now */ 
		if(ACDC_TimeDelay(guw32StartCnt, VAC_DROP_MAX_DURATION)) 
		{
			gsACDC_Ctrl.uiCtrl |= SM_CTRL_STOP;
			//gsACDC_Drive.u16WorkModeCmd = DC_TO_AC; 
			bACDC_Run = 0;
		}
#endif	
	}
	else
	{
		if((gsACDC_Drive.u16CurrentInvMode==GRIDCONNECTED_INV)&&(gsACDC_Drive.u16InvModeCmd==OFFGRID_INV))
		{
			//OPEN_SW_GRID();//immediately open the grid switch upon detecting an off grid command
		    OPEN_SW_ALL(); //because of hardware risk, all thyristors are closed, so no load can be added in this condition
		}
	}
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   FAULT to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransFaultInit(void)
{
	gsACDC_Drive.sFlag.VarInitReady = 0;
}

/***************************************************************************//*!
*
* @brief   INIT to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransInitFault(void)
{
	 guw32StartCnt = gu32TimerCnt;
}

/***************************************************************************//*!
*
* @brief   INIT to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransInitStop(void)
{
	gsACDC_Drive.sFlag.ZeroCross = 0;
}

/***************************************************************************//*!
*
* @brief   STOP to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransStopFault(void)
{
	guw32StartCnt = gu32TimerCnt;
}

/***************************************************************************//*!
*
* @brief   STOP to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransStopInit(void)
{
	OPEN_SW_ALL();
	PIT0_STOP();
}

#pragma section CODES_IN_RAM begin
/***************************************************************************//*!
*
* @brief   STOP to RUN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransStopRun(void)
{	
	if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)	   				/* DC to AC */ 
	{
		if(gsACDC_Drive.u16CurrentInvMode==OFFGRID_INV)
		{
			/* because of hardware risk, don't uncomment this commad when grid voltage is conncected for off-grid inverter test, no laod can be added through load port. 
	         * If you want to test off-grid inverter with load on load port, uncomment this command and make sure no grid voltage is connected. */
		    //CLOSE_SW_LOAD();  
		    gu16VInvCycleCnt = 0;
		    gsACDC_Drive.sVacCtrl.sSinGen.f32Angle = 0;
		    gsACDC_Drive.sFlag.VInvPol = 1;
		    gsACDC_Drive.sFlag.VInvReadyforMetering = 1;
		    gsACDC_Drive.sVacDetect.u16FirstCycle = 1;
		    gsACDC_Drive.sPowerMetering.a32VInvSqrSum = 0;
		    gsACDC_Drive.sPowerMetering.a32IacSqrSum = 0;
		    gsACDC_Drive.sPowerMetering.a32VIinstSum = 0;
		    gsACDC_Drive.sFlag.VInvZeroCrossforMeter = 0;
		    PWMA->FAULT[0].FCTRL |= PWM_FCTRL_FAUTO(3); //enable cycle by cycle protection
		    
		}
		else if(gsACDC_Drive.u16CurrentInvMode==GRIDCONNECTED_INV)
		{
			gsACDC_Drive.sFlag.VacFaultEn = 1; // enable vac protection only in RUN state
			CLOSE_SW_ALL();
			PWMA->FAULT[0].FCTRL &= ~PWM_FCTRL_FAUTO(3); //disable cycle by cycle protection
		}
#if BOARD_TEST
		gsACDC_Drive.sFaultThresholds.f16VdcUnder = 0;
#else
		gsACDC_Drive.sFaultThresholds.f16VdcUnder = gsACDC_Drive.sVdcCtrl.f16VdcBusLowLimit;
#endif
		
		if((!ACDC_SLOWBRIDGE_HIGHMOS_STATE()))
		{
			ACDC_SLOWBRIDGE_LOWMOS_ON();
		}
	}
	else     /* AC to DC */                                                                                                                         
	{
		gsACDC_Drive.sFlag.VacFaultEn = 1;
		guw32StartCnt = gu32TimerCnt; /* reset the timing start value to prevent wrong judgement of AC drop time */
		ACDC_FASTBRIDGE_PWM_MASK(); /* avoid current spike during startup */ 
		gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 = gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim;
		gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut = gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim;
		if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt < MLIB_Add_F16(gsACDC_Drive.sVdcCtrl.f16VdcUnctrlRec, FRAC16(8.0/VDC_SCALE)))
		{
			gsACDC_Drive.sVdcCtrl.f16VdcRef = MLIB_Add_F16(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt,FRAC16(8.0/VDC_SCALE));
		}
		else
		{
			gsACDC_Drive.sVdcCtrl.f16VdcRef = gsACDC_Drive.sVdcCtrl.f16VdcBusFilt;
		}  /* initialize the soft-start parameters when working at PFC mode */
		GFLIB_RampInit_F16(gsACDC_Drive.sVdcCtrl.f16VdcRef, &gsACDC_Drive.sVdcCtrl.sVdcRamp.sRamp);
	}
	
	gsACDC_Drive.sIlCtrl.uw16CurLimNumCnt = 0;		/* restart current protection */
	gsACDC_Drive.sIlCtrl.uw16CurLimTimerCnt = 0;
	gsACDC_Drive.sPowerMetering.a32PSum = 0;
	gsACDC_Drive.sPowerMetering.a32VASum = 0;
	gsACDC_Drive.sPowerMetering.u16PowerUpdateCnt = 0;
	gsACDC_Drive.bCloseLoopOnOff = 1;
	ACDC_FASTBRIDGE_PWM_EN(); 
	gsACDC_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

/***************************************************************************//*!
*
* @brief   RUN to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransRunFault(void)
{
	gsACDC_Drive.bCloseLoopOnOff = 0;
	guw32StartCnt = gu32TimerCnt;
	
	if(gsACDC_Drive.u16CurrentInvMode==OFFGRID_INV)  gsACDC_Drive.sFlag.VInvReadyforMetering = 0;
	
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(1);  /* ensure restart from a small duty cycle to avoid current spike */
	PWMA->SM[0].VAL2 = 0;
	PWMA->SM[0].VAL3 = 0;
	PWMA->MCTRL |= PWM_MCTRL_LDOK(1);
	RESETOUTPUTPOLARITY;
}

/***************************************************************************//*!
*
* @brief   RUN to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void ACDC_TransRunStop(void)
{
	
	ACDC_FASTBRIDGE_PWM_DIS();
	ACDC_SLOWBRIDGE_PWM_DIS();
	
	gsACDC_Drive.bCloseLoopOnOff = 0;
	gsACDC_Drive.f16Duty = 0;
	
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(1);  /* ensure restart from a small duty cycle to avoid current surge */
	PWMA->SM[0].VAL2 = 0;
	PWMA->SM[0].VAL3 = 0;
	PWMA->MCTRL |= PWM_MCTRL_LDOK(1);
	RESETOUTPUTPOLARITY;

	/* reset controller to avoid current surge when works again */
	if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)	   				/* DC to AC */ 
	{
		OPEN_SW_ALL();
		if(gsACDC_Drive.u16CurrentInvMode == OFFGRID_INV)
		{
		    gsACDC_Drive.sFlag.VInvReadyforMetering = 0;
		    PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams);
		    PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams);
		    PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams);
		    PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams);
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f16InErrK_1 = 0; 
		    gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
		}
		else
		{
			PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams);
			PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams);
			PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams);
			PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams);
			gsACDC_Drive.sIGridCtrl.sPIpAWParams.f16InErrK_1 = 0;
			gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1 = 0;
		}
	}
	else
	{
		gsPFC_Runsub = SOFTSTART; /* always start from soft start mode */
		gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16InErrK_1 = 0; 
		gsACDC_Drive.sVdcCtrl.sPIpAWParams.f32IAccK_1 = (frac32_t)gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim<<16;
		gsACDC_Drive.sIlCtrl.sPIpAWParams.f16InErrK_1 = 0; 
		gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
	}
	
	gsACDC_Drive.sFaultThresholds.f16VdcUnder = 0; /* any voltage is acceptable when converter is stopped, avoid entering fault state by mistake */
	gsACDC_Drive.sFlag.VacFaultEn=0; 
	
	gsACDC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}
#pragma section CODES_IN_RAM end

