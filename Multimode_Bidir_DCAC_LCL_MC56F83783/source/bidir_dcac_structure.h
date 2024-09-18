/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef BIDIR_DCAC_STRUCTURE_H_
#define BIDIR_DCAC_STRUCTURE_H_

#include "mlib.h"
#include "gflib.h"
#include "gdflib.h"
#include "PR_controller.h"
/******************************************************************************
* Types
******************************************************************************/
typedef struct 
{
	frac32_t f32Angle;        /* calculated angle */
	frac32_t f32AnglePrev;    /* angle calculated in previous cycle */
	frac32_t f32AngleStep;    /* change step for angle calculation */
	frac16_t f16Sin;          /* sine value of current angle */
	uint16_t uw16SinCnt;      /* cycle count of calculated sine */
}ACDCSTRUC_SIN_GEN_T;

typedef struct
{
	uint32_t             ZeroCross:1;               /* used to make sure the PFC is started at zero-crossing point */
	uint32_t     		 SampOffsetReady:1;    		/* flag that indicate whether the offset value is obtained */
	uint32_t     		 VacFaultEn:1;         		/* flag that control if enable part of Vac fault detection */
	uint32_t  			 Ensoftzero:1;        	   	/* when ensoftzero = 1, enable soft start drivers at detected zero crossing point */
	uint32_t  			 AcDrop:1;            	   	/* AC input drop flag */
	uint32_t             AcFirstCycleDetect:1;		/* flag that indicate the starting of zero-crossing detect in AC-DC mode*/
	uint32_t             VGridPol:1;				/* polarity of the grid voltage, 1 stands for positive cycle, 0 stands for negative cycle */
	uint32_t             VGridPolTemp:1;			/* temporary polarity of the grid voltage, 1 stands for positive cycle, 0 stands for negative cycle */
	uint32_t             VInvPol:1;                 /* polarity of the off-grid inverter output voltage, 1 stands for positive cycle, 0 stands for negative cycle */
	uint32_t             VInvPolTemp:1;             /* temporary polarity of the off-grid inverter output voltage, 1 stands for positive cycle, 0 stands for negative cycle */
	uint32_t             VGridReadyforMetering:1;   /* offset detection is completed, the sampling value can be used for power metering */
	uint32_t             VInvReadyforMetering:1;    /* inverter start work in off-grid mode*/
	uint32_t     		 VGridMeteringDone:1;       /* flag that indicate if power factor calculation done */
	uint32_t     		 VInvMeteringDone:1;        /* flag that indicate if power factor calculation done */
	uint32_t             VGridZeroCrossforMeter;    /* grid voltage zero crossing flag for metering */
	uint32_t             VInvZeroCrossforMeter;     /* zero crossing flag for metering in off-grid mode */
	uint32_t             GridCheckDone:1;           /* flag that indicate whether the grid check is done*/
	uint32_t             GridOK:1;                  /* flag that indicate the grid is OK */
	uint32_t             ReadyforINVModeChange:1;   /* flag that indicate the INV mode can switch from GRIDCONNECTED to OFFGRID , 		                                               prevent the situation that grid sw is off but the mode doesn't switch */
	uint32_t             PreChargeDone:1;			/* flag that indicate if the pre-charging is done */
	uint32_t             VarInitReady:1;            /* variables init status flag */

	uint32_t             Reserved:11;
} ACDCSTRUC_FLAG_T;

typedef struct
{
	GFLIB_RAMP_T_F16       sRamp; 		/* output voltage ramp parameters, used on softstart sub-state*/
	frac16_t               f16InitVal; 	/* ramp initial value */
	frac16_t               f16Target; 	/* required value, ramp limit*/
}ACDCSTRUC_VDC_RAMP_T;

typedef struct
{	
	GDFLIB_FILTER_IIR1_T_F32   sVGridFilter;				/* filter for grid voltage */
	GDFLIB_FILTER_MA_T_A32     sVGridOffsetFilter;			/* grid voltage sampling offset filter */
	
	frac16_t       f16VacLoad;     	 	      /* detected load side output voltage */
	frac16_t       f16VGrid;      			  /* detected grid side output voltage */
	frac16_t       f16ACVoltBias;             /* detected DC bias of the AC voltage */
	
	frac16_t       f16VGridOffsetDeviation;   /* grid side voltage sample offset deviation */
	frac16_t       f16VGridFilt;			  /* filtered grid side voltage */
	frac16_t       f16VGridPeak;              /* grid peak voltage */
	frac16_t       f16VGridFiltAbs;			  /* Absolute value of filtered Vgrid. */
	frac16_t       f16VGridFiltlast1;
	frac16_t       f16VGridFiltlast2;
	frac16_t       f16VGriddiff1;
	frac16_t       f16VGriddiff2;
	
	frac16_t       f16VacrmsNominal;    /* nominal grid voltage */
	acc32_t        a32VacFreqNominal;   /* nominal grid frequency */ 
	
	uint16_t       u16VGridPolConfirmCnt; /* the polarity is accepted at least this value of consecutive times are consistent */
	uint16_t       u16VInvPolConfirmCnt;
	uint16_t       u16ZeroCrossingCnt;    /* recorded grid voltage zero crossing times */
	uint16_t       u16VGridWrongCnt;      /* record the number of consecutive vgrid error cycles*/
    uint16_t       u16FirstCycle;         /* flag to indicate the first half cycle that inverter starts to work*/ 
}INVSTRUC_VAC_DETECT_T;

typedef struct
{	
	frac16_t     f16IBus;     			/* detected current sharing bus current */
	frac16_t     f16ICap;     			/* detected filter capacitor current */
	
	GDFLIB_FILTER_MA_T_A32 sICapOffsetFilter;   /* capacitor current sampling offset filter */
	frac16_t     f16ICapOffsetDeviation;/* capacitor current offset deviation */
	
	frac16_t     f16ICapFeedbackcoef;   /* capacitor current feedback coefficient */
	frac16_t     f16ICapFeedback;       /* feedback capacitor current after multiplication by the coefficient */
}INVSTRUC_CUR_DETECT_T;

typedef struct
{
	ACDCSTRUC_PR_PARAMS_T      sVacFundamentalPRParams;   	/* AC voltage fundamental wave: continuous PR parameters */
	ACDCSTRUC_PR_PARAMS_T      sVac3rdHarmonicPRParams;   	/* AC voltage 3rd harmonic: continuous PR parameters */
	ACDCSTRUC_PR_PARAMS_T      sVac5thHarmonicPRParams;     /* AC voltage 5th harmonic: continuous PR parameters */
	ACDCSTRUC_PR_PARAMS_T      sVac7thHarmonicPRParams;     /* AC voltage 7th harmonic: continuous PR parameters */

	FILTER_2P2Z_TRANS_T        sVacFundamental2p2zParams; 	/* AC voltage fundamental wave: discrete PR parameters */
	FILTER_2P2Z_TRANS_T        sVac3rdHarmonic2p2zParams; 	/* AC voltage 3rd harmonic wave: discrete PR parameters */
	FILTER_2P2Z_TRANS_T        sVac5thHarmonic2p2zParams; 	/* AC voltage 5th harmonic wave: discrete PR parameters */
	FILTER_2P2Z_TRANS_T        sVac7thHarmonic2p2zParams; 	/* AC voltage 7th harmonic wave: discrete PR parameters */

	ACDCSTRUC_SIN_GEN_T        sSinGen;						/* generate sinusoidal reference for AC voltage in INV mode */
	GDFLIB_FILTER_IIR1_T_F32   sVInvFilter;					/* filter for inverter side voltage in INV mode */
	GDFLIB_FILTER_MA_T_A32     sVInvOffsetFilter;			/* inverter voltage sampling offset filter */
	
	frac16_t       f16VInvOffsetDeviation; 	    /* off-grid inverter voltage sample offset deviation */	
	frac16_t       f16VInvFilt;					/* filtered inverter side voltage */
	frac16_t       f16VInv;          			/* detected inverter side output voltage */
	
	frac16_t       f16VacRefAmpReq; 	/* required reference AC voltage amplitude */
	acc32_t        a32VInvRefFreq;      /* required reference AC voltage frequency */
	frac16_t       f16VacRef;       	/* AC voltage reference */

	frac16_t       f16VacErr;       	/* AC voltage error */
	
	frac16_t       f16VacCtrl1stOut;    /* AC voltage loop fundamental wave regulator output */
	frac16_t       f16VacCtrl3rdOut;    /* AC voltage loop 3rd wave regulator output */
	frac16_t       f16VacCtrl5thOut;    /* AC voltage loop 5th wave regulator output */
	frac16_t       f16VacCtrl7thOut;    /* AC voltage loop 7th wave regulator output */
	
	frac16_t       f16ViVgPhaseDif;     /* phase difference between inverter output and grid voltage */
	frac16_t       f16FreqAdjustment;    /* inverter output frequency adjustment during off-grid to grid-connected switching process */    
	GFLIB_CTRL_PI_P_AW_T_A32  sSyncPIpAWParams; 		/* invert output and grid voltage sync PI parameters */
	bool_t                    bSyncPIStopIntegFlag;     /* PI controller integration stop flag */ 
		
	uint16_t       uw16PeriodNumCnt;   	/* counter for the ac voltage period number, enable AC under voltage after specific number of period */		
}ACDCSTRUC_VAC_CTRL_T;

typedef struct
{
	frac16_t       f16VdcBus;      /* DC bus voltage */
	frac16_t       f16VdcBusFilt;  /* filtered DC bus voltage */
	GDFLIB_FILTER_IIR1_T_F32   sVdcBusFilter;	/* DC bus voltage filter */
	ACDCSTRUC_VDC_RAMP_T       sVdcRamp; 		/* DC bus voltage ramp parameters */
	frac16_t       f16VdcRef; 		    /* DC bus voltage reference */
	frac16_t       f16VdcErr;       	/* DC voltage error */
	GFLIB_CTRL_PI_P_AW_T_A32  	sPIpAWParams;  	/* DC bus voltage PI controller parameters */
	bool_t                 		StopIntegFlag; 	/* PI controller integration stop flag */
	frac16_t       f16VdcCtrlOut;  	/* output of DC voltage controller for current reference generation*/

	frac16_t       f16VBurston; 	/* Start PFC operation when output voltage is less than this value in light-load mode */
	frac16_t       f16VBurstoff; 	/* Stop PFC operation when output voltage is larger than this value in light-load mode */
	
	frac16_t       f16VdcUnctrlRec; /* dc bus uncontrolled rectified voltage in AC-DC mode */
	frac16_t       f16VacThreshold; /* Vac threshold to close the TRAIC in pre-charge stage */

	bool_t         bVacRise;        /* grid voltage trend */
	uint16_t       uw16VacRiseTrendCnt;  /* counter for confirm the grid voltage rising changing trend  */
	uint16_t       uw16VacFallTrendCnt;  /* counter for confirm the grid voltage falling changing trend */
	
	frac16_t       f16VdcBusLowLimit;
}ACDCSTRUC_VDC_CTRL_T;

typedef struct
{
	GFLIB_CTRL_PI_P_AW_T_A32  sPIpAWParams; 		/* PI parameters */
	bool_t                    bStopIntegFlag; 		/* PI controller integration stop flag */ 
	
	GDFLIB_FILTER_MA_T_A32    sIlOffsetFilter;		/* inductor current offset filter */
	GDFLIB_FILTER_IIR1_T_F32  sIlFilter;			/* Il IIR1 filter */
	
	frac16_t     f16Il;       			/* detected inverter side current */	
	frac16_t     f16IlFilt;   			/* detected filtered inverter side current */
	
	frac16_t     f16IlRef;    			/* current reference */
	acc32_t      a32IrefReq;     		/* f16UDcCtrlout/vrms^2, used for current reference generation */
	frac16_t     f16IlErr;    			/* current error */
	frac16_t     f16IlOffsetDeviation;  /* current sampling offset deviation*/
	
	uint16_t     uw16CurLimNumCnt;    	/* counter for cycle by cycle current limitation time */
	uint16_t     uw16CurLimTimerCnt;    /* counter for timing 200ms to check if over-current protection should be triggered */
	
	/* AC-DC part */
	acc32_t      a32HVinkp; 				/* kp used when input voltage is greater than a certain threshold, 
	                                   	   	   different from the zero-crossing point */
	acc32_t      a32HVinki; 				/* ki used when input voltage is greater than a certain threshold, 
	                                   	   	   different from the zero-crossing point */
	frac16_t	 f16RefHlim; 				/* high limit for current reference in case of vac flash, the limitation of voltage loop is decided by vrms  */
	frac16_t     f16IlRec; 					/* rectified value of inductor current */	
	frac16_t	 f16CurCtrlOut;				/* current loop output */

	frac16_t	 f16DutyCCM;				/* calculated duty in CCM mode for duty feed-forward */
	frac16_t	 f16DutyDCM;				/* calculated duty in CCM mode for duty feed-forward */
	frac16_t	 f16PFCDCMRatioNum;			/* numerator for theoretical duty calculation */
	frac16_t	 f16PFCDCMRatioDen;			/* denominator for theoretical duty calculation */
	
	frac16_t       f16VacLowTh; 	        /* input voltage low threshold, different PI parameters are used when input voltage is less than this threshold */
}ACDCSTRUC_IND_CUR_CTRL_T;


typedef struct
{
	ACDCSTRUC_PR_PARAMS_T     sIGridFundamentalPRParams;   	/* grid current fundamental wave: continuous PR parameters */
	FILTER_2P2Z_TRANS_T       sIGridFundamental2p2zParams; 	/* grid current fundamental wave: discrete PR parameters */
	ACDCSTRUC_PR_PARAMS_T     sIGrid3rdHarmonicPRParams;
	FILTER_2P2Z_TRANS_T       sIGrid3rdHarmonic2p2zParams; 	/* grid current 3rd harmonic wave: discrete PR parameters */
	ACDCSTRUC_PR_PARAMS_T     sIGrid5thHarmonicPRParams;
    FILTER_2P2Z_TRANS_T       sIGrid5thHarmonic2p2zParams; 	/* grid current 5th harmonic wave: discrete PR parameters */
	ACDCSTRUC_PR_PARAMS_T     sIGrid7thHarmonicPRParams;
    FILTER_2P2Z_TRANS_T       sIGrid7thHarmonic2p2zParams; 	/* grid current 7th harmonic wave: discrete PR parameters */
    GFLIB_CTRL_PI_P_AW_T_A32  sPIpAWParams; 		        /* PI parameters */
    bool_t                    bStopIntegFlag; 		        /* PI controller integration stop flag */ 
    
	GDFLIB_FILTER_MA_T_A32    sIGridOffsetFilter;		/* grid current offset filter */
	GDFLIB_FILTER_IIR1_T_F32  sIGridFilter;			    /* grid current IIR1 filter */
	frac16_t                  f16IGridOffsetDeviation;  /* current sampling offset deviation*/


	frac16_t     f16IGrid;    			/* detected grid side current */	
	frac16_t     f16IGridFilt;   		/* filtered grid side current */
	         	
	frac16_t     f16IGridRefPeak;       /* current reference peak */
	frac16_t     f16IGridRef;    		/* current reference */
	frac16_t     f16IGridErr;    		/* current error */
	
	frac16_t     f16IGridCtrl1stOut;    /* grid current loop fundamental wave regulator output */
	frac16_t     f16IGridCtrl3rdOut;    /* grid current loop 3rd wave regulator output */
	frac16_t     f16IGridCtrl5thOut;    /* grid current loop 5th wave regulator output */
	frac16_t     f16IGridCtrl7thOut;    /* grid current loop 7th wave regulator output */
	frac16_t     f16IGridPRCtrlOut;
	frac16_t     f16IGridPICtrlOut;
	frac16_t     f16IGridCtrlOut;
	
	frac16_t    f16ProportinalComp;   /* calculated proportional compensation duty */
	frac32_t    f321stdiffComp;       /* calculated 1st differential compensation duty */
	frac32_t    f322nddiffComp;       /* calculated 2nd differential compensation duty */	
}INVSTRUC_GRID_CUR_CTRL_T;


typedef struct
{
	frac16_t f16VInvRms;           /* inverter output voltage RMS */
	frac16_t f16VInvSqr;           /* square of inverter output voltage */
	acc32_t  a32VInvSqrSum;        /* sum for VInvRms calculation over half cycle */
	acc32_t  a32VInvSqrSumSav;     /* saving sum of VInvRms calculation over half cycle for RMS calculation */
	frac16_t f16VGridRms;          /* grid voltage RMS */
	frac16_t f16VGridRmsSqr;       /* square of grid voltage RMS */
	frac16_t f16VGridSqr;          /* square of grid voltage */
	acc32_t  a32VGridSqrSum;       /* sum for VGridRms calculation over half cycle */
	acc32_t  a32VGridSqrSumSav;    /* saving sum of VacRms calculation over half cycle for RMS calculation */
	frac16_t f16IacRms;            /* ac current RMS */
	frac16_t f16IacSqr;            /* square of ac current */
	acc32_t  a32IacSqrSum;         /* sum for IacRms calculation over half cycle*/
	acc32_t  a32IacSqrSumSav;      /* saving sum of IacRms calculation over half cycle for RMS calculation */
	frac16_t f16VIinst;            /* instantaneous value of voltage and current product */
	acc32_t  a32VIinstSum;         /* sum for active power calculation over half cycle*/
	acc32_t  a32VIinstSumSav;      /* saving sum of active power calculation over half cycle*/
	acc32_t  a32PSum;              /* active power sum of 100 consecutive half-cycles */
	acc32_t  a32VASum;             /* apparent power sum of 100 consecutive half-cycles */
	frac16_t f16ActivePower;   
	frac16_t f16ApparentPower;
	frac16_t f16PowerFactor;
	acc32_t  a32VInvFreq;          /* inverter output voltage frequency */
	acc32_t  a32VGridFreq;         /* Grid voltage frequency */
	acc32_t  a32VGridFreqSum;      /* grid voltage frequency sum of 50 consecutive half-cycles */
	acc32_t  a32VGridFreqAvg;      /* average grid voltage frequency */
    uint16_t u16PowerUpdateCnt;    /* counter that used for average power calculation */
    uint16_t u16GridFreqUpdateCnt; /* counter that used for average grid voltage frequency calculation */
}ACDCSTRUC_METERING_T;

typedef union
{
	uint16_t R;
	struct
	{
		uint16_t   VacOver          : 1; /* AC voltage over voltage flag */
		uint16_t   VacRMSOver       : 1; /* AC voltage RMS over voltage flag */
		uint16_t   VacRMSUnder      : 1; /* AC voltage RMS under voltage flag */
		uint16_t   VacFreqOver      : 1; /* AC voltage over frequency flag */
		uint16_t   VacFreqUnder     : 1; /* AC voltage under frequency flag */
		uint16_t   IlOver           : 1; /* inductor current over current flag */
		uint16_t   IGridOver        : 1; /* grid current over current flag */
		uint16_t   HW_IlOver        : 1; /* HW protection - inductor current over current flag */
		uint16_t   HW_VdcOver       : 1; /* HW protection - DC bus over voltage flag */
		uint16_t   TempOver         : 1; /* over temperature flag */
		uint16_t   VdcOver          : 1; /* DC bus voltage over voltage flag */
		uint16_t   VdcUnder         : 1; /* DC bus voltage under voltage flag */
	} B;
}ACDCSTRUC_FAULT_STATUS_T;

typedef struct
{
	frac16_t     f16VacOver;       /* AC voltage maximum level */ 
	frac16_t     f16VacRMSOver;    /* AC voltage maximum rms level */ 
	frac16_t     f16VacRMSUnder;   /* AC voltage minimum rms level */
	acc32_t      a32VInvFreqOver;  /* inverter voltage maximum frequency level */
	acc32_t      a32VInvFreqUnder; /* inverter voltage minimum frequency level */
	acc32_t      a32VGridFreqOver; /* grid voltage maximum frequency level */
	acc32_t      a32VGridFreqUnder;/* grid voltage minimum frequency level */
	frac16_t     f16IlOver;        /* inductor current maximum level */
	frac16_t     f16IGridOver;     /* grid current maximum level */
	frac16_t     f16TempOver;      /* IGBT temperature maximum level */
	frac16_t     f16VdcOver;       /* DC bus voltage maximum level */
	frac16_t     f16VdcUnder;      /* DC bus voltage minimum level */
}ACDCSTRUC_FAULT_THRESHOLDS_T;

typedef struct
{
	ACDCSTRUC_VAC_CTRL_T             sVacCtrl;
	ACDCSTRUC_VDC_CTRL_T             sVdcCtrl;
	ACDCSTRUC_IND_CUR_CTRL_T         sIlCtrl;
	INVSTRUC_GRID_CUR_CTRL_T         sIGridCtrl;
	INVSTRUC_VAC_DETECT_T            sVacDetect;
	INVSTRUC_CUR_DETECT_T            sCurDetect;
	ACDCSTRUC_FAULT_THRESHOLDS_T     sFaultThresholds;
	ACDCSTRUC_FAULT_STATUS_T         sFaultId;
	ACDCSTRUC_FAULT_STATUS_T         sFaultIdPending;
	ACDCSTRUC_METERING_T             sPowerMetering;
	ACDCSTRUC_FLAG_T                 sFlag;
	
	frac16_t     f16VacRecoveryTh; 			/* input voltage recovery judgement threshold  */
	frac16_t     f16VacLackTh; 				/* input voltage power down judgement threshold */
	uint16_t	 u16VacFailCnt; 			/* a counter to record how many consecutive times input voltage is lower than the drop threshold */
	uint16_t	 u16VacRecoveryOkCnt; 		/* a counter to record how many consecutive times input voltage is higher than the recovery threshold */
	
	frac16_t    f16Duty;
	frac16_t    f16DutyComp;                    /* calculated compensation duty */
	bool_t      bCloseLoopOnOff;  				/* 1: enable close loop control, 0: disable close loop control */ 
	
	frac16_t    f16TempSamp;        			/* ADC result of temperature sensing of the IGBT */
	uint16_t    u16ActualTempVal;               /* actual temperature value for display */
	
	uint16_t 	u16WorkModeCmd;				/* work mode command, 1: inverter, 2: PFC */
	uint16_t    u16WorkModeUsed;            /* work mode used now */
	uint16_t    u16InvModeCmd;              /* inverter operating mode command, used for active mode selection*/
	uint16_t    u16CurrentInvMode;          /* current inverter operating mode */
}ACDCSTRUC_BI_DIR_T;

/******************************************************************************
* Global functions
******************************************************************************/

#endif /* BIDIR_DCAC_STRUCTURE_H_ */
