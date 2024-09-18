/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef BIDIR_DCAC_STATEMACHINE_H_
#define BIDIR_DCAC_STATEMACHINE_H_

#include "cpu.h"
#include "bidir_dcac_structure.h"
#include "bidir_dcacctrl.h"
#include "state_machine.h"
#include "spll_1ph_sogi.h"
#include "PR_controller.h"

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
	SOFTSTART       = 0,
	NORMAL          = 1,
	LIGHTLOAD       = 2
} PFC_RUN_SUBSTATE_T;
/******************************************************************************
* Global variables
******************************************************************************/
extern SM_APP_CTRL_T            gsACDC_Ctrl;
extern SPLL_1PH_SOGI_T 			spll_obj;
extern ACDCSTRUC_BI_DIR_T       gsACDC_Drive;
extern PFC_RUN_SUBSTATE_T   	gsPFC_Runsub;
extern uint32_t  gu32TimerCnt, guw32StartCnt, gu32IOTogTimeCnt;
extern uint16_t  gu16VGridCycleCnt, gu16VInvCycleCnt, gu16SlowBriDTCnt, gu16TriacOnCnt, gu16SoftStartStepCnt;
__pmem extern const PFCN_VOID_VOID mPFC_STATE_RUN_TABLE[3];
/******************************************************************************
* Global functions
******************************************************************************/
extern bool_t ACDC_TimeDelay(uint32_t uw32timerstartval, uint32_t uw32delaytime);
extern void ACDC_FaultDetection(void);
extern void OffGrid_CtrlparamTune();
#endif /* BIDIR_DCAC_STATEMACHINE_H_ */
