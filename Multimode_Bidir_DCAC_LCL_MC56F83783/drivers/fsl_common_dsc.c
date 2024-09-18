/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.common_dsc"
#endif

#define COMMON_INTC_TYPE_REG_INDEX(x) (((uint8_t)(x)) >> 3U)
#define COMMON_INTC_TYPE_BIT_INDEX(x) ((((uint8_t)(x)) & 0x7U) << 1U)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Interrupt priority table. */
static uint16_t s_intPrioTable[(NUMBER_OF_INT_IRQ - 1U) / 8U + 1U] = {0U};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t EnableIRQWithPriority(IRQn_Type irq, uint8_t priNum)
{
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t prioMask;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = COMMON_INTC_TYPE_REG_INDEX(irq);
    bitIndex = COMMON_INTC_TYPE_BIT_INDEX(irq);
    prioMask = ((uint16_t)3U << bitIndex);
    /* Valid priority number is 0-3 */
    priNum = priNum & 0x03U;

    if (0U == priNum)
    {
        priNum = SDK_DSC_DEFAULT_INT_PRIO;
    }

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    /* Save the priority in s_intPrioTable */
    reg = s_intPrioTable[regIndex];
    reg = (reg & ~prioMask) | (priNum << bitIndex);

    s_intPrioTable[regIndex] = reg;

    /* Set new priority in interrupt controller register. */
    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];
    reg = (reg & ~prioMask) | (priNum << bitIndex);

    ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

status_t DisableIRQ(IRQn_Type irq)
{
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = COMMON_INTC_TYPE_REG_INDEX(irq);
    bitIndex = COMMON_INTC_TYPE_BIT_INDEX(irq);

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];
    reg = reg & (~((uint16_t)3U << bitIndex));

    ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

status_t EnableIRQ(IRQn_Type irq)
{
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t prioMask;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = COMMON_INTC_TYPE_REG_INDEX(irq);
    bitIndex = COMMON_INTC_TYPE_BIT_INDEX(irq);
    prioMask = ((uint16_t)3U << bitIndex);

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    /* If priority in s_intPrioTable is 0, use SDK_DSC_DEFAULT_INT_PRIO. */
    if (0U == (s_intPrioTable[regIndex] & prioMask))
    {
        s_intPrioTable[regIndex] = (s_intPrioTable[regIndex] & ~prioMask) | (SDK_DSC_DEFAULT_INT_PRIO << bitIndex);
    }

    /* Set the interrupt priority with the priority in s_intPrioTable. */
    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];
    reg = (reg & ~prioMask) | (s_intPrioTable[regIndex] & prioMask);

    ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

/*
 * brief Set the IRQ priority.
 *
 * note The parameter priNum is range in 1~3, and its value is **NOT**
 * directly map to interrupt priority.
 *
 * - Some IPs maps 1 to priority 1, 2 to priority 2, 3 to priority 3
 * - Some IPs maps 1 to priority 0, 2 to priority 1, 3 to priority 2
 *
 * User should check chip's RM to get its corresponding interrupt priority
 */
status_t IRQ_SetPriority(IRQn_Type irq, uint8_t priNum)
{
    /*
     * If the interrupt is already enabled, the new priority will be set
     * to the register. If interrupt is not enabled, the new priority is
     * only saved in priority table s_intPrioTable, when interrupt enabled,
     * the priority value is set to register.
     */
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t prioMask;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = COMMON_INTC_TYPE_REG_INDEX(irq);
    bitIndex = COMMON_INTC_TYPE_BIT_INDEX(irq);
    prioMask = ((uint16_t)3U << bitIndex);

    /* Valid priority number is 0-3 */
    priNum = priNum & 0x03U;

    if (0U == priNum)
    {
        priNum = SDK_DSC_DEFAULT_INT_PRIO;
    }

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    /* Save the priority in s_intPrioTable */
    reg = s_intPrioTable[regIndex];
    reg = (reg & ~prioMask) | (priNum << bitIndex);

    s_intPrioTable[regIndex] = reg;

    /*
     * If interrupt already enabled, set new priority
     * in interrupt controller register.
     */
    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];

    if (0U != (reg & prioMask))
    {
        reg = (reg & (~prioMask)) | (priNum << bitIndex);

        ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;
    }

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

/*!
 * brief Delay core cycles.
 *  Please note that, this API uses software loop for delay, the actual delayed
 *  time depends on core clock frequency, where the function is located (ram or flash),
 *  flash clock, possible interrupt.
 *
 * param u32Num  Number of core clock cycle which needs to be delayed.
 */
void SDK_DelayCoreCycles(uint32_t u32Num)
{
    /*
     *  if(u32Num < 22)
     *  {
     *      ActualDelayCycle = 21;
     *  }
     *  else
     *  {
     *      ActualDelayCycle = 35 + ((u32Num-22)/8) * 8 = 13 + u32Num - ((u32Num-22)%8)
     *  }
     */

    /*  JSR - 4 cycles
     *  RTS - 8 cycles
     */

    asm {
       cmp.l #21,A // 2 cycle
       bls   ret // 5 cycles when jump occurs. 3 cycles when jump doesn't occur
       nop // 1 cycle
       nop // 1 cycle
       sub.l  #22, A // 2 cycle
       asrr.l #3, A // 2 cycle
       bra   test // 5 cycle

       loop:
       dec.l A // 1 cycle

       test:
       tst.l A // 1 cycle
       nop // 1 cycle
       bne   loop // 5 cycles when jump occurs. 3 cycles when jump doesn't occur

       ret:
       nop // 1 cycle
       nop // 1 cycle
    }
}

/*!
 * brief Delay at least for some time.
 *  Please note that, this API uses while loop for delay, different run-time environments make the time not precise,
 *  if precise delay count was needed, please implement a new delay function with hardware timer.
 *  The maximum delay time is limited with MCU core clock, for example CPU core clock = 100MHz, then the maximum delay
 *  time should be less than 0xFFFFFFFF/100 = 42,949,672us
 *
 * param delayTime_us Delay time in unit of microsecond.
 * param coreClock_Hz  Core clock frequency with Hz.
 */
void SDK_DelayAtLeastUs(uint32_t delayTime_us, uint32_t coreClock_Hz)
{
    assert(0U != delayTime_us);
    uint32_t count = USEC_TO_COUNT(delayTime_us, coreClock_Hz);

    SDK_DelayCoreCycles(count);
}
