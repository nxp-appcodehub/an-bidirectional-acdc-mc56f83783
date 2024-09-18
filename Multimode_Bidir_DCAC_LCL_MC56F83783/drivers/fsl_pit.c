/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_pit.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dsc_pit"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address to be used to gate or ungate the module clock
 *
 * @param base PIT peripheral base address
 *
 * @return The PIT instance
 */
static uint32_t PIT_GetInstance(PIT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to PIT bases for each instance. */
static PIT_Type *const s_pitBases[] = PIT_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to PIT clocks for each instance. */
static const clock_ip_name_t s_pitClocks[] = PIT_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t PIT_GetInstance(PIT_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_pitBases); instance++)
    {
        if (s_pitBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_pitBases));

    return instance;
}

/*!
 * brief Ungates the PIT clock, configures the PIT features.
 * The configurations are:
 * - Clock source selection for PIT module
 * - Prescaler configuration to the input clock source
 * - PIT period interval
 * - PIT slave mode enable/disable
 * - Interrupt enable/disable
 * - PIT timer enable/disable
 *
 * note This API should be called at the beginning of the application using the PIT driver and call PIT_StartTimer()
 * API to start PIT timer.
 *
 * param base   PIT peripheral base address
 * param config Pointer to the user's PIT config structure
 */
void PIT_Init(PIT_Type *base, const pit_config_t *psConfig)
{
    assert(NULL != psConfig);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the module clock */
    CLOCK_EnableClock(s_pitClocks[PIT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if defined(FSL_FEATURE_PIT_32BIT_COUNTER) && FSL_FEATURE_PIT_32BIT_COUNTER
    PIT_SetTimerPeriod(base, psConfig->u32PeriodCount);
#else
    PIT_SetTimerPeriod(base, psConfig->u16PeriodCount);
#endif

    base->CTRL = PIT_CTRL_PRESCALER(psConfig->ePrescaler) | PIT_CTRL_CLKSEL(psConfig->eClockSource) |
                 PIT_CTRL_SLAVE(psConfig->bEnableSlaveMode) | PIT_CTRL_PRIE(psConfig->bEnableInterrupt) |
                 PIT_CTRL_CNT_EN(psConfig->bEnableTimer);
}

/*!
 * brief Gates the PIT clock and disables the PIT module.
 *
 * param base PIT peripheral base address
 */
void PIT_Deinit(PIT_Type *base)
{
    /* Stop the counter */
    base->CTRL &= ~((uint16_t)PIT_CTRL_CNT_EN_MASK);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the module clock */
    CLOCK_DisableClock(s_pitClocks[PIT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * brief  Fill in the PIT config structure with the default settings
 *
 * This function initializes the PIT configuration structure to default values.
 * code
 *    psConfig->ePrescaler = kPIT_PrescalerDivBy1;
 *    psConfig->bEnableInterrupt = false;
 *    psConfig->bEnableSlaveMode = false;
 *    psConfig->bEnableTimer = false;
 *    psConfig->eClockSource = kPIT_CountClockSource0;
 *    psConfig->u16PeriodCount = 0xFFFFU;
 * endcode
 * param psConfig Pointer to user's PIT config structure.
 */
void PIT_GetDefaultConfig(pit_config_t *psConfig)
{
    assert(NULL != psConfig);

    /* Initializes the psConfigure structure to zero. */
    (void)memset(psConfig, 0, sizeof(*psConfig));

    /* PIT clock source is IP bus clock */
    psConfig->eClockSource = kPIT_CountClockSource0;
    /* Disable PIT timer */
    psConfig->bEnableTimer = false;
    /* Disable PIT slave mode */
    psConfig->bEnableSlaveMode = false;
    /* Count rate is PIT clock divider by 1 */
    psConfig->ePrescaler = kPIT_PrescalerDivBy1;
    /* Disable PIT Roll-Over Interrupt */
    psConfig->bEnableInterrupt = false;
#if defined(FSL_FEATURE_PIT_32BIT_COUNTER) && FSL_FEATURE_PIT_32BIT_COUNTER
    psConfig->u32PeriodCount = 0xFFFFFFFFU;
#else
    psConfig->u16PeriodCount = 0xFFFFU;
#endif
}
