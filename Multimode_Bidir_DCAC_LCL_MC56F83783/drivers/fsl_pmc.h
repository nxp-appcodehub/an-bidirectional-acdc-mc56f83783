/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_PMC_H_
#define _FSL_PMC_H_

#include "fsl_common.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dsc_pmc"
#endif

/*!
 * @defgroup pmc PMC: Power Management Controller Driver
 * @details This document consists of sections titled with <b>Driver Overview</b>, <b>Data Structures</b>,
 *          <b>Enumerations</b>, <b>Functions</b>, etc., each with an overview list and detailed documentation.
 *          It is recommended to read the <b>Driver Overview</b> first for it includes a comprehensive description
 *          of the peripheral, driver and driver changes. Other sections give detailed information for APIs, enums,
 *          macros, etc., for your further reference.
 * @{
 */

/*! @name Driver version */
/*@{*/
/*! @brief PMC driver version. */
#define FSL_PMC_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*******************************************************************************
 * ChangeLog
 ******************************************************************************/
/*!
 * @defgroup pmc_driver_log The Driver Change Log
 * @ingroup pmc
 * @{
 * The current PMC driver version is 2.0.0.
 *
 *     - 2.0.0
 *         - Initial version.
 * @}
 */

/*******************************************************************************
 * Introduction of peripheral and driver
 ******************************************************************************/

/*!
 * @defgroup pmc_intro PMC Peripheral and Driver Overview
 * @ingroup pmc
 * @brief Content including 1) peripheral features, work logic and work method; 2) driver design logic and use method;
 *        3) typical use case.
 * @{
 *
 * Peripheral features and how this peripheral works
 * ==================================================
 *  The PMC module contains the core voltage regulators and power monitoring circuitry. Its function is to ensure that
 *  the chip is operated only within legal voltage ranges and to assist in the orderly shutdown of the chip in the event
 *  that the power supply is interrupted. The PMC module also regulates the internal voltage rails for the core digital
 *  and analog logic.
 *
 *  Features
 *  ---------
 *  + Supports multi-level low voltage detection(including 2.0V, 2.2V, and 2.7V), and multi-type low voltage interrupt(
 *    including #kPMC_2P2VLowVoltageInterruptEnable, #kPMC_2P7VLowVoltageInterruptEnable,
 *    #kPMC_2P2VHighVoltageInterruptEnable, #kPMC_2P7VHighVoltageInterruptEnable). When the input supply drops
 below 2.0V,
 *    the PMC module will assert the POR(Power On Reset). If the input supply drops consistently below 2.2V, the PMC
 module
 *    can be configured to generate the LVI22 low voltage interrupt. If the input supply drops consistently below 2.7V,
 *    the PMC module can be configured to generate the LVI27 low voltage interrupt.
 *  + Inside the regulator, there is the bandgap reference, the buffer can be enabled to drive the 1.2V bandgap
 reference
 *    to the ADC
 *
 *  How this peripheral works
 *  -------------------------
 *  Inside the PMC module, the input supply is processed by the internal \b POR_LVI analog module to generate an
 *  internal power-on reset and 2.2V low voltage and 2.7V low voltage detection signals. The PMC module performs
 *  de-glitch functions on these signals and uses them to generate noise-free versions of the raw POR and low voltage
 *  detects. When the input supply is below 2.0V, the internal \b POR circuit can assert the internal PORT. Inside the
 *  PMC module, the \b LVI logic uses four interrupts enables and two low voltages detects for 2.7V and 2.2V to generate
 *  a single low voltage interrupt. By properly enabling those four interrupts based upon the current input supply
 *  voltage range, the \b LVI can be configured to generate interrupts at any possible falling or rising transition of
 *  two fixed voltage levels.
 *
 *
 * How this driver is designed to make this peripheral works.
 * ===========================================================
 *  Based on the features of the PMC module, the APIs can be divided in 3 groups:
 *  + Bandgap Control Interfaces
 *    The APIs in this function group can be used to control the bandgap in regulator, such as to set the bandgap trim
 *    value.
 *  + Interrupt Control Interfaces
 *    The APIs in this function group can be used to enable/disable the PMC module's interrupts.
 *  + Status Flags Inferfaces
 *    The APIs in this function group can be used to get/clear status flags.
 *
 *
 * How to use this driver
 * =======================
 *  + To get the voltage detection status flag, invoke PMC_GetStatusFlags() function. In the result value
 *    @ref kPMC_Sticky2P7VLowVoltageFlag indicates that the input supply dropped below the 2.7V level at some point,
 *    and @ref kPMC_Sticky2P2VLowVoltageFlag indicates that the input supply dropped below the 2.2V level at some point.
 *    To clear those sticky flags, users can invoke PMC_ClearStatusFlags().
 *
 *
 * Typical Use Case
 * ================
 *  + 2.7V low voltage detection
 *      When the input supply is above 2.7V, and the users want to enable 2.7 low voltage detection, the template is
 *      shown below.
 *      @code
            uint16_t u16Status;
            u16Status = PMC_GetStatusFlags(PMC);
            if ((u16Status & (kPMC_2P7VLowVoltageFlag | kPMC_2P2VLowVoltageFlag)) == 0U)
            {
                PMC_DisableInterrupts(PMC, kPMC_AllInterruptsEnable);
                PMC_EnableInterrupts(PMC, kPMC_2P7VLowVoltageInterruptEnable);
            }
        @endcode
 *      In this type of use case, when the input supply is above 2.7V, to enable 2.7V low voltage interrupt, both
 *      @ref kPMC_2P2VHighVoltageInterruptEnable and @ref kPMC_2P7VHighVoltageInterruptEnable should be disabled.
 *      Enabling @ref kPMC_2P7VLowVoltageInterruptEnable and disabling @ref kPMC_2P2VLowVoltageInterruptEnable will
 *      assert low voltage interrupt when the input supply drops below 2.7V.
 *
 *  + 2.2V high voltage detection
 *      When the input supply is below 2.2V, and the users want to enable 2.2 high voltage detection, the template is
 *      shown below.
 *      @code
            uint16_t u16Status;
            u16Status = PMC_GetStatusFlags(PMC);
            if ((u16Status & (kPMC_2P7VLowVoltageFlag | kPMC_2P2VLowVoltageFlag)) ==
                    (kPMC_2P7VLowVoltageFlag | kPMC_2P2VLowVoltageFlag))
            {
                PMC_DisableInterrupts(PMC, kPMC_AllInterruptsEnable);
                PMC_EnableInterrupts(PMC, kPMC_2P2VHighVoltageInterruptEnable);
            }
        @endcode
 *      In this type of use case, when the input supply is below 2.2V, to enable 2.2V high voltage interrupt, both
 *      @ref kPMC_2P2VLowVoltageInterruptEnable and @ref kPMC_2P7VLowVoltageInterruptEnable should be cleared. Enabling
 *      @ref kPMC_2P2VHighVoltageInterruptEnable and disabling @ref kPMC_2P7VHighVoltageInterruptEnable assert low
 *      voltage interrupt when the input supply rises above 2.2V.
 *  + 2.7V high voltage detection
 *      When the input supply is below 2.7V and above 2.2V, and the users want to enable 2.7 high voltage detection, the
 *      template is shown below.
 *      @code
            uint16_t u16Status;
            u16Status = PMC_GetStatusFlags(PMC);
            if ((u16Status & (kPMC_2P7VLowVoltageFlag | kPMC_2P2VLowVoltageFlag)) == kPMC_2P2VLowVoltageFlag)
            {
                PMC_DisableInterrupts(PMC, kPMC_AllInterruptsEnable);
                PMC_EnableInterrupts(PMC, kPMC_2P7VHighVoltageInterruptEnable);
            }
        @endcode
 *      In this type of use case, when the input supply is below 2.7V and above 2.2V, to enable 2.7 high voltage
 interrupt,
 *      both @ref kPMC_2P7VHighVoltageInterruptEnable and @ref kPMC_2P2VHighVoltageInterruptEnable should be disabled.
 *      Enabling @ref kPMC_2P7VLowVoltageInterruptEnable and disabling @ref kPMC_2P2VLowVoltageInterruptEnable assert
 *      low voltage interrupt only if the input supply rises above 2.7V.
 * @}
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief The enumeration of PMC voltage detection interrupts.
 */
enum _pmc_interrupt_enable
{
    kPMC_2P2VLowVoltageInterruptEnable = PMC_CTRL_LV22IE_MASK,  /*!< If the input supply is currently dropped below the
                                                                    2.2V level, generate the low voltage interrupt. */
    kPMC_2P7VLowVoltageInterruptEnable = PMC_CTRL_LV27IE_MASK,  /*!< If the input supply is currently dropped below the
                                                                    2.7V level, generate the low voltage interrupt. */
    kPMC_2P2VHighVoltageInterruptEnable = PMC_CTRL_HV22IE_MASK, /*!< If the input supply is currently raised above the
                                                                    2.2V level, generate the low voltage interrupt.  */
    kPMC_2P7VHighVoltageInterruptEnable = PMC_CTRL_HV27IE_MASK, /*!< If the input supply is currently raised above the
                                                                    2.2V level, generate the low voltage interrupt. */
    kPMC_AllInterruptsEnable =
        PMC_CTRL_LV22IE_MASK | PMC_CTRL_LV27IE_MASK | PMC_CTRL_HV22IE_MASK | PMC_CTRL_HV27IE_MASK,
};

/*!
 * @brief The enumeration of PMC status flags.
 */
enum _pmc_status_flags
{
    kPMC_SmallRegulator2P7VActiveFlag = PMC_STS_SR27_MASK, /*!< The small regulator 2.7V supply is ready to be used. */
    kPMC_LowVoltageInterruptFlag      = PMC_STS_LVI_MASK,  /*!< The low voltage interrupt flag, used to indicate whether
                                                               the low voltage interrupt is asserted. */
    kPMC_Sticky2P7VLowVoltageFlag = PMC_STS_SLV27F_MASK,   /*!< Input supply has dropped below the 2.7V threshold.
                                                               This sticky flag indicates that the input supply
                                                               dropped below the 2.7V level at some point. */
    kPMC_Sticky2P2VLowVoltageFlag = PMC_STS_SLV22F_MASK,   /*!< Input supply has dropped below the 2.2V threshold.
                                                               This sticky flag indicates that the input supply
                                                               dropped below the 2.2V threshold at some point. */
    kPMC_2P7VLowVoltageFlag = PMC_STS_LV27F_MASK,          /*!< Input supply is below the 2.7V threshold. */
    kPMC_2P2VLowVoltageFlag = PMC_STS_LV22F_MASK,          /*!< Input supply is below the 2.2V threshold. */
    kPMC_AllStatusFlags     = kPMC_SmallRegulator2P7VActiveFlag | kPMC_LowVoltageInterruptFlag |
                          kPMC_Sticky2P7VLowVoltageFlag | kPMC_Sticky2P2VLowVoltageFlag | kPMC_2P7VLowVoltageFlag |
                          kPMC_2P2VLowVoltageFlag,
};

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name Bandgap Control Interfaces
 * @{
 */

/*!
 * @brief Sets the trim value of the bandgap reference in the regulator.
 *
 * @param base PMC peripheral base address.
 * @param u8TrimValue The bandgap's trim value, ranges from 0 to 15.
 */
static inline void PMC_SetBandgapTrim(PMC_Type *base, uint8_t u8TrimValue)
{
    base->CTRL = (base->CTRL & (~PMC_CTRL_TRIM_MASK)) | PMC_CTRL_TRIM(u8TrimValue);
}

/*!
 * @brief Enables/Disables a buffer that drivers the 1.2V bandgap reference to the ADC.
 *
 * If the users want to calibrate the ADC using the 1.2V reference voltage, then the voltage reference buffer should be
 * enabled. When ADC calibration is not being performed, the voltage reference buffer should be disabled to save power.
 *
 * @param base PMC peripheral base address.
 * @param bEnable Used to control the behaviour of voltage reference buffer.
 *                - \b true Enable voltage reference buffer.
 *                - \b false Disable voltage reference buffer.
 */
static inline void PMC_EnableVoltageReferenceBuffer(PMC_Type *base, bool bEnable)
{
    if (bEnable)
    {
        base->CTRL |= PMC_CTRL_VRBEN_MASK;
    }
    else
    {
        base->CTRL &= ~PMC_CTRL_VRBEN_MASK;
    }
}

/*! @} */

/*!
 * @name Interrupts Control Interfaces
 * @{
 */

/*!
 * @brief Enables the interrups, including 2.2V high voltage interrupt, 2.7V high voltage interrupt, 2.2V low voltage
 * interrupt, 2.7V low voltage interrupt.
 *
 * @param base PMC peripheral base address.
 * @param u16Interrupts The interupts to be enabled, should be the OR'ed value of @ref _pmc_interrupt_enable.
 */
static inline void PMC_EnableInterrupts(PMC_Type *base, uint16_t u16Interrupts)
{
    base->CTRL |= u16Interrupts & 0xFU;
}

/*!
 * @brief Disables the interrups, including 2.2V high voltage interrupt, 2.7V high voltage interrupt, 2.2V low voltage
 * interrupt, 2.7V low voltage interrupt.
 *
 * @param base PMC peripheral base address.
 * @param u16Interrupts The interupts to be disabled, should be the OR'ed value of @ref _pmc_interrupt_enable.
 */
static inline void PMC_DisableInterrupts(PMC_Type *base, uint16_t u16Interrupts)
{
    base->CTRL &= ~(u16Interrupts & 0xFU);
}

/*! @} */

/*!
 * @name Status Flags Inferfaces
 * @{
 */

/*!
 * @brief Gets the status flags of PMC module, such as low voltage interrpt flag, small regulator 2.7 active flag, etc.
 *
 * @param base PMC peripheral base address.
 * @return The status flags of PMC module, should be the OR'ed value of @ref _pmc_status_flags.
 */
static inline uint16_t PMC_GetStatusFlags(PMC_Type *base)
{
    return base->STS;
}

/*!
 * @brief Clears the status flags of PMC module, only low voltage interrupt flag, sticky 2.7V low voltage flag, and
 * sticky 2.2V low voltage flag can be cleared.
 *
 * @param base PMC peripheral base address.
 * @param u16StatusFlags The status flags to be cleared, should be the OR'ed value of @ref kPMC_LowVoltageInterruptFlag,
 *                    @ref kPMC_Sticky2P7VLowVoltageFlag, and @ref kPMC_Sticky2P2VLowVoltageFlag.
 */
static inline void PMC_ClearStatusFlags(PMC_Type *base, uint16_t u16StatusFlags)
{
    base->STS = u16StatusFlags;
}

/*! @} */

#if defined(__cplusplus)
}
#endif;

/*! @} */

#endif /* _FSL_GPIO_H_*/
