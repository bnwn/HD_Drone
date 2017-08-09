/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * $Revision:  2$
 * $Date: 16/02/25 15:53 $
 * @brief    PN020 series ADC driver source file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
*****************************************************************************/
#include "PN020Series.h"

/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_ADC_Driver ADC Driver
  @{
*/


/** @addtogroup PN020_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief Enable ADC Power
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_Enable(ADC_T *adc)
{
	uint32_t data;
	
	ADC->CTL |= ADC_CTL_ADCEN_Msk;
	SYS_delay_10nop(100);
	ADC->CTL |= ADC_CTL_SWTRG_Msk;
	while(ADC->STATUS & ADC_STATUS_BUSY_Msk);
	data = (ADC->DAT & ADC_DAT_RESULT_Msk);
	data = data;
    return;
}

/**
  * @brief Disable ADC Power
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_Disable(ADC_T *adc)
{
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;
	
	ADC->CTL &= ~ADC_CTL_ADCEN_Msk;
    return;

}

/**
  * @brief This API configures ADC module to be ready for convert the input from selected channel
  * @param[in] adc Base address of ADC module
  * @param[in] u32InputMode This parameter is unused
  * @param[in] u32OpMode This parameter is unused
  * @param[in] u32ChMask Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1...
  * @return  None
  * @note PN020 series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * @note This API does not turn on ADC power nor does trigger ADC conversion
  */
void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask)
{
    ADC->CHEN  = (ADC->CHEN & ~(ADC_CHEN_CHEN0_Msk |
                                ADC_CHEN_CHEN1_Msk |
                                ADC_CHEN_CHEN2_Msk |
                                ADC_CHEN_CHEN3_Msk |
                                ADC_CHEN_CHEN4_Msk |
                                ADC_CHEN_CHEN5_Msk |
                                ADC_CHEN_CHEN6_Msk |
                                ADC_CHEN_CHEN7_Msk)) | u32ChMask;
    return;
}

/**
  * @brief Disable ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_Close(ADC_T *adc)
{
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;
    return;

}

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] adc Base address of ADC module
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                 - \ref ADC_TRIGGER_BY_EXT_PIN
  *                 - \ref ADC_TRIGGER_BY_PWM
  * @param[in] u32Param While ADC trigger by PWM, this parameter is used to set the delay between PWM
  *                     trigger and ADC conversion. Valid values are from 0 ~ 0xFF, and actual delay
  *                     time is (4 * u32Param * HCLK). While ADC trigger by external pin, this parameter
  *                     is used to set trigger condition. Valid values are:
  *                 - \ref ADC_FALLING_EDGE_TRIGGER
  *                 - \ref ADC_RISING_EDGE_TRIGGER
  * @return None
  */
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param)
{
    ADC->CTL &= ~(ADC_TRIGGER_BY_PWM | ADC_RISING_EDGE_TRIGGER | ADC_CTL_HWTRGEN_Msk);
    if(u32Source == ADC_TRIGGER_BY_EXT_PIN) {
        ADC->CTL |= u32Source | u32Param | ADC_CTL_HWTRGEN_Msk;
    } else {

        ADC->CTL |=  u32Source|ADC_CTL_HWTRGEN_Msk;
    }
    return;
}

/**
  * @brief Disable hardware trigger ADC function.
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_DisableHWTrigger(ADC_T *adc)
{
    ADC->CTL &= ~(ADC_TRIGGER_BY_PWM | ADC_RISING_EDGE_TRIGGER | ADC_CTL_HWTRGEN_Msk);
    return;
}

/**
  * @brief Set ADC sample time for designated channel.
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum This parameter is not used
  * @param[in] u32SampleTime ADC sample ADC time, valid values 1 --1023
  * @return None
  */
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime)
{
    ADC->EXTSMPT = (ADC->EXTSMPT & ~ADC_EXTSMPT_EXTSMPT_Msk) | u32SampleTime;
}

/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be enabled.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask)
{
    if(u32Mask & ADC_ADIF_INT)
        ADC->CTL |= ADC_CTL_ADCIEN_Msk;
    if(u32Mask & ADC_CMP0_INT)
        ADC->CMP0 |= ADC_CMP0_ADCMPIE_Msk;
    if(u32Mask & ADC_CMP1_INT)
        ADC->CMP1 |= ADC_CMP1_ADCMPIE_Msk;

    return;
}

/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be disabled.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask)
{
    if(u32Mask & ADC_ADIF_INT)
        ADC->CTL &= ~ADC_CTL_ADCIEN_Msk;
    if(u32Mask & ADC_CMP0_INT)
        ADC->CMP0 &= ~ADC_CMP0_ADCMPIE_Msk;
    if(u32Mask & ADC_CMP1_INT)
        ADC->CMP1 &= ~ADC_CMP1_ADCMPIE_Msk;

    return;
}

/**
  * @brief ADC PWM Sequential Mode Control.
  * @param[in] adc Base address of ADC module
  * @param[in] u32SeqTYPE   This parameter decides which type will be selected.
  *                     - \ref ADC_SEQMODE_TYPE_23SHUNT
  *                     - \ref ADC_SEQMODE_TYPE_1SHUNT
  * @param[in] u32ModeSel  This parameter decides which mode will be selected.
  *                     - \ref ADC_SEQMODE_MODESELECT_CH01
  *                     - \ref ADC_SEQMODE_MODESELECT_CH12
  *                     - \ref ADC_SEQMODE_MODESELECT_CH02
  * @return None
  */
void ADC_SeqModeEnable(ADC_T *adc, uint32_t u32SeqTYPE, uint32_t u32ModeSel)
{
    // Enable ADC Sequential Mode
    ADC->SEQCTL = ADC->SEQCTL  | ADC_SEQCTL_SEQEN_Msk;

    // Select ADC Sequential Mode Type
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_SEQTYPE_Msk)) | (u32SeqTYPE << ADC_SEQCTL_SEQTYPE_Pos);

    // Select ADC Sequential Mode Type
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_MODESEL_Msk)) | (u32ModeSel << ADC_SEQCTL_MODESEL_Pos);

    return;
}

/**
  * @brief ADC PWM Sequential Mode PWM Trigger Source and type.
  * @param[in] adc Base address of ADC module
  * @param[in] u32SeqModeTriSrc1  This parameter decides first PWM trigger source and type.
  * @param[in] u32SeqModeTriSrc2  This parameter decides second PWM trigger source and type.
  *
  *
  * @return None
  */
void ADC_SeqModeTriggerSrc(ADC_T *adc, uint32_t u32SeqModeTriSrc1, uint32_t u32SeqModeTriSrc2)
{
    // Select PWM Trigger Source Selection for TRG1CTL or TRG2CTL
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_TRG1CTL_Msk)) | (u32SeqModeTriSrc1 << ADC_SEQCTL_TRG1CTL_Pos);
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_TRG2CTL_Msk)) | (u32SeqModeTriSrc2 << ADC_SEQCTL_TRG2CTL_Pos);
    return;
}

/*@}*/ /* end of group PN020_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_ADC_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
