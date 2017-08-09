/**************************************************************************//**
 * @file     adc.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/25 16:08 $ 
 * @brief    PN020 series ADC driver header file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/ 
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_ADC_Driver ADC Driver
  @{
*/

/** @addtogroup PN020_ADC_EXPORTED_CONSTANTS ADC Exported Constants
  @{
*/

#define ADC_CH8_EXT                     (0UL)                       		/*!< Use external input pin as ADC channel 8 source */
#define ADC_CH8_BGP                     (ADC_CHEN_CH8SEL_Msk)       		/*!< Use internal band-gap voltage (VBG) as channel 8 source. */
#define ADC_CMP0_LESS_THAN               (0UL << ADC_CMP0_CMPCOND_Pos)	/*!< ADC compare condition less than */
#define ADC_CMP1_LESS_THAN               (0UL << ADC_CMP1_CMPCOND_Pos)	/*!< ADC compare condition less than */
#define ADC_CMP0_GREATER_OR_EQUAL_TO     (1ul << ADC_CMP0_CMPCOND_Pos)	/*!< ADC compare condition greater or equal to */
#define ADC_CMP1_GREATER_OR_EQUAL_TO     (1ul << ADC_CMP1_CMPCOND_Pos)	/*!< ADC compare condition greater or equal to */
#define ADC_TRIGGER_BY_EXT_PIN          (0UL)                       		/*!< ADC trigger by STADC (P3.2) pin */
#define ADC_TRIGGER_BY_PWM              (ADC_CTL_HWTRGSEL_Msk)      		/*!< ADC trigger by PWM events */
#define ADC_FALLING_EDGE_TRIGGER        (0UL)                       		/*!< External pin falling edge trigger ADC */
#define ADC_RISING_EDGE_TRIGGER         (ADC_CTL_HWTRGCOND_Msk)     		/*!< External pin rising edge trigger ADC */
#define ADC_ADIF_INT                    (ADC_STATUS_ADIF_Msk)       		/*!< ADC convert complete interrupt */
#define ADC_CMP0_INT                    (ADC_STATUS_ADCMPIF0_Msk)    		/*!< ADC comparator 0 interrupt */
#define ADC_CMP1_INT                    (ADC_STATUS_ADCMPIF1_Msk)    		/*!< ADC comparator 0 interrupt */
#define ADC_SEQMODE_TYPE_23SHUNT        (0UL)                       		/*!< ADC sequential mode 23-shunt type */
#define ADC_SEQMODE_TYPE_1SHUNT         (1UL)                       		/*!< ADC sequential mode 1-shunt type */
#define ADC_SEQMODE_MODESELECT_CH01     (0UL)                       		/*!< ADC channel 0 then channel 1 conversion */
#define ADC_SEQMODE_MODESELECT_CH12     (1UL)                       		/*!< ADC channel 1 then channel 2 conversion */
#define ADC_SEQMODE_MODESELECT_CH02     (2UL)                       		/*!< ADC channel 0 then channel 2 conversion */
#define ADC_SEQMODE_PWM0_RISING		      (0UL)                       		/*!< ADC sequential mode PWM0 rising trigger ADC*/
#define ADC_SEQMODE_PWM0_CENTER		      (1UL)                       		/*!< ADC sequential mode PWM0 center trigger ADC*/
#define ADC_SEQMODE_PWM0_FALLING	      (2UL)                       		/*!< ADC sequential mode PWM0 falling trigger ADC*/
#define ADC_SEQMODE_PWM0_PERIOD 	      (3UL)                       		/*!< ADC sequential mode PWM0 period trigger ADC*/
#define ADC_SEQMODE_PWM2_RISING		      (4UL)                       		/*!< ADC sequential mode PWM2 rising trigger ADC*/
#define ADC_SEQMODE_PWM2_CENTER		      (5UL)                       		/*!< ADC sequential mode PWM2 center trigger ADC*/
#define ADC_SEQMODE_PWM2_FALLING	      (6UL)                       		/*!< ADC sequential mode PWM2 falling trigger ADC*/
#define ADC_SEQMODE_PWM2_PERIOD 	      (7UL)                       		/*!< ADC sequential mode PWM2 period trigger ADC*/
#define ADC_SEQMODE_PWM4_RISING		      (8UL)                       		/*!< ADC sequential mode PWM4 rising trigger ADC*/
#define ADC_SEQMODE_PWM4_CENTER		      (9UL)                       		/*!< ADC sequential mode PWM4 center trigger ADC*/
#define ADC_SEQMODE_PWM4_FALLING	      (10UL)                       		/*!< ADC sequential mode PWM4 falling trigger ADC*/
#define ADC_SEQMODE_PWM4_PERIOD 	      (11UL)                       		/*!< ADC sequential mode PWM4 period trigger ADC*/
/*@}*/ /* end of group PN020_ADC_EXPORTED_CONSTANTS */


/** @addtogroup PN020_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/


/**
  * @brief Configure the analog input source of channel 8
  * @param[in] adc Base address of ADC module
  * @param[in] u32Source Decides the analog input source of channel 8, valid values are
  *                     - \ref ADC_CH8_EXT
  *                     - \ref ADC_CH8_BGP
  * @return None
  * @note While using VBG as channel 8 source, ADC module clock must /b not exceed 300kHz 
  * \hideinitializer
  */
#define ADC_CONFIG_CH8(adc, u32Source) (ADC->CHEN = ((adc)->CHEN & ~ADC_CHEN_CH8SEL_Msk) | (u32Source))

/**
  * @brief Get the latest ADC conversion data
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum Currently not used
  * @return  Latest ADC conversion data
  * \hideinitializer
  */
#define ADC_GET_CONVERSION_DATA(adc, u32ChNum) ((adc)->DAT & ADC_DAT_RESULT_Msk)

/**
  * @brief Return the user-specified interrupt flags
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return  User specified interrupt flags
  * \hideinitializer
  */
#define ADC_GET_INT_FLAG(adc, u32Mask) ((adc)->STATUS & (u32Mask))


/**
  * @brief This macro clear the selected interrupt status bits
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return  None
  * \hideinitializer
  */
#define ADC_CLR_INT_FLAG(adc, u32Mask) ((adc)->STATUS = ((adc)->STATUS & ~(ADC_STATUS_ADIF_Msk | \
                                                                       ADC_STATUS_ADCMPF0_Msk | \
                                                                       ADC_STATUS_ADCMPF1_Msk)) | (u32Mask))
/**
  * @brief Return the user-specified event flags
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following evnet status bits. 
  *                     - \ref ADC_STATUS_ADIF_Msk
  *                     - \ref ADC_STATUS_ADCMPF0_Msk
  *                     - \ref ADC_STATUS_ADCMPF1_Msk
  * @return  User specified interrupt flags
  * \hideinitializer
  */
#define ADC_GET_FLAG(adc, u32Mask) ((adc)->STATUS & (u32Mask))

/**
  * @brief This macro clear the selected interrupt status bits
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_STATUS_ADIF_Msk
  *                     - \ref ADC_STATUS_ADCMPF0_Msk
  *                     - \ref ADC_STATUS_ADCMPF1_Msk
  * @return  None
  * \hideinitializer
  */
#define ADC_CLR_INT_FLAG(adc, u32Mask) ((adc)->STATUS = ((adc)->STATUS & ~(ADC_STATUS_ADIF_Msk | \
                                                                       ADC_STATUS_ADCMPF0_Msk | \
                                                                       ADC_STATUS_ADCMPF1_Msk)) | (u32Mask))
/**
  * @brief Get the busy state of ADC
  * @param[in] adc Base address of ADC module
  * @return busy state of ADC
  * @retval 0 ADC is not busy
  * @retval 1 ADC is busy
  * \hideinitializer
  */
#define ADC_IS_BUSY(adc) ((adc)->STATUS & ADC_STATUS_BUSY_Msk ? 1 : 0)
     
/**
  * @brief Check if the ADC conversion data is over written or not
  * @param[in] adc Base address of ADC module
  * @return Over run state of ADC data
  * @retval 0 ADC data is not overrun
  * @retval 1 ADC data is overrun
  * \hideinitializer
  */     
#define ADC_IS_DATA_OVERRUN(adc) ((adc)->STATUS & ADC_STATUS_OV_Msk ? 1 : 0)

/**
  * @brief Check if the ADC conversion data is valid or not
  * @param[in] adc Base address of ADC module
  * @return Valid state of ADC data
  * @retval 0 ADC data is not valid
  * @retval 1 ADC data us valid
  * \hideinitializer
  */  
#define ADC_IS_DATA_VALID(adc) ((adc)->STATUS & ADC_STATUS_VALID_Msk ? 1 : 0)

/**
  * @brief Power down ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_POWER_DOWN(adc) ((adc)->CTL &= ~ADC_CTL_ADCEN_Msk)

/**
  * @brief Power on ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_POWER_ON(adc) ((adc)->CTL |= ADC_CTL_ADCEN_Msk)                                                        

/**
  * @brief Configure the comparator 0 and enable it
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 7
  * @param[in] u32Condition Specifies the compare condition
  *                     - \ref ADC_CMP0_LESS_THAN
  *                     - \ref ADC_CMP0_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the compare value. Valid value are between 0 ~ 0x3FF
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return None
  * @details For example, ADC_ENABLE_CMP0(ADC, 5, ADC_CMP_GREATER_OR_EQUAL_TO, 0x800, 10);
  *          Means ADC will assert comparator 0 flag if channel 5 conversion result is 
  *          greater or equal to 0x800 for 10 times continuously.
  * \hideinitializer
  */ 
#define ADC_ENABLE_CMP0(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (ADC->CMP0 = ((u32ChNum) << ADC_CMP0_CMPCH_Pos) | \
                                                                   (u32Condition) | \
                                                                   ((u32Data) << ADC_CMP0_CMPDAT_Pos) | \
                                                                   (((u32MatchCount) - 1) << ADC_CMP0_CMPMCNT_Pos) |\
																																	 ADC_CMP0_ADCMPIE_Msk |\
                                                                   ADC_CMP0_ADCMPEN_Msk) 
																																	 
/**
  * @brief Disable comparator 0
  * @param[in] adc Base address of ADC module
  * \hideinitializer
  */  
#define ADC_DISABLE_CMP0(adc) ((adc)->CMP0 = 0)              

/**
  * @brief Configure the comparator 1 and enable it
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 7
  * @param[in] u32Condition Specifies the compare condition
  *                     - \ref ADC_CMP1_LESS_THAN
  *                     - \ref ADC_CMP1_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the compare value. Valid value are between 0 ~ 0x3FF
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return None
  * @details For example, ADC_ENABLE_CMP1(ADC, 5, ADC_CMP1_GREATER_OR_EQUAL_TO, 0x800, 10);
  *          Means ADC will assert comparator 1 flag if channel 5 conversion result is 
  *          greater or equal to 0x800 for 10 times continuously.
  * \hideinitializer
  */                     
#define ADC_ENABLE_CMP1(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (ADC->CMP1 = ((u32ChNum) << ADC_CMP1_CMPCH_Pos) | \
                                                                   (u32Condition) | \
                                                                   ((u32Data) << ADC_CMP1_CMPDAT_Pos) | \
                                                                   ((u32MatchCount - 1) << ADC_CMP1_CMPMCNT_Pos) |\
                                                                   ADC_CMP1_ADCMPEN_Msk)  

/**
  * @brief Disable comparator 1
  * @param[in] adc Base address of ADC module
  * \hideinitializer
  */                          
#define ADC_DISABLE_CMP1(adc) ((adc)->CMP1 = 0)

/**
  * @brief Set ADC input channel. Enabled channel will be converted while ADC starts.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1...
  * @return None
  * @note PN020 series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert. ADC_CHEN_CHEN0_Msk
  * \hideinitializer
  */   
#define ADC_SET_INPUT_CHANNEL(adc, u32Mask) ((adc)->CHEN = (ADC->CHEN & ~0x1FF) | (u32Mask))

/**
  * @brief Start the A/D conversion.
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_START_CONV(adc) ((adc)->CTL |= ADC_CTL_SWTRG_Msk)

/**
  * @brief Stop the A/D conversion.
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_STOP_CONV(adc) ((adc)->CTL &= ~ADC_CTL_SWTRG_Msk)


/**
  * @brief Enable the A/D test mode.
  * @return None
  * \hideinitializer
  */
#define ADC_ENABLE_TESTMODE(adc) ((adc)->CTL2 |= ADC_CTL2_TESTMODE_Msk)

/**
  * @brief Disable the A/D test mode.
  * @return None
  * \hideinitializer
  */
#define ADC_DISABLE_TESTMODE(adc) ((adc)->CTL2 &= ~ADC_CTL2_TESTMODE_Msk)

/**
  * @brief Set the A/D clock division.
  * @return None
  * \hideinitializer
  */
#define ADC_SET_CLKDIV(adc,div)  ((adc)->CTL2 = ((adc)->CTL2 &= ~ADC_CTL2_CLKDIV_Msk) | div)

/**
  * @brief Set the A/D compare current control.
  * @return None
  * \hideinitializer
  */
#define ADC_SET_CMPCTL(adc,current)  ((adc)->CTL2 = ((adc)->CTL2 &= ~ADC_CTL2_CMPCTL_Msk) | current )


void ADC_Enable(ADC_T *adc);
void ADC_Disable(ADC_T *adc);
void ADC_Open(ADC_T *adc,
               uint32_t u32InputMode, 
               uint32_t u32OpMode,  
               uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
                   
void ADC_SeqModeEnable(ADC_T *adc, uint32_t u32SeqTYPE, uint32_t u32ModeSel);
void ADC_SeqModeTriggerSrc(ADC_T *adc, uint32_t u32SeqModeTriSrc1, uint32_t u32SeqModeTriSrc2);

/*@}*/ /* end of group PN020_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_ADC_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ADC_H__

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
