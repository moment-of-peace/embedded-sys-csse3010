/**   
 ******************************************************************************   
 * @file    mylib/s4402815_joystick.c    
 * @author  YI LIU – 44028156 
 * @date    24/03/2016   
 * @brief   joystick peripheral driver   
 *	     REFERENCE:  
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_joystick_init() - Initialise joystick
 * s4402815_joystick_x_read() - read x value
 * s4402815_joystick_y_read() - read y value
 * s4402815_joystick_z_read() - read z value
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4402815_joystick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/**
  * @brief  initialise joystick
  * @param  None
  * @retval None
  */
extern void s4402815_joystick_init(void) {
	/* Enable Ax GPIO Clock */
	__BRD_A0_GPIO_CLK();
	__BRD_A1_GPIO_CLK();

	/* Configure A0 as analog input */
  	GPIO_InitStructure.Pin = BRD_A0_PIN;			//Set A0 pin
  	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
  	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister

  	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO
	
	/* Configure A1 as analog input */
  	GPIO_InitStructure.Pin = BRD_A1_PIN;			//Set A1 pin
  	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
  	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister

  	HAL_GPIO_Init(BRD_A1_GPIO_PORT, &GPIO_InitStructure);	//Initialise A1

	/* Configure A2 as analog input */
  	/*GPIO_InitStructure.Pin = BRD_A2_PIN;			//Set A2 pin
  	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
  	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister

  	HAL_GPIO_Init(BRD_A2_GPIO_PORT, &GPIO_InitStructure);	//Initialise A2
*/

	/* Enable ADC1 clock */
	__ADC1_CLK_ENABLE();

    /* Configure ADC1 */
    AdcHandle.Instance = (ADC_TypeDef *)(ADC1_BASE);						//Use ADC1
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
    AdcHandle.Init.ScanConvMode          = DISABLE;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.NbrOfDiscConversion   = 0;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = DISABLE;

    HAL_ADC_Init(&AdcHandle);		//Initialise ADC
	/* Configure ADC Channel */
	AdcChanConfig.Channel = BRD_A0_ADC_CHAN;							//Use AO pin
	AdcChanConfig.Rank         = 1;
    AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    AdcChanConfig.Offset       = 0;    

	HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);		//Initialise ADC channel
}

/**
  * @brief  read joystick x signal
  * @param  None
  * @retval None
  */
extern unsigned int s4402815_joystick_x_read(void) {

	unsigned int adc_value;

	/* Configure ADC Channel */
	AdcChanConfig.Channel = BRD_A0_ADC_CHAN;							//Use AO pin
	AdcChanConfig.Rank         = 1;
    AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    AdcChanConfig.Offset       = 0;    

	HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);		//Initialise ADC channel
	
	HAL_ADC_Start(&AdcHandle); // Start ADC conversion

	//Wait for ADC Conversion to complete
    while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
    adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));

	return adc_value;
}

/**
  * @brief  read joystick y signal
  * @param  None
  * @retval None
  */
extern unsigned int s4402815_joystick_y_read(void) {

	unsigned int adc_value;

	/* Configure ADC Channel */
	AdcChanConfig.Channel = BRD_A1_ADC_CHAN;							//Use A1 pin
	AdcChanConfig.Rank         = 1;
    AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    AdcChanConfig.Offset       = 0;    

	HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);		//Initialise ADC channel
	
	HAL_ADC_Start(&AdcHandle); // Start ADC conversion

	//Wait for ADC Conversion to complete
    while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
    adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));

	return adc_value;
}

/**
  * @brief  read joystick z signal
  * @param  None
  * @retval None
  */
extern unsigned int s4402815_joystick_z_read(void) {

	unsigned int adc_value = 0;

	/* Configure ADC Channel */
	AdcChanConfig.Channel = BRD_A2_ADC_CHAN;							//Use A2 pin
	AdcChanConfig.Rank         = 1;
    AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    AdcChanConfig.Offset       = 0;    

	HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);		//Initialise ADC channel
	
	HAL_ADC_Start(&AdcHandle); // Start ADC conversion

	//Wait for ADC Conversion to complete
    while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
    adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));
	adc_value++;
	return adc_value;
}
