/**
  ******************************************************************************
  * @file    ex5_timer_interrupt.c 
  * @author  MDS
  * @date    02022015
  * @brief   Enable Timer 2 update compare interrupt. Use interrupt to flash
  *			 LED every second.
  *			 See Section 18 (TIM2), P576 of the STM32F4xx Reference Manual.
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4402815_lightbar.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConfig;

int count_interrupt = 0;	//increment each time a timer interrupt occurs
int write_value = 0;
float wave_peak;			//length of square wave peak
int wave_offpeak;		//length of square wave offpeak
int duty_cycle;
float timer_period;

/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void tim2_irqhandler (void);


/**
  * @brief  Main program - Timer 2 update compare interrupt.
  * @param  None
  * @retval None
  */
void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules

	unsigned int adc_value;

  	/* Main processing loop waiting for interrupt */
  	while (1) {

		HAL_ADC_Start(&AdcHandle); // Start ADC conversion

		//Wait for ADC Conversion to complete
	    while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
    	adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));

		wave_peak = adc_value * 19/4096;
		wave_offpeak = 18 - ((int)wave_peak);
		duty_cycle = adc_value * 100/4096;
		if (duty_cycle == 99) duty_cycle = 100;

		/* Print ADC conversion values and wave infomation */
		debug_printf("ADC Value: %X\t\tDuty cycle: %d\%\n\r", adc_value, duty_cycle);

		/*Set LED bar*/
		int LED_values[10] = {0};
		int i = 0;
		for (;i <= duty_cycle/10; i++)
			LED_values[i] = 1;
		if (duty_cycle == 0) LED_values[0] = 0;
		s4402815_lightbar_set(LED_values);
		
		BRD_LEDToggle();	//Toggle LED on/off
		HAL_Delay(500);	//Delay for 0.5s

	}
}

/**
  * @brief  Configure the hardware, 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	unsigned short PrescalerValue;

	GPIO_InitTypeDef  GPIO_InitStructure;

	BRD_LEDInit();		//Initialise onboard LED
	BRD_LEDOff();		//Turn off onboard LED
	
	/* Enable Timer 2, D10, A0 Clocks */
	__TIM2_CLK_ENABLE();
	__BRD_D10_GPIO_CLK();
	__BRD_A0_GPIO_CLK();

	s4402815_lightbar_init();	//Initialise LED bar

	/* Configure the D10 pin as an output */
	GPIO_InitStructure.Pin = BRD_D10_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D10_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Compute the prescaler value */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/500000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

  	/* Time base configuration */
	TIM_Init.Instance = TIM2;				//Enable Timer 2
  	TIM_Init.Init.Period = 500000/1000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM2_IRQn, (uint32_t)&tim2_irqhandler);  
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);

	/* Configure A0 as analog input */
  	GPIO_InitStructure.Pin = BRD_A0_PIN;			//Set A0 pin
  	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
  	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister

  	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	//Initialise AO

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
  * @brief  Timer 2 Interrupt handler
  * @param  None.
  * @retval None
  */
void tim2_irqhandler (void) {

	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	if (write_value == 1)
		timer_period = wave_peak;
	else
		timer_period = wave_offpeak;

	if (duty_cycle == 0){		
			write_value = 0;
			HAL_GPIO_WritePin(BRD_D10_GPIO_PORT, BRD_D10_PIN, write_value & 0x01);	//Write Digital 0 bit value
		}
	else if (duty_cycle == 100){
			write_value = 1;
			HAL_GPIO_WritePin(BRD_D10_GPIO_PORT, BRD_D10_PIN, write_value & 0x01);
		}
	else if (count_interrupt > timer_period) {

		/* Toggle D10 */		
		write_value = 1 - write_value;
		HAL_GPIO_WritePin(BRD_D10_GPIO_PORT, BRD_D10_PIN, write_value & 0x01);	//Write Digital 0 bit value
		count_interrupt = 0;
	}
		
	count_interrupt++;		//increment counter, when the interrupt occurs
	

}


