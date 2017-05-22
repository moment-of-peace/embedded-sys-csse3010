/**   
 ******************************************************************************   
 * @file    mylib/s4402815_pantilt.c    
 * @author  YI LIU – 44028156 
 * @date    23/03/2016   
 * @brief   servo peripheral driver   
 *	    
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_pwm_init() – intialise pwm
 * s4402815_pwm_set(float pwm_period, float pwm_pulse)
 * s4402815_pwm_set(float, float) – set pwm
 * s4402815_TaskPanTilt(void* laserStatus)
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4402815_pantilt.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/**
  * @brief  Initialise pwm with 20ms period and 1.45ms pulse.
  * @param  None
  * @retval None
  */
extern void s4402815_pwm_init(void) {
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM_OC_InitTypeDef PWMConfig;
	TIM_HandleTypeDef TIM_Init;

	uint16_t PrescalerValue = 0;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Timer x clock enable */
	PWM_CLK();

	/* Enable the Dx Clock */
	PIN_CLK();
	PIN_CLK2();

	/* Configure the Dx pin with TIMx output*/
	GPIO_InitStructure.Pin = PWM_PIN;				//Pin
	GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
	GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	GPIO_InitStructure.Alternate = PWM_AF;	//Set alternate function to be timer 2
	HAL_GPIO_Init(PWM_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the Dy pin with TIMx output*/
	GPIO_InitStructure.Pin = PWM_PIN2;				//Pin
	GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
	GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	GPIO_InitStructure.Alternate = PWM_AF;	//Set alternate function to be timer 4
	HAL_GPIO_Init(PWM_PORT2, &GPIO_InitStructure);	//Initialise Pin

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

	/* Configure Timer2 settings */
	TIM_Init.Instance = PWM_TIM;					//Enable Timer 2
	TIM_Init.Init.Period = 20*500000/1000;			//Set for 20ms (50Hz) period
	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* PWM Mode configuration for Channel x - set pulse width*/
	PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
	PWMConfig.Pulse				= 1.45*500000/1000;		//1.45ms pulse width
	PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
	PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
	PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
	PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	/* Enable PWM for Timer x, channel x */
	HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CH);

	/* Start PWM */
	HAL_TIM_PWM_Start(&TIM_Init, PWM_CH);

	/* PWM Mode configuration for Channel y - set pulse width*/
	PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
	PWMConfig.Pulse				= 2*500000/1000;		//1.45ms pulse width
	PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
	PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
	PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
	PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	/* Enable PWM for Timer x, channel x */
	HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CH2);

	/* Start PWM */
	HAL_TIM_PWM_Start(&TIM_Init, PWM_CH2);

	/*Enable D7 clock to output laser signal*/
	__BRD_D7_GPIO_CLK();

	/* Configure the D7 pin as an output */
	GPIO_InitStructure.Pin = BRD_D7_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D7_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

}

/**
  * @brief  Set pwm with special period and pulse width.
  * @param  float period, float pulse
  * @retval None
  */
extern void s4402815_pwm_set(float pwm_period, float pwm_pulse) {
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM_OC_InitTypeDef PWMConfig;
	TIM_HandleTypeDef TIM_Init;

	uint16_t PrescalerValue = 0;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

	/* Configure Timer 2 settings */
	TIM_Init.Instance = PWM_TIM;					//Enable Timer 2
	TIM_Init.Init.Period = pwm_period * 500000/1000;			//Set for period
	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* PWM Mode configuration for Channel x - set pulse width*/
	PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
	PWMConfig.Pulse				= pwm_pulse * 500000/1000;		
	PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
	PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
	PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
	PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	/* Enable PWM for Timer 2, channel x */
	HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CH);

	/* Start PWM */
	HAL_TIM_PWM_Start(&TIM_Init, PWM_CH);

}

/**
  * @brief  set the angle of pan or tilt
  * @param  type 1 is for pan set, type 2 is for tilt set. int angle
  * @retval None
  */
extern void s4402815_pantilt_angle_write(int type, int angle) {
	
	// pan:1, tilt:2
	float pulse_width = 1.45;

	//set the limits of pan and tilt angle
	if (angle > 70) angle = 70;
	if (angle < -70) {
		if (type ==1) angle = -70;
		if (type ==2) angle = -60;
	}

	pulse_width += angle * 0.011;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM_OC_InitTypeDef PWMConfig;
	TIM_HandleTypeDef TIM_Init;

	uint16_t PrescalerValue = 0;

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

	if (type == 1) {
		/* Configure Timer settings */
		TIM_Init.Instance = PWM_TIM;					//Enable Timer 
		TIM_Init.Init.Period = 20 * 500000/1000;			//Set for period
		TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
		TIM_Init.Init.ClockDivision = 0;			//Set clock division
		TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
		TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

		/* PWM Mode configuration for Channel x - set pulse width*/
		PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
		PWMConfig.Pulse				= pulse_width * 500000/1000;		
		PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
		PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
		PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
		PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
		PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

		/* Enable PWM for Timer 3, channel x */
		HAL_TIM_PWM_Init(&TIM_Init);
		HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CH);

		/* Start PWM */
		HAL_TIM_PWM_Start(&TIM_Init, PWM_CH);
	} 

	else if (type == 2) {
		/* Configure Timer settings */
		TIM_Init.Instance = PWM_TIM;					//Enable Timer 
		TIM_Init.Init.Period = 20 * 500000/1000;			//Set for period
		TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
		TIM_Init.Init.ClockDivision = 0;			//Set clock division
		TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
		TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

		/* PWM Mode configuration for Channel y - set pulse width*/
		PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
		PWMConfig.Pulse				= (pulse_width - 0.22) * 500000/1000;		
		PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
		PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
		PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
		PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
		PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

		/* Enable PWM for Timer 3, channel x */
		HAL_TIM_PWM_Init(&TIM_Init);
		HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CH2);

		/* Start PWM */
		HAL_TIM_PWM_Start(&TIM_Init, PWM_CH2);
	}
}

/**
  * @brief  FreeRTOS task used to control servo and laser
  * @param  void
  * @retval None

  */
extern void s4402815_TaskPanTilt(void* laserStatus) {

	int status;
	pan_angle = 0;
	tilt_angle = 0;

	s4402815_QueuePan = xQueueCreate(5, sizeof(pan_angle));		//create queue to transfer pan values
	s4402815_QueueTilt = xQueueCreate(5, sizeof(tilt_angle));		//create queue to transfer pan values

	/* Create Semaphores */
	s4402815_SemaphoreLaser = xSemaphoreCreateBinary();
	s4402815_SemaphorePanLeft = xSemaphoreCreateBinary();
	s4402815_SemaphorePanRight = xSemaphoreCreateBinary();
	s4402815_SemaphoreTiltUp = xSemaphoreCreateBinary();
	s4402815_SemaphoreTiltDown = xSemaphoreCreateBinary();

	s4402815_pwm_init();

	for (;;) {

		/*receive laser semaphore to control on or off*/
		if (s4402815_SemaphoreLaser != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreLaser, 10 ) == pdTRUE ) {
				status = *((int*)laserStatus);
				if (status == 1)
					HAL_GPIO_WritePin(BRD_D7_GPIO_PORT, BRD_D7_PIN, 1 & 0x01);
				else if (status == 0)
					HAL_GPIO_WritePin(BRD_D7_GPIO_PORT, BRD_D7_PIN, 0 & 0x01);
			}
		}

		/*receive pan angle value from queue pan and set pan angle*/
		if (s4402815_QueuePan != NULL) {
	
			/* Check for item received - block atmost for 10 ticks */
			if (xQueueReceive( s4402815_QueuePan, &pan_angle, 10)) {
				s4402815_pantilt_angle_write(1, pan_angle);
			}
		}

		/*receive pan left semaphore to turn pan left*/
		if (s4402815_SemaphorePanLeft != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphorePanLeft, 30 ) == pdTRUE ) {
				pan_angle += 5;
				s4402815_pantilt_angle_write(1, pan_angle);
			}
		}

		/*receive pan right semaphore to turn pan right*/
		if (s4402815_SemaphorePanRight != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphorePanRight, 30 ) == pdTRUE ) {
				pan_angle -= 5;
				s4402815_pantilt_angle_write(1, pan_angle);
			}
		}

		/*receive tilt angle value from queue tilt and set tilt angle*/
		if (s4402815_QueueTilt != NULL) {
	
			/* Check for item received - block atmost for 10 ticks */
			if (xQueueReceive( s4402815_QueueTilt, &tilt_angle, 10)) {
				s4402815_pantilt_angle_write(2, tilt_angle);
			}
		}

		/*receive tilt up semaphore to turn tilt up*/
		if (s4402815_SemaphoreTiltUp != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreTiltUp, 30 ) == pdTRUE ) {
				tilt_angle -= 5;
				s4402815_pantilt_angle_write(2, tilt_angle);
			}
		}

		/*receive tilt down semaphore to turn tilt down*/
		if (s4402815_SemaphoreTiltDown != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreTiltDown, 30 ) == pdTRUE ) {
				tilt_angle += 5;
				s4402815_pantilt_angle_write(2, tilt_angle);
			}
		}
		vTaskDelay(20);
	}
}
