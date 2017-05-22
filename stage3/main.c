/**
  ******************************************************************************
  * @file    stage3/main.c 
  * @author  YI LIU - 44028156
  * @date    23/03/2016
  * @brief   
  *		     
  *			 
  *			 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "debug_printf.h"
#include "stm32f4xx_hal_conf.h"
#include "s4402815_pantilt.h"
#include "s4402815_joystick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

float pulse_width = 1.45;
char dir = 'r';

void Hardware_init(void);
void exti_A2_irqhandler(void);

#define PRINTF_REFLECT			//Comment out to use putc otherwise printf.

/**
  * @brief  Main program - flashes onboard blue LED
  * @param  None
  * @retval None
  */
void main(void) {

	BRD_init();			//Initalise NP2 board
	Hardware_init();	//Initalise hardware modules
	HAL_Delay(3000);

	char RxChar = 'r';
	unsigned int adc_val;

	debug_printf("pulse width = 1.45 ms  %c\n", RxChar);
  	
	/* Main processing loop */
    while (1) {

		/* Receive characters using getc */
		RxChar = debug_getc();

		/* Check if character is not Null */
		if (RxChar != '\0') {

#ifdef PRINTF_REFLECT
			debug_printf("%c", RxChar);		//reflect byte using printf - must delay before calling printf again.
#else
			debug_putc(RxChar);				//reflect byte using putc - puts character into buffer
			debug_flush();					//Must call flush, to send character
#endif
		}

		if (RxChar != '\0') dir = RxChar;

		adc_val = s4402815_joystick_x_read();
		

		if (adc_val < 100){
			if (pulse_width < 2.33) pulse_width += 0.004;
		
		s4402815_pwm_set(20, pulse_width);
		HAL_Delay(40);
		}

		else if (adc_val > 4000){
			if (pulse_width > 0.57) pulse_width -= 0.002;
		
		s4402815_pwm_set(20, pulse_width);
		HAL_Delay(40);
		}

		else {
    	BRD_LEDToggle();	//Toggle 'Alive' LED on/off	
		HAL_Delay(1000);	//Delay 100ms.
		}

	}

}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	BRD_LEDInit();	//Initialise Blue LED
	BRD_LEDOff();	//Turn off Blue LED
	s4402815_pwm_init();	//Initialise pwm
	

	GPIO_InitTypeDef  GPIO_InitStructure;
  	
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

  	/* Enable A2 clock */
  	__BRD_A2_GPIO_CLK();

	/* Set priority of external GPIO Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(BRD_A2_EXTI_IRQ, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0

	//Enable external GPIO interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_A2_EXTI_IRQ, (uint32_t)&exti_A2_irqhandler);  
	NVIC_EnableIRQ(BRD_A2_EXTI_IRQ);

  	/* Configure A2 pin as pull down input */
	GPIO_InitStructure.Pin = BRD_A2_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
  	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_A2_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	s4402815_joystick_init();
}


/**
  * @brief  NP2_A2_EXTI Interrupt handler - see netduinoplus2.h
  * @param  None.
  * @retval None
  */
void exti_A2_irqhandler(void) {
	
	HAL_Delay(300);		//Debouncing

	HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);				//Clear A2 pin external interrupt flag

	if (dir == 'r') {
		if (pulse_width > 0.6)
			pulse_width -= 0.11;
		debug_printf("Direction: right");
	}

	if (dir == 'l') {
		if (pulse_width < 2.3)
			pulse_width += 0.11;
		debug_printf("Direction: left ");
	}
	
	s4402815_pwm_set(20, pulse_width);
    
	BRD_LEDToggle();
	debug_printf("\t Pulse width = %.3f ms\n\r", dir, pulse_width);    //Print pulse_width value

}
