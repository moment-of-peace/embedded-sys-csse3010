/**
  ******************************************************************************
  * @file    stage1/main.c 
  * @author  YI LIU
  * @date    10-March-2016
  * @brief   Prac 1 Template C main file - BCD timer and press counter.
  *			 NOTE: THIS CODE IS PSEUDOCODE AND DOES NOT COMPILE. 
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex11_character
  ******************************************************************************
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
uint16_t counter_value = 64;
uint16_t press_counter_val = 0;

int delay_length = 1000;	//Set the length of delay
int press_count = 0;		//Times of speed increase

/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void exti_a2_interrupt_handler(void);

/**
  * @brief  Main program - timer and press counter.
  * @param  None
  * @retval None
  */

void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	
	HAL_Delay(4000);	//Delay for preperation
	
	/* Main processing loop */
  	while (1) {
		
		debug_printf("Counter value: %d\n\r", counter_value);	//Print debug message		

		/****************** Display counter. ***************/
		/* First, turn off each LED light bar segment
			write 0 to D0
			Write 0 to D1
			....
			Write 0 to D6

		*/
		s4402815_lightbar_write(counter_value);

		counter_value--;	//Decrement counter
		BRD_LEDToggle();	//Toggle LED on/off

		/*Return to 64 when counter value is 0*/
		if(counter_value < 0 || counter_value > 64)
			counter_value = 64;

		/* Toggle 'Keep Alive Indicator' BLUE LED */
    	HAL_Delay(delay_length);		//Delay

	}
}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;	

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Initialise LEDBar*/
	s4402815_lightbar_init();
	
	/* Configure A2 interrupt for Prac 1, Task 2 or 3 only */
  	__BRD_A2_GPIO_CLK();		// Enable A2 clock

	/* Set priority of external GPIO Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(BRD_A2_EXTI_IRQ, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0

	//Enable external GPIO interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_A2_EXTI_IRQ, (uint32_t)&exti_a2_interrupt_handler);  
	NVIC_EnableIRQ(BRD_A2_EXTI_IRQ);

  	/* Configure A2 pin as pull down input */
	GPIO_InitStructure.Pin = BRD_A2_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;		//interrupt Mode
  	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_A2_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

}

/**
  * @brief  exti_a2 GPIO Interrupt handler
  * @param  None.
  * @retval None
  */
void exti_a2_interrupt_handler(void) {

	HAL_Delay(300);		//Debouncing

	HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);				//Clear A2 pin external interrupt flag

	BRD_LEDToggle();
	HAL_Delay(200);
	BRD_LEDToggle();

	press_count++;		//increment press count, everytime the interrupt occurs
	debug_printf("Triggered : %d\n\r", press_count);    //Print press count value

	/* Speed up the counter by reducing the delay value */
	delay_length = (int) (delay_length/2);
}
