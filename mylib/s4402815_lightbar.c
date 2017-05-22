/**   
 ******************************************************************************   
 * @file    mylib/s4402815_ledbar.c    
 * @author  YI LIU – 44028156 
 * @date    03/03/2016   
 * @brief   LED Light Bar peripheral driver   
 *	     REFERENCE: LEDLightBar_datasheet.pdf   
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_lightbar_init() – intialise LED Light BAR
 * s4402815_lightbar_write() – set LED Light BAR value
 * s4402815_lightbar_set() – set every LED
 * s4402815_TaskLightBar( void ) -- led bar freertos dirver task
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4402815_lightbar.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


extern void s4402815_lightbar_set(int values[]) {
	
	/* Set D0, D1 .... D9 */
	HAL_GPIO_WritePin(BRD_D0_GPIO_PORT, BRD_D0_PIN, values[0] & 0x01);	//Write Digital 0 bit value 0
	HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, values[1] & 0x01);	//Write Digital 1 bit value 1
	HAL_GPIO_WritePin(BRD_D2_GPIO_PORT, BRD_D2_PIN, values[2] & 0x01);	//Write Digital 2 bit value 2
	HAL_GPIO_WritePin(BRD_D3_GPIO_PORT, BRD_D3_PIN, values[3] & 0x01);	//Write Digital 3 bit value 3
	HAL_GPIO_WritePin(BRD_D4_GPIO_PORT, BRD_D4_PIN, values[4] & 0x01);	//Write Digital 4 bit value 4
}

/**
  * @brief  Initialise LEDBar.
  * @param  None
  * @retval None
  */
extern void s4402815_lightbar_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable the D0, D1 .... D4 Clock */
  	__BRD_D0_GPIO_CLK();
	__BRD_D1_GPIO_CLK();
	__BRD_D2_GPIO_CLK();
	__BRD_D3_GPIO_CLK();
	__BRD_D4_GPIO_CLK();

	/* Configure the D0 pin as an output */
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	
	/* Configure the D1 pin as an output */
	GPIO_InitStructure.Pin = BRD_D1_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	
	/* Configure the D2 pin as an output */
	GPIO_InitStructure.Pin = BRD_D2_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D2_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	
	/* Configure the D3 pin as an output */
	GPIO_InitStructure.Pin = BRD_D3_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D3_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	
	/* Configure the D4 pin as an output */
	GPIO_InitStructure.Pin = BRD_D4_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D4_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	
	int origin_values[10] = {0};
	s4402815_lightbar_set(origin_values);
	
}

/**
  * @brief  Set the LED Bar GPIO pins high or low, depending on the bit of ‘value’
  *         i.e. value bit 0 is 1 – LED Bar 0 on
  *          value bit 1 is 1 – LED BAR 1 on
  *
  * @param  value
  * @retval None
  */
	
extern void s4402815_lightbar_write(unsigned short value) {

	int led_values[10] = { 0 };

	//extract every bit of received value
	for (int i = 0; i < 10; i++) {
		led_values[i] = (value >> i) & 0x0001;
	}

	//after extracting bits, set led bar
	s4402815_lightbar_set(led_values);
}

/**
  * @brief  Receiver Task. Used to receive messages.
  * @param  None
  * @retval None
  */
void s4402815_TaskLightBar( void ) {

	unsigned short left_value = 0;
	unsigned short right_value = 0;
	unsigned short entire_value = 0;

	struct dualtimer_msg RecvMessage;

	s4402815_QueueLightBar = xQueueCreate(5, sizeof(RecvMessage));		/* Create queue of length 10 Message items */

	//initialise led bar
	s4402815_lightbar_init();	

	for (;;) {

		if (s4402815_QueueLightBar != NULL) {	/* Check if queue exists */

			/* Check for item received - block atmost for 10 ticks */
			if (xQueueReceive( s4402815_QueueLightBar, &RecvMessage, 10 )) {

				/* display received item */
				if (RecvMessage.type == 'r')
					// the value of right part
					right_value = RecvMessage.timer_value;

				if (RecvMessage.type == 'l')
					// the value of left part
					left_value = RecvMessage.timer_value;
				// the entire value
				if (RecvMessage.type == 'e')
					s4402815_lightbar_write(RecvMessage.timer_value);
				else
					s4402815_lightbar_write((left_value << 5) | right_value);
				
        	}
		}	

		/* Delay for 20ms */
		vTaskDelay(20);
	}
}

