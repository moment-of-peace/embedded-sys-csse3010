/**   
 ******************************************************************************   
 * @file    mylib/s4402815_sysmon.c    
 * @author  YI LIU – 44028156 
 * @date    26/04/2016   
 * @brief   FreeRTOS system monitor 
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_sysmon_init() – intialise gpios
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4402815_sysmon.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern void s4402815_sysmon_init(void) {

	/*Configurate gpios*/
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable the A3, A4, A5 Clock */
  	__BRD_A3_GPIO_CLK();
	__BRD_A4_GPIO_CLK();
	__BRD_A5_GPIO_CLK();
	

	/* Configure the A3 pin as an output */
	GPIO_InitStructure.Pin = BRD_A3_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_A3_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	
	/* Configure the A4 pin as an output */
	GPIO_InitStructure.Pin = BRD_A4_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_A4_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	
	/* Configure the A5 pin as an output */
	GPIO_InitStructure.Pin = BRD_A5_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_A5_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

}
