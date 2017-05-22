/**   
 ******************************************************************************   
 * @file    mylib/s4402815_crc.c    
 * @author  YI LIU – 44028156 
 * @date    17/05/2016   
 * @brief   crc 16 calculation
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * extern uint16_t s4402815_crc(char *str, int length)
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define POLY 0x1021

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
  * @brief  crc calculation
  * @param  pointer to target string and the length of string
  * @retval crc code

  */
extern uint16_t s4402815_crc(char *str, int length) {
	uint16_t crc_value = 0;
	
	while(length > 0) {
		crc_value ^= *str++ << 8;
		for(int i = 0; i < 8; i++) {
			if(crc_value & 0x8000)
				crc_value = (crc_value << 1) ^ POLY;
			else
				crc_value = (crc_value << 1);
		}
		length--;
	}
	return crc_value;
}
