/**   
 ******************************************************************************   
 * @file    mylib/s4402815_ledbar.h  
 * @author  YI LIU – 44028156   
 * @date    10/03/2016   
 * @brief   LED Light Bar peripheral driver   
 *	     REFERENCE: LEDLightBar_datasheet.pdf   
 *
 *			NOTE: REPLACE s4402815 with your student login.
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_ledbar_init() – intialise LED Light BAR
 * s4402815_ledbar_set() – set LED Light BAR value
 * s4402815_TaskLightBar( void ) -- led bar freertos dirver task
 ******************************************************************************   
 */

#ifndef s4402815_LIGHTBAR_H
#define s4402815_LIGHTBAR_H

/* Includes ------------------------------------------------------------------*/
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External function prototypes -----------------------------------------------*/
struct dualtimer_msg {	/* Message consists of sequence number and payload string */
	char type;
	unsigned char timer_value;
};

QueueHandle_t s4402815_QueueLightBar;	/* Queue used to transfer led value*/

extern void s4402815_lightbar_init(void);
extern void s4402815_lightbar_write(unsigned short value);
extern void s4402815_lightbar_set(int[]);
extern void s4402815_TaskLightBar( void );
#endif

