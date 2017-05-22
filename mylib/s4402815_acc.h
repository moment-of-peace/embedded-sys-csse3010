/**   
 ******************************************************************************   
 * @file    mylib/s4402815_acc.h 
 * @author  YI LIU – 44028156 
 * @date    15/05/2016   
 * @brief   a freertos driver for accelerometer
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 * extern void s4402815_TaskAcc(void)
 ******************************************************************************
 * 
 ******************************************************************************   
 */

#ifndef s4402815_ACC_H
#define s4402815_ACC_H

/* Includes ------------------------------------------------------------------*/
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
I2C_HandleTypeDef  I2CHandle;

/* Private define ------------------------------------------------------------*/
#define MMA8452Q_ADDRESS	0x3A

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct Accvalue {
	int x_value;
	int y_value;
	int z_value;
	char pl;
	char bf;
};

QueueHandle_t s4402815_QueueAccValue;

SemaphoreHandle_t s4402815_SemaphoreAccOn;
SemaphoreHandle_t s4402815_SemaphoreAccOff;

/* External function prototypes -----------------------------------------------*/

extern void s4402815_TaskAcc(void);

#endif
