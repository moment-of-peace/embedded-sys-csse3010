/**   
 ******************************************************************************   
 * @file    mylib/s4402815_pantilt.h  
 * @author  YI LIU – 44028156   
 * @date    23/03/2016   
 * @brief   servo peripheral driver   
 *	     REFERENCE: np2 quicksheet   
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_pwm_init() – intialise pwm
 * s4402815_pwm_set() – set pwm wav parameter
 * extern void s4402815_pantilt_angle_write(int, int)
 * s4402815_TaskPanTilt() - FreeRTOS task used to control servo and laser
 ******************************************************************************   
 */

#ifndef s4402815_PANTILT_H
#define s4402815_PANTILT_H

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

#define PIN_CLK() __BRD_D5_GPIO_CLK()
#define PWM_PIN BRD_D5_PIN
#define PWM_PORT BRD_D5_GPIO_PORT
#define PWM_CLK() __TIM4_CLK_ENABLE()
#define PWM_TIM TIM4
#define PWM_AF GPIO_AF2_TIM4
#define PWM_CH TIM_CHANNEL_3

#define PIN_CLK2() __BRD_D6_GPIO_CLK()
#define PWM_PIN2 BRD_D6_PIN
#define PWM_PORT2 BRD_D6_GPIO_PORT
#define PWM_CLK2() __TIM2_CLK_ENABLE()
#define PWM_TIM2 TIM2
#define PWM_AF2 GPIO_AF1_TIM4
#define PWM_CH2 TIM_CHANNEL_4

QueueHandle_t s4402815_QueuePan;
QueueHandle_t s4402815_QueueTilt;

SemaphoreHandle_t s4402815_SemaphoreLaser;
SemaphoreHandle_t s4402815_SemaphorePanLeft;
SemaphoreHandle_t s4402815_SemaphorePanRight;
SemaphoreHandle_t s4402815_SemaphoreTiltUp;
SemaphoreHandle_t s4402815_SemaphoreTiltDown;
SemaphoreHandle_t s4402815_SemaphoreBox;

int pan_angle;
int tilt_angle;

extern void s4402815_pwm_init(void);
extern void s4402815_pwm_set(float, float);
extern void s4402815_pantilt_angle_write(int, int);
extern void s4402815_TaskPanTilt(void* laserStatus);
#endif

