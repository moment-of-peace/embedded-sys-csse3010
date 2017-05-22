/**   
 ******************************************************************************   
 * @file    mylib/s4402815_joystick.h  
 * @author  YI LIU – 44028156   
 * @date    24/03/2016   
 * @brief   joystick peripheral driver   
 *	     REFERENCE: np2 quicksheet   
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4402815_joystick_init() - Initialise joystick
 * s4402815_joystick_x_read() - read x value
 * s4402815_joystick_y_read() - read y value
 * s4402815_joystick_z_read() - read z value
 ******************************************************************************   
 */

#ifndef s4402815_JOYSTICK_H
#define s4402815_JOYSTICK_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConfig;
GPIO_InitTypeDef  GPIO_InitStructure;
/* External function prototypes -----------------------------------------------*/

extern void s4402815_joystick_init(void);
extern unsigned int s4402815_joystick_x_read(void);
extern unsigned int s4402815_joystick_y_read(void);
extern unsigned int s4402815_joystick_z_read(void);
#endif

