/**   
 ******************************************************************************   
 * @file    mylib/s4402815_sysmon.h  
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

#ifndef s4402815_SYSMON_H
#define s4402815_SYSMON_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define s4402815_LA_CHAN0_CLR() HAL_GPIO_WritePin(BRD_A3_GPIO_PORT, BRD_A3_PIN, 0 & 0x01)
#define s4402815_LA_CHAN0_SET() HAL_GPIO_WritePin(BRD_A3_GPIO_PORT, BRD_A3_PIN, 1 & 0x01)
#define s4402815_LA_CHAN1_CLR() HAL_GPIO_WritePin(BRD_A4_GPIO_PORT, BRD_A4_PIN, 0 & 0x01)
#define s4402815_LA_CHAN1_SET() HAL_GPIO_WritePin(BRD_A4_GPIO_PORT, BRD_A4_PIN, 1 & 0x01)
#define s4402815_LA_CHAN2_CLR() HAL_GPIO_WritePin(BRD_A5_GPIO_PORT, BRD_A5_PIN, 0 & 0x01)
#define s4402815_LA_CHAN2_SET() HAL_GPIO_WritePin(BRD_A5_GPIO_PORT, BRD_A5_PIN, 1 & 0x01)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* External function prototypes -----------------------------------------------*/

extern void s4402815_sysmon_init(void);

#endif
