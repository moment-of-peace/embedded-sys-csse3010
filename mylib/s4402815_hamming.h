/**
  ******************************************************************************
  * @file    s4402815_hamming.c 
  * @author  YI LIU
  * @date    042016
  * @brief   hamming decode and encode
  ******************************************************************************
  *  
  *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * extern void s4402815_laser_encode(char packet, int* code);
 * extern char s4402815_laser_decode(int* raw, float unit, int* lasertest);
 * extern void s4402815_error_insertion(char packet, int erro, int* code);
 * extern void s4402815_hamenc(int value);
 * extern int s4402815_hamdec(uint16_t codedword);
 ******************************************************************************   
 */;

#ifndef s4402815_PANTILT_H
#define s4402815_PANTILT_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* External function prototypes -----------------------------------------------*/
extern void s4402815_laser_encode(char packet, int* code);
extern char s4402815_laser_decode(int* raw, float unit, int* lasertest);
extern void s4402815_error_insertion(char packet, int erro, int* code);
extern void s4402815_hamenc(int value);
extern int s4402815_hamdec(uint16_t codedword);

#endif
