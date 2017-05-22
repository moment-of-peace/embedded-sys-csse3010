/**   
 ******************************************************************************   
 * @file    mylib/s4402815_radio.h  
 * @author  YI LIU – s4402815  
 * @date    032016   
 * @brief   radio peripheral driver   
 *
 *			NOTE: REPLACE sxxxxxx with your student login.
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 * extern void s4402815_radio_init(void);
 * extern void s4402815_radio_setchan(unsigned char);
 * extern char s4402815_radio_getchan();
 * extern void s4402815_radio_settxaddress(unsigned char *);
 * extern void s4402815_radio_gettxaddress(unsigned char *);
 * void s4402815_radio_fsmprocessing();
 * extern void s4402815_TaskRadio(void);
 ******************************************************************************
 *
 ******************************************************************************   
 */

#ifndef S4402815_RADIO_H
#define S4402815_RADIO_H

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
uint8_t re_address[5];
uint8_t re_channel;
uint8_t tr_address[5];
uint8_t tr_channel;

struct Message {
	int error_state;
	char packet[32];
};

struct Msg_send {
	int8_t chan;
	char msg[32];
};

QueueHandle_t s4402815_QueueRadioMsg;
QueueHandle_t s4402815_QueueRadioTx;

/* External function prototypes -----------------------------------------------*/
extern void s4402815_radio_init(void);
extern void s4402815_radio_setchan(unsigned char);
extern char s4402815_radio_getchan();
extern void s4402815_radio_settxaddress(unsigned char *);
extern void s4402815_radio_gettxaddress(unsigned char *);
void s4402815_radio_fsmprocessing();
extern void s4402815_TaskRadio(void);

#endif


