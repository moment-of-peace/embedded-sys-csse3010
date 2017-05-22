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

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "string.h"
#include "s4402815_radio.h"
#include "s4402815_cli.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define IDLE_STATE  0
#define TX_STATE  1
#define RX_STATE  2
#define WAIT_STATE  3

int send_flag = 0;
int receive_flag = 0;
char channel;
unsigned char channel_address[5];
unsigned char channel_package[32];
int fsm_state = IDLE_STATE;	
uint8_t receive_buffer[32];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//initialse radio
extern void s4402815_radio_init() {

	/* Initialise radio FSM */ 
	radio_fsm_init();

	/* set radio FSM state to IDLE */
	radio_fsm_setstate(RADIO_FSM_IDLE_STATE);

}

//set channel for radio
extern void s4402815_radio_setchan(unsigned char channel) {
    
    uint8_t chara;
    chara = channel;

	nrf24l01plus_wr(NRF24L01P_RF_CH, chara); 	// Select RF channel
	nrf24l01plus_wr(NRF24L01P_RF_SETUP, 0x06);   							// TX_PWR:0dBm, Datarate:1Mbps

}

//get radio channel
extern char s4402815_radio_getchan(){

	char channel;

	radio_fsm_register_read(NRF24L01P_RF_CH, &channel);	//Read channel

	return channel;

}

//set radio addr
extern void s4402815_radio_settxaddress(unsigned char *addr) {	

	nrf24l01plus_wb(NRF24L01P_WRITE_REG | NRF24L01P_TX_ADDR, addr, 5);		  // Writes TX_Address to nRF24L01
	nrf24l01plus_wb(NRF24L01P_WRITE_REG | NRF24L01P_RX_ADDR_P0, addr, 5);	//NRF24L01P_TX_ADR_WIDTH);

}

//get radio addr
extern void s4402815_radio_gettxaddress(unsigned char *addr) {

	radio_fsm_buffer_read(NRF24L01P_TX_ADDR, addr, 5);

}
  
extern void s4402815_radio_sendpacket(char chara, unsigned char *addr, unsigned char *txpack){

	channel = chara;
	memcpy(channel_address, addr, 5);		
	memcpy(channel_package, txpack, 32);	

	send_flag = 1;	

}

//change radio state
void s4402815_radio_fsmprocessing(){

	switch(fsm_state) {

		case IDLE_STATE:	//Idle state for reading current channel

			/* Get current channel , if radio FSM is in IDLE State */
			if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

				//debug_printf("idle\n");
	
				if(send_flag == 1){
				
					send_flag = 0;

					//debug_printf("idle1\n");

					fsm_state = TX_STATE;

				}

				else {
				
                    if(!receive_flag) {
						//debug_printf("idle2\n");
						fsm_state = RX_STATE;	//Set next state as RX state.
					}
				}

			} else {

					/* if error occurs, set state back to IDLE state */
					debug_printf("ERROR: Radio FSM not in Idle state\n\r");
					radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
			}

			break;

		case TX_STATE:	//TX state for writing packet to be sent.

			/* Put radio FSM in TX state, if radio FSM is in IDLE state */
			if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

				if (radio_fsm_setstate(RADIO_FSM_TX_STATE) == RADIO_FSM_ERROR) {
					debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
					HAL_Delay(100);
				} else {

					debug_printf("Sending via radio...\n\r");
					//debug_printf("tx1: %d, %d\n", radio_fsm_getstate(), fsm_state);

					/* Send packet - radio FSM will automatically go to IDLE state, after write completes. */
					radio_fsm_write(channel_package);

					//for (int i = 0; i <16; i++) debug_printf("tx check: %c", channel_package[i]);
					debug_printf("tx\n");	
					fsm_state = IDLE_STATE;		//set next state as Waiting state

					//debug_printf("tx2: %d, %d\n", radio_fsm_getstate(), fsm_state);
				}
			} else {

					/* if error occurs, set state back to IDLE state */
					debug_printf("ERROR: Radio FSM not in Idle state\n\r");
					radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
			}

			break;

		case RX_STATE:	//RX state for putting radio transceiver into receive mode.

				/* Put radio FSM in RX state, if radio FSM is in IDLE or in waiting state */
				if ((radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) || (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE)) {
					//debug_printf("after rx1: %d, %d\n", radio_fsm_getstate(), fsm_state);
					if (radio_fsm_setstate(RADIO_FSM_RX_STATE) == RADIO_FSM_ERROR) {
						debug_printf("ERROR: Fail to set RX state\n\r");
					
					} else {
						//debug_printf("rx1: %d, %d\n", radio_fsm_getstate(), fsm_state);
						fsm_state = WAIT_STATE;		//set next state as Waiting state
						
						//debug_printf("rx2: %d, %d\n", radio_fsm_getstate(), fsm_state);
					}
				} else {
	
						/* if error occurs, set state back to IDLE state */
						debug_printf("ERROR\n\r");
						radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
				}


				break;

		case WAIT_STATE:	//Waiting state for reading received packet.

				//debug_printf("for wait: %d, %d\n", radio_fsm_getstate(), fsm_state);
				/* Check if radio FSM is in WAITING STATE */
				if (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE) {
					//debug_printf("w1: %d, %d\n", radio_fsm_getstate(), fsm_state);

					/* Check for received packet and display  */
					if (radio_fsm_read(receive_buffer) == RADIO_FSM_DONE) {

                        receive_flag = 1;			
						radio_fsm_setstate(RADIO_FSM_IDLE_STATE);	//Set Radio to IDLE state.
						fsm_state = IDLE_STATE;

						//debug_printf("w2: %d, %d\n", radio_fsm_getstate(), fsm_state);
                      	break;
					}
						
				}	       

				if(send_flag == 1){
					radio_fsm_setstate(RADIO_FSM_IDLE_STATE);	//Set Radio to IDLE state.

					fsm_state = IDLE_STATE;

					//debug_printf("w3: %d, %d\n", radio_fsm_getstate(), fsm_state);
				}
				else
                HAL_Delay(50);
				
				break;

	}		

}

//get radio state
int s4402815_radio_getrxstatus() {

    return receive_flag;

}

void s4402815_radio_getpacket(int8_t *packetbuffer) {
	int i;
	for(i=0; i<32; i++){
		packetbuffer[i] = receive_buffer[i];
		receive_buffer[i] = 0;
    }

	receive_flag = 0;
}
	
void s4402815_radio_setfsmrx() {

	receive_flag = 0;

}

/**
  * @brief  FreeRTOS driver for radio
  * @param  void
  * @retval None

  */
extern void s4402815_TaskRadio(void) {

	struct Message rec_msg;		//struct used to store received message
	struct Msg_send sen_msg;	//struct used to store message received from other task or functions
	int send_flag = 0;

	//Initialise radio address 
	re_address[0] = 0x31;
	re_address[1] = 0x34;
	re_address[2] = 0x22;
	re_address[3] = 0x11;
	re_address[4] = 0x00;

	//initialise radio channel
	re_channel = 43;

	fsm_state = IDLE_STATE;
	s4402815_radio_init();
	s4402815_radio_setchan(re_channel);
	s4402815_radio_settxaddress(re_address);

	s4402815_QueueRadioMsg = xQueueCreate(5, sizeof(rec_msg));	//create queue to transfer received message
	s4402815_QueueRadioTx = xQueueCreate(5, sizeof(sen_msg));	//create queue to receive message to be transferred

	for (;;) {
		// shift state
		switch(fsm_state) {

		case IDLE_STATE:	//Idle state for reading current channel

			/* Get current channel , if radio FSM is in IDLE State */
			if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

				//debug_printf("idle\n");
				if (send_flag == 1) {
					//set channel and address
					s4402815_radio_setchan(re_channel);
					s4402815_radio_settxaddress(re_address);
					send_flag = 0;
				}
				
				if (s4402815_QueueRadioTx != NULL) {
					//Check for item received - block atmost for 10 ticks
					if (xQueueReceive( s4402815_QueueRadioTx, &sen_msg, 10)) {
						//set channel and address
						s4402815_radio_setchan(tr_channel);
						s4402815_radio_settxaddress(tr_address);

						// next state is tx
						fsm_state = TX_STATE;
					}
					else
						fsm_state = RX_STATE;
				}
				else {
				
						fsm_state = RX_STATE;	//Set next state as RX state.
				}

			} else {

				/* if error occurs, set state back to IDLE state */
				debug_printf("ERROR: Radio FSM not in Idle state\n\r");
				radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
			}

			break;

		case TX_STATE:	//TX state for writing packet to be sent.

			/* Put radio FSM in TX state, if radio FSM is in IDLE state */
			if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

				if (radio_fsm_setstate(RADIO_FSM_TX_STATE) == RADIO_FSM_ERROR) {
					debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
				} else {

					//check the type 
					if (sen_msg.msg[0] == 0x32) send_flag =1;

					/* Send packet - radio FSM will automatically go to IDLE state, after write completes. */
					radio_fsm_write(sen_msg.msg);

					fsm_state = IDLE_STATE;		//set next state as Waiting state
				}
			} else {

				/* if error occurs, set state back to IDLE state */
				debug_printf("ERROR: Radio FSM not in Idle state\n\r");
				radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
			}

			break;

		case RX_STATE:	//RX state for putting radio transceiver into receive mode.

			/* Put radio FSM in RX state, if radio FSM is in IDLE or in waiting state */
			if ((radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) || (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE)) {
				if (radio_fsm_setstate(RADIO_FSM_RX_STATE) == RADIO_FSM_ERROR) {
					debug_printf("ERROR: Fail to set RX state\n\r");
				
				} else {
					fsm_state = WAIT_STATE;		//set next state as Waiting state
				}
			} else {
	
				/* if error occurs, set state back to IDLE state */
				debug_printf("ERROR\n\r");
				radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
			}


			break;

		case WAIT_STATE:	//Waiting state for reading received packet.

			/* Check if radio FSM is in WAITING STATE */
			if (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE) {

				/* Check for received packet and display  */
				if (radio_fsm_read(rec_msg.packet) == RADIO_FSM_DONE) {
					rec_msg.error_state = 0;

					//Send received message to the front of the queue - wait atmost 10 ticks
					if (s4402815_QueueRadioMsg != NULL) {
						if( xQueueSendToFront(s4402815_QueueRadioMsg, (void *)&rec_msg, ( portTickType ) 20 ) != pdPASS ) {
							debug_printf("Failed to post the message in radio task, after 10 ticks.\n\r");
						}
					}

					radio_fsm_setstate(RADIO_FSM_IDLE_STATE);	//Set Radio to IDLE state.
					fsm_state = IDLE_STATE;

                   	break;
				}
				else {
					radio_fsm_setstate(RADIO_FSM_IDLE_STATE);	//Set Radio to IDLE state.
					fsm_state = IDLE_STATE;

					break;
				}		
			}	       

			else {
				radio_fsm_setstate(RADIO_FSM_IDLE_STATE);	//Set Radio to IDLE state.
				fsm_state = IDLE_STATE;
			}
				
			break;

		}
		//task delay
		vTaskDelay(60);
	}
}

