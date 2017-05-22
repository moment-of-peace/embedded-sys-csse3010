/**
  ******************************************************************************
  * @file    stage4/main.c
  * @author  YI LIU
  * @date    21022016
  * @brief   send and receive message
  *			 
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "s4402815_radio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CLOCK_PRESCALER		50000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int8_t packetbuffer[32];
uint8_t base_address[] = {0x7b, 0x56, 0x34, 0x12, 0x00};
unsigned char base_channel = 50;
int mainloop_counter = 0;

/* Private function prototypes -----------------------------------------------*/
void HardwareInit();
void packetaddr_set(void);
void packetbuff_init(void);

/**
  * @brief  Main program,
  * @param  None
  * @retval None
  */

int main(void) {

	/*int i;
	uint8_t current_channel;*/
    int input_counter=0;

	BRD_init();
	HardwareInit();
	
	/* Initialise radio FSM */ 
	s4402815_radio_init();
	s4402815_radio_setchan(base_channel);
	s4402815_radio_settxaddress(base_address);

	HAL_Delay(3000);

	while (1) {
		
		s4402815_radio_fsmprocessing();

		char RxChar;
		RxChar = debug_getc();
		if (RxChar != '\0') {
           mainloop_counter= -50;

			if (RxChar != 13) {
				packetbuffer[9 + input_counter] = RxChar;
				input_counter ++;

				if (input_counter >= 23){
					
					packetaddr_set();
					s4402815_radio_sendpacket( base_channel ,base_address, packetbuffer);
					s4402815_radio_setfsmrx();

					packetbuff_init();//clean packetbuff
					input_counter = 0;
				}
			}
			else {
				packetaddr_set();	
				s4402815_radio_sendpacket( base_channel ,base_address, packetbuffer);
				s4402815_radio_setfsmrx();
					
				packetbuff_init();//clean packetbuff
				input_counter = 0;
			}

        }



		mainloop_counter++;
		
				/*if (mainloop_counter >= 50) {
	
				mainloop_counter = 0;
				packetaddr_set();
				packetbuffer[9] = 'l';
				packetbuffer[10] = 'i';
				packetbuffer[11] = 'u';
				packetbuffer[12] = ' ';
				packetbuffer[13] = 'y';
				packetbuffer[14] = '1';
				packetbuffer[15] = '-';

				s4402815_radio_sendpacket( base_channel ,base_address, packetbuffer);
				s4402815_radio_setfsmrx();
				packetbuff_init();
              }
*/
			
		//print received message if has already received	  
		if(s4402815_radio_getrxstatus() == 1) {
			char packet_receive[32];
			s4402815_radio_getpacket(packet_receive);	

			int i;
			debug_printf("Receive from: ");
			for (i = 1; i < 5; i++ )
				debug_printf("%02x", packet_receive[i]);

			debug_printf(" ");

			for (i = 9; i < 32; i++ ) {
		
				if(packet_receive[i])
					debug_printf("%c", packet_receive[i]);		
			}
			debug_printf("\n\r");                
		}
		HAL_Delay(10);			
	}
}


/**
  * @brief Hardware Initialisation Function.
  * @param  None
  * @retval None
  */
void HardwareInit() {

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

}

/**
  *write address part of packetbuffer[]
  */
void packetaddr_set(void) {
	packetbuffer[0] = 0x20;
	packetbuffer[1] = 0x7b;
	packetbuffer[2] = 0x56;
	packetbuffer[3] = 0x34;
	packetbuffer[4] = 0x12;
	packetbuffer[5] = 0x56;
	packetbuffer[6] = 0x81;
	packetbuffer[7] = 0x02;
	packetbuffer[8] = 0x44;	
}

/**
  *set all elements in packetbuffer[] to 0
  */
void packetbuff_init(void) {
	int i;
	for(i =0; i<32;i++) {
		packetbuffer[i] = 0;
	}
}
