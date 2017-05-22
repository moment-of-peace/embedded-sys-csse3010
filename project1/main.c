/**
  ******************************************************************************
  * @file    project1/main.c 
  * @author  YI LIU - 44028156
  * @date    10/04/2016
  * @brief   a complux communication system contains radio communication, laser 
  *		     optical communication and servo controll		 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "s4402815_pantilt.h"
#include "s4402815_joystick.h"
#include "s4402815_radio.h"
#include "s4402815_hamming.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;

int lasertest[70] = {0};
char keyboard_input = 'w'; //store the value of keyboard input
int pan_angle = 0;
int tilt_angle = 68;
int laser_send_flag = 0;		//a signal show whether the laser signal is sent
int laser_receive_flag = 0;		//a signal show whether the laser signal is received
int laser_receive_counter = 0;
int laser_send[44] = {0};		//store the message which will be sent by laser
int tim4_counter = 0;
char laser_chara;
int tim3_period = 20000;
char laser_receive_chara;
int unit = 0;					//laser signal unit length
double offset =0;				//capture distance
int pb_flag = 1;

int8_t packetbuffer[32];		//store the message which will be sent by radio
uint8_t base_address[] = {0x78, 0x56, 0x34, 0x12, 0x00};
int laser_receive[22] = {0};
unsigned char base_channel = 43;
int mainloop_counter = 0;

/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void tim3_irqhandler (void);
void tim4_irqhandler (void);
void exti_PB_irqhandler(void);

void packetaddr_set(void);
void packetbuff_init(void);

#define PRINTF_REFLECT			//Comment out to use putc otherwise printf.

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void main(void) {

	char RxChar = 'w';
	unsigned int x_signal;
	unsigned int y_signal;
	int loop_counter = 0;
	int input_flag = 0;
	int error_insert = 0;
	char laser_chara_error;

	int input_counter=0;

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	
	/* Initialise radio FSM */ 
	s4402815_radio_init();
	s4402815_radio_setchan(base_channel);
	s4402815_radio_settxaddress(base_address);

	s4402815_pantilt_angle_write(2, tilt_angle);
	//HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1 & 0x01);

	HAL_Delay(3000);

  	while (1) {
	
		//fsm state shift
		s4402815_radio_fsmprocessing();

		//procssing received laser signal
		if (laser_receive_flag == 1) {
			//a test laser receive[]
			for (int i = 0; i < 22; i++ ) { 
					debug_printf("%d ", laser_receive[i]);		
			}
			debug_printf("\n\r");
			debug_printf("unit origin :%d\n", (laser_receive[1] - laser_receive[0]) / 2);
			unit = (laser_receive[1] - laser_receive[0]) / 2;
			debug_printf("unit1: %d\n", unit);
			if (unit < 0) unit = unit + tim3_period/2;
			debug_printf("unit2: %d\n", unit);
			
			laser_receive_chara = s4402815_laser_decode(laser_receive, (laser_receive[1] - laser_receive[0]) / 2, lasertest);
			debug_printf("Receive from laser: %d\n", laser_receive_chara);
			laser_receive_flag = 0;
			for (int i = 0; i < 70; i++ ) {
					debug_printf("%d ", lasertest[i]);		
			}
			debug_printf("\n\r");
		}

		/* Receive characters using getc */
		RxChar = debug_getc();

		/* Check if character is not Null */
		if (RxChar != '\0') {

#ifdef PRINTF_REFLECT
			//debug_printf("%c", RxChar);		//reflect byte using printf - must delay before calling printf again.
#else
			debug_putc(RxChar);				//reflect byte using putc - puts character into buffer
			debug_flush();					//Must call flush, to send character
#endif
		}

		//set the input model
		if (RxChar != '\0') keyboard_input = RxChar;
		if (keyboard_input == '@') input_flag = 1;
		if (keyboard_input == '#') input_flag = 2;
		if (keyboard_input == '!') input_flag = 3;

		//in model 1, message will be transmitted by radio
		if (input_flag == 1) {
			if (keyboard_input != 13) {
				packetbuffer[8 + input_counter] = RxChar;
				input_counter ++;

				//send message when the input message length is over the limit
				if (input_counter >= 23){
					packetaddr_set();
					s4402815_radio_sendpacket(base_channel ,base_address, packetbuffer);
					s4402815_radio_setfsmrx();

					input_counter = 0;
					input_flag = 0;
					packetbuff_init();
				}

			}else {
				packetaddr_set();	
				s4402815_radio_sendpacket(base_channel ,base_address, packetbuffer);
				s4402815_radio_setfsmrx();
					
				input_counter = 0;
				input_flag = 0;
				packetbuff_init();
			}
		} 

		//read x and y signal
		x_signal = s4402815_joystick_x_read();
		y_signal = s4402815_joystick_y_read();

		//in model 2, keyboard input will be transmitted by laser
		if (input_flag == 2) {
		//debug_printf("keyboard_input %d\n", keyboard_input);
			if (keyboard_input != '#' && keyboard_input != '\0') {
				if (keyboard_input != 13) {
					laser_chara = keyboard_input;
					debug_printf("keyboard input %c\n", laser_chara);
				}else {
					s4402815_laser_encode(laser_chara, laser_send);
					laser_send_flag = 1;
					input_flag = 0;
				}
			}
		}

		//in model 3, 1 or 2 errors will be inser
		if (input_flag == 3) {
			
			if (keyboard_input != '!' && keyboard_input != '\0') {
				if (keyboard_input != 13) {
					error_insert = (error_insert << 4) | (keyboard_input - 48);
				}else {
					s4402815_error_insertion(laser_chara, error_insert, laser_send);
					//s4402815_laser_encode(laser_chara_error, laser_send);
					laser_send_flag = 1;
					input_flag = 0;
				}
			}
		}

		//in model 0, laser transmitter is controlled by keyboard
			if (input_flag == 0) {
				switch (keyboard_input) {
					case 'w': s4402815_pantilt_angle_write(2, --tilt_angle); break;
					case 's': s4402815_pantilt_angle_write(2, ++tilt_angle); break;
					case 'a': s4402815_pantilt_angle_write(1, ++pan_angle); break;
					case 'd': s4402815_pantilt_angle_write(1, --pan_angle); break;
					case 13: ;
					case '\0': break;
					default : debug_printf("erro, input again\n"); break;
				}
			}
		

		//in model 0, laser transmitter is controlled by joystick
			if (x_signal < 100){	
				if (pan_angle < 78) 
					pan_angle++;
				s4402815_pantilt_angle_write(1, pan_angle);
			}

			else if (x_signal > 4000){
			
				if (pan_angle > -82) 
					pan_angle--;
				s4402815_pantilt_angle_write(1, pan_angle);
			}

			if (y_signal < 100){
			
				if (tilt_angle < 82) 
					tilt_angle++;
				s4402815_pantilt_angle_write(2, tilt_angle);
			}

			else if (y_signal > 4000){
			
				if (tilt_angle > -82) 
					tilt_angle--;
				s4402815_pantilt_angle_write(2, tilt_angle);
			}
		
		//print received message if has already received
		if(s4402815_radio_getrxstatus() == 1) {
			char packet_receive[32];
			s4402815_radio_getpacket(packet_receive);	

			int i;
			debug_printf("Receive from radio: ");
			/*for (i = 1; i < 5; i++ )
				debug_printf("%02x", packet_receive[i]);

			debug_printf(" ");*/

			for (i = 9; i < 32; i++ ) {
		
				if(packet_receive[i])
					debug_printf("%c", packet_receive[i]);		
			}
			debug_printf("\n\r");                
		}

		//toggle led once every 2 loops
		if (loop_counter > 2) {
			
			BRD_LEDToggle();	//Toggle 'Alive' LED on/off	
			debug_printf("Pan: %d  Tilt read: %d\n", pan_angle, tilt_angle + 10);

			loop_counter = 1;
		}

		loop_counter++;
		keyboard_input = '\0';

		HAL_Delay(10);	//Delay 10ms.
	}
}

/**
  * @brief  Initialise and configure the hardware, 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {
	
	s4402815_pwm_init();	//Initialise pwm
	s4402815_joystick_init();

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_IC_InitTypeDef  TIM_ICInitStructure;
	uint16_t PrescalerValue = 0;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

  	/* Timer 3 clock enable */
  	__TIM3_CLK_ENABLE();

  	/* Enable the D0 Clock */
  	__BRD_D0_GPIO_CLK();

  	/* Configure the D0 pin with TIM3 input capture */
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
  	GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
  	GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;	//Set alternate function to be timer 2
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;

	/* Configure Timer 3 settings */
	TIM_Init.Instance = TIM3;					//Enable Timer 3
  	TIM_Init.Init.Period = 4*50000/10;			//Set for 100ms (10Hz) period
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.
	
	/* Configure TIM3 Input capture */
  	TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;			//Set to trigger on rising edge
  	TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
  	TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
  	TIM_ICInitStructure.ICFilter = 0;

	/* Set priority of Timer 3 Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0.

	//Enable Timer 3 interrupt and interrupt vector
	NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim3_irqhandler);  
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable input capture for Timer 3, channel 2 */
	HAL_TIM_IC_Init(&TIM_Init);
	HAL_TIM_IC_ConfigChannel(&TIM_Init, &TIM_ICInitStructure, TIM_CHANNEL_2);

	/* Start Input Capture */
	HAL_TIM_IC_Start_IT(&TIM_Init, TIM_CHANNEL_2);

	/* Enable timer4 to generate laser signal*/
	__TIM4_CLK_ENABLE();

	/* Timer4 configuration */
	TIM_Init.Instance = TIM4;				//Enable Timer 2
  	TIM_Init.Init.Period = 2 * 500000/100000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 10 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM4_IRQn, 9, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM4_IRQn, (uint32_t)&tim4_irqhandler);  
	NVIC_EnableIRQ(TIM4_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);

	/*Enable D1 clock to output laser signal*/
	__BRD_D1_GPIO_CLK();

	/* Configure the D1 pin as an output */
	GPIO_InitStructure.Pin = BRD_D1_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Enable PB clock for model shift*/
  	__BRD_PB_GPIO_CLK();
	
	/* Set priority of external GPIO Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(BRD_PB_EXTI_IRQ, 8, 0);	//Set Main priority ot 10 and sub-priority ot 0

	//Enable external GPIO interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_PB_EXTI_IRQ, (uint32_t)&exti_PB_irqhandler);  
	NVIC_EnableIRQ(BRD_PB_EXTI_IRQ);

  	/* Configure PB pin as pull down input */
	GPIO_InitStructure.Pin = BRD_PB_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
  	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_PB_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
}


/**
  * @brief  Timer 3 Interrupt handler for Input Capture 
  * @param  None.
  * @retval None
  */
void tim3_irqhandler (void) {

	unsigned int input_capture_value;
	
	//Clear Input Capture Flag
	TIM_Init.Instance = TIM3;
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_TRIGGER);

  	//input_capture_value = HAL_TIM_ReadCapturedValue(&TIM_Init, TIM_CHANNEL_2);
	laser_receive[laser_receive_counter] = HAL_TIM_ReadCapturedValue(&TIM_Init, TIM_CHANNEL_2);
	laser_receive_counter++;
	//debug_printf("laser receive: %d", laser_receive[laser_receive_counter]);
	//debug_printf("receive_counter: %d  %d\n",laser_receive_counter, laser_receive[laser_receive_counter]); 
	if (laser_receive_counter > 4) {
		
		unit = (laser_receive[1] - laser_receive[0]) / 2;
		//debug_printf("unit1: %f\n", unit);
		if (unit < 0) unit = unit + tim3_period/2;
		//debug_printf("unit2: %f\n", unit);
		offset = laser_receive[laser_receive_counter] - laser_receive[0];
		//debug_printf("offset1: %f\n", offset);
		if (offset < 0) offset = offset + tim3_period;
		//debug_printf("offset2: %f\n", offset);
		if (offset > 17.5*unit) {
			laser_receive_flag = 1;
			//debug_printf("Receive from laser: %c\n", laser_receive_chara);
			//debug_printf("offset fianl: %d\n", offset);
			laser_receive_counter = 0;
		}
	}
}

/**
  * @brief  Timer 4 Interrupt handler for laser signal generate 
  * @param  None.
  * @retval None
  */
void tim4_irqhandler (void) {
	
	//Clear Update Flag
	TIM_Init.Instance = TIM4;
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
	
	if (laser_send_flag ==1) {
		//debug_printf("%d ", laser_send[tim4_counter]);
		HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, laser_send[tim4_counter] & 0x01);
		tim4_counter++;
	}

	if (tim4_counter > 43) {
		HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0 & 0x01);
		tim4_counter = 0;
		laser_send_flag = 0;
		packetbuff_init();//clean packetbuff
	}
	//tim4_counter = 1-tim4_counter;
	//HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, tim4_counter & 0x01);
}

/**
  * @brief  NP2_PB_EXTI Interrupt handler for change transmit speed
  * @param  None.
  * @retval None
 */ 
void exti_PB_irqhandler(void) {
	
	HAL_Delay(200);		//Debouncing

	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);				//Clear PB pin external interrupt flag

	if (pb_flag == 1) pb_flag = 2;
	else pb_flag = 1;

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_IC_InitTypeDef  TIM_ICInitStructure;
	uint16_t PrescalerValue = 0;

	/* Enable timer4 to generate laser signal*/
	__TIM4_CLK_ENABLE();

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;

	/* Timer4 configuration */
	TIM_Init.Instance = TIM4;				//Enable Timer 2
  	TIM_Init.Init.Period = pb_flag * 500000/100000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 10 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM4_IRQn, 9, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM4_IRQn, (uint32_t)&tim4_irqhandler);  
	NVIC_EnableIRQ(TIM4_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);

	BRD_LEDToggle();

}

/**
  *write tx address part of packetbuffer[]
  */
void packetaddr_set(void) {
	packetbuffer[0] = 0x20;
	packetbuffer[1] = 0x78;
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
