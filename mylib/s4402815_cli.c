/**   
 ******************************************************************************   
 * @file    mylib/s4402815_cli.c    
 * @author  YI LIU – 44028156 
 * @date    01/05/2016   
 * @brief   cli callback functions 
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * * extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvTopCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvSuspendCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvResumeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvHamencCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvHamdecCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvAccCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvTrackingCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 * extern BaseType_t prvCRCCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include "math.h"
#include "s4402815_pantilt.h"
#include "s4402815_cli.h"
#include "s4402815_acc.h"
#include "s4402815_radio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int GetHandle(char* TaskName);
extern double s4402815_current_time(void);
void sethead(uint8_t *str);
void setcrc(uint8_t *str);
extern s4402815_rover_control(uint8_t type, uint8_t speed_l, uint8_t speed_r, int dura, uint8_t dir);

/**
  * @brief  CLI function for laser Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	
	if (strcmp("on", cCmd_string) == 0) {
		
		if (s4402815_SemaphoreLaser != NULL) {	//Check if semaphore exists
			laser_status = 1;
			/* Give Semaphore */
			xSemaphoreGive(s4402815_SemaphoreLaser);
		}
	}

	if (strcmp("off", cCmd_string) == 0) {
		
		if (s4402815_SemaphoreLaser != NULL) {	//Check if semaphore exists
			laser_status = 0;
			/* Give Semaphore */
			xSemaphoreGive(s4402815_SemaphoreLaser);
		}
	}


	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for pan Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cCmd_string;
	int angle = 0;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if (*cCmd_string > 47 && *cCmd_string < 58) {
		angle = *cCmd_string - 48;
		if (cCmd_string[1] > 47 && cCmd_string[1] < 58)
			angle = angle * 10 + cCmd_string[1] - 48;

		if (s4402815_QueuePan != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSend(s4402815_QueuePan, ( void * ) &angle, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
	}
	else if (*cCmd_string == '-') {
		angle = cCmd_string[1] - 48;
		if (cCmd_string[2] > 47 && cCmd_string[2] < 58)
			angle = angle * 10 + cCmd_string[2] - 48;
		angle = 0 - angle;

		if (s4402815_QueuePan != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSend(s4402815_QueuePan, ( void * ) &angle, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
	}
	else {
		if (strcmp("left", cCmd_string) == 0) {
			
			if (s4402815_SemaphorePanLeft != NULL)
				xSemaphoreGive(s4402815_SemaphorePanLeft);
		}
		if (strcmp("right", cCmd_string) == 0) {
			
			if (s4402815_SemaphorePanRight != NULL)
				xSemaphoreGive(s4402815_SemaphorePanRight);
		}
	}

	/* Write command pan output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for tilt Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cCmd_string;
	int angle = 0;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if (*cCmd_string > 47 && *cCmd_string < 58) {
		angle = *cCmd_string - 48;
		if (cCmd_string[1] > 47 && cCmd_string[1] < 58)
			angle = angle * 10 + cCmd_string[1] - 48;
		
		if (s4402815_QueueTilt != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSend(s4402815_QueueTilt, ( void * ) &angle, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
	}
	else if (*cCmd_string == '-') {
		angle = cCmd_string[1] - 48;
		if (cCmd_string[2] > 47 && cCmd_string[2] < 58)
			angle = angle * 10 + cCmd_string[2] - 48;
		angle = 0 - angle;
		
		if (s4402815_QueueTilt != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSend(s4402815_QueueTilt, ( void * ) &angle, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
	}
	else {
		if (strcmp("up", cCmd_string) == 0) {
			
			if (s4402815_SemaphoreTiltUp != NULL)
				xSemaphoreGive(s4402815_SemaphoreTiltUp);
		}
		if (strcmp("down", cCmd_string) == 0) {
			
			if (s4402815_SemaphoreTiltDown != NULL)
				xSemaphoreGive(s4402815_SemaphoreTiltDown);
		}
	}

	/* Write command pan output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for box Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	/*give semaphore to start drawing box*/
	xSemaphoreGive(s4402815_SemaphoreBox);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for top command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvTopCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	/*give semaphore */
	xSemaphoreGive(s4402815_SemaphoreTop);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for suspend command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvSuspendCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	char* cmd_string;
	long lParam_len; 
	struct Task task_ctrl;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string
	task_ctrl.handle = GetHandle(cmd_string);
	task_ctrl.type = 's';

	if (s4402815_QueueTaskControl != NULL) {		//Check if queue exists

		//Send message the queue - wait atmost 10 ticks
		if( xQueueSend(s4402815_QueueTaskControl, ( void * )&task_ctrl, ( portTickType ) 10 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}
	}

	//Write command output string to write buffer.
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");
	
	//Return pdFALSE, as there are no more strings to return 
	//Only return pdTRUE, if more strings need to be printed
	return pdFALSE;
}

/**
  * @brief  CLI function for resume command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvResumeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	char* cmd_string;
	long lParam_len; 
	struct Task task_ctrl;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	task_ctrl.handle = GetHandle(cmd_string);
	task_ctrl.type = 'r';

	if (s4402815_QueueTaskControl != NULL) {		//Check if queue exists

		//Send message the queue - wait atmost 10 ticks
		if( xQueueSend(s4402815_QueueTaskControl, ( void * )&task_ctrl, ( portTickType ) 10 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}
	}

	//Write command output string to write buffer.
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");
	
	//Return pdFALSE, as there are no more strings to return 
	//Only return pdTRUE, if more strings need to be printed
	return pdFALSE;
}

/**
  * @brief  CLI function for hamenc command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvHamencCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	int value = 0;
 
	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string
	if (cmd_string[0] > 47 && cmd_string[0] < 58)
		value = (cmd_string[0] - 48) * 16;
	else if (cmd_string[0] > 64 && cmd_string[0] < 71)
		value = (cmd_string[0] - 55) * 16;
	else if (cmd_string[0] > 96 && cmd_string[0] < 103)
		value = (cmd_string[0] - 87) * 16;
	else debug_printf("invalid number\n");

	if (cmd_string[1] > 47 && cmd_string[1] < 58)
		value += cmd_string[1] - 48;
	else if (cmd_string[1] > 64 && cmd_string[1] < 71)
		value += cmd_string[1] - 55;
	else if (cmd_string[1] > 96 && cmd_string[1] < 103)
		value += cmd_string[1] - 87;
	else debug_printf("invalid number\n");

	debug_printf("Hamming encode: 0x%04x\n", s4402815_hamenc(value));

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for hamdec command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvHamdecCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	uint16_t codedword = 0x0000;
 
	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string
	if (cmd_string[0] == '0' && (cmd_string[1] == 'x' || cmd_string[1] == 'X')) {
		for (int i = 0; i < 4; i++) {
			
			if (cmd_string[i+2] > 47 && cmd_string[i+2] < 58)
				codedword |= (cmd_string[i+2] - 48) << (12 - 4*i);
			else if (cmd_string[i+2] > 64 && cmd_string[i+2] < 71)
				codedword |= (cmd_string[i+2] - 55) << (12 - 4*i);
			else if (cmd_string[i+2] > 96 && cmd_string[i+2] < 103)
				codedword |= (cmd_string[i+2] - 87) << (12 - 4*i);
			else debug_printf("invalid number1\n");
		}
	}
	else {
		int i = 0;
		for (; cmd_string[i] != '\0'; i++);
		debug_printf("%s, %d\n", cmd_string, i);
		for (int j = 0; j < i; j++) {
			if (cmd_string[j] > 47 && cmd_string[j] < 58)
				codedword += (cmd_string[j] - 48) * pow(10, i-j-1);
			else debug_printf("invalid number\n");
		}

	}
	
	if (s4402815_hamdec(codedword) == -1) 
		debug_printf("two or more bits wrong and cannot correct\n");
	else
		debug_printf("hamming decode: 0x%04x\n", s4402815_hamdec(codedword));

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for crc
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvCRCCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	uint32_t hex_value = 0;
	uint16_t crc_value;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	//if the input is hex number
	if(cmd_string[0] == '0' && (cmd_string[1] == 'x' || cmd_string[1] == 'X')) {
		for (int i = 0; i < 32; i++) {
			
			if (cmd_string[i] > 47 && cmd_string[i] < 58)
				hex_value |= (cmd_string[i] - 48) << i;
			else if (cmd_string[i+2] > 64 && cmd_string[i+2] < 71)
				hex_value |= (cmd_string[i] - 55) << i;
			else if (cmd_string[i+2] > 96 && cmd_string[i+2] < 103)
				hex_value |= (cmd_string[i] - 87) << i;
		}
		crc_value = s4402815_crc((char*) &hex_value, 4);
		debug_printf("crc: 0x%04x\n", crc_value);
	}
	else {
		crc_value = s4402815_crc((char*)cmd_string, strlen(cmd_string));
		debug_printf("crc: 0x%04x\n", crc_value);
	}

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for acc command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvAccCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	if (strcmp(cmd_string, "raw") == 0) {
		if (s4402815_SemaphoreAccReadRaw != NULL) {
			/*give semaphore to read acc x, y, z values*/
			xSemaphoreGive(s4402815_SemaphoreAccReadRaw);
		}
	}
	else if (strcmp(cmd_string, "pl") == 0) {
		if (s4402815_SemaphoreAccReadPL != NULL) {
			/*give semaphore to read acc pl status */
			xSemaphoreGive(s4402815_SemaphoreAccReadPL);
		}
	}
	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for tracking command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvTrackingCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	//when type "on"
	if (strcmp(cmd_string, "on") == 0) {
		if (s4402815_SemaphoreTrackOn != NULL) {
			/*give semaphore to enable send radio message*/
			xSemaphoreGive(s4402815_SemaphoreTrackOn);
		}
	}
	//when type "off"
	if (strcmp(cmd_string, "off") == 0) {
		if (s4402815_SemaphoreTrackOff != NULL) {
			/*give semaphore to enable send radio message*/
			xSemaphoreGive(s4402815_SemaphoreTrackOff);
		}
	}

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for getpasskey command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvGetpasskeyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	// send message to rover
	s4402815_rover_control(0x30, 0, 0, 0, 0);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for getsensor command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvGetsensorCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	// send message to rover
	s4402815_rover_control(0x31, 0, 0, 0, 0);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for gettime command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvGettimeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	double time = s4402815_current_time();

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "current time: %.2fs\n\r", time);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for rfchanset command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvRfchansetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string
	
	re_channel = (cmd_string[0] - 48) * 10 + cmd_string[1] - 48;
	s4402815_radio_setchan(re_channel);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "chan test: %d\n\r", re_channel);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for txaddset command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvTxaddsetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	for (int i = 0; i < 4; i++) {
		re_address[3 - i] = ((cmd_string[i*2] - 48) << 4) | (cmd_string[i*2 + 1] - 48);
	}
	s4402815_radio_settxaddress(re_address);

	for (int i = 0; i < 5; i++)
		debug_printf("%02x ", re_address[i]);
	debug_printf("\n");

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for roverchan command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvRoverchanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string
	
	tr_channel = (cmd_string[0] - 48) * 10 + cmd_string[1] - 48;

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "rover channel is %d\n\r", tr_channel);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for roveradd command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvRoveraddCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	for (int i = 0; i < 4; i++) {
		tr_address[3 - i] = ((cmd_string[i*2] - 48) << 4) | (cmd_string[i*2 + 1] - 48);
	}
	
	for (int i = 0; i < 5; i++)
		debug_printf("%02x ", tr_address[i]);
	debug_printf("\n");

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for forward command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvForwardCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	double distance = 0;
	uint8_t time = 0;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	//calculate a distance which a wheel should move
	for (int i = 0; i < strlen(cmd_string); i++)
		distance = distance * 10 + cmd_string[i] - 48;

	//calculate the time which the wheel should move on for
	time = (int)(distance / 15);

	//set each bit of message to be sent to rover
	s4402815_rover_control(0x32, speed1, speed2, time, 0x05);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for reverse command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvReverseCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	double distance = 0;
	uint8_t time = 0;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	//calculate a distance which a wheel should move
	for (int i = 0; i < strlen(cmd_string); i++)
		distance = distance * 10 + cmd_string[i] - 48;

	//calculate the time which the wheel should move on for
	time = (int)(distance / 15);

	//set each bit of message to be sent to rover
	s4402815_rover_control(0x32, speed1r, speed2r, time, 0x0a);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for tracking command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	float distance = 0;
	uint8_t angle_value = 0;
	uint8_t time = 0;
	uint16_t dir = 0x0a;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	//if input angle is negative
	if (cmd_string[0] == '-') {
		//extract angle value
		for (int i = 1; i < strlen(cmd_string); i++)
			angle_value = angle_value * 10 + cmd_string[i] - 48;
		dir = 0x05;		//change the direction to reverse if input angle is negative
	}
	else {
		for (int i = 0; i < strlen(cmd_string); i++)
			angle_value = angle_value * 10 + cmd_string[i] - 48;
	}

	//calculate a distance which the right wheel should move
	distance = 92 * 3.14 * angle_value / 180;
	debug_printf("dis: %.2f\n", distance);

	//calculate the time which the wheel should move on for
	time = (int)(distance / 15);
	debug_printf("time: 0x%02x\n", time);

	//set each bit of message to be sent to rover
	s4402815_rover_control(0x32, speed3, 0, time, dir);

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for distance command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvDistanceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	//check whether semophore exists
	if (s4402815_SemaphoreDistance != NULL) {
		//give semaphore to diplay pass key
		xSemaphoreGive(s4402815_SemaphoreDistance);
	}

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for position command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvPositionCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	EventBits_t uxBitsPos;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	if (s4402815_SemaphoreLaser != NULL) {	//Check if semaphore exists
		laser_status = 1;
		/* Give Semaphore */
		xSemaphoreGive(s4402815_SemaphoreLaser);
	}

	//when type "on"
	if (strcmp(cmd_string, "on") == 0) {
		//check whether semophore exists
		if (s4402815_SemaphorePositionOn != NULL) {
			//give semaphore to diplay pass key
			xSemaphoreGive(s4402815_SemaphorePositionOn);
		}
	}

	//when type "off"
	if (strcmp(cmd_string, "off") == 0) {
		//check whether semophore exists
		if (s4402815_SemaphorePositionOff != NULL) {
			//give semaphore to diplay pass key
			xSemaphoreGive(s4402815_SemaphorePositionOff);
		}
	}
	
	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for Calibration command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvCalibrationCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	int new_speed = 0;
	
	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	for (int i = 1; i < strlen(cmd_string); i++)
		new_speed = new_speed * 10 + cmd_string[i] - 48;

	//set speed1
	if (cmd_string[0] == 'a') {
		speed1 = new_speed;
		speed1r = new_speed;
		debug_printf("new speed1 %d\n", speed1);
	}

	//set_speed2
	if (cmd_string[0] == 'b') {
		speed2 = new_speed;
		speed2r = new_speed;
		debug_printf("new speed2 %d\n", speed2);
	}

	//set_speed3
	if (cmd_string[0] == 'c') {
		speed3 = new_speed;
		speed3r = new_speed;
		debug_printf("new speed3 %d\n", speed3);
	}

	//set_speed1r
	if (cmd_string[0] == 'd') {
		speed1r = new_speed;
		debug_printf("new speed1r %d\n", speed1r);
	}

	//set_speed2r
	if (cmd_string[0] == 'e') {
		speed2r = new_speed;
		debug_printf("new speed2r %d\n", speed2r);
	}

	//set_speed3r
	if (cmd_string[0] == 'f') {
		speed3r = new_speed;
		debug_printf("new speed3r %d\n", speed3r);
	}

	//set_speedx
	if (cmd_string[0] == 'x') {
		speedx = new_speed;
		debug_printf("new speedx %d\n", speedx);
	}
	
	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for roverid command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvRoveridCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	rover_id = cmd_string[0] - 48;

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "rover id %d\n\r", rover_id);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for waypoint command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvWaypointCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;
	int waypoint_id;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	waypoint_id = cmd_string[0] - 48;

	//Send received message to the front of the queue - wait atmost 10 ticks
	if (s4402815_QueueWayPointID != NULL) {
		if( xQueueSendToFront(s4402815_QueueWayPointID, (void *)&waypoint_id, ( portTickType ) 10 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}
	}

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "id is %d\n\r", waypoint_id);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for follower command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvFollowerCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	//check whether semophore exists
	if (s4402815_SemaphoreFollow!= NULL) {
		//give semaphore to get rover width and height
		xSemaphoreGive(s4402815_SemaphoreFollow);
	}

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  CLI function for acccontrol command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
extern BaseType_t prvAcccontrolCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len; 
	const char *cmd_string;

	cmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);	//Get parameters from command string

	//when type "on"
	if (strcmp(cmd_string, "on") == 0) {
		//check whether semophore exists
		if (s4402815_SemaphoreAccCtrlOn != NULL) {
			//give semaphore to diplay pass key
			xSemaphoreGive(s4402815_SemaphoreAccCtrlOn);
		}
	}

	//when type "off"
	if (strcmp(cmd_string, "off") == 0) {
		//check whether semophore exists
		if (s4402815_SemaphoreAccCtrlOff != NULL) {
			//give semaphore to diplay pass key
			xSemaphoreGive(s4402815_SemaphoreAccCtrlOff);
		}
	}

	/* Write command output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\r");

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  set the crc of packet to be sent
  * @param  the name of task
  * @retval None
  */
extern double s4402815_current_time(void) {

	double time = xTaskGetTickCount();
	time = time / 1000;

	return time;
}

/**
  * @brief  get the handle of task
  * @param  the name of task
  * @retval None
  */
int GetHandle(char* TaskName) {

	if (strcmp(TaskName, "CLI") == 0) return 0;
	else if (strcmp(TaskName, "Top") == 0) return 1;
	else if (strcmp(TaskName, "ACC") == 0) return 2;
	else if (strcmp(TaskName, "AccRead") == 0) return 3;
	else if (strcmp(TaskName, "Radio") == 0) return 4;
	else if (strcmp(TaskName, "RadioRec") == 0) return 5;
	else if (strcmp(TaskName, "Track") == 0) return 6;
	else if (strcmp(TaskName, "RoverMsg") == 0) return 7;
	else if (strcmp(TaskName, "LightBar") == 0) return 8;
	else if (strcmp(TaskName, "PanTilt") == 0) return 9;
	else if (strcmp(TaskName, "AutoCtrl") == 0) return 10;
	else if (strcmp(TaskName, "AccCtrl") == 0) return 11;
	else if (strcmp(TaskName, "Led") == 0) return 12;
	else if (strcmp(TaskName, "IDLE") == 0) return 13;
	else return 0;
}

/**
  * @brief  send message to rover to control rover.
  * @param  type, left speed, right speed , duration, direction
  * @retval None
  */
extern s4402815_rover_control(uint8_t type, uint8_t speed_l, uint8_t speed_r, int dura, uint8_t dir) {

	struct Msg_send msg_rover;
	uint8_t message[32] = {0};
	int dura_temp = dura;
	int i = 1;

	if (dura == 0 && type == 0x32) dura = 1;
	if (dura == 28 && speed_r == 0) {
		dura = 7;
		speed_l += speedx * 3;
	}
	else if (dura > 0x0f) {
		
		while (dura > 0x0f) {
			i++;
			dura = dura_temp / i;
		}
		speed_l += speedx * (i-1);
		if (speed_r != 0)
			speed_r += speedx * (i-1);
	}

	message[0] = type;
	message[1] = tr_address[0];
	message[2] = tr_address[1];
	message[3] = tr_address[2];
	message[4] = tr_address[3];
	message[5] = 0x56;
	message[6] = 0x81;
	message[7] = 0x02;
	message[8] = 0x44;
	message[9] = sequence++;
	message[10] = key;
	message[11] = s4402815_hamenc(speed_l) >> 8;
	message[12] = s4402815_hamenc(speed_l) & 0x00ff;
	message[13] = s4402815_hamenc(speed_r) >> 8;
	message[14] = s4402815_hamenc(speed_r) & 0x00ff;
	message[15] = s4402815_hamenc((dura << 4) | dir) >> 8;
	message[16] = s4402815_hamenc((dura << 4) | dir) & 0x00ff;
	setcrc(message);

	for (int i = 0; i < 32; i++)
		msg_rover.msg[i] = message[i];

	msg_rover.chan = 0;

	//Send message used to get sensor value to the front of the queue - wait atmost 10 ticks
	if (s4402815_QueueRadioTx != NULL) {
		if( xQueueSendToFront(s4402815_QueueRadioTx, (void *)&msg_rover, ( portTickType ) 20 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}
	}
}

/**
  * @brief  set the address part of packet to be sent
  * @param  the name of task
  * @retval None
  */
void sethead(uint8_t *str) {
	str[1] = tr_address[0];
	str[2] = tr_address[1];
	str[3] = tr_address[2];
	str[4] = tr_address[3];
	str[5] = 0x56;
	str[6] = 0x81;
	str[7] = 0x02;
	str[8] = 0x44;
	str[9] = sequence++;
	str[10] = key;
}

/**
  * @brief  set the crc of packet to be sent
  * @param  the name of task
  * @retval None
  */
void setcrc(uint8_t *str) {
	uint8_t string[30] = {0};
	uint16_t crc_value = 0;

	for (int i = 0; i < 30; i++) {
		string[i] = str[i];
	}
	crc_value = s4402815_crc(string, 30);
	str[30] = crc_value & 0x00ff;
	str[31] = (crc_value & 0xff00) >> 8;
}

