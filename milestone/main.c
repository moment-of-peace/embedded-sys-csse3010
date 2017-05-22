/**
  ******************************************************************************
  * @file    milestone/main.c 
  * @author  YI LIU
  * @date    06052016
  * @brief   the task utilities, FreeRTOS diver tasks and contolling tasks
  *
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <math.h>

#include "s4402815_cli.h"
#include "s4402815_hamming.h"
#include "s4402815_radio.h"
#include "s4402815_acc.h"
#include "s4402815_lightbar.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;	//timer4 type define

/* Private define ------------------------------------------------------------*
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void ApplicationIdleHook( void ); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void CLI_Task(void);
void LedToggle_Task(void);
void Top_Task(signed char *);
void Suspend_Task(void);
void Resume_Task(void);
void Hamenc_Task(void);
void CRC_Task(void);
void AccRead_Task(void);
void RadioRec_Task(void);
void Tracking_Task(void);
void RoverMsg_Task(void);
void tim4_irqhandler (void);

/* Private variables ---------------------------------------------------------*/
// Structure that defines the "top" command line command.
CLI_Command_Definition_t xTop = {
	"top",
	"top: list the number and information of current tasks\r\n",
	prvTopCommand,
	0
};

// Structure that defines the "suspend" command line command.
CLI_Command_Definition_t xSuspend = {
	"suspend",
	"suspend: suspend a task with a given name\r\n",
	prvSuspendCommand,
	1
};

// Structure that defines the "resume" command line command.
CLI_Command_Definition_t xResume = {
	"resume",
	"resume: resume a task with a given name\r\n",
	prvResumeCommand,
	1
};

// Structure that defines the "hamenc" command line command.
CLI_Command_Definition_t xHamenc = {
	"hamenc",
	"hamenc: hamming encoder\r\n",
	prvHamencCommand,
	1
};

// Structure that defines the "hamdec" command line command.
CLI_Command_Definition_t xHamdec = {
	"hamdec",
	"hamdec: hamming decoder\r\n",
	prvHamdecCommand,
	1
};

// Structure that defines the "crc" command line command.
CLI_Command_Definition_t xCRC = {
	"crc",
	"crc: calculate crc 16 of a hex value or a string\r\n",
	prvCRCCommand,
	1
};

// Structure that defines the "acc" command line command.
CLI_Command_Definition_t xAcc = {
	"acc",
	"acc: receive data from accelerometer and display\r\n",
	prvAccCommand,
	1
};

// Structure that defines the "tracking" command line command.
CLI_Command_Definition_t xTracking = {
	"tracking",
	"tracking: input on or off to enable or disable the tracking output\r\n",
	prvTrackingCommand,
	1
};

// Structure that defines the "getpasskey" command line command.
CLI_Command_Definition_t xGetpasskey = {
	"getpasskey",
	"getpasskey: get current pass key\r\n",
	prvGetpasskeyCommand,
	0
};

// Structure that defines the "getsensor" command line command.
CLI_Command_Definition_t xGetsensor = {
	"getsensor",
	"getsensor: get current sensor value\r\n",
	prvGetsensorCommand,
	0
};

// Structure that defines the "gettime" command line command.
CLI_Command_Definition_t xGettime = {
	"gettime",
	"gettime: get the time which is since np2 was turned on\r\n",
	prvGettimeCommand,
	0
};

// Structure that defines the "rfchanset" command line command.
CLI_Command_Definition_t xRfchanset = {
	"rfchanset",
	"rfchanset: set rf channel\r\n",
	prvRfchansetCommand,
	1
};

// Structure that defines the "txaddset" command line command.
CLI_Command_Definition_t xTxaddset = {
	"txaddset",
	"txaddset: set tx address\r\n",
	prvTxaddsetCommand,
	1
};

// Structure that defines the "forward" command line command.
CLI_Command_Definition_t xForward = {
	"forward",
	"forward: move the rover forward a specified distance\r\n",
	prvForwardCommand,
	1
};

// Structure that defines the "reverse" command line command.
CLI_Command_Definition_t xReverse = {
	"reverse",
	"recerse: move the rover in reverse a specified distance\r\n",
	prvReverseCommand,
	1
};

// Structure that defines the "angle" command line command.
CLI_Command_Definition_t xAngle = {
	"angle",
	"angle: move the rover to a certain angle\r\n",
	prvAngleCommand,
	1
};

TaskHandle_t handle_list[15];			//used to store tasks handles
char* name_list[15];					//used to store tasks names
unsigned int priority_list[15] = {0};	//used to store tasks priorities
char* state_list[15] = {0};				//used to store tasks state
int number_list[15] = {0};				//used to store tasks number
double runtime_list[15] = {0};			//used to store tasks runtime
unsigned long ulPercentage[15] = {0};	//used to store tasks runtime precentage
struct Message rec_msg;					//used to store radio message from queue
int sensor_value = 0;					//sensor value of rover

/* Task Priorities ------------------------------------------------------------*/
#define mainCLI_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainTop_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainSuspend_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainResume_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainACC_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainAccRead_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainRadio_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainRadioRec_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainTracking_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainLed_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainRoverMsg_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainLightBar_PRIORITY				( tskIDLE_PRIORITY + 1 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainCLI_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 2 )
#define mainTop_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 4 )
#define mainSuspend_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 1 )
#define mainResume_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 1 )
#define mainACC_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 2 )
#define mainAccRead_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
#define mainRadio_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
#define mainRadioRec_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainTracking_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLed_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 1 )
#define mainRoverMsg_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLightBar_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 1 )

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {
	BRD_init();
	Hardware_init();
	
	/* Start the tasks and start the CLI. */
	xTaskCreate( (void *) &CLI_Task, (const signed char *) "CLI", mainCLI_TASK_STACK_SIZE, NULL, mainCLI_PRIORITY, handle_list+0 );
	xTaskCreate( (void *) &LedToggle_Task, (const signed char *) "Led", mainLed_TASK_STACK_SIZE, NULL, mainLed_PRIORITY, handle_list+1 );
	xTaskCreate( (void *) &Top_Task, (const signed char *) "Top", mainTop_TASK_STACK_SIZE, NULL, mainTop_PRIORITY, handle_list+2 );
	xTaskCreate( (void *) &Suspend_Task, (const signed char *) "Suspend", mainSuspend_TASK_STACK_SIZE, NULL, mainSuspend_PRIORITY, handle_list+3 );
	xTaskCreate( (void *) &Resume_Task, (const signed char *) "Resume", mainResume_TASK_STACK_SIZE, NULL, mainResume_PRIORITY, handle_list+4 );
	xTaskCreate( (void *) &s4402815_TaskAcc, (const signed char *) "ACC", mainACC_TASK_STACK_SIZE, NULL, mainACC_PRIORITY, handle_list+5 );
	xTaskCreate( (void *) &AccRead_Task, (const signed char *) "AccRead", mainAccRead_TASK_STACK_SIZE, NULL, mainAccRead_PRIORITY, handle_list+6 );
	xTaskCreate( (void *) &s4402815_TaskRadio, (const signed char *) "Radio", mainRadio_TASK_STACK_SIZE, NULL, mainRadio_PRIORITY, handle_list+7 );
	xTaskCreate( (void *) &RadioRec_Task, (const signed char *) "RadioRec", mainRadioRec_TASK_STACK_SIZE, NULL, mainRadioRec_PRIORITY, handle_list+8 );
	xTaskCreate( (void *) &Tracking_Task, (const signed char *) "Track", mainTracking_TASK_STACK_SIZE, NULL, mainTracking_PRIORITY, handle_list+9 );
	xTaskCreate( (void *) &RoverMsg_Task, (const signed char *) "RoverMsg", mainRoverMsg_TASK_STACK_SIZE, NULL, mainRoverMsg_PRIORITY, handle_list+10 );
	xTaskCreate( (void *) &s4402815_TaskLightBar, (const signed char *) "LightBar", mainLightBar_TASK_STACK_SIZE, NULL, mainLightBar_PRIORITY, handle_list+11 );
	
	
	/* Create Semaphores */
	s4402815_SemaphoreTop = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccRaw = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccPL = xSemaphoreCreateBinary();
	s4402815_SemaphoreRadioOn = xSemaphoreCreateBinary();
	s4402815_SemaphoreRadioOff = xSemaphoreCreateBinary();
	s4402815_SemaphoreGetKey = xSemaphoreCreateBinary();
	s4402815_SemaphoreGetSensor = xSemaphoreCreateBinary();

	/* Register CLI commands */
	FreeRTOS_CLIRegisterCommand(&xTop);
	FreeRTOS_CLIRegisterCommand(&xSuspend);
	FreeRTOS_CLIRegisterCommand(&xResume);
	FreeRTOS_CLIRegisterCommand(&xHamenc);
	FreeRTOS_CLIRegisterCommand(&xHamdec);
	FreeRTOS_CLIRegisterCommand(&xCRC);
	FreeRTOS_CLIRegisterCommand(&xAcc);
	FreeRTOS_CLIRegisterCommand(&xTracking);
	FreeRTOS_CLIRegisterCommand(&xGetpasskey);
	FreeRTOS_CLIRegisterCommand(&xGetsensor);
	FreeRTOS_CLIRegisterCommand(&xGettime);
	FreeRTOS_CLIRegisterCommand(&xRfchanset);
	FreeRTOS_CLIRegisterCommand(&xTxaddset);
	FreeRTOS_CLIRegisterCommand(&xForward);
	FreeRTOS_CLIRegisterCommand(&xReverse);
	FreeRTOS_CLIRegisterCommand(&xAngle);

	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();
	
	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}

/**
  * @brief  CLI Receive Task.
  * @param  None
  * @retval None
  */
void CLI_Task(void) {

	char cRxedChar;
	char cInputString[100];
	int InputIndex = 0;
	char *pcOutputString;
	BaseType_t xReturned;

	/* Initialise pointer to CLI output buffer. */
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for (;;) {

		/* Receive character */
		cRxedChar = debug_getc();

		/* Process if chacater if not Null */
		if (cRxedChar != '\0') {

			/* Put byte into USB buffer if input is not backspace */
			if (cRxedChar != 127)
				debug_putc(cRxedChar);

			/* if input is backspace, clear previous character*/
			if (cRxedChar == 127)
				debug_printf("\b \b");
		
			/* Process only if return is received. */
			if (cRxedChar == '\r') {

				//Put new line and transmit buffer
				debug_putc('\n');
				debug_flush();

				/* Put null character in command input string. */
				cInputString[InputIndex] = '\0';
				
				xReturned = pdTRUE;
				/* Process command input string. */
				while (xReturned != pdFALSE) {

					/* Returns pdFALSE, when all strings have been returned */
					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

					/* Display CLI output string */
					debug_printf("%s\n",pcOutputString);
				    vTaskDelay(5);	//Must delay between debug_printfs.
				}

				memset(cInputString, 0, sizeof(cInputString));
				InputIndex = 0;

			} else {

				debug_flush();		//Transmit USB buffer

				if( cRxedChar == '\r' ) {

					/* Ignore the character. */
				} else if( cRxedChar == 127 ) {

					/* Backspace was pressed.  Erase the last character in the
					 string - if any.*/
					if( InputIndex > 0 ) {
						InputIndex--;
						cInputString[ InputIndex ] = '\0';
					}

				} else {

					/* A character was entered.  Add it to the string
					   entered so far.  When a \n is entered the complete
					   string will be passed to the command interpreter. */
					if( InputIndex < 20 ) {
						cInputString[ InputIndex ] = cRxedChar;
						InputIndex++;
					}
				}
			}		
		}
		/* task delay */
		vTaskDelay(50);
	}
}

/**

  * @brief  Task to list the number and information of current tasks.
  * @param  None
  * @retval None
  */
void Top_Task(signed char *writeBuff) {

	TaskStatus_t *TasksStatusList;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;

	// get the idle task handle before the for loop
	handle_list[12] = xTaskGetIdleTaskHandle();
	
	for (;;) {

		//receive semaphore, wait most 10 ticks
		if (s4402815_SemaphoreTop != NULL) {
			if (xSemaphoreTake(s4402815_SemaphoreTop, 10) == pdTRUE) {

				//Make sure the write buffer does not contain a string
				*writeBuff = 0x00;

				//take a snapshot of the number of tasks in case it changes
				uxArraySize = uxTaskGetNumberOfTasks();

				// get tasks state before using "uxTaskGetSystemState" function, otherwise the state could 
				//be changed
				for (int i = 0; i < uxArraySize; i++) {
					switch(eTaskGetState(handle_list[i])) {
						case eReady: state_list[i] = "ready"; break;
						case eRunning: state_list[i] = "running"; break;
						case eBlocked: state_list[i] = "blocked"; break;
						case eSuspended: state_list[i] = "suspend"; break;
					}
				}

				//Allocate a TaskStatus_t structure for each task
				TasksStatusList = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

				if( TasksStatusList != NULL ){

					// Generate raw status information about each task.
					uxArraySize = uxTaskGetSystemState( TasksStatusList, uxArraySize, &ulTotalRunTime );
					debug_printf("total run time %d\n", ulTotalRunTime);
					//for percentage calculations
					ulTotalRunTime /= 100UL;

					//print the parameters name of each task
					debug_printf("Name\tNum\tPrior\tState\t\tRuntime(%)\tRuntime\n");

					//avoid divided by zero
					if(ulTotalRunTime > 0) {
						for(x = 0; x < uxArraySize; x++) {
							//calculate the percentage of runtime
							ulPercentage[x] = TasksStatusList[x].ulRunTimeCounter / ulTotalRunTime;

							if(ulPercentage > 0UL) {
								sprintf(writeBuff, "%lu\t\t%lu\n", TasksStatusList[x].ulRunTimeCounter, ulPercentage);
							}
							else {
								//If the percentage is zero here then the task has consumed less than 
								//1% of the total run time
								sprintf(writeBuff, "%lu\n", TasksStatusList[x].ulRunTimeCounter);
							}
							writeBuff += strlen((char*)writeBuff);

							//get other parameters of each task
							name_list[x] = TasksStatusList[x].pcTaskName;
							number_list[x] = (int)TasksStatusList[x].xTaskNumber;
							priority_list[x] = (unsigned int)TasksStatusList[x].uxCurrentPriority;
							runtime_list[x] = TasksStatusList[x].ulRunTimeCounter;
	
							//set the output color and print details of each task
							color_set(state_list[number_list[x]-1]);
							debug_printf("%s\t%d\t%d\t%s\t\t%lu%%\t\t%f\n", name_list[x], number_list[x], priority_list[x], state_list[number_list[x]-1], ulPercentage[x], runtime_list[x]);

							//set the output color to default
							debug_printf("\033[0m\r");
						}
					}
				}
				//The array is not needed, free the memory it consumes
				vPortFree( TasksStatusList );
			}
		}
		//task delay 
		vTaskDelay(50);
	}
}


/**
  * @brief  Task to suspend a task with a given name.
  * @param  None
  * @retval None
  */
void Suspend_Task(void) {

	int TaskToSuspend;
	//create queue to transfer the task needed to suspend
	s4402815_QueueSuspend = xQueueCreate(5, sizeof(TaskToSuspend));	

	for (;;) {

		//Check if queue exists
		if (s4402815_QueueSuspend != NULL) {
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueSuspend, &TaskToSuspend, 10)) {
				//suspend the target task
				vTaskSuspend(handle_list[TaskToSuspend]);
				debug_printf("already suspended\n");
			}
		}
		//task delay
		vTaskDelay(50);
	}
}

/**
  * @brief  Task to resume a task with a given name.
  * @param  None
  * @retval None
  */
void Resume_Task(void) {

	int TaskToResume;
	//create queue to transfer the task needed to resume
	s4402815_QueueResume = xQueueCreate(5, sizeof(TaskToResume));

	for (;;) {
		
		//Check if queue exists
		if (s4402815_QueueResume != NULL) {
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueResume, &TaskToResume, 10)) {
				//resume the target task
				vTaskResume(handle_list[TaskToResume]);
				debug_printf("already resumed\n");
			}
		}
		// task delay
		vTaskDelay(50);
	}
}

/**
  * @brief  Task for receiving data from accelerometer, including x, y, z value and portrait and landscape
  *			status.
  * @param  None
  * @retval None
  */
void AccRead_Task(void) {

	struct Raw raw_value;	//struct used to receive x, y, z value f
	struct Pl pl_status;	//struct used to receive portrait and landscape status
		
	for (;;) {

		//receive 12-bit acc x, y, z values from queue
		if (s4402815_QueueAccRaw != NULL) {
	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueAccRaw, &raw_value, 10)) {
				//print out x, y, z value of accelerometer
				debug_printf("acc x value: %d\n", raw_value.x_value);
				debug_printf("acc y value: %d\n", raw_value.y_value);
				debug_printf("acc z value: %d\n\n", raw_value.z_value);
			}
		}

		//receive acc pl status from queue
		if (s4402815_QueueAccPL != NULL) {
	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueAccPL, &pl_status, 10)) {
				debug_printf("pl status: %c\tbf status: %c\n\n", pl_status.pl, pl_status.bf);
			}
		}
		//task delay
		vTaskDelay(50);
	}
}

void RadioRec_Task(void) {

	for (;;) {

		//receive radio message from queue
		if (s4402815_QueueRadioMsg != NULL) {
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueRadioMsg, &rec_msg, 10)) {
			}
		}
		rec_msg.packet[0] = 0x31;
		//task delay
		vTaskDelay(25);
	}
}
/**
  * @brief  Task for tracking a marker, get the parameters of different markers, including id, x, y, width
  * 		and heigh values, and calculate the velocity of marker.
  * @param  None
  * @retval None
  */
void Tracking_Task(void) {

	uint16_t temp_raw;
	uint8_t temp_decoded[10];		//decoded playload
	uint32_t position_list[4][3] = {0};	//used to record the positions of different markers
	int id, x_value, y_value, w_value, h_value;	//position of current marker
	int ham_error = 0;				//whether contains hamming error
	double time;					//the time that a moving marker consumes

	for (;;) {

		//receive radio message from queue
		if (s4402815_QueueRadioMsg != NULL) {
	
			//Check whether received message comes from orb or rover
			if (rec_msg.packet[0] == 0x40) {

				// in this case, there is a crc error
				if(rec_msg.error_state == 1) {
					debug_printf("crc error\n");
					continue;
				}

				//extract the information from the raw message
				for (int i = 0; i < 10; i++) {
					temp_raw = (rec_msg.packet[10 + 2*i] << 8) | rec_msg.packet[10 + 2*i + 1];
					//hamming decode
					temp_decoded[i] = s4402815_hamdec(temp_raw);
					if (temp_decoded[i] == -1)
						ham_error = 1;
				}
				//in this case, there are two or more hamming errors
				if(ham_error == 1) {
					ham_error = 0;
					debug_printf("two or more bits wrong and cannot be corrected\n");
					continue;
				}

				// get the id, x, y, w, h value from received message
				id = temp_decoded[0] | (temp_decoded[1] << 8);
				x_value = temp_decoded[2] | (temp_decoded[3] << 8);
				y_value = temp_decoded[4] | (temp_decoded[5] << 8);
				w_value = temp_decoded[6] | (temp_decoded[7] << 8);
				h_value = temp_decoded[8] | (temp_decoded[9] << 8);

				// print the parameters of current marker
				debug_printf("ID: %d\tX: %d\tY: %d\tW: %d\tH: %d\n\n", id, x_value, y_value, w_value, h_value);

				// calculate the time that current marker consumed
				double time_temp = (HAL_GetTick() - position_list[id][2])/1000000000;
				if (time_temp != 0) time = time_temp;

				// calculate the displacement
				double distance = 6 * sqrt(pow(abs(x_value - position_list[id][0]), 2) + pow(abs(y_value - position_list[id][1]), 2));

				//refresh the position of current marker
				position_list[id][0] = x_value;
				position_list[id][1] = y_value;
				position_list[id][2] = HAL_GetTick;

				// print marker's id and its velocity
				debug_printf("velocity of No. %d marker is %f\n\n", id, distance/time);
			}
		}
		//task delay
		vTaskDelay(30);
	}
}

void RoverMsg_Task(void) {

	int value_temp;
	struct dualtimer_msg led_msg;	//used to store led value
	char cInputString[] = {"gettime"};
	char *pcOutputString;

	/* Initialise pointer to CLI output buffer. */
	//memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for (;;) {

		//Check the type of received message
		if (rec_msg.packet[0] == 0x30) {
			//key = rec_msg.packet[10];
			//check crc error
			if (rec_msg.error_state == 1)
				debug_printf("crc error, try again please\n");

			//hamming decode and error checking
			value_temp = s4402815_hamdec((rec_msg.packet[11] << 8) | rec_msg.packet[12]);
			if (value_temp == -1)
				debug_printf("2 or more bits hamming error, try again please\n");
			else key = value_temp;

			//if receive semaphore, display the key value
			if (s4402815_SemaphoreGetKey != NULL) {
				//If the semaphore is not available, wait 10 ticks to see if it becomes free.
				if( xSemaphoreTake( s4402815_SemaphoreGetKey, 10 ) == pdTRUE ) {
					debug_printf("current passkey is: 0x%02x\n", key);
					FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
					debug_printf("%s\r",pcOutputString);
					//vTaskDelay(5);	//Must delay between debug_printfs.
				}
			}
		}

		//Check the type of received message
		if (rec_msg.packet[0] == 0x31) {

			//check crc error
			if (rec_msg.error_state == 1)
				debug_printf("crc error, try again please\n");

			//hamming decode and error checking
			value_temp = s4402815_hamdec((rec_msg.packet[11] << 8) | rec_msg.packet[12]);
			if (value_temp == -1)
				debug_printf("2 or more bits hamming error, try again please\n");
			else sensor_value = value_temp;

			//if receive semaphore, display the sensor value
			if (s4402815_SemaphoreGetSensor != NULL) {
				//If the semaphore is not available, wait 10 ticks to see if it becomes free.
				if( xSemaphoreTake( s4402815_SemaphoreGetSensor, 10 ) == pdTRUE ) {
					debug_printf("sensor value is: 0x%02x\n", sensor_value);
					FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
					debug_printf("%s\r",pcOutputString);//
vTaskDelay(5);	//Must delay between debug_printfs.

					led_msg.type = 'e';
					led_msg.timer_value = 0x61;
					//Send received message to the front of the queue - wait atmost 10 ticks
					if (s4402815_QueueLightBar != NULL) {
						if( xQueueSendToFront(s4402815_QueueLightBar, (void *)&led_msg, ( portTickType ) 20 ) != pdPASS ) {
							debug_printf("Failed to post the message, after 10 ticks.\n\r");
						}
						//else debug_printf("led queue success\n");
					}
				}
			}
		}
		//task delay
		vTaskDelay(15);
	}
}
/**
  * @brief  Task for led.
  * @param  None
  * @retval None
  */
void LedToggle_Task(void) {

	for(;;) {
		//toggle led
		BRD_LEDToggle();

		// task delay 
		vTaskDelay(500);
	}
}
/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
void Hardware_init( void ) {
	
	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED

	GPIO_InitTypeDef  GPIO_InitStructure;

	portENABLE_INTERRUPTS();	//Enable interrupts

	ulHighFrequencyTimerTicks = 0;
	sequence = 0;
	key = 0;

	unsigned short PrescalerValue;

	/* Timer 4 clock enable */
	__TIM4_CLK_ENABLE();

	/* Compute the prescaler value */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

  	/* Time base configuration */
	TIM_Init.Instance = TIM4;				//Enable Timer 2
  	TIM_Init.Init.Period = 50000/50000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 4 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM4_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM4_IRQn, (uint32_t)&tim4_irqhandler);  
	NVIC_EnableIRQ(TIM4_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);
}

/**
  * @brief  set the output color of tasks in different states
  * @param  None
  * @retval None
  */
void color_set(char* state) {

	if(strcmp(state, "ready") == 0) debug_printf("\033[1;33m\r");
	else if(strcmp(state, "running") == 0) debug_printf("\033[0;32m\r");
	else if(strcmp(state, "blocked") == 0) debug_printf("\033[0;31m\r");
	else if(strcmp(state, "suspend") == 0) debug_printf("\033[0;34m\r");
}

/**
  * @brief  Timer 4 Interrupt handler
  * @param  None.
  * @retval None
  */
void tim4_irqhandler (void) {

	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	//increase the couter
	ulHighFrequencyTimerTicks++;
}

/**
  * @brief  Application Tick Task.
  * @param  None
  * @retval None
  */
void vApplicationTickHook( void ) {

	BRD_LEDOff();
}

/**
  * @brief  Idle Application Task
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static portTickType xLastTx = 0;

	BRD_LEDOff();

	for (;;) {

		/* The idle hook simply prints the idle tick count, every second */
		if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {

			xLastTx = xTaskGetTickCount();

			//debug_printf("IDLE Tick %d\n", xLastTx);

			/* Blink Alive LED */
			BRD_LEDToggle();		
		}
	}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

