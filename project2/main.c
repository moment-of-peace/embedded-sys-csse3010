/**
  ******************************************************************************
  * @file    project2/main.c 
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

#include "s4402815_pantilt.h"
#include "s4402815_hamming.h"
#include "s4402815_radio.h"
#include "s4402815_acc.h"
#include "s4402815_lightbar.h"
#include "s4402815_cli.h"

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
void CRC_Task(void);
void AccRead_Task(void);
void RadioRec_Task(void);
void Tracking_Task(void);
void RoverMsg_Task(void);
void AutoCtrl_Task(void);
void AccCtrl_Task(void);
void tim2_irqhandler (void);
void exti_pb_irqhandler(void);

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

// Structure that defines the "roverchan" command line command.
CLI_Command_Definition_t xRoverchan = {
	"roverchan",
	"roverchan: set rover channel\r\n",
	prvRoverchanCommand,
	1
};

// Structure that defines the "roveradd" command line command.
CLI_Command_Definition_t xRoveradd = {
	"roveradd",
	"roveradd: set rover address\r\n",
	prvRoveraddCommand,
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

// Structure that defines the "distance" command line command.
CLI_Command_Definition_t xDistance = {
	"distance",
	"distance: display the distance of the rover from the starting edge\r\n",
	prvDistanceCommand,
	0
};

// Structure that defines the "position" command line command.
CLI_Command_Definition_t xPosition = {
	"position",
	"position: display the current position of the rover using laser\r\n",
	prvPositionCommand,
	1
};

// Structure that defines the "Calibration" command line command.
CLI_Command_Definition_t xCalibration = {
	"Calibration",
	"Calibration: Calibrate linear motion of rover\r\n",
	prvCalibrationCommand,
	1
};

// Structure that defines the "roverid" command line command.
CLI_Command_Definition_t xRoverid = {
	"roverid",
	"roverid: set the id of a rover\r\n",
	prvRoveridCommand,
	1
};

// Structure that defines the "waypoint" command line command.
CLI_Command_Definition_t xWaypoint = {
	"waypoint",
	"waypoint: move to the waypoint defined by the marker ID\r\n",
	prvWaypointCommand,
	1
};

// Structure that defines the "follower" command line command.
CLI_Command_Definition_t xFollower = {
	"follower",
	"follower: follow the marker\r\n",
	prvFollowerCommand,
	0
};

// Structure that defines the "laser" command line command.
CLI_Command_Definition_t xLaser = {
	"laser",
	"laser: iput on to turn on and off to turn off.\r\n",
	prvLaserCommand,
	1
};

// Structure that defines the "acccontrol" command line command.
CLI_Command_Definition_t xAcccontrol = {
	"acccontrol",
	"acccontrol: iput on to turn on and off to turn off.\r\n",
	prvAcccontrolCommand,
	1
};

// structure used for autocontrl of rover
struct AutoCtrlMsg {
	int rover_x;
	int rover_y;
	int target_x;
	int target_y;
	int id;
};

TaskHandle_t handle_list[15];			//used to store tasks handles
char* name_list[15];					//used to store tasks names
unsigned int priority_list[15] = {0};	//used to store tasks priorities
char* state_list[15] = {0};				//used to store tasks state
int number_list[15] = {0};				//used to store tasks number
double runtime_list[15] = {0};			//used to store tasks runtime
unsigned long ulPercentage[15] = {0};	//used to store tasks runtime precentage
int ctrl_flag = 0;

QueueHandle_t QueueTrack;			// used to transfer position message for tracking
QueueHandle_t QueueRover;			// used to transfer message received from rover
QueueHandle_t QueueAutoCtrl;		// used to transfer rover autocontrol message
QueueHandle_t QueueAccCtrl;			// used for menu selecting

SemaphoreHandle_t SemaphorePB;	/* Semaphore for pushbutton interrupt */

/* Task Priorities ------------------------------------------------------------*/
#define mainCLI_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainTop_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainSuspend_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainResume_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainACC_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainAccRead_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainRadio_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainRadioRec_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainTracking_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainLed_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainRoverMsg_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainLightBar_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainPanTilt_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainAutoCtrl_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainAccCtrl_PRIORITY				( tskIDLE_PRIORITY + 1 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainCLI_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 2 )
#define mainTop_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 4 )
#define mainSuspend_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 1 )
#define mainResume_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 1 )
#define mainACC_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 3 )
#define mainAccRead_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 3 )
#define mainRadio_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
#define mainRadioRec_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainTracking_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLed_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 1 )
#define mainRoverMsg_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLightBar_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 1 )
#define mainPanTilt_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
#define mainAutoCtrl_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainAccCtrl_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 6 )

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
	xTaskCreate( (void *) &Top_Task, (const signed char *) "Top", mainTop_TASK_STACK_SIZE, NULL, mainTop_PRIORITY, handle_list+1 );
	xTaskCreate( (void *) &s4402815_TaskAcc, (const signed char *) "ACC", mainACC_TASK_STACK_SIZE, NULL, mainACC_PRIORITY, handle_list+2 );
	xTaskCreate( (void *) &AccRead_Task, (const signed char *) "AccRead", mainAccRead_TASK_STACK_SIZE, NULL, mainAccRead_PRIORITY, handle_list+3 );
	xTaskCreate( (void *) &s4402815_TaskRadio, (const signed char *) "Radio", mainRadio_TASK_STACK_SIZE, NULL, mainRadio_PRIORITY, handle_list+4 );
	xTaskCreate( (void *) &RadioRec_Task, (const signed char *) "RadioRec", mainRadioRec_TASK_STACK_SIZE, NULL, mainRadioRec_PRIORITY, handle_list+5 );
	xTaskCreate( (void *) &Tracking_Task, (const signed char *) "Track", mainTracking_TASK_STACK_SIZE, NULL, mainTracking_PRIORITY, handle_list+6 );
	xTaskCreate( (void *) &RoverMsg_Task, (const signed char *) "RoverMsg", mainRoverMsg_TASK_STACK_SIZE, NULL, mainRoverMsg_PRIORITY, handle_list+7 );
	xTaskCreate( (void *) &s4402815_TaskLightBar, (const signed char *) "LightBar", mainLightBar_TASK_STACK_SIZE, NULL, mainLightBar_PRIORITY, handle_list+8 );
	xTaskCreate( (void *) &s4402815_TaskPanTilt, (const signed char *) "PanTilt", mainPanTilt_TASK_STACK_SIZE, (void*)&laser_status, mainPanTilt_PRIORITY, handle_list+9 );
	xTaskCreate( (void *) &AutoCtrl_Task, (const signed char *) "AutoCtrl", mainAutoCtrl_TASK_STACK_SIZE, NULL, mainAutoCtrl_PRIORITY, handle_list+10 );
	xTaskCreate( (void *) &AccCtrl_Task, (const signed char *) "AccCtrl", mainAccCtrl_TASK_STACK_SIZE, NULL, mainAccCtrl_PRIORITY, handle_list+11 );
	xTaskCreate( (void *) &LedToggle_Task, (const signed char *) "Led", mainLed_TASK_STACK_SIZE, NULL, mainLed_PRIORITY, handle_list+12 );
	
	
	/* Create Semaphores */
	s4402815_SemaphoreTop = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccReadRaw = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccReadPL = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccOn = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccOff = xSemaphoreCreateBinary();
	s4402815_SemaphoreTrackOn = xSemaphoreCreateBinary();
	s4402815_SemaphoreTrackOff = xSemaphoreCreateBinary();
	s4402815_SemaphorePositionOn = xSemaphoreCreateBinary();
	s4402815_SemaphorePositionOff = xSemaphoreCreateBinary();
	s4402815_SemaphoreFollow = xSemaphoreCreateBinary();
	s4402815_SemaphoreDistance = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccCtrlOn = xSemaphoreCreateBinary();
	s4402815_SemaphoreAccCtrlOff = xSemaphoreCreateBinary();
	SemaphorePB = xSemaphoreCreateBinary();

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
	FreeRTOS_CLIRegisterCommand(&xRoverchan);
	FreeRTOS_CLIRegisterCommand(&xRoveradd);
	FreeRTOS_CLIRegisterCommand(&xForward);
	FreeRTOS_CLIRegisterCommand(&xReverse);
	FreeRTOS_CLIRegisterCommand(&xAngle);
	FreeRTOS_CLIRegisterCommand(&xDistance);
	FreeRTOS_CLIRegisterCommand(&xPosition);
	FreeRTOS_CLIRegisterCommand(&xCalibration);
	FreeRTOS_CLIRegisterCommand(&xRoverid);
	FreeRTOS_CLIRegisterCommand(&xWaypoint);
	FreeRTOS_CLIRegisterCommand(&xFollower);
	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xAcccontrol);

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
	int flag = 0;

	/* Initialise pointer to CLI output buffer. */
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for (;;) {

		
		/* Receive character */
		cRxedChar = debug_getc();

		if (cRxedChar == 27) flag = 1;
		if (cRxedChar == 91 && flag == 1) flag = 2;
		if (cRxedChar > 64 && cRxedChar < 69 && flag == 2) flag = 3;

		/* Process if chacater if not Null */
		if (cRxedChar != '\0') {

			//set color
			debug_printf("\033[0;36m");

			/* Put byte into USB buffer if input is not backspace */
			if (cRxedChar != 127 && cRxedChar != 27 && flag != 2 && flag != 3) {
				
				debug_putc(cRxedChar);
			}

			/* if input is backspace, clear previous character*/
			if (cRxedChar == 127)
				debug_printf("\b \b");
		
			/* Process only if return is received. */
			if (cRxedChar == '\r') {

				debug_printf("\033[1;35m");
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

				} else if (cRxedChar == 65 && flag == 3) {
					flag = 0;
					
					debug_printf("\033[1A");
				} else if (cRxedChar == 66 && flag == 3) {
					flag = 0;
					
					debug_printf("\033[1B");
				} else if (cRxedChar == 67 && flag == 3) {
					flag = 0;
					
					debug_printf("\033[1C");
				} else if (cRxedChar == 68 && flag == 3) {
					flag = 0;

					debug_printf("\033[1D");
					debug_printf("\033[1D");
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
	struct Task task_ctrl;

	s4402815_QueueTaskControl = xQueueCreate(3, sizeof(task_ctrl));

	// get the idle task handle before the for loop
	handle_list[13] = xTaskGetIdleTaskHandle();
	
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

		//Check if queue exists
		if (s4402815_QueueTaskControl != NULL) {
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueTaskControl, &task_ctrl, 10)) {

				//suspend or resume a task
				if (task_ctrl.type == 's') {
					//suspend the target task
					vTaskSuspend(handle_list[task_ctrl.handle]);
					debug_printf("already suspended\n");
				}
				else if (task_ctrl.type == 'r') {
					//resume the target task
					vTaskResume(handle_list[task_ctrl.handle]);
					debug_printf("already resumed\n");
				}
			}
		}

		//task delay 
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

	struct Accvalue acc_value;	//struct used to receive x, y, z value f
	int counter = 0;

	QueueAccCtrl = xQueueCreate(3, sizeof(acc_value));

	//check whether semophore exists
	if (s4402815_SemaphoreAccOn != NULL) {
		//give semaphore to enable acc measurement
		xSemaphoreGive(s4402815_SemaphoreAccOn);
	}

	for (;;) {

		//receive 12-bit acc x, y, z values and pl status from queue
		if (s4402815_QueueAccValue != NULL) {	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueAccValue, &acc_value, 10)) {

				//if receive semaphore, display the raw value
				if (s4402815_SemaphoreAccReadRaw != NULL) {
					//If the semaphore is not available, wait 10 ticks to see if it becomes free.
					if( xSemaphoreTake( s4402815_SemaphoreAccReadRaw, 10 ) == pdTRUE ) {
						//print out x, y, z value of accelerometer
						debug_printf("acc x value: %d\n", acc_value.x_value);
						debug_printf("acc y value: %d\n", acc_value.y_value);
						debug_printf("acc z value: %d\n\n", acc_value.z_value);
					}
				}

				//if receive semaphore, display the pl value
				if (s4402815_SemaphoreAccReadPL != NULL) {
					//If the semaphore is not available, wait 10 ticks to see if it becomes free.
					if( xSemaphoreTake( s4402815_SemaphoreAccReadPL, 10 ) == pdTRUE ) {
						//print out pl status of accelerometer
						debug_printf("pl status: %c\tbf status: %c\n\n", acc_value.pl, acc_value.bf);
					}
				}
			}
		}

		// this loop is used to delay
		if (counter % 10 == 0) {
			counter = 0;

			// control rouver using accelarometer
			if ( acc_value.z_value < 950 && acc_value.y_value > -800) {

				// Send received message to the front of the queue - wait atmost 10 ticks
				if (QueueAccCtrl != NULL) {
					if( xQueueSendToFront(QueueAccCtrl, (void *)&acc_value, ( portTickType ) 10 ) != pdPASS ) {
						debug_printf("Failed to post acc ctrl message, after 10 ticks.\n\r");
					}
				}
				// control rover using accelerometer
				if (ctrl_flag == 0) {
					if (acc_value.pl == '<') {
						s4402815_rover_control(0x32, speed3, 0, 1, 0x0a);
					}
					else if (acc_value.pl == '>'){
						s4402815_rover_control(0x32, speed3, 0, 1, 0x05);
					}
					else if (acc_value.bf == 'f' && acc_value.z_value > 500){
						s4402815_rover_control(0x32, speed1, speed2, 1, 0x05);
					}
					else if (acc_value.bf == 'b' && acc_value.z_value < -500){
						s4402815_rover_control(0x32, speed1, speed2, 1, 0x0a);
					}
				}				
			}
		}

		counter++;
		vTaskDelay(40);
	}
}

void RadioRec_Task(void) {

	struct Message rec_msg;			//used to store radio message from queue
	int counter = 0;
	uint16_t crc_value;				// calculated crc value
	uint16_t crc_byte;				// received crc vlaue

	QueueTrack = xQueueCreate(3, sizeof(rec_msg));
	QueueRover = xQueueCreate(3, sizeof(rec_msg));

	for (;;) {

		//receive radio message from queue
		if (s4402815_QueueRadioMsg != NULL) {
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueRadioMsg, &rec_msg, 10)) {

				crc_value = s4402815_crc((char *)rec_msg.packet, 30);
				crc_byte = rec_msg.packet[30] | (rec_msg.packet[31] << 8);

				//send crc error message and received message
				if (crc_value != crc_byte) {
					//debug_printf("crc error\n");
					continue;
				}

				// rececive 0x40 type message, send to tracking task
				if (rec_msg.packet[0] == 0x40) {
					//Send received message to the front of the queue - wait atmost 10 ticks
					if (QueueTrack != NULL) {
						if( xQueueSendToFront(QueueTrack, (void *)&rec_msg, ( portTickType ) 20 ) != pdPASS ) {
							debug_printf("Failed to post the message, after 10 ticks.\n\r");
						}
					}
				}

				// receive 0x30 or 0x31 type message
				if ((rec_msg.packet[0] == 0x30 || rec_msg.packet[0] == 0x31) && rec_msg.packet[1] == 0x56 && rec_msg.packet[2] == 0x81 && rec_msg.packet[3] == 0x02 && rec_msg.packet[4] == 0x44) {

					//Send received message to the front of the queue - wait atmost 10 ticks
					if (QueueRover != NULL) {
						if( xQueueSendToFront(QueueRover, (void *)&rec_msg, ( portTickType ) 20 ) != pdPASS ) {
							debug_printf("Failed to post the message, after 10 ticks.\n\r");
						}
					}
				}

				//0x36 type message
				else if (rec_msg.packet[0] == 0x36 && rec_msg.packet[1] == 0x56 && rec_msg.packet[2] == 0x81 && rec_msg.packet[3] == 0x02 && rec_msg.packet[4] == 0x44) {
					debug_printf("crc wrong, send again\n");

					//set channel and address
					s4402815_radio_setchan(re_channel);
					s4402815_radio_settxaddress(re_address);
				}
			}
		}
		//task delay
		vTaskDelay(30);
	}
}
/**
  * @brief  Task for tracking a marker, get the parameters of different markers, including id, x, y, width
  * 		and heigh values, and calculate the velocity of marker.
  * @param  None
  * @retval None
  */
void Tracking_Task(void) {

	struct Message track_msg;					//used to store radio message from queue
	struct AutoCtrlMsg auto_msg;
	uint16_t temp_raw;
	uint8_t temp_decoded[10];		//decoded playload
	uint16_t position_list[4][4] = {0};	//used to record the positions of different markers
	//uint16_t rover_position[
	int marker_id = 0;
	int follow_id = 1;
	int ham_error = 0;				//whether contains hamming error
	//double time;					//the time that a moving marker consumes
	int counter = 0;
	int track_flag = 0;
	int position_flag = 0;
	int pan_angle = 22;
	int tilt_angle = 70;

	QueueAutoCtrl = xQueueCreate(3, sizeof(auto_msg));

	for (;;) {

		// receive semaphore to enable track
		if (s4402815_SemaphoreTrackOn != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreTrackOn, 10 ) == pdTRUE ) {
				// if receive, start to send message to queue
				track_flag = 1;
			}
		}

		// receive semaphore to disable track
		if (s4402815_SemaphoreTrackOff != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreTrackOff, 10 ) == pdTRUE ) {
				// if receive, stop sending message to queue
				track_flag = 0;
			}
		}

		// receive semaphore to enable track
		if (s4402815_SemaphorePositionOn != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphorePositionOn, 10 ) == pdTRUE ) {
				// if receive, start to send message to queue
				position_flag = 1;
				//debug_printf("receive pos sema\n");
			}
		}

		// receive semaphore to disable track
		if (s4402815_SemaphorePositionOff != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphorePositionOff, 10 ) == pdTRUE ) {
				// if receive, stop sending message to queue
				position_flag = 0;
			}
		}

		//receive radio message from queue
		if (QueueTrack != NULL) {
	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( QueueTrack, &track_msg, 10)) {
				//extract the information from the raw message
				for (int i = 0; i < 10; i++) {
					temp_raw = (track_msg.packet[10 + 2*i] << 8) | track_msg.packet[10 + 2*i + 1];
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

				// get the id
				marker_id = temp_decoded[0] | (temp_decoded[1] << 8);

				//set the id of the target which rover follows
				if (marker_id != rover_id) follow_id = marker_id;

				// x, y, w, h value from received message
				position_list[marker_id][0] = temp_decoded[2] | (temp_decoded[3] << 8);
				position_list[marker_id][1] = temp_decoded[4] | (temp_decoded[5] << 8);
				position_list[marker_id][2] = temp_decoded[6] | (temp_decoded[7] << 8);
				position_list[marker_id][3] = temp_decoded[8] | (temp_decoded[9] << 8);
			}
		}

		// display tracking information
		if (track_flag == 1) {
			if (counter % 20 == 0) {
				// print the parameters of current marker
				debug_printf("ID: %d\tX: %d\tY: %d\tW: %d\tH: %d\n\n", marker_id, position_list[marker_id][0], position_list[marker_id][1], position_list[marker_id][2], position_list[marker_id][3]);
			}
		}

		// show the position using pan tilt and laser
		if (position_flag == 1) {
			// calculate the angle of pan and tilt
			pan_angle = 22 - position_list[rover_id][0] * 44 / 300;
			tilt_angle = 48 + position_list[rover_id][1] * 22 / 200;

			// used to delay
			if (counter %3 == 0) {
				if (s4402815_QueuePan != NULL) {	// Check if queue exists

					// Send message to the front of the queue - wait atmost 10 ticks
					if( xQueueSend(s4402815_QueuePan, ( void * ) &pan_angle, ( portTickType ) 10 ) != pdPASS ) {
						debug_printf("Failed to post pos message, after 10 ticks.\n\r");
					}
				}
				if (s4402815_QueueTilt != NULL) {	// Check if queue exists

					// Send message to the front of the queue - wait atmost 10 ticks
					if( xQueueSend(s4402815_QueueTilt, ( void * ) &tilt_angle, ( portTickType ) 10 ) != pdPASS ) {
						debug_printf("Failed to post the message, after 10 ticks.\n\r");
					}
				}
			}
		}

		//if receive semaphore, display the distance value
		if (s4402815_SemaphoreDistance != NULL) {
			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreDistance, 10 ) == pdTRUE ) {
				double rover_dis = sqrt(position_list[rover_id][0]*position_list[rover_id][0] + position_list[rover_id][1]*position_list[rover_id][1]);
				debug_printf("rover distance: %.2fmm\n", rover_dis * 3);
			}
		}

		// send poisition information for rover autocontrol
		if (counter %10 == 0) {
			auto_msg.rover_x = position_list[rover_id][0];
			auto_msg.rover_y = position_list[rover_id][1];
			auto_msg.target_x = position_list[follow_id][0];
			auto_msg.target_y = position_list[follow_id][1];
			auto_msg.id = follow_id;

			if (QueueAutoCtrl != NULL) {	// Check if queue exists
				// Send message to the front of the queue - wait atmost 10 ticks
				if( xQueueSend(QueueAutoCtrl, ( void * ) &auto_msg, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post autoctrl message, after 10 ticks.\n\r");
				}
			}
		}
		counter++;
		if (counter == 20) counter = 0;	
		//task delay
		vTaskDelay(30);
	}
}

void RoverMsg_Task(void) {

	int value_temp;
	struct dualtimer_msg led_msg;	//used to store led value
	struct Message rover_msg;
	int sensor_value = 0;					//sensor value of rover

	for (;;) {

		if (QueueRover != NULL) {
	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( QueueRover, &rover_msg, 10)) {
				if (rover_msg.packet[0] == 0x30) {
					
					//hamming decode and error checking
					value_temp = s4402815_hamdec((rover_msg.packet[11] << 8) | rover_msg.packet[12]);

					if (value_temp != -1) {
						// display key value
						key = value_temp;
						debug_printf("time: %.2fs  current passkey is: 0x%02x\n", s4402815_current_time(), key);
					}
					else 
						debug_printf("2 or more bits hamming error, try again please\n");

				}

				//Check the type of received message
				if (rover_msg.packet[0] == 0x31) {

					//hamming decode and error checking
					value_temp = s4402815_hamdec((rover_msg.packet[11] << 8) | rover_msg.packet[12]);

					if (value_temp != -1) {
						// display sensor value
						sensor_value = value_temp;
						debug_printf("time: %.2fs  current sensor value is: 0x%02x\n", s4402815_current_time(), sensor_value);

						//set led values
						led_msg.type = 'e';
						led_msg.timer_value = sensor_value;
						//Send message to the front of the queue - wait atmost 10 ticks
						if (s4402815_QueueLightBar != NULL) {
							if( xQueueSendToFront(s4402815_QueueLightBar, (void *)&led_msg, ( portTickType ) 20 ) != pdPASS ) {
								debug_printf("Failed to post the message, after 10 ticks.\n\r");
							}
						}
					}
					else 
						debug_printf("2 or more bits hamming error, try again please\n");
				}
			}
		}
		//task delay
		vTaskDelay(15);
	}
}

/**
  * @brief  Task for rover autocontrol.
  * @param  None
  * @retval None
  */
void AutoCtrl_Task(void) {

	struct AutoCtrlMsg auto_msg;
	int target_id = 0;
	int flag = 0;
	int mode = 0;				//mode 0 is waypoint, and mode 1 is follower
	int counter = 0;
	double orien_rover = 0;		// the oientation of rover
	double distance = 0;		// distance between rover and target
	double pre_dis = 0;
	double angle = 0;			// the angle of target
	double x_diff = 1;			// difference of x coordinate
	double y_diff = 1;			// difference of y coordinate

	//Create queue to receive target marker id
	s4402815_QueueWayPointID = xQueueCreate(5, sizeof(target_id));

	for(;;) {

		//receive radio message from queue
		if (s4402815_QueueWayPointID != NULL) {
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( s4402815_QueueWayPointID, &target_id, 10)) {
				mode = 0;
				flag = 1;
			}
		}

		//if receive semaphore, display the key value
		if (s4402815_SemaphoreFollow != NULL) {
			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreFollow, 10 ) == pdTRUE ) {
				mode = 1;
				flag = 1;
				target_id = -1;
			}
		}

		if (QueueAutoCtrl != NULL) {
	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( QueueAutoCtrl, &auto_msg, 5)) {
				if (flag == 1) {

					// judge whether received position is target position
					if (target_id == -1 || target_id == auto_msg.id) {
						// calculate x , y difference and distance
						x_diff = auto_msg.rover_x - auto_msg.target_x;
						y_diff = auto_msg.rover_y - auto_msg.target_y;
						distance = pow(x_diff, 2) + pow(y_diff, 2);

						//calculate the angle rover should rotate
						if (y_diff / x_diff > 100)
							angle = 90;
						else if (y_diff / x_diff < -100)
							angle = -90;
						else
							angle = atan(y_diff / x_diff) * 180 / 3.14;
					}

					// judge whether the distance is larger than threshold
					if (distance > 4000) {
						if (angle - orien_rover > 5) {
							// rotate anticlockwise
							s4402815_rover_control(0x32, speed3, 0, 1, 0x05);//angle
							orien_rover += 10;
						}
						else if (angle - orien_rover < -5) {
							// rotate clockwise
							s4402815_rover_control(0x32, speed3, 0, 1, 0x0a);//angle
							orien_rover -= 10;
					
						}
						else {
							// move the rover forward
							s4402815_rover_control(0x32, speed1, speed2, 1, 0x05);//forward
						}
					}
					// when arriving at target
					else {
						if (mode == 0) {
							flag = 0;
							orien_rover = 0;
						}
						debug_printf("end\n");
						pre_dis = 0;
					}
					
					// orientation calibration
					if(counter%5 == 0) {
						if ((distance - pre_dis) > 2000 && pre_dis != 0) {
							s4402815_rover_control(0x32, speed3, 0, 28, 0x0a);
						}
						pre_dis = distance;
					}

					counter++;
				}
			}
		}

		
//debug_printf("auto e\n");
		// task delay 
		vTaskDelay(80);
	}
}

/**
  * @brief  Task for AccCtrl.
  * @param  None
  * @retval None
  */
void AccCtrl_Task(void) {

	struct Accvalue acc_value;	//struct used to receive x, y, z value and pl status
	int i = 0;
	int j = 0;
	char* commands[7];		// all commands need to display in menu
	//all parameters of each command
	char* para[5][5] = {"  10", "  20", "  30", "  40", "  50","  10", "  20", "  30", "  40", "  50","  10", "  20", "  30", "  40", "  50" ," a50", " a60", " b50", " b60", " c50","   1", "   2", "   3", "   4","   5"};
	commands[0] = "forward     ";
	commands[1] = "reverse     ";
	commands[2] = "angle       ";
	commands[3] = "Calibration ";
	commands[4] = "waypoint    ";
	commands[5] = "distance    ";
	commands[6] = "follower    ";
	char *pcOutputString;

	/* Initialise pointer to CLI output buffer. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for(;;) {

		//if receive semaphore, enable acc control
		if (s4402815_SemaphoreAccCtrlOn != NULL) {
			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreAccCtrlOn, 10 ) == pdTRUE ) {
				ctrl_flag = 1;

			// display the menu
		menu:	debug_printf("\n");
				for (int m = 0; m < 7; m++) {
					if (m == 0) debug_printf("\033[0;36m\r");
					else debug_printf("\033[0m\r"); //set the output color to default
					debug_printf("%s\n", commands[m]);
				}
				// adjust cursor position
				debug_printf("\033[7A");
				debug_printf("\033[12C");
				i = 0;
				j = 0;
			}
		}

		//if receive semaphore, disable acc control
		if (s4402815_SemaphoreAccCtrlOff != NULL) {
			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreAccCtrlOff, 10 ) == pdTRUE ) {
				ctrl_flag = 0;
			}
		}

		//pushbutton interrupt semaphore
		if (SemaphorePB != NULL) {
			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( SemaphorePB, 10 ) == pdTRUE ) {

				// show the sub menu in this case
				if (j == 0 && i < 5) {
					for (int m = 0; m < 5; m++) {
						if (m == 0) debug_printf("\033[0;36m");
						else debug_printf("\033[0m");
						
						debug_printf("%s", para[i][m]);
					}
					debug_printf("\033[20D");
					j = 1;
				}
				// excute command in this case
				else {
					// command with parameter
					if (i < 5) {
						char* input_string;
						input_string = pvPortMalloc(17 * sizeof(char));
						strcpy(input_string, commands[i]);
						strcat(input_string, para[i][j-1]);

						debug_printf("\033[6B\n");
						FreeRTOS_CLIProcessCommand( input_string, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
						debug_printf("%s\r",pcOutputString);
						//debug_printf("%s", input_string);
						vTaskDelay(10);
						goto menu;
					}
					// command without parameter
					else {
						char* input_string = commands[i];

						debug_printf("\033[6B\n");
						FreeRTOS_CLIProcessCommand( input_string, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
						debug_printf("%s\r",pcOutputString);
						//debug_printf("%s", input_string);
						vTaskDelay(10);
						goto menu;
					}
				}
			}
		}

		//receive acc message from queue
		if (QueueAccCtrl != NULL) {
	
			//Check for item received - block atmost for 10 ticks
			if (xQueueReceive( QueueAccCtrl, &acc_value, 10) && ctrl_flag == 1) {
				// move cursor toward left
				if (acc_value.pl == '<' && j > 1 && i < 5) {
					debug_printf("\033[0m%s", para[i][j-1]);
					debug_printf("\033[8D");
					j--;
				}
				// move cursor toward right
				else if (acc_value.pl == '>' && j > 0 && j < 5 && i < 5){
					
					debug_printf("\033[0m%s", para[i][j-1]);

					j++;
				}
				// move cursor toward up
				else if (acc_value.bf == 'f' && acc_value.z_value > 500 && i > 0){
					debug_printf("\033[12D");
					debug_printf("\033[0m\r%s", commands[i]);
					i--;
					j=0;
					debug_printf("\033[1A");
					debug_printf("\033[12D");
					debug_printf("\033[0;36m\r%s\033[0m", commands[i]);
				}
				// move cursor toward down
				else if (acc_value.bf == 'b' && acc_value.z_value < -500 && i < 6){
					debug_printf("\033[12D");
					debug_printf("\033[0m\r%s", commands[i]);
					i++;
					j=0;
					debug_printf("\033[1B");
					debug_printf("\033[12D");
					debug_printf("\033[0;36m\r%s\033[0m", commands[i]);
				}
				// show elements of a sub menu
				if (j != 0 && i < 5) {
					debug_printf("\033[0;36m%s\033[0m", para[i][j-1]);
					debug_printf("\033[4D");
				}
			}
		}

		// task delay 
		vTaskDelay(50);
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

	GPIO_InitTypeDef  GPIO_InitStructure;
	
	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED

	ulHighFrequencyTimerTicks = 0;

	//initialise extern variables
	sequence = 0;
	key = 0;
	speed1 = 46;
	speed2 = 40;
	speed3 = 44;
	speed1r = 46;
	speed2r = 40;
	speed3r = 44;
	speedx = 7;
	rover_id = 2;

	//set default value of rover channel
	tr_channel = 49;
	//set default value of rover address
	tr_address[0] = 0x49;
	tr_address[1] = 0x33;
	tr_address[2] = 0x22;
	tr_address[3] = 0x11;
	tr_address[4] = 0x00;

	unsigned short PrescalerValue;

	/* Timer 2 clock enable */
	__TIM2_CLK_ENABLE();

	/* Compute the prescaler value */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

  	/* Time base configuration */
	TIM_Init.Instance = TIM2;				//Enable Timer 2
  	TIM_Init.Init.Period = 50000/50000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM2_IRQn, (uint32_t)&tim2_irqhandler);  
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM_Init);

	/* Enable PB clock */
  	__BRD_PB_GPIO_CLK();

	/* Set priority of PB Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	HAL_NVIC_SetPriority(BRD_PB_EXTI_IRQ, 4, 0);	//Set Main priority ot 10 and sub-priority ot 0.

	//Enable PB interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_PB_EXTI_IRQ, (uint32_t)&exti_pb_irqhandler);  
	NVIC_EnableIRQ(BRD_PB_EXTI_IRQ);

  	/* Configure PB pin as pull down input */
	GPIO_InitStructure.Pin = BRD_PB_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
  	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_PB_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	portENABLE_INTERRUPTS();	//Enable interrupts
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
  * @brief  Timer 2 Interrupt handler
  * @param  None.
  * @retval None
  */
void tim2_irqhandler (void) {

	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	//increase the couter
	ulHighFrequencyTimerTicks++;
}

/**
  * @brief  Pushbutton Interrupt handler. Gives PB Semaphore
  * @param  None.
  * @retval None
  */
void exti_pb_irqhandler(void) {

	HAL_Delay(150);				//for deboucing

	BaseType_t xHigherPriorityTaskWoken;
	 
    /* Is it time for another Task() to run? */
    xHigherPriorityTaskWoken = pdFALSE;

	/* Check if Pushbutton external interrupt has occured */
  	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);				//Clear D0 pin external interrupt flag
    	
	if (SemaphorePB != NULL) {	/* Check if semaphore exists */
		xSemaphoreGiveFromISR( SemaphorePB, &xHigherPriorityTaskWoken );		/* Give PB Semaphore from ISR*/
	}
    
	/* Perform context switching, if required. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

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
