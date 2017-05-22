/**
  ******************************************************************************
  * @file    stage6/main.c 
  * @author  YI LIU
  * @date    29042016
  * @brief   
  *			 See the FreeRTOSPlus CLI API for more information
  *			 http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>
#include "s4402815_pantilt.h"
#include "s4402815_cli.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void ApplicationIdleHook( void ); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void Box_Task( void );
void CLI_Task(void);

/* Private variables ---------------------------------------------------------*/
// Structure that defines the "laser" command line command.
CLI_Command_Definition_t xLaser = {
	"laser",
	"laser: iput on to turn on and off to turn off.\r\n",
	prvLaserCommand,
	1
};

// Structure that defines the "pan" command line command.
CLI_Command_Definition_t xPan = {
	"pan",
	"pan: input degree or direction. note: the angle limit is -70 to 70\r\n",
	prvPanCommand,
	1
};

// Structure that defines the "tilt" command line command.
CLI_Command_Definition_t xTilt = {
	"tilt",
	"tilt: input degree or direction. note: the angle limit is -70 to 70\r\n",
	prvTiltCommand,
	1
};

// Structure that defines the "box" command line command.
CLI_Command_Definition_t xBox = {
	"box",
	"box: when typing box, the laser will draw a box\r\n",
	prvBoxCommand,
	0
};

/* Task Priorities ------------------------------------------------------------*/
#define mainBox_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainCLI_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainPanTilt_PRIORITY				( tskIDLE_PRIORITY + 1 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainBox_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 1 )
#define mainCLI_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 1 )
#define mainPanTilt_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 1 )

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	
	/* Start the tasks and start the CLI. */
    xTaskCreate( (void *) &Box_Task, (const signed char *) "Box", mainBox_TASK_STACK_SIZE, NULL, mainBox_PRIORITY, NULL );
	xTaskCreate( (void *) &CLI_Task, (const signed char *) "CLI", mainCLI_TASK_STACK_SIZE, NULL, mainCLI_PRIORITY, NULL );
	xTaskCreate( (void *) &s4402815_TaskPanTilt, (const signed char *) "PanTilt", mainPanTilt_TASK_STACK_SIZE, (void*)&laser_status, mainPanTilt_PRIORITY, NULL );

	/* Create Semaphores */
	s4402815_SemaphoreLaser = xSemaphoreCreateBinary();
	s4402815_SemaphorePanLeft = xSemaphoreCreateBinary();
	s4402815_SemaphorePanRight = xSemaphoreCreateBinary();
	s4402815_SemaphoreTiltUp = xSemaphoreCreateBinary();
	s4402815_SemaphoreTiltDown = xSemaphoreCreateBinary();
	s4402815_SemaphoreBox = xSemaphoreCreateBinary();
	
	/* Register CLI commands */
	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xPan);
	FreeRTOS_CLIRegisterCommand(&xTilt);
	FreeRTOS_CLIRegisterCommand(&xBox);

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
  * @brief  Box Flashing Task.
  * @param  None
  * @retval None
  */
void Box_Task( void ) {

	BRD_LEDOff();
	int draw_counter = 0;
	int draw_flag = 0;

	for (;;) {

		/*receive semaphore to start draw a box, wait most 10 ticks*/
		if (s4402815_SemaphoreBox != NULL) {
			if (xSemaphoreTake(s4402815_SemaphoreBox, 10) == pdTRUE) {

				//when typing box, start draw a box, type again, stop drawing
				/*if (draw_flag == 1) {
					draw_flag = 0;
					draw_counter = 0;
				}
				else */draw_flag = 1;
			}
		}
		if (draw_flag == 1) {

			switch(draw_counter) {
				case 0: s4402815_pantilt_angle_write(1, pan_angle = 20);
						s4402815_pantilt_angle_write(2, tilt_angle = 70);
						break;
				case 1: s4402815_pantilt_angle_write(2, tilt_angle = 50); break;
				case 2: s4402815_pantilt_angle_write(1, pan_angle = -20);break;
				case 3: s4402815_pantilt_angle_write(2, tilt_angle = 70);break;
				case 4: s4402815_pantilt_angle_write(1, pan_angle = 20);break;
				default: break;
			}

			draw_counter++;
			if (draw_counter > 4) {
				draw_counter = 0;
				draw_flag = 0;
			}
			
		}
		BRD_LEDToggle();	//toggle led
		
		vTaskDelay(300);	//Delay the task for 300ms

	}
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

			/* Put byte into USB buffer */
			debug_putc(cRxedChar);
		
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

		vTaskDelay(50);
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
	s4402815_pwm_init();

	GPIO_InitTypeDef  GPIO_InitStructure;

	/*Enable D0 clock to output laser signal*/
	__BRD_D0_GPIO_CLK();

	/* Configure the D0 pin as an output */
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	portENABLE_INTERRUPTS();	//Enable interrupts
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

