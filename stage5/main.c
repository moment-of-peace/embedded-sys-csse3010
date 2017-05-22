/**
  ******************************************************************************
  * @file    stage5/main.c 
  * @author  YI LIU
  * @date    26/04/2016
  * @brief   FreeRTOS practice, build tasks and semaphores to controll light bar
  *
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include "s4402815_lightbar.h"
#include "s4402815_sysmon.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Hardware_init();
void ApplicationIdleHook( void );
void TaskTimerRight( void );
void TaskTimerLeft( void );
void TaskPushButton(void);
void exti_pb_irqhandler(void);

SemaphoreHandle_t PBSemaphore;	/* Semaphore for pushbutton interrupt */
int press_counter = 0;

/* Task Priorities ------------------------------------------------------------*/
#define mainTIMERRIGHTTASK_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainTIMERLEFTTASK_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainLIGHTBARTASK_PRIORITY					( tskIDLE_PRIORITY + 3 )
#define mainPBTASK_PRIORITY							( tskIDLE_PRIORITY + 3 )

/* Task Stack Allocations -----------------------------------------------------*/
#define mainTIMERRIGHTTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainTIMERLEFTTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLIGHTBARTASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
#define mainPBTASK_STACK_SIZE				( configMINIMAL_STACK_SIZE * 2 )

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
	
	/* Start sender and receiver tasks */
	xTaskCreate( (void *) &TaskTimerLeft, (const signed char *) "SEND", mainTIMERLEFTTASK_STACK_SIZE, NULL, mainTIMERLEFTTASK_PRIORITY, NULL );
    xTaskCreate( (void *) &TaskTimerRight, (const signed char *) "SEND", mainTIMERRIGHTTASK_STACK_SIZE, NULL, mainTIMERRIGHTTASK_PRIORITY, NULL );
	xTaskCreate( (void *) &s4402815_TaskLightBar, (const signed char *) "RECV", mainLIGHTBARTASK_STACK_SIZE, NULL, mainLIGHTBARTASK_PRIORITY, NULL );
	xTaskCreate( (void *) &TaskPushButton, (const signed char *) "PB", mainPBTASK_STACK_SIZE, NULL, mainPBTASK_PRIORITY, NULL );

	/* Create Semaphores */
	PBSemaphore = xSemaphoreCreateBinary();

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
  * @brief  Sender Task.Send a message to the queue, every second.
  * @param  None
  * @retval None
  */
void TaskTimerLeft( void ) {
	
	struct dualtimer_msg LeftMessage; 

	/*Initialise Message Item payload */
	LeftMessage.type = 'l';
	
	LeftMessage.timer_value = 0;

	for (;;) {

		s4402815_LA_CHAN0_SET();		//for system monitor detect

		if (s4402815_QueueLightBar != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSendToFront(s4402815_QueueLightBar, ( void * ) &LeftMessage, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}

		if (press_counter % 4 == 0 || press_counter % 4 == 1)
			LeftMessage.timer_value++;		/* Increment Sequence Number */
		if (LeftMessage.timer_value > 31) LeftMessage.timer_value = 0;

		/* Wait for 1000ms in total */
		vTaskDelay(999);
		s4402815_LA_CHAN0_CLR();		//for system monitor detect
		vTaskDelay(1);
	}
}

/**
  * @brief  Sender Task.Send a message to the queue, every second.
  * @param  None
  * @retval None
  */
void TaskTimerRight( void ) {
	
	struct dualtimer_msg RightMessage; 

	/*Initialise Message Item payload */
	RightMessage.type = 'r';
	
	RightMessage.timer_value = 0;

	for (;;) {

		s4402815_LA_CHAN1_SET();		//for system monitor detect

		if (s4402815_QueueLightBar != NULL) {	/* Check if queue exists */

			/*Send message to the front of the queue - wait atmost 10 ticks */
			if( xQueueSendToFront(s4402815_QueueLightBar, ( void * ) &RightMessage, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}

		if (press_counter % 4 == 0 || press_counter % 4 == 3)
			RightMessage.timer_value++;		/* Increment Sequence Number */
		if (RightMessage.timer_value > 31) RightMessage.timer_value = 0;
	
		/* Wait for 100ms in total */
		vTaskDelay(99);
		s4402815_LA_CHAN1_CLR();		//for system monitor detect
		vTaskDelay(1);
	}
}

/**

  * @brief  Sender Task.Send a message to the queue, every second.
  * @param  None
  * @retval None
  */
void TaskPushButton( void ) {

	for (;;) {

		if (PBSemaphore != NULL) {	/* Check if semaphore exists */

			/* See if we can obtain the PB semaphore. If the semaphore is not available
           	wait 10 ticks to see if it becomes free. */
			if( xSemaphoreTake( PBSemaphore, 10 ) == pdTRUE ) {
				press_counter++;
				if (press_counter > 999) press_counter = 0;

				switch (press_counter % 4) {
					case 0: debug_printf("the both timers start\n"); break;
					case 1: debug_printf("the right timer stops\n"); break;
					case 2: debug_printf("the both timers stop\n"); break;
					case 3: debug_printf("the left timer stops\n"); break;
					default : break;
				}
			}				
		}
		//toggle led
		BRD_LEDToggle();

		vTaskDelay(200);
	}
}

/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
static void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED

	s4402815_sysmon_init();		//Initialise systme monitor

	portENABLE_INTERRUPTS();	//Enable interrupts

	/*Initialise pushbutton interrupt for semaphore*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED

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
  * @brief  Pushbutton Interrupt handler. Gives PB Semaphore
  * @param  None.
  * @retval None
  */
void exti_pb_irqhandler(void) {

	HAL_Delay(150);				//for deboucing

	s4402815_LA_CHAN2_SET();		//for system monitor detect

	BaseType_t xHigherPriorityTaskWoken;
	 
    /* Is it time for another Task() to run? */
    xHigherPriorityTaskWoken = pdFALSE;

	/* Check if Pushbutton external interrupt has occured */
  	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);				//Clear D0 pin external interrupt flag
    	
	if (PBSemaphore != NULL) {	/* Check if semaphore exists */
		xSemaphoreGiveFromISR( PBSemaphore, &xHigherPriorityTaskWoken );		/* Give PB Semaphore from ISR*/
		debug_printf("Triggered \n\r");    //Print press count value
	}
    
	/* Perform context switching, if required. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	s4402815_LA_CHAN2_CLR();		//for system monitor detect
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
  * @brief  Idle Application Task (Disabled)
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static TickType_t xLastTx = 0;

	BRD_LEDOff();

	for (;;) {
		/* The idle hook simply prints the idle tick count */
		if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {
			xLastTx = xTaskGetTickCount();
			//debug_printf("IDLE Tick %d\n", xLastTx);
			//BRD_LEDToggle();		
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

