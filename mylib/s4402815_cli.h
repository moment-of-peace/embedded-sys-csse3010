/**   
 ******************************************************************************   
 * @file    mylib/s4402815_cli.h 
 * @author  YI LIU – 44028156 
 * @date    01/05/2016   
 * @brief   cli callback functions 
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
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

#ifndef s4402815_CLI_H
#define s4402815_CLI_H

/* Includes ------------------------------------------------------------------*/
/* Scheduler includes. */
#include "s4402815_pantilt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"
#include "event_groups.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define EVT_POS_ON			1 << 0		//LED Event Flag
#define EVT_POS_OFF			1 << 1		//Pushbutton Event Flag
#define POSITIONCTRL_EVENT		EVT_POS_ON | EVT_POS_OFF	//Control Event Group Mask

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External function prototypes -----------------------------------------------*/
struct Task {
	int handle;
	char type;
};

int laser_status;			//used in "laser" command to store laser status
uint16_t sequence;
uint8_t key;
uint8_t speed1;
uint8_t speed2;
uint8_t speed3;
uint8_t speed1r;
uint8_t speed2r;
uint8_t speed3r;
uint8_t speedx;
int rover_id;		//the id number of a rover

SemaphoreHandle_t s4402815_SemaphoreTop;
SemaphoreHandle_t s4402815_SemaphoreAccReadRaw;
SemaphoreHandle_t s4402815_SemaphoreAccReadPL;
SemaphoreHandle_t s4402815_SemaphoreTrackOn;
SemaphoreHandle_t s4402815_SemaphoreTrackOff;
SemaphoreHandle_t s4402815_SemaphorePositionOn;
SemaphoreHandle_t s4402815_SemaphorePositionOff;
SemaphoreHandle_t s4402815_SemaphoreDistance;
SemaphoreHandle_t s4402815_SemaphoreFollow;
SemaphoreHandle_t s4402815_SemaphoreAccCtrlOn;
SemaphoreHandle_t s4402815_SemaphoreAccCtrlOff;

QueueHandle_t s4402815_QueueTaskControl;
QueueHandle_t s4402815_QueueWayPointID;

EventGroupHandle_t positionctrl_EventGroup;		//Control Event Group

extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvTopCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvSuspendCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvResumeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvHamencCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvHamdecCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvAccCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvTrackingCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvCRCCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvGetpasskeyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvGetsensorCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvGettimeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern double s4402815_current_time(void);
extern BaseType_t prvRfchansetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvTxaddsetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvRoverchanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvRoveraddCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvForwardCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvReverseCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvCalibrationCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvDistanceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvPositionCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvRoveridCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvWaypointCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern BaseType_t prvFollowerCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern s4402815_rover_control(uint8_t type, uint8_t speed_l, uint8_t speed_r, int dura, uint8_t dir);
extern BaseType_t prvAcccontrolCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

#endif
