/**   
 ******************************************************************************   
 * @file    mylib/s4402815_acc.c    
 * @author  YI LIU – 44028156 
 * @date    15/05/2016   
 * @brief   a freertos driver for mma8452q accelerometer
 *
 ******************************************************************************   
 *     extern void s4402815_TaskAcc(void)
 ******************************************************************************
 * 
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4402815_acc.h"
#include "s4402815_cli.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
  * @brief  FreeRTOS driver task used to control accelerometer
  * @param  void
  * @retval None

  */
extern void s4402815_TaskAcc(void) {

	// initialise the accelerometer
	acc_int();

	struct Accvalue acc_value;
	uint8_t pl_value;
	int flag = 0;

	//create queue to transfer acc x, y, z values and pl status
	s4402815_QueueAccValue = xQueueCreate(5, sizeof(acc_value));

	for (;;) {

		// receive semaphore to enable acc measurement
		if (s4402815_SemaphoreAccOn != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreAccOn, 10 ) == pdTRUE ) {
				flag = 1;
			}
		}

		// receive semaphore to disable acc measurement
		if (s4402815_SemaphoreAccOff != NULL) {

			//If the semaphore is not available, wait 10 ticks to see if it becomes free.
			if( xSemaphoreTake( s4402815_SemaphoreAccOff, 10 ) == pdTRUE ) {
				flag = 0;
			}
		}

		if (flag == 1) {

			//check whether the queue exists
			if (s4402815_QueueAccValue != NULL) {
				//get 12-bit x, y, z values
				acc_value.x_value = value_combine(0x01, 0x02);
				acc_value.y_value = value_combine(0x03, 0x04);
				acc_value.z_value = value_combine(0x05, 0x06);

				//get pl status;
				pl_value = read_reg(0x10);
				int state_pl = (pl_value & 0x06) >> 1;
				int state_bf = pl_value & 0x01;

				acc_value.pl = get_pl_state(state_pl);
				acc_value.bf = get_bf_state(state_bf);

				//Send raw values to the front of the queue - wait atmost 10 ticks
				if( xQueueSendToFront(s4402815_QueueAccValue, ( void * ) &acc_value, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post acc message, after 10 ticks.\n\r");
				}
			}
		}
		// task delay 
		vTaskDelay(80);
	}
}

/**
  * @brief  Initialise accelerometer, and enable detections
  * @param  void
  * @retval None

  */
void acc_int(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable GPIO clocks */
	__BRD_SCL_GPIO_CLK();
	__BRD_SDA_GPIO_CLK();

	/* Enable I2C CLK */
	__BRD_I2C_CLK();

	/******************************************************/
	/* IMPORTANT NOTE: SCL Must be Initialised BEFORE SDA */
	/******************************************************/
	/* enable GPIO pins for I2C */
	GPIO_InitStructure.Pin = BRD_SCL_PIN;			//SCL
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP ;
	GPIO_InitStructure.Alternate = BRD_SCL_AF;
	HAL_GPIO_Init(BRD_SCL_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = BRD_SDA_PIN;			//SDA
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP ;
	GPIO_InitStructure.Alternate = BRD_SDA_AF;
	HAL_GPIO_Init(BRD_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the I2C peripheral */
	I2CHandle.Instance = BRD_I2C;
	I2CHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;                               // 7bit addressing mode
	I2CHandle.Init.ClockSpeed      = 1000000;						// Transmission Frequency
	I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2CHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2CHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2CHandle.Init.OwnAddress1     = 0;
	I2CHandle.Init.OwnAddress2     = 0;

	/* Initialise and Start the I2C peripheral */
	HAL_I2C_Init(&I2CHandle);

	/* -> Wait for the end of the transfer */
	/* Before starting a new communication transfer, you need to check the current
	* state of the peripheral; if it’s busy you need to wait for the end of current
	* transfer before starting a new one.
	* For simplicity reasons, this example is just waiting till the end of the
	* transfer, but application may perform other tasks while transfer operation
	* is ongoing.
	*/
	while (HAL_I2C_GetState(&I2CHandle) != HAL_I2C_STATE_READY);

	write_reg(0x11, 0xc0);
	write_reg(0x2a, 0x01);
	write_reg(0x2b, 0x84);
	write_reg(0x2c, 0x60);
	write_reg(0x2d, 0xb1);
}

/**
  * @brief  read register value of MMA8462Q
  * @param  register address
  * @retval register value

  */
int read_reg(int address) {
	
	uint8_t read_reg_val;

	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag
		
	/* Send Read Register Address - WHO_AM_I Register Address */
	I2CHandle.Instance->DR = address;	

	/* Wait until register Address byte is transmitted */
	while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));

	/* Generate the START condition, again */
	I2CHandle.Instance->CR1 |= I2C_CR1_START;

	/* Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);
      
	/* Send Read Address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_READ(MMA8452Q_ADDRESS);

	/* Wait address is acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag
		
	/* Wait to read */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_RXNE) == RESET);
		
	/* Read received value */
	read_reg_val = I2CHandle.Instance->DR;

	/* Generate NACK */
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;

	/* Generate the STOP condition */
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;

	return read_reg_val;
}

/**
  * @brief  write a register of MMA8462Q
  * @param  register address, and the value to be written
  * @retval None

  */
void write_reg(int address, int data) {
	
	//debug_printf("1\n");
	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/* Send Read Register Address - WHO_AM_I Register Address */
	I2CHandle.Instance->DR = address;	

	/* Wait until register Address byte is transmitted */
	while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));
      
	/* Send Read Address */
	I2CHandle.Instance->DR = data;

	/* Wait address is acknowledged */
	while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));

	/* Generate the STOP condition */
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;
}

/**
  * @brief  generate a 12-bit raw value
  * @param  address of first register, address of second register
  * @retval 12-bit combined value

  */
int value_combine(int address_1, int address_2) {

	int value;
	int signal = 1;
	int value1 = read_reg(address_1);
	int value2 = read_reg(address_2);

	//combine the value of first register and the first 4 bits of second register
	if ((value1 >> 7) == 1) {
		value1 = ~((value1 & 0x7f) -1);
		signal = -1;
	}
	value = ((value1 & 0x7f) << 4) | (value2 >>4);

	//judge whether the value is positive or negative
	return value * signal;
}

/**
  * @brief  judge pl state, including up, down, right and left
  * @param  bit 2 and bit 1 value of 0x10 register
  * @retval a character representing the pl state

  */
int get_pl_state(int value) {

	// the hex value 00: up, 01:down, 10: right, 11: left
	switch(value) {
		case 0: return '|';
		case 1: return '_';
		case 2: return '>';
		case 3: return '<';
		default: return '|';
	}
}

/**
  * @brief  judge bf state, including back and front
  * @param  bit 0 value of 0x10 register
  * @retval a character representing the bf state

  */
int get_bf_state(int value) {

	// front: value = 0, back: value = 1
	if (value == 0)
		return 'f';
	else
		return 'b';
}

