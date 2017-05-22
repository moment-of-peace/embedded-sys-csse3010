/**
  ******************************************************************************
  * @file    s4402815_hamming.c 
  * @author  YI LIU
  * @date    042016
  * @brief   hamming decode and encode
  ******************************************************************************
  *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * extern void s4402815_laser_encode(char packet, int* code);
 * extern char s4402815_laser_decode(int* raw, float unit, int* lasertest);
 * extern void s4402815_error_insertion(char packet, int erro, int* code);
 * extern void s4402815_hamenc(int value);
 * extern int s4402815_hamdec(uint16_t codedword);
 ******************************************************************************   
  */ 

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "nrf24l01plus.h"
#include "math.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
uint16_t hamming_byte_encoder(uint8_t input);

/**
  *transfer a character to a string of manchester code, and write binary infomation to
  *an array
  */
extern void s4402815_laser_encode(char packet, int* code) {

	int code_temp[22] = {0};
	int i;
	int j;
	uint16_t codedword;

	//if (packet != '\0')
	codedword = hamming_byte_encoder(packet);
	code_temp[0] = 1;
	code_temp[1] = 1;
	code_temp[10] = 0;
	code_temp[11] = 1;
	code_temp[12] = 1;
	code_temp[21] = 0;
	for (i = 0; i < 8; i++) {
		code_temp[i+2] = !!(codedword & (0x1 << i));
	}

	for (i = 8; i < 16; i++) {
		code_temp[i+5] = !!(codedword & (0x1 << i));
	}

	for (i = 0; i < 22; i++ ) { //test code temp
					debug_printf("%d ", code_temp[i]);		
			}
			debug_printf("\n\r");

	for (i = 0; i < 22; i++) {
		if (code_temp[i] == 0) {
			code[i * 2] = 1;
			code[i * 2 + 1] = 0;
		}
		else {
			code[i * 2] = 0;
			code[i * 2 + 1] = 1;
		}
	}
}

/**
  *transfer a string of manchester code to a character, and return the char
  *
  */
extern char s4402815_laser_decode(int* raw, float unit, int* lasertest) {

	int tim3_period = 20000;
	int j = 1;
	int i = 0;
	char chara;
	double chara_value = 0;
	float diff = 0;
	int signal[22] = {0};
	int raw_temp[21] = {0}; //signal length-1 
	debug_printf("raw test\n");
	for (i = 0; i < 22; i++) {
		*(lasertest+i) = *(raw+i);
	}
	for (int i = 0; i < 22; i++ ) { //test raw[]
		debug_printf("%d ", raw[i]);		
	}
	debug_printf("\n");
	debug_printf("unit in haming %f\n", unit);
	raw_temp[0] = 2;
	for (i = 1; i < 21 && raw[i+1] != 0; i++) { //
		diff = (raw[i+1] - raw[i]);
		if (diff < 0) diff = diff + tim3_period;
		diff = diff / unit;
		if (diff > 1.6 && diff < 2.4) raw_temp[i] = 2;
		if (diff > 2.6 && diff < 3.4) raw_temp[i] = 3;
		if (diff > 3.6 && diff < 4.4) raw_temp[i] = 4;
	}

	for (i = 0; i < 21; i++) {
		lasertest[i+22] = raw_temp[i];
	}

	signal[0] = 1;
	for (i = 0; i < 21 && j < 22; i++) {
		switch (raw_temp[i]) {
			case 2: signal[j] = signal[j-1]; j++; break;
			case 3: signal[j] = 0; signal[j+1] = 1 - signal[j-1]; j += 2; break;
			case 4: signal[j] = 0; signal[j+1] = 1; j += 2; break;
		}
	}
	for (i = 0; i < 22; i++) {
		lasertest[i+43] = signal[i];
	}
	
	for (i = 0; i < 4; i++) {
		chara_value = signal[i+6] * pow(2, i) + chara_value;
	}
	for (i = 0; i < 4; i++) {
		chara_value = signal[i+17] * pow(2, i) + chara_value;
	}
	chara = (int)chara_value;
	lasertest[66] = chara;
	lasertest[68] = unit;
	debug_printf("2\n");
	lasertest[69] = 6;
	return chara;
}

/**
  *This function is used for hamming encode 
  */
extern uint16_t s4402815_hamenc(int value) {

	uint16_t codedword;
	uint8_t in = value;

	codedword = hamming_byte_encoder(in);
	return codedword;
}

/**
 *This function is used for hamming decode
 */
extern int s4402815_hamdec(uint16_t codedword) {

	uint8_t temp1;
	uint8_t temp2;
	int value;

	//extract bytes
	temp1 = (codedword & 0x00ff);
	temp2 = (codedword & 0xff00) >> 8;

	//decode each bytes and combine
	temp1 = ham_byte_dec(temp1);
	temp2 = ham_byte_dec(temp2);
	if (temp1 == -1 || temp2 == -1)
		//in this case, there are 2 or more errors
		return -1;
	else {
		value = (temp1 << 4) | temp2;
		return (value);
	}
}
/**
  *this function is to insert one or two error into a correct a string of
  *manchester code
  */
extern void s4402815_error_insertion(char packet, int erro, int* code) {
	uint16_t codedword;
	int i;
	int code_temp[22] = {0};

	codedword = hamming_byte_encoder(packet);
	codedword = codedword ^ erro;

	code_temp[0] = 1;
	code_temp[1] = 1;
	code_temp[10] = 0;
	code_temp[11] = 1;
	code_temp[12] = 1;
	code_temp[21] = 0;
	for (i = 0; i < 8; i++) {
		code_temp[i+2] = !!(codedword & (0x1 << i));
	}

	for (i = 8; i < 16; i++) {
		code_temp[i+5] = !!(codedword & (0x1 << i));
	}

	for (i = 0; i < 22; i++ ) { //test code temp
					debug_printf("%d ", code_temp[i]);		
			}
			debug_printf("\n\r");

	for (i = 0; i < 22; i++) {
		if (code_temp[i] == 0) {
			code[i * 2] = 1;
			code[i * 2 + 1] = 0;
		}
		else {
			code[i * 2] = 0;
			code[i * 2 + 1] = 1;
		}
	}

}
/**
  * Implement Hamming Code + parity checking
  * Hamming code is based on the following generator and parity check matrices
  */
uint8_t hamming_hbyte_encoder(uint8_t in) {

	uint8_t d0, d1, d2, d3;
	uint8_t p0 = 0, h0, h1, h2;
	uint8_t z;
	uint8_t out;
	
	/* extract bits */
	d0 = !!(in & 0x1);
	d1 = !!(in & 0x2);
	d2 = !!(in & 0x4);
	d3 = !!(in & 0x8);
	
	/* calculate hamming parity bits */
	h0 = d0 ^ d1 ^ d2;
	h1 = d0 ^ d1 ^ d3;
	h2 = d0 ^ d2 ^ d3;
	
	out = (h0) | (h1 << 1) | (h2 << 2) |
		(d0 << 3) | (d1 << 4) | (d2 << 5) | (d3 << 6);

	/* calculate even parity bit */
	p0 = h0 ^ h1 ^ h2 ^ d0 ^ d1 ^ d2 ^ d3;
	
	out = (p0) | (out << 1);

	return(out);

}

/**
  * Implement Hamming Code on a full byte of input
  * This means that 16-bits out output is needed
  */
uint16_t hamming_byte_encoder(uint8_t input) {

	uint16_t out;
	
	/* first encode D0..D3 (first 4 bits), 
	 * then D4..D7 (second 4 bits).
	 */
	out = (hamming_hbyte_encoder(input & 0xF) << 8)| 
		hamming_hbyte_encoder(input >> 4);
	
	return(out);

}

int ham_byte_dec(int in) {

	int value;
	int error_state = 0;
	uint8_t d3, d2, d1, d0, h2, h1, h0, p0;

	//extract bits
	d3 = (in & 0x80) >> 7;
	d2 = (in & 0x40) >> 6;
	d1 = (in & 0x20) >> 5;
	d0 = (in & 0x10) >> 4;
	h2 = (in & 0x08) >> 3;
	h1 = (in & 0x04) >> 2;
	h0 = (in & 0x02) >> 1;
	p0 = (in & 0x01);

	//calculate even parity
	uint8_t parity = h0 ^ h1 ^ h2 ^ d0 ^ d1 ^ d2 ^ d3;

	//check whether there are errors
	uint8_t s0 = d0 ^ d1 ^ d2 ^ h0;
	uint8_t s1 = d0 ^ d1 ^ d3 ^ h1;
	uint8_t s2 = d0 ^ d2 ^ d3 ^ h2;

	//correct single error
	switch(s0 | (s1 << 1) | (s2 << 2)) {
		case 0: error_state = 0; break;
		case 1: h0 = !h0; error_state = 1; break;
		case 2: h1 = !h1; error_state = 1; break;
		case 3: d1 = !d1; error_state = 1; break;
		case 4: h2 = !h2; error_state = 1; break;
		case 5: d2 = !d2; error_state = 1; break;
		case 6: d3 = !d3; error_state = 1; break;
		case 7: d0 = !d0; error_state = 1; break;
	}

	//if there two or more errors
	if (parity == p0 && error_state == 1)
		return -1;
	else {
		value = (d3 << 3) | (d2 << 2) | (d1 << 1) | d0;
		return (value);
	}
}

