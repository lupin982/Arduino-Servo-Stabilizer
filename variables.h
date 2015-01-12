/*
 * variables.h
 *
 * Created: 12/01/2015 22:22:59
 *  Author: Beatella
 */ 


#ifndef VARIABLES_H_
#define VARIABLES_H_

#include <stdint.h>


struct config_t
{
	uint8_t versEEPROM;
	uint8_t configSet;
	float PitchKp;
	float PitchKi;
	float PitchKd;
	float YawKp;
	float YawKi;
	float YawKd;
	
	uint8_t crc8;
}config;


void setDefaultParameters()
{
	config.versEEPROM = 0.1;
	config.PitchKp = 2.0;
	config.PitchKi = 5.0;
	config.PitchKd = 1.0;
	config.YawKp = 2.0;
	config.YawKi = 5.0;
	config.YawKd = 1.0;
	
	config.crc8 = 0;
}

// CRC definitions
#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
typedef uint8_t crc;


#endif /* VARIABLES_H_ */