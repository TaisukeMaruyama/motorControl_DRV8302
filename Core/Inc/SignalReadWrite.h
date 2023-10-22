/*
 * signalReadWrite.h
 *
 *  Created on: May 7, 2023
 *      Author: r720r
 */

#ifndef MCLIB_SIGNALREADWRITE_H_
#define MCLIB_SIGNALREADWRITE_H_

#include <stdint.h>
#include "main.h"


// Global Functions
uint8_t readButton1(void);
uint32_t readInputCaptureCnt(void);
float readTimeInterval(uint32_t inputCaptureCnt, uint32_t inputCaptureCnt_pre);
float readVolume(void);
float readVdc(void);
void readCurrent(uint16_t* Iuvw_AD, float* Iuvw);
void readHallSignal(uint8_t* Hall);
void writeOutputMode(int8_t* outputMode);
void writeDuty(float* Duty);

#endif /* MCLIB_SIGNALREADWRITE_H_ */
