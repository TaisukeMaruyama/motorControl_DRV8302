/*
 * signalReadWrite.c
 *
 *  Created on: May 7, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "GlogalVariables.h"
#include "SignalReadWrite.h"


uint16_t Bemf_AD[3];

uint8_t readButton1(void){
	volatile uint8_t B1;

	B1 = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	return B1;
}

uint32_t readInputCaptureCnt(void){
	// Read Input Capture Count of GPIO
	// CCR1:TIM2 Channel1 = H1, CCR2:Channel2 = H2, CCR3:Channel3 = H3
	volatile uint32_t inputCaptureCnt;

	inputCaptureCnt = TIM2 -> CCR1;

	return inputCaptureCnt;
}

float readTimeInterval(uint32_t inputCaptureCnt, uint32_t inputCaptureCnt_pre){

	float inputCaptureCntDiff;
	float timeInterval;
	uint32_t inputCaptureCntMax;
	uint32_t inputCaptureCntHalf;

	// TIM2 -> ARR Means Counter Period of TIM2
	inputCaptureCntMax = TIM2 -> ARR;
	inputCaptureCntHalf = (inputCaptureCntMax + 1) >> 1;


	inputCaptureCntDiff = (float)inputCaptureCnt - (float)inputCaptureCnt_pre;

	if( inputCaptureCntDiff < - (float)inputCaptureCntHalf)
	  inputCaptureCntDiff += (float)inputCaptureCntMax;

	timeInterval = inputCaptureCntDiff * SYSTEMCLOCKCYCLE;

	return timeInterval;
}

float readVolume(void){
	// P-NUCLEO-IHM001(or 002), Volume is connected to PB1(ADC12)
	// BLM_KIT_Ver1_5, Accel is connected  is connected to PC2(ADC8)
	float Volume;
	uint16_t Volume_ad = gAdcValue[1];

	Volume = ((int16_t)Volume_ad - 99)* 0.0002442f;
	//Volume = ((int16_t)Volume_ad - 950) * 0.000573394f;
	if( Volume < 0) Volume = 0;
	return Volume;
}

float readVdc(void){
	// P-NUCLEO-IHM001(or 002), Vdc is connected to PA1(ADC2)
	float Vdc;
	uint16_t Vdc_ad = gAdcValue[0];
	Vdc = Vdc_ad * AD2VOLTAGE;
	return Vdc;
}

void readCurrent(uint16_t* Iuvw_AD, float* Iuvw){
	Iuvw_AD[0] = ADC1 -> JDR1; // Iu
	Iuvw_AD[1] = ADC1 -> JDR2; // Iv
	Iuvw_AD[2] = ADC1 -> JDR3; // Iw

	Iuvw[0] = ((float)Iuvw_AD[0] - IU_ADOffSET) * AD2CURRENT;
	Iuvw[1] = ((float)Iuvw_AD[1] - IV_ADOffSET) * AD2CURRENT;
	Iuvw[2] = ((float)Iuvw_AD[2] - IW_ADOffSET) * AD2CURRENT;
}

void readHallSignal(uint8_t* Hall){
	//Hall[0] = u, Hall[1] = v, Hall[2] = w
	Hall[0] = 0b00000001 & ~HAL_GPIO_ReadPin(H1_GPIO_Port, H1_Pin);
	Hall[1] = 0b00000001 & ~HAL_GPIO_ReadPin(GPIOB, H2_Pin);
	Hall[2] = 0b00000001 & ~HAL_GPIO_ReadPin(GPIOB, H3_Pin);
}

void writeOutputMode(int8_t* outputMode){

	// if the outputMode is OPEN, set Enable Pin to RESET.
	if(outputMode[0] == OUTPUTMODE_OPEN )
		HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);

	if(outputMode[1] == OUTPUTMODE_OPEN )
		HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);

	if(outputMode[2] == OUTPUTMODE_OPEN )
		HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
}

void writeDuty(float* Duty){
	// TIM1 -> ARR Means Counter Period of TIM1
	TIM1 -> CCR1 = Duty[0] * (TIM1 -> ARR);
	TIM1 -> CCR2 = Duty[1] * (TIM1 -> ARR);
	TIM1 -> CCR3 = Duty[2] * (TIM1 -> ARR);
}

