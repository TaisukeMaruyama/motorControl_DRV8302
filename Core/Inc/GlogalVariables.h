/*
 * common.h
 *
 *  Created on: May 13, 2023
 *      Author: r720r
 */

#ifndef MCLIB_GLOGALVARIABLES_H_
#define MCLIB_GLOGALVARIABLES_H_

#include <stdint.h>
#include "main.h"

#define OUTPUTMODE_OPEN 		0
#define OUTPUTMODE_POSITIVE 	1
#define OUTPUTMODE_NEGATIVE 	-1
//#define SYSTEMCLOCKFREQ 		72000000.0f //for F302
#define SYSTEMCLOCKFREQ 		170000000.0f
#define SYSTEMCLOCKCYCLE		1 / SYSTEMCLOCKFREQ
#define CARRIERFREQ				17000.0f
#define CARRIERCYCLE 			1 / CARRIERFREQ //0.00005555555f
#define DUTYMAXCOUNT 			SYSTEMCLOCKFREQ / (CARRIERFREQ * 2) - 1 //Center Aligned
#define BITMAX32	 			4294967296
#define BITMAX32HALF 			2147483648
#define PI 3.141592654f
#define TWOPI 6.283185307f
#define ONEDIVTWOPI 0.159154943f
#define PIDIV3 1.047197551f
#define PIDIV6 0.523598776f
#define SQRT3DIV2_DIV2		0.612372436f
#define SQRT_2DIV3			0.816496581f
#define SQRT3_DIV3			0.86602540378f
#define DUTYUPPER			1.0f
#define DUTYLOWER			-1.0f
#define IU_ADOffSET			1993
#define IV_ADOffSET			2002
#define IW_ADOffSET			2024
#define AD2CURRENT			-0.00193586253f // for IHM07M1
// #define AD2CURRENT			-0.014767822f // for DRV8302 tekito
//#define AD2CURRENT			-0.358844273 // for CQKIT
#define AD2VOLTAGE			0.0154305f; // for IHM07M1 1/(9.31/(9.31+169)*4096/3.3V)
//#define AD2VOLTAGE			0.025210084f; // for CQKIT


#define POSMODE_HALL				0
#define POSMODE_HALL_PLL			1
#define POSMODE_SENSORLESS			2
#define DRVMODE_OFF					0
#define DRVMODE_SIXSTEP				1
#define DRVMODE_OPENLOOP			2
#define DRVMODE_VECTORCONTROL		3


// for NIDEC Motor
#define Ra					1.680596498f
#define La					0.00048f
#define Ke					1.03E-03f

// for CQKit Motor
//#define Ra					0.027f
//#define La					0.000035f
//#define Ke					9.36E-03f

// Global Variables
extern uint16_t gAdcValue[2];
extern uint8_t gHall[3];
extern uint8_t gButton1;
extern uint32_t gTIMCounter;
extern uint32_t gTIMCounter_pre;
extern float gTIMCounterDiff;
extern uint32_t gInputCaptureCnt;
extern uint32_t gInputCaptureCnt_pre;

extern float gElectFreq;
extern float gTheta;
extern float gElectAngVelo;
extern uint32_t gTheta_DAC;
extern float gVdc;
extern float gTwoDivVdc;
extern float gVolume;
extern float gPropoDuty;
extern float gIuvw[3];
extern uint16_t gIuvw_AD[3];

extern int8_t gOutputMode[3];
extern float gDutyRef;
extern float gDuty[3];

extern uint8_t gPosMode;
extern uint8_t gDrvMode;
extern uint16_t gInitCnt;


#endif /* MCLIB_GLOGALVARIABLES_H_ */
