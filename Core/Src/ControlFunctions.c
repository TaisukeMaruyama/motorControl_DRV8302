/*
 * ControlFunctions.c
 *
 *  Created on: Jun 4, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include "main.h"
#include "ControlFunctions.h"

float cfPhaseLockedLoop(float ElectAngleErr, float Kp_PLL, float Ki_PLL, float *Integral_ElectAngleErr_Ki){
	float ElectAngVeloEstimate;


	*Integral_ElectAngleErr_Ki += ElectAngleErr * Ki_PLL;
	ElectAngVeloEstimate = Kp_PLL * ElectAngleErr + *Integral_ElectAngleErr_Ki;

	return ElectAngVeloEstimate;
}

