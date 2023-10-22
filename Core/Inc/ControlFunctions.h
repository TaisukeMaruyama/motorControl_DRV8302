/*
 * ControlFunctions.h
 *
 *  Created on: Jun 4, 2023
 *      Author: r720r
 */

#ifndef MCLIB_CONTROLFUNCTIONS_H_
#define MCLIB_CONTROLFUNCTIONS_H_

#include <stdint.h>
#include "main.h"

// Control Functions
float cfPhaseLockedLoop(float ElectAngleErr, float Kp_PLL, float Ki_PLL, float *Integral_ElectAngleErr_Ki_Ts);


#endif /* MCLIB_CONTROLFUNCTIONS_H_ */
