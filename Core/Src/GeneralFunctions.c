/*
 * GeneralFunction.c
 *
 *  Created on: Jun 4, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "GeneralFunctions.h"
#include "GlogalVariables.h"

float gfDivideAvoidZero(float num, float den, float  threshold){
	float result;
	if ( den >= 0 && den < threshold )
		den = threshold;
	else if( den < 0 && den > -threshold)
		den = -threshold;

	result = num / den;
	return result;
}

float gfWrapTheta(float theta){
	theta = fmodf(theta, TWOPI);
	if( theta > PI)
		theta -= TWOPI;
	else if( theta < -PI)
		theta += TWOPI;

	return theta;
}

void gfOmega2Theta(float omega, float Ts, float *theta){
	float wrapTheta;

	*theta += omega * Ts;
	wrapTheta = gfWrapTheta(*theta);
	*theta = wrapTheta;
}

float gUpperLowerLimit(float input, float Upper, float Lower){
	if(input > Upper) input = Upper;
	if(input < Lower) input = Lower;
	return input;
}


