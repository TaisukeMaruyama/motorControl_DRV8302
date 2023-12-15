/*
 * VectorControl.c
 *
 *  Created on: Jan 7, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "math.h"
#include "GlogalVariables.h"
#include "GeneralFunctions.h"
#include "VectorControl.h"
#include "ControlFunctions.h"

static float sIab[2];
static float sIdq[2];
static float sIdq_ref_1000[2];
static float sIdq_1000[2];
static float sVdq[2];
static float sVdq_i[2];
static float sVab[2];
static float sVuvw[3];
static float sVamp;
static float sMod;
static float sEdq[2];
static float sAngleErr;

static inline void uvw2ab(float *uvw, float *ab);
static inline void ab2uvw(float *ab, float *uvw);
static inline void ab2dq(float theta, float *ab, float *dq);
static inline void dq2ab(float theta, float *dq, float *ab);
static inline float calcAmpFromVect(float* Vect);
static inline float calcModFromVamp(float Vamp, float twoDivVdc);
static inline void Vuvw2Duty(float twoDivVdc, float *Vuvw, float *Duty);
static inline void CurrentFbControl(float *Igd_ref, float *Igd, float electAngVelo, float Vdc, float *Vgd, float* Vamp);

void VectorControlTasks(float *Idq_ref, float theta, float electAngVelo, float *Iuvw, float Vdc, float twoDivVdc, uint8_t flgFB, float* Duty, int8_t* outputMode){
	float Vq_ref_open;
	if ( flgFB == 0 ){
		Vq_ref_open = Vdc * SQRT3DIV2_DIV2 * gPropoDuty;
			OpenLoopTasks(Vq_ref_open, theta, Iuvw, twoDivVdc, Duty, outputMode);
sVdq[0] = 0.0f;
			sVdq[1] = Vq_ref_open;
						sVdq_i[0] = 0.0f;
			sVdq_i[1] = 0.0f;
		}
	else{

		outputMode[0] = OUTPUTMODE_POSITIVE;
		outputMode[1] = OUTPUTMODE_POSITIVE;
		outputMode[2] = OUTPUTMODE_POSITIVE;


		uvw2ab(gIuvw, sIab);
		ab2dq(theta, sIab, sIdq);

		CurrentFbControl(Idq_ref, sIdq, electAngVelo, Vdc, sVdq, &sVamp);
		sMod = calcModFromVamp(sVamp, gTwoDivVdc);

		sEdq[0] = sVdq[0] - Ra * sIdq[0];// + La * electAngVelo * sIdq[1];
		sEdq[1] = sVdq[1] - Ra * sIdq[1];// - La * electAngVelo * sIdq[0];
		sAngleErr = atan2f(-1.0f * sEdq[0], sEdq[1]);

		dq2ab(theta, sVdq, sVab);
		ab2uvw(sVab, sVuvw);
		Vuvw2Duty(twoDivVdc, sVuvw, Duty);

		sIdq_ref_1000[0] = Idq_ref[0] * 1000.0f;
		sIdq_ref_1000[1] = Idq_ref[1] * 1000.0f;
		sIdq_1000[0] = sIdq[0] * 1000.0f;
		sIdq_1000[1] = sIdq[1] * 1000.0f;
	}

}

void OpenLoopTasks(float VamRef, float theta, float *Iuvw, float twoDivVdc, float* Duty, int8_t* outputMode){
	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;

	uvw2ab(gIuvw, sIab);
	ab2dq(theta, sIab, sIdq);
	sVdq[0] = 0.0f;
	sVdq[1] = VamRef;
	dq2ab(theta, sVdq, sVab);
	ab2uvw(sVab, sVuvw);
	Vuvw2Duty(twoDivVdc, sVuvw, Duty);

	sIdq_1000[0] = sIdq[0] * 1000.0f;
	sIdq_1000[1] = sIdq[1] * 1000.0f;
}

static void uvw2ab(float* uvw, float* ab){
	ab[0] = SQRT_2DIV3 * ( uvw[0] - 0.5f * uvw[1] - 0.5f * uvw[2] );
	ab[1] = SQRT_2DIV3 * ( SQRT3_DIV3 * uvw[1] - SQRT3_DIV3 * uvw[2] );
}

static void ab2uvw(float* ab, float* uvw){
	uvw[0] = SQRT_2DIV3 * ab[0];
	uvw[1] = SQRT_2DIV3 * ( -0.5f * ab[0] + SQRT3_DIV3 * ab[1] );
	uvw[2] = - uvw[0] - uvw[1];
}

static void ab2dq(float theta, float* ab, float* dq){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	dq[0] = ab[0] * cosTheta + ab[1] * sinTheta;
	dq[1] = - ab[0] * sinTheta + ab[1] * cosTheta;
}

static float calcAmpFromVect(float* Vect){
	float amp;
	float amp2;

	amp2 = Vect[0] * Vect[0] + Vect[1] * Vect[1];
	amp = sqrtf(amp2);
	return amp;
}

static float calcModFromVamp(float Vamp, float twoDivVdc){
	float mod;

	mod = Vamp * twoDivVdc * SQRT_2DIV3;
	return mod;
}

static void dq2ab(float theta, float* dq, float* ab){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	ab[0] = dq[0] * cosTheta - dq[1] * sinTheta;
	ab[1] = dq[0] * sinTheta + dq[1] * cosTheta;
}

static void Vuvw2Duty(float twoDivVdc, float* Vuvw, float* Duty){

	float max;
	float min;
	float vo;

	// third-harmonic injection
	max = Vuvw[0];
	if(Vuvw[1] > max)
		max = Vuvw[1];
	if(Vuvw[2] > max)
		max = Vuvw[2];

	min = Vuvw[0];
	if(Vuvw[1] < min)
		min = Vuvw[1];
	if(Vuvw[2] < min)
		min = Vuvw[2];

	vo = (max + min) * 0.5f;

	Vuvw[0] = Vuvw[0] - vo;
	Vuvw[1] = Vuvw[1] - vo;
	Vuvw[2] = Vuvw[2] - vo;


	Duty[0] = (Vuvw[0] * twoDivVdc);
	Duty[1] = (Vuvw[1] * twoDivVdc);
	Duty[2] = -Duty[0] - Duty[1];

	Duty[0] = gUpperLowerLimit(Duty[0], DUTYUPPER, DUTYLOWER);
	Duty[1] = gUpperLowerLimit(Duty[1], DUTYUPPER, DUTYLOWER);
	Duty[2] = gUpperLowerLimit(Duty[2], DUTYUPPER, DUTYLOWER);

	//50% CENTER
	Duty[0] = Duty[0] * 0.5f + 0.5f;
	Duty[1] = Duty[1] * 0.5f + 0.5f;
	Duty[2] = Duty[2] * 0.5f + 0.5f;

}



static void CurrentFbControl(float* Igd_ref, float* Igd, float electAngVelo, float Vdc, float* Vgd, float* Vamp){
	float Ierr[2];
	float Kp;
	float Kig;
	float Kid;
	float VampLimit;
	float Vphase;
	float wc;

	wc = 10.0f * TWOPI;

	Kp = La * wc;
	Kig = Ra * wc * CARRIERCYCLE;
	Kid = Kig;

	Ierr[0] = Igd_ref[0] - Igd[0];
	Ierr[1] = Igd_ref[1] - Igd[1];

	sVdq_i[0] += Kig * Ierr[0];
	sVdq_i[1] += Kid * Ierr[1];

	Vgd[0] = Kp * Ierr[0] + sVdq_i[0];
	Vgd[1] = Ke * electAngVelo + Kp * Ierr[1] + sVdq_i[1];// + Vgd[1] + Kid * Ierr[1] + ;

	Vphase = atan2f(Vgd[1], Vgd[0]);

	*Vamp = calcAmpFromVect(Vgd);

	VampLimit = Vdc * SQRT3DIV2_DIV2 * 1.15f;
	if( *Vamp > VampLimit ){
		Vgd[0] = VampLimit * cosf(Vphase);
		sVdq_i[0] = Vgd[0];
		Vgd[1] = VampLimit * sinf(Vphase);
		sVdq_i[1] = Vgd[1] -  Ke * electAngVelo;

	}
}

/*
float FluxObserver(float* Igd, float* Vgd, float* Egd){
	float Theta_est;
	Egd[0] = Vgd[0] - Rmotor * Igd[0];
	Egd[1] = Vgd[1] - Rmotor * Igd[1];
	//arm_atan2_f32(Egd[0], Egd[1], &result);
	Theta_est = atan2f(Egd[1], Egd[0]);
	return Theta_est;
}
*/
