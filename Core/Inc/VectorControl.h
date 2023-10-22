#include <stdint.h>
#include "math.h"
#include "GlogalVariables.h"

void VectorControlTasks(float *Idq_ref, float theta, float electAngVelo, float *Iuvw, float Vdc, float twoDivVdc, uint8_t flgFB, float *Duty, int8_t* outputMode);
void OpenLoopTasks(float VamRef, float omega, float *Iuvw, float twoDivVdc, float *Duty, int8_t* outputMode);
