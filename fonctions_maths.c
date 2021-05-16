#include <fonctions_maths.h>
#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chprintf.h>
#include <stdbool.h>
#include <stdint.h>


float StepsToCm(int32_t nbSteps)
{
	return (float)((float)(nbSteps*13)/(float)(1000));
}

float MmToCm(uint16_t ValeurMm)
{
	return (float)((float)ValeurMm)/((float)10);
}

int16_t CmToSteps(float ValeurCm)
{
	return (int16_t)((int16_t)ValeurCm*1000)/((int16_t)13);
}

float Multi(float val1, uint16_t val2)
{
	return (float)(val1*(float)val2);
}

