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
#include <deplcament.h>
#include <mesure.h>


float StepsToCm(int32_t nbSteps)
{
	return (float)((float)(nbSteps*PERIM_ROUE_CM)/(float)(TICS_1_TOUR));
}

float MmToCm(uint16_t ValeurMm)
{
	return (float)((float)ValeurMm)/((float)DIX);
}

int16_t CmToSteps(float ValeurCm)
{
	return (int16_t)((int16_t)ValeurCm*TICS_1_TOUR)/((int16_t)PERIM_ROUE_CM);
}

