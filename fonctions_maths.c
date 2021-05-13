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


float StepsToCm(int32_t nbSteps){
	return (float)((float)(nbSteps*13)/(float)(1000));
}

float MmToCm(uint16_t ValeurMm){
	return (float)((float)ValeurMm)/((float)10);
}
