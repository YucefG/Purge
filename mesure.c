#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>


//defines du hardware
#define PI						3.14
#define ANGLE360				360
#define D_ENTRE_ROUES			53			//en mm
#define PERIM_CERC_PARC			D_ENTRE_ROUES*PI		//perimetre du cercle parcouru par les roues en 360
#define TICS_1_TOUR				1000
#define DISTANCE_1_TOUR			130			//en mm

//defines qu'on peut modifier
#define NB_MESURES				30
#define TICS_360				(PERIM_CERC_PARC*TICS_1_TOUR)/DISTANCE_1_TOUR   //nb de tics pour faire un 360
#define TICS_1_MESURE			TICS_360/NB_MESURES

//variable globale: tableau de mesures
uint8_t tab_mesures[NB_MESURES];

//1ere etape: remplir le tableau avec le TOF
void tour_mesures(void){
	//initialiser les compteurs des moteurs
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	palSetPad(GPIOB, GPIOB_LED_BODY);

	//bouger aux NB_MESURES positions et faire la mesure (remplir le tab)
	for(uint8_t i=0; i<NB_MESURES; i++){
		//faire la mesure
		tab_mesures[i]=(uint8_t)VL53L0X_get_dist_mm() - (uint8_t)30;   //est ce que les valeurs vont etre restreintes par le uint8? (max:255)
		chprintf((BaseSequentialStream *)&SD3, "  %u-eme mesure à une distance de : %u mm   ||",i,tab_mesures[i]);


		//bouger les moteurs
		while((left_motor_get_pos()<TICS_1_MESURE)&&(right_motor_get_pos()<TICS_1_MESURE)){
			left_motor_set_speed(100);
			right_motor_set_speed(-100);
		}
		//on arrete et on initialise pour la prochaine mesure
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		left_motor_set_pos(0);
		right_motor_set_pos(0);
	}
	//message pour dire qu'il a fini
	palClearPad(GPIOB, GPIOB_LED_BODY);
	chThdSleepMilliseconds(300);
	palSetPad(GPIOB, GPIOB_LED_BODY);
	chThdSleepMilliseconds(300);
	palClearPad(GPIOB, GPIOB_LED_BODY);
	chThdSleepMilliseconds(300);
	palSetPad(GPIOB, GPIOB_LED_BODY);
	chThdSleepMilliseconds(300);
	palClearPad(GPIOB, GPIOB_LED_BODY);
}

uint8_t get_mesure_i(uint8_t i){
	return tab_mesures[i];			//return la i-ème mesure
}

void set_mesure_i(uint8_t distance_i, uint8_t i){
	tab_mesures[i]=distance_i;		//set la i-eme mesure
}
