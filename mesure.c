#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>
#include <lumiere.h>
#include <stdbool.h>
#include <analyse_couleur.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>

//Defin du hardware
#define PI						3.14
#define ANGLE360				360
#define D_ENTRE_ROUES			53			//en mm
#define PERIM_CERC_PARC			D_ENTRE_ROUES*PI		//perimetre du cercle parcouru par les roues en 360
#define TICS_1_TOUR				1000
#define DISTANCE_1_TOUR			130			//en mm
#define OFFSET					30

#define PROX_FRONT_R17			0
#define PROX_FRONT_R49			1
#define PROX_R					2
#define PROX_BACK_R				3
#define PROX_BACK_L				4
#define PROX_L					5
#define PROX_FRONT_L49			6
#define PROX_FRONT_L17			7

// Defin que l'on peut modifier
#define NB_MESURES				30    //taille max limité par uint8
#define TICS_360				(PERIM_CERC_PARC*TICS_1_TOUR)/DISTANCE_1_TOUR   //nb de tics pour faire un 360
#define TICS_1_MESURE			TICS_360/NB_MESURES
#define DIAM_ARENE				200
#define TICS_1_ALLER			(DIAM_ARENE*TICS_1_TOUR)/DISTANCE_1_TOUR
#define LIMITE_COLLISION		1000							//pour pas de chocs via capt de prox

#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

//variable globale: tableau de mesures
uint16_t tab_mesures[NB_MESURES];			// uint16 ou 8 dicte la distance max
uint8_t compteur;
static uint8_t compte_g = 0;
static uint8_t compte_d = 0;

_Bool prox_distance(void){
	//chprintf((BaseSequentialStream *)&SD3, "capt : %u, ",get_calibrated_prox(PROX_FRONT_R17));
	if((get_calibrated_prox(PROX_FRONT_R17)>LIMITE_COLLISION) ||
	  (get_calibrated_prox(PROX_FRONT_L17)>LIMITE_COLLISION))
		return false;
	else
		return true;
}

void ajustement_angle(void)
{
	uint16_t ajustement = 0;
	ajustement = abs(get_calibrated_prox(PROX_FRONT_R17) - get_calibrated_prox(PROX_FRONT_L17));
	chThdSleepMilliseconds(1000);

	while((get_calibrated_prox(PROX_FRONT_R17) > get_calibrated_prox(PROX_FRONT_L17)) &&
			ajustement > 20){ // seuil a modif
			// recentrer
		chThdSleepMilliseconds(100);

			left_motor_set_speed(100);
			right_motor_set_speed(-100);
			compte_d++;
		}

	while((get_calibrated_prox(PROX_FRONT_R17) < get_calibrated_prox(PROX_FRONT_L17)) &&
			ajustement > 20){
					// recentrer
		chThdSleepMilliseconds(100);

		left_motor_set_speed(-100);
		right_motor_set_speed(100);
		compte_g ++;
	}
}

void next_angle(uint16_t speed){
	//aller
			while((left_motor_get_pos()<TICS_1_MESURE)&&(right_motor_get_pos()<TICS_1_MESURE)){
				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			}
			//on arrete et on initialise pour la prochaine mesure
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			left_motor_set_pos(0);
			right_motor_set_pos(0);
}

//1ere etape: remplir le tableau avec le TOF
void tour_mesures(void){
	//initialiser les compteurs des moteurs
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	palSetPad(GPIOB, GPIOB_LED_BODY);
	//joue la melodie de mission imposssible quand il inspecte

//	playMelody(IMPOSSIBLE_MISSION, ML_FORCE_CHANGE, NULL);

	//bouger aux NB_MESURES positions et faire les mesures
	for(uint8_t i=0; i<NB_MESURES; i++){
		tab_mesures[i]=(uint16_t)VL53L0X_get_dist_mm() - (uint16_t)OFFSET;
		next_angle(200);
	}
	signal_fin();
}

//2eme etape: detecter les objets
void object_detec_proche(void){

	if(tab_mesures[0]<DIAM_ARENE && tab_mesures[NB_MESURES-1]<DIAM_ARENE){
		uint16_t shortest_dist_g=tab_mesures[NB_MESURES-1];
		uint8_t pos_shortest_g = NB_MESURES-1;
		uint16_t shortest_dist_d=tab_mesures[0];
		uint8_t pos_shortest_d = 0;
		//compter la taille de l'objet sens normal
		uint8_t j=0;
		while((tab_mesures[j+1]<DIAM_ARENE)&&((j+1)<NB_MESURES))
			j++;
		uint8_t k=0;
		while((tab_mesures[NB_MESURES-1-k-1]<DIAM_ARENE)&& (k<NB_MESURES))
			k++;

		//taille de l'objet k+j+2
		for(uint8_t l=1; l<=j;l++){			//< ou <=
			if(tab_mesures[l]<shortest_dist_d){
				shortest_dist_d = tab_mesures[l];
				tab_mesures[pos_shortest_d]=0;
				pos_shortest_d = l;
				tab_mesures[l]=1;
			}
			else
				tab_mesures[l]=0;
		}

		for(uint8_t t=1; t<=k;t++){
			if(tab_mesures[NB_MESURES-1-t]<shortest_dist_g){

		    	tab_mesures[pos_shortest_g]=0;
				shortest_dist_g = tab_mesures[NB_MESURES-1-t];

				pos_shortest_g = NB_MESURES-1-t;
				tab_mesures[NB_MESURES-1-t]=1;

			}
			else
				tab_mesures[NB_MESURES-1-t]=0;
		}
 	//on compare les deux minimas des deux cotés du monde
		if(shortest_dist_d<shortest_dist_g){
			tab_mesures[pos_shortest_d]=1;
			tab_mesures[pos_shortest_g]=0;

		}
		else
		{
			tab_mesures[pos_shortest_d]=0;
			tab_mesures[pos_shortest_g]=1;
		}
	}

	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]!=0 && tab_mesures[i]!=1){
			if(tab_mesures[i]<DIAM_ARENE){	//rayon ou diam
				uint8_t j = i;
				if(tab_mesures[j]<DIAM_ARENE){				//inutile..?
					while((tab_mesures[j+1]<DIAM_ARENE)&&((j+1)<NB_MESURES))
						j++;
					if(j==i)
						tab_mesures[i]=1;
					else{
						uint16_t shortest_dist=DIAM_ARENE;
						uint8_t pos_shortest = i;
						for(uint8_t k=0; k+i<=j; k++){				//<= ou <?
							if(tab_mesures[k+i]<shortest_dist){
								shortest_dist = tab_mesures[k+i];
								tab_mesures[pos_shortest]=0;
								pos_shortest = k+i;
								tab_mesures[k+i]=1;
							}
							else
							tab_mesures[k+i]=0;
						}
					}
				}
				i=j;
			}
			else
				tab_mesures[i]=0;
		}
	}

	//mesure du nombre d'objets
	compteur = 0;	//init compteur
	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]==1)
			compteur++;
	}
	check_compteur(compteur);

}

//3eme etape: pousser les objets
void object_push(void){
	for(uint8_t i=0;i<NB_MESURES;i++){
		if(tab_mesures[i]==1){
		deplacement();
		compteur--;
		check_compteur(compteur);
		}
		next_angle(200);
	}
	playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
	chThdSleepMilliseconds(3000);
}

void deplacement(void){
	//bouger les moteurs : marche avant jusqu'a la base
	uint16_t last_pos =0;
	while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)&&
		   prox_distance()){
		pi_regulator(distance, (TICS_1_ALLER * DISTANCE_1_TOUR) / TICS_1_TOUR )
		//marche_avant(600);
		//allumer la LED 1
		last_pos = right_motor_get_pos();		//relever le compteur d'un des deux moteurs car vont dans le meme sens meme vitesse
	}
	palSetPad(GPIOD, GPIOD_LED1);//eteindre la LED 1
	//on arrete et on initialise pour le chemin retour
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	chThdSleepMilliseconds(100); //pour marquer un temps d'arret avant analyse

	//ajouter la fonction qui tourne l'epuck face à l'objet (angle env celui du capteur activé)
	ajustement_angle();

	//joue la mort de mario pour indiquer que l'objet est dead
		playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
		while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)){
			palSetPad(GPIOD, GPIOD_LED_FRONT);
			marche_avant(800);
		}
		palClearPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED1);//eteindre la LED 1

		left_motor_set_speed(0);
		right_motor_set_speed(0);
		chThdSleepMilliseconds(100);

		while((left_motor_get_pos()>last_pos)&&(right_motor_get_pos()>last_pos)){
			marche_arriere(800);
		}
		palClearPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED5);//eteindre la LED 5

	left_motor_set_speed(0);
	right_motor_set_speed(0);
	chThdSleepMilliseconds(1000);

	while(compte_d !=0){
		chThdSleepMilliseconds(100);
		left_motor_set_speed(-100);
		right_motor_set_speed(100);
		compte_d --;
	}

	while(compte_g !=0 ){
		chThdSleepMilliseconds(100);
		left_motor_set_speed(100);
		right_motor_set_speed(-100);
		compte_g --;
	}

	//marche arriere jusqu'a la base
	while((left_motor_get_pos()>0)&&(right_motor_get_pos()>0)){
		marche_arriere(600);
	}
	palSetPad(GPIOD, GPIOD_LED5);	//éteindre la LED5

		//allumer la LED5
	//on arrete et on initialise pour la prochaine mesure
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void object_push_couleur(void){
	for(uint8_t i=0;i<NB_MESURES;i++){
		if(tab_mesures[i]==1){
		deplacement_couleur();
		compteur--;
		check_compteur(compteur);
		}
		next_angle(200);
	}
	playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
	chThdSleepMilliseconds(3000);
}

void deplacement_couleur(void){
	//bouger les moteurs : marche avant jusqu'a la base
	uint16_t last_pos =0;
	while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)&&
		   prox_distance()){
		marche_avant(600);
		//allumer la LED 1
		last_pos = right_motor_get_pos();		//relever le compteur d'un des deux moteurs car vont dans le meme sens meme vitesse
	}
	palSetPad(GPIOD, GPIOD_LED1);//eteindre la LED 1
	//on arrete et on initialise pour le chemin retour
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	chThdSleepMilliseconds(100); //pour marquer un temps d'arret avant analyse

	//ajouter la fonction qui tourne l'epuck face à l'objet (angle env celui du capteur activé)
	ajustement_angle();

	// Si analyse couleur image est true, le robot avance jusqu'arene
	if(detec_rouge()){
		//joue la mort de mario pour indiquer que l'objet est dead
		playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
		while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)){
			palSetPad(GPIOD, GPIOD_LED_FRONT);
			marche_avant(800);
		}
		palClearPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED1);//eteindre la LED 1

		left_motor_set_speed(0);
		right_motor_set_speed(0);
		chThdSleepMilliseconds(100);

		while((left_motor_get_pos()>last_pos)&&(right_motor_get_pos()>last_pos)){
			marche_arriere(800);
		}
		palClearPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED5);//eteindre la LED 5
	}
	else{
		//signe de victoire quand il trouve un bleu
//		playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
	}


	left_motor_set_speed(0);
	right_motor_set_speed(0);
	chThdSleepMilliseconds(1000);

	while(compte_d !=0){
		chThdSleepMilliseconds(100);
		left_motor_set_speed(-100);
		right_motor_set_speed(100);
		compte_d --;
	}

	while(compte_g !=0 ){
		chThdSleepMilliseconds(100);
		left_motor_set_speed(100);
		right_motor_set_speed(-100);
		compte_g --;
	}

	//marche arriere jusqu'a la base
	while((left_motor_get_pos()>0)&&(right_motor_get_pos()>0)){
		marche_arriere(600);
	}
	palSetPad(GPIOD, GPIOD_LED5);	//éteindre la LED5

		//allumer la LED5
	//on arrete et on initialise pour la prochaine mesure
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

uint16_t get_mesure_i(uint8_t i){
	return tab_mesures[i];			//return la i-ème mesure
}

void set_mesure_i(uint16_t distance_i, uint8_t i){
	tab_mesures[i]=distance_i;		//set la i-eme mesure
}

void show_mesure(void){
	for(uint8_t i=0;i<NB_MESURES;i++){
    	chprintf((BaseSequentialStream *)&SD3, "§ %u-ieme mesure = %u ",i,tab_mesures[i]);
    	chThdSleepMilliseconds(1000);		//permet de lire en direct sur le realterm
	}
}


//va en avant et allume la LED1 (à eteindre apres)
void marche_avant(uint16_t speed){
	lumiere_eteinte();
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
	lumiere_eteinte();
	palClearPad(GPIOD, GPIOD_LED1);
}

//va en arriere et allume la LED5 (à eteindre apres)
void marche_arriere(uint16_t speed){
	lumiere_eteinte();
	left_motor_set_speed(-speed);
	right_motor_set_speed(-speed);
	palClearPad(GPIOD, GPIOD_LED5);			//allumer la LED5
}

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

