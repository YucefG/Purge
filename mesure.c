#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>
#include <lumiere.h>


//defines du hardware
#define PI						3.14
#define ANGLE360				360
#define D_ENTRE_ROUES			53			//en mm
#define PERIM_CERC_PARC			D_ENTRE_ROUES*PI		//perimetre du cercle parcouru par les roues en 360
#define TICS_1_TOUR				1000
#define DISTANCE_1_TOUR			130			//en mm
#define OFFSET					30

//defines qu'on peut modifier
#define NB_MESURES				30    //taille max limité par uint8
#define TICS_360				(PERIM_CERC_PARC*TICS_1_TOUR)/DISTANCE_1_TOUR   //nb de tics pour faire un 360
#define TICS_1_MESURE			TICS_360/NB_MESURES
#define DIAM_ARENE				200
#define TICS_1_ALLER			(DIAM_ARENE*TICS_1_TOUR)/DISTANCE_1_TOUR

//variable globale: tableau de mesures
uint16_t tab_mesures[NB_MESURES];			// uint16 ou 8 dicte la distance max
uint8_t compteur;

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

	//bouger aux NB_MESURES positions et faire les mesures
	for(uint8_t i=0; i<NB_MESURES; i++){
		tab_mesures[i]=(uint16_t)VL53L0X_get_dist_mm() - (uint16_t)OFFSET;
		next_angle(200);
	}
	signal_fin();
}

//2eme etape: detecter les objets
/*
void object_detec_centre(void){
	//place des 1 ou la distance est inf a DIAM_ARENE
	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]<DIAM_ARENE){			//pour eviter les erreurs de bruit, on posera que c un objet a partir de ... 1 de suite
			tab_mesures[i]=1;
	//    	chprintf((BaseSequentialStream *)&SD3, "§ %u-ieme: objet TROUVE ",i);
	//    	chThdSleepMilliseconds(1000);
		}
		else{
	//		chprintf((BaseSequentialStream *)&SD3, "§ %u-ieme: objet ABSENT ",i);
			tab_mesures[i]=0;
	//		chThdSleepMilliseconds(1000);
		}
	}
	show_mesure();
 	 DECOMMENTER SI BESOIN DE LAUTRE TECHNIQUE

	for(uint8_t i=0; i<NB_MESURES; i++){
//		chprintf((BaseSequentialStream *)&SD3, "\n\nDans la mesure: %u ",i);
	//	chThdSleepMilliseconds(1000);
		if(tab_mesures[i]==1){
	//	if(tab_mesures[i]<DIAM_ARENE){
			uint8_t j = i+1;

			if(tab_mesures[j]==1){
				while((tab_mesures[j+1]==1)&&(j<NB_MESURES)){  //j<=?
					j++;
				}
				 	METHODE DU CENTRE DE LOBJET
							for(uint8_t k=0; k<=j-i; k++){		//<= ou <?
								if(k==(j-i)/2)
									tab_mesures[k+i]=1;
								else
									tab_mesures[k+i]=0;
							}
				}
			}
			i=j;
		}
		else
			tab_mesures[i]=0;
	show_mesure();


	//nombre d'objets détéctés
	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]==1)
			compteur++;
	}

//	show_mesure();

	chprintf((BaseSequentialStream *)&SD3, "Il y a %u objets detectes",compteur);
	check_compteur(compteur);

}
*/

void object_detec_proche(void){

	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]<DIAM_ARENE){	//rayon ou diam
			uint8_t j = i;
			if(tab_mesures[j]<DIAM_ARENE){

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
}

void deplacement(void){
	//bouger les moteurs : marche avant jusqu'a la base
	while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)){
		lumiere_eteinte();
		left_motor_set_speed(600);
		right_motor_set_speed(600);
		lumiere_eteinte();
		palClearPad(GPIOD, GPIOD_LED1);			//allumer la LED 1
	}
	palSetPad(GPIOD, GPIOD_LED1);				//eteindre la LED 1

	//on arrete et on initialise pour le chemin retour
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	//marche arriere jusqu'a la base
	while((left_motor_get_pos()>-TICS_1_ALLER)&&(right_motor_get_pos()>-TICS_1_ALLER)){
		lumiere_eteinte();
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
		palClearPad(GPIOD, GPIOD_LED5);			//allumer la LED5
	}
	palSetPad(GPIOD, GPIOD_LED5);				//éteindre la LED5

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
