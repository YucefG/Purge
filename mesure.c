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

//defines qu'on peut modifier
#define NB_MESURES				16    //taille max limité par uint8
#define TICS_360				(PERIM_CERC_PARC*TICS_1_TOUR)/DISTANCE_1_TOUR   //nb de tics pour faire un 360
#define TICS_1_MESURE			TICS_360/NB_MESURES
#define DIAM_ARENE				300
#define TICS_1_ALLER			(DIAM_ARENE*TICS_1_TOUR)/DISTANCE_1_TOUR

//variable globale: tableau de mesures
uint16_t tab_mesures[NB_MESURES];			// uint16 ou 8 dicte la distance max
uint8_t compteur =0;

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

	//bouger aux NB_MESURES positions et faire la mesure (remplir le tab)
	for(uint8_t i=0; i<NB_MESURES; i++){
		//faire la mesure
		tab_mesures[i]=(uint16_t)VL53L0X_get_dist_mm() - (uint16_t)30;   //est ce que les valeurs vont etre restreintes par le uint8? (max:255)
		next_angle(100);
	}
	//jeu de lumiere pour dire qu'il a fini
	signal_fin();
}

//2eme etape: detecter les objets
void object_detec(void){
	//place des 1 ou la distance est inf a DIAM_ARENE
	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]<DIAM_ARENE)			//pour eviter les erreurs de bruit, on posera que c un objet a partir de ... 1 de suite
			tab_mesures[i]=1;
		else
			tab_mesures[i]=0;	}

	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]==1){
			uint8_t j = i;					//pour eviter les erreurs de bruit, on posera que c un objet a partir de ... 1 de suite
			while(tab_mesures[j]==1){
				j++;
			}

			for(uint8_t k=0; k<=j-i; k++){		//<= ou <?
				if(k==(j-i)/2)
					tab_mesures[k+i]=1;
				else
					tab_mesures[k+i]=0;
			}
			i =j+1;  //saute a la fin de l'objet
		}
	}
	//nombre d'objets détéctés
	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]==1)
			compteur++;
	}

	show_mesure();

	chprintf((BaseSequentialStream *)&SD3, "Il y a %u objets detectes",compteur);
	check_compteur();

}

//3eme etape: pousser les objets
void object_push(void){
	for(uint8_t i=0;i<NB_MESURES;i++){
		if(tab_mesures[i]==1)
		deplacement();
		next_angle(100);
		compteur--;

		check_compteur();
	}
}

void deplacement(void){
	//bouger les moteurs : marche avant jusqu'a la base
	while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)){
		lumiere_eteinte();
		left_motor_set_speed(400);
		right_motor_set_speed(400);
		palTogglePad(GPIOD, GPIOD_LED1);
	}palSetPad(GPIOD, GPIOD_LED1);

	//on arrete et on initialise pour la prochaine mesure
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	//marche arriere jusqu'a la base
	while((left_motor_get_pos()>-TICS_1_ALLER)&&(right_motor_get_pos()>-TICS_1_ALLER)){
		lumiere_eteinte();
		left_motor_set_speed(-400);
		right_motor_set_speed(-400);
		palTogglePad(GPIOD, GPIOD_LED5);
	}palSetPad(GPIOD, GPIOD_LED5);

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
    	chprintf((BaseSequentialStream *)&SD3, " | la %u-ieme mesure est à %u ",i,tab_mesures[i]);
	}
}
