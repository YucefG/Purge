#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mesure.h>
#include <math.h>
#include <arm_math.h>
#include <motors.h>
#include <chprintf.h>
#include <lumiere.h>
#include <stdbool.h>
#include <analyse_couleur.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <fonctions_maths.h>


//variable globale: tableau de mesures
uint16_t tab_mesures[NB_MESURES];			// uint16 ou 8 dicte la distance max
uint8_t compteur;


//simple PI regulator implementation - entrées distances en cm


_Bool prox_distance(void){
	//chprintf((BaseSequentialStream *)&SD3, "capt : %u, ",get_calibrated_prox(PROX_FRONT_R17));
	if((get_calibrated_prox(PROX_FRONT_R17)>LIMITE_COLLISION) ||
	  (get_calibrated_prox(PROX_FRONT_L17)>LIMITE_COLLISION))
		return false;
	else
		return true;
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

	if(tab_mesures[0]<RAYON_ARENE && tab_mesures[NB_MESURES-1]<RAYON_ARENE){
		uint16_t shortest_dist_g=tab_mesures[NB_MESURES-1];
		uint8_t pos_shortest_g = NB_MESURES-1;
		uint16_t shortest_dist_d=tab_mesures[0];
		uint8_t pos_shortest_d = 0;
		//compter la taille de l'objet sens normal
		uint8_t j=0;
		while((tab_mesures[j+1]<RAYON_ARENE)&&((j+1)<NB_MESURES))
			j++;
		uint8_t k=0;
		while((tab_mesures[NB_MESURES-1-k-1]<RAYON_ARENE)&& (k<NB_MESURES))
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
			tab_mesures[pos_shortest_d]=shortest_dist_d;
			tab_mesures[pos_shortest_g]=0;

		}
		else
		{
			tab_mesures[pos_shortest_d]=0;
			tab_mesures[pos_shortest_g]=shortest_dist_g;
		}
	}

	for(uint8_t i=0; i<NB_MESURES; i++){
		if(tab_mesures[i]<RAYON_ARENE){	//rayon ou diam
			uint8_t j = i;
			while((tab_mesures[j+1]<RAYON_ARENE)&&((j+1)<NB_MESURES))
				j++;
			if(j==i){}	//ne rien toucher si le point de mesure est isolé	
			else{
				uint16_t shortest_dist=RAYON_ARENE;
				uint8_t pos_shortest = i;
				for(uint8_t k=0; k+i<=j; k++){				//<= ou <?
					if(tab_mesures[k+i]<shortest_dist){
						shortest_dist = tab_mesures[k+i];
						tab_mesures[pos_shortest]=0;
						pos_shortest = k+i;
						tab_mesures[k+i]=shortest_dist;
					}
					else
					tab_mesures[k+i]=0;
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
		if(tab_mesures[i]!=0)
			compteur++;
	}
	check_compteur(compteur);

}



uint16_t get_mesure_i(uint8_t i){
	return tab_mesures[i];			//return la i-ème mesure
}

void set_mesure_i(uint16_t distance_i, uint8_t i){
	tab_mesures[i]=distance_i;		//set la i-eme mesure
}

void show_mesures(void){
	for(uint8_t i=0;i<NB_MESURES;i++){
    	chprintf((BaseSequentialStream *)&SD3, "§ %u-ieme mesure = %u ",i,tab_mesures[i]);
    	chThdSleepMilliseconds(1000);		//permet de lire en direct sur le realterm
	}
}

uint8_t get_compteur(void){
	return compteur;
}

void dec_compteur(void){
	compteur--; //decremente le compteur
}



