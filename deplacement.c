#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>
#include <stdbool.h>
#include <analyse_couleur.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <fonctions_maths.h>
#include <deplacement.h>
#include <mesure.h>
#include <fonctions_maths.h>

static uint8_t compte_g = 0;
static uint8_t compte_d = 0;
bool onRoad = 1;

int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = goal - distance;

	//disables the PI regulator if the error is to small
	if(fabs(error) < ERROR_THRESHOLD){
		onRoad = 0 ;
		return 0;
	}
	else
		onRoad = 1;

	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}
	
	speed = KP * error + KI * sum_error;
	if(abs(speed)<40){
		onRoad = 0 ;
		return 0;
	}
    
    return (int16_t)speed;
}

void ligne_droite_pi(float objectif, bool avancer, bool charge){		//objectif qui est en cm (positif ou negatif)
	if (avancer==true)
	{
		if(charge==true){		//coup de boule
			onRoad = 1;
			palSetPad(GPIOD, GPIOD_LED_FRONT);		//allumer led frontale
			while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)&&onRoad){
				marche_avant(pi_regulator(StepsToCm(right_motor_get_pos()), objectif));
			}		//ajouter a prox distance un booleen qui le desactive SI avancer ==0 (donc qu'il recule)
			palClearPad(GPIOD, GPIOD_LED_FRONT);		//eteindre led frontale
		}
		else{
			onRoad = 1;
			palClearPad(GPIOD, GPIOD_LED1);//allumer la LED1
			while((left_motor_get_pos()<TICS_1_ALLER)&&(right_motor_get_pos()<TICS_1_ALLER)&&
				prox_distance()&&onRoad){
				marche_avant(pi_regulator(StepsToCm(right_motor_get_pos()), objectif));
			}		//ajouter a prox distance un booleen qui le desactive SI avancer ==0 (donc qu'il recule)
			palSetPad(GPIOD, GPIOD_LED1);//éteindre la LED1
		}

	}
	if (avancer==false)
	{
		onRoad = 1;
		palClearPad(GPIOD, GPIOD_LED5);//allumer la LED5
		while((left_motor_get_pos()>0)&&(right_motor_get_pos()>0)&&onRoad){
			marche_avant(pi_regulator(StepsToCm(right_motor_get_pos()), objectif));
		}		//ajouter a prox distance un booleen qui le desactive SI avancer ==0 (donc qu'il recule)
		palSetPad(GPIOD, GPIOD_LED5);//éteindre la LED1
	}
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

//3eme etape: pousser les objets
void object_push(void){
	for(uint8_t i=0;i<NB_MESURES;i++){

		if(get_mesure_i(i)!=0){	
		//peut etre prob de define
		deplacement(i);
		dec_compteur();
		check_compteur(get_compteur());
		}
		next_angle(200);
	}
//	playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
	chThdSleepMilliseconds(1000);
}

void deplacement(uint8_t indice){
	//bouger les moteurs : marche avant jusqu'a la base
	bool avancer = 1;	//va dire si l'epuck avance ou non (oui au debut)
	bool charge = 0;	//true=coup de boule false=s'arreter si obstacle
	uint16_t last_pos =0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	float objectif = MmToCm(get_mesure_i(indice));	//conversion en cm float des valeurs entieres en mm

    ligne_droite_pi(objectif, avancer, charge);	//objectif qui est en nb de cycles (positif ou negatif)
	//on arrete et on initialise pour le chemin retour
	
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	last_pos = StepsToCm(right_motor_get_pos());
	chThdSleepMilliseconds(100); //pour marquer un temps d'arret avant analyse

	//ajouter la fonction qui tourne l'epuck face à l'objet (angle env celui du capteur activé)
	ajustement_angle();

	if(get_selector()==1){
		if(detec_rouge()==1){
			//joue la mort de mario pour indiquer que l'objet est dead
			//playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
			charge = 1;
			
			ligne_droite_pi(MmToCm((uint16_t)DIAM_ARENE),avancer,charge);

			palClearPad(GPIOD, GPIOD_LED_FRONT);	//éteindre led frontale

			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(100);
			charge = 0;
			avancer = false; 	//instruction de reculer
			ligne_droite_pi(last_pos,avancer,charge);
		}
	}
	else if(get_selector()==2){
		//joue la mort de mario pour indiquer que l'objet est dead
		//playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
		
		charge = 1;
		ligne_droite_pi(MmToCm((uint16_t)DIAM_ARENE), avancer, charge);
		
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		chThdSleepMilliseconds(100);

		avancer = false; 	//instruction de reculer
		charge = 0;
		ligne_droite_pi(last_pos,avancer, charge);
	}

	//robot re-axe son angle initial
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	chThdSleepMilliseconds(100);		//petite pause arbitraire

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
	avancer = false;  	//normalement deja a false mais c'est une sécurité
	charge = 0; 	//pareil pour les charge: tous les charges a 0 ne servent a rien
	ligne_droite_pi(0,avancer, charge);

	//on arrete et on initialise pour la prochaine mesure
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

//va en avant et allume la LED1 (à eteindre apres)
void marche_avant(int16_t speed){
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


