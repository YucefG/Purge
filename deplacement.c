#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>
#include <stdbool.h>
#include <arm_math.h>
#include <fonctions_maths.h>
#include <analyse_couleur.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <fonctions_maths.h>
#include <deplacement.h>
#include <mesure.h>
#include <lumiere.h>
#include <audio_processing.h>
#include <selector.h>

//compteurs pour ajustement d'angle
static uint8_t compte_g = 0;
static uint8_t compte_d = 0;
//si true: deplacement permis, sinon arret
bool onRoad = 1;		
//transmet a get_out_arena le dernier objet rouge poussé					 
static uint8_t indice_sortie = 0;			

int16_t pi_regulator(float distance, float goal)
{
	float error = 0;
	float speed = 0;
	static float sum_error = 0;
	error = goal - distance;
	//disables the PI regulator if the error is to small
	if(fabs(error) < ERROR_THRESHOLD)
	{
		onRoad = 0 ;
		return 0;
	}
	else
		onRoad = 1;

	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR)
	{
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR)
	{
		sum_error = -MAX_SUM_ERROR;
	}
	
	speed = KP * error + KI * sum_error;

	//Mettre le booleen onRoad a 0 car on arrive a destination
	if(abs(speed)<40)
	{
		onRoad = 0 ;
		return 0;
	}

	//Ajustement de la vitesse max positive ou négative car problémes au max du hardware
	if(speed>MAX_VITESSE_PI)
		speed = MAX_VITESSE_PI;

	if(speed<-MAX_VITESSE_PI)
		speed = -MAX_VITESSE_PI;
    
    return (int16_t)speed;
}

//tourne jusqu'au prochain angle 360/NB_MESURES à la vitesse speed 
void next_angle(uint16_t speed)
{	
	while((left_motor_get_pos()<TICS_1_MESURE)&&(right_motor_get_pos()<TICS_1_MESURE))
	{
		left_motor_set_speed(speed);
		right_motor_set_speed(-speed);
	}
	//on arrete et on initialise pour la prochaine mesure
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

/*
*	avance en ligne droite jusqu'a objectif en cm, avance si avancer = true, recule sinon
*	charge si charge = true, sinon s'arrete si detection d'objet devant
*	important: n'initialise pas les compteurs de moteurs, à lancer avant la fonction
*/
void ligne_droite_pi(float objectif, bool avancer, bool charge)//objectif qui est en cm (positif ou negatif)
{		
	if(avancer==true)
	{
		//avance sans s'arreter si detection d'objet devant (charge) + LED frontale
		if(charge==true)
		{	
			onRoad = 1;
			palSetPad(GPIOD, GPIOD_LED_FRONT);
			while(onRoad)
			{
				marche_avant(pi_regulator(StepsToCm(right_motor_get_pos()), objectif));
			}
			palClearPad(GPIOD, GPIOD_LED_FRONT);
		}
		//avance en s'arretant si detection d'objet devant + LED 1 
		else
		{
			onRoad = 1;
			palClearPad(GPIOD, GPIOD_LED1);
			while(prox_distance()&&onRoad)
			{
				marche_avant(pi_regulator(StepsToCm(right_motor_get_pos()), objectif));
			}
			palSetPad(GPIOD, GPIOD_LED1);
		}
	}
	//recule sans s'arreter si objet derriere (passage propre) + LED 5 
	if(avancer==false)
	{
		onRoad = 1;
		palClearPad(GPIOD, GPIOD_LED5);
		while((left_motor_get_pos()>0)&&(right_motor_get_pos()>0)&&onRoad)
		{
			marche_avant(pi_regulator(StepsToCm(right_motor_get_pos()), objectif));
		}
		palSetPad(GPIOD, GPIOD_LED5);
	}
}

/*
*	centre le devant de l'epuck face à l'objet le plus proche
*	permet une meilleure prise de la caméra 
*/
void ajustement_angle(void)
{
	uint16_t ajustement = 0;
	ajustement = abs(get_calibrated_prox(PROX_FRONT_R17) - get_calibrated_prox(PROX_FRONT_L17));
	chThdSleepMilliseconds(1000);			 
	while((get_calibrated_prox(PROX_FRONT_R17) > get_calibrated_prox(PROX_FRONT_L17)) &&
			ajustement > SEUIL_AJUSTEMENT)
	{
		// recentrer
		chThdSleepMilliseconds(100); 
		left_motor_set_speed(100);
		right_motor_set_speed(-100);
		compte_d++;
	}

	while((get_calibrated_prox(PROX_FRONT_R17) < get_calibrated_prox(PROX_FRONT_L17)) &&
			ajustement > SEUIL_AJUSTEMENT)
	{
		// recentrer
		chThdSleepMilliseconds(100); 
		left_motor_set_speed(-CENT);
		right_motor_set_speed(CENT);
		compte_g ++;
	}
}

/*
*	pousse les objets (rouges ou tous en fonction de la position du selecteur)
* 	vers l'exterieur en parcourant le tableau de mesures
*/ 
void object_push(void)
{
	for(uint8_t i=0;i<NB_MESURES;i++)
	{
		if(get_mesure_i(i)!=0)
		{	
			deplacement_push(i);
			dec_compteur();
			check_compteur(get_compteur());
		}
		next_angle(VITESSE_ROTA_ANGLES);	
	}
	playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
	chThdSleepMilliseconds(1000);
}

void re_axage_angle(void)
{
	while(compte_d !=0)
	{
		chThdSleepMilliseconds(100);
		left_motor_set_speed(-CENT);
		right_motor_set_speed(CENT);
		compte_d --;
	}

	while(compte_g !=0 )
	{
		chThdSleepMilliseconds(100);
		left_motor_set_speed(CENT);
		right_motor_set_speed(-CENT);
		compte_g --;
	}
}

/*
*	s'approche d'un objet détecté, en fonction de la valeur du selecteur 
*	va charger l'objet ou non en dehors de la région de travail,
*	délimitée par RAYON_ARENA+MARGE_PERIPH+MARGE_POUSSEE. 
*/
void deplacement_push(uint8_t indice)
{
	//étape 1: avancer jusqu'a l'objet détecté
	bool avancer = MARCHE_AVANT;
	bool charge = PAS_CHARGE;
	init_pos_mot();	
	uint16_t pos_detection =0;					
	float objectif = MmToCm(get_mesure_i(indice));
    ligne_droite_pi(objectif, avancer, charge);

	//étape 2: faire face à la caméra à l'arret
	init_vitesse_mot();
	pos_detection = StepsToCm(right_motor_get_pos());	//stocke la position a la detection d'un objet 	
	chThdSleepMilliseconds(100); 
	ajustement_angle();

	//étape 3: en fonction de la valeur du selecteur: charger si rouge ou charger si objet
	if((get_selector()==1)||(get_selector()==2||get_selector()==5))
	{
		if(detec_rouge()==1)
		{
			playMelody(MARIO_DEATH, ML_FORCE_CHANGE, NULL);
			//charge l'objet s'il est rouge jusqu'à l'exterieur de l'arène 
			charge = CHARGE;
			ligne_droite_pi(MmToCm((uint16_t)(RAYON_ARENE+MARGE_PERIPH+MARGE_POUSSEE)), avancer, charge);
			init_vitesse_mot();
			chThdSleepMilliseconds(100);
			//revient à sa position avant la charge
			charge = PAS_CHARGE;
			avancer = MARCHE_ARRIERE;
			ligne_droite_pi(pos_detection,avancer,charge);
			//efface l'objet rouge pousse comme objet d'interet pour ne pas le collect ensuite
			set_mesure_i(ABSENCE_OBJET, indice);
			//memorisation du dernier rouge deplace pour sortir avant objet_collect
			indice_sortie = indice;
		}
	}
	else if(get_selector()==3)
	{
		//charge dans tous les cas l'objet jusqu'à l'exterieur de l'arène
		charge = CHARGE;
		ligne_droite_pi(MmToCm((uint16_t)RAYON_ARENE+MARGE_PERIPH+MARGE_POUSSEE), avancer, charge);
		init_vitesse_mot();
		chThdSleepMilliseconds(100);
		//revient à sa position avant la charge
		charge = PAS_CHARGE;
		avancer = MARCHE_ARRIERE;
		ligne_droite_pi(pos_detection,avancer,charge);
	}

	//étape 4: se ré-axer parallèle à la trajectoire d'arrivée avant détection
	init_vitesse_mot();
	chThdSleepMilliseconds(100);
	re_axage_angle();

	//étape 5: faire marche arrière jusqu'à la base de mesure
	avancer = MARCHE_ARRIERE; 
	charge = PAS_CHARGE; 
	chThdSleepMilliseconds(100);
	ligne_droite_pi(COMPTEUR_BASE, avancer, charge);

	//étape 6: arrêt et initialisation pour la prochaine fonction
	init_vitesse_mot();
	init_pos_mot();
	chThdSleepMilliseconds(100);

}

/*
* 	Donne la meme vitesse aux deux moteurs pour aller tout droit, en avant ou en arriere
*/
 
void marche_avant(int16_t speed)
{
	lumiere_eteinte();			
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
	lumiere_eteinte();		
	palClearPad(GPIOD, GPIOD_LED1);	 
}

/*
*		Donne deux vitesses et deux distances differentes pour chaque moteur pour 
*		parcourir le bon arc.
*/
void next_arc(float speed)
{
	init_vitesse_mot();
	init_pos_mot();

	//apres de nombreux bugs avec les defines, préférences pour variables locales
	static float coeff_convers = (float)100/(float)13;
	static float angle_rad=2*PI/NB_MESURES;

	float temps = (float)(((float)ARC_1_MESURE_EPUCK)*(MM_2_CM)/speed); //	mm->cm
	float rayon_int=(float)RAYON_ROUE_INT;
	float rayon_ext=(float)RAYON_ROUE_EXT;
	float l_int = angle_rad*rayon_int;
	float l_ext = angle_rad*rayon_ext;
	int16_t step_int = (uint16_t)(l_int*coeff_convers);
	int16_t step_ext = (uint16_t)(l_ext*coeff_convers);
	int16_t vitesse_int = (int16_t)(coeff_convers*angle_rad*rayon_int/temps);
	int16_t vitesse_ext = (int16_t)(coeff_convers*angle_rad*rayon_ext/temps);

	//Le moteur droit et gauche parcourent des arcs differents
	while((abs(left_motor_get_pos())<step_ext)&&(abs(right_motor_get_pos())<step_int))
	{
		right_motor_set_speed(vitesse_int);
		left_motor_set_speed(vitesse_ext);
	}

	//arret et initialisation pour la prochaine fonction
	init_pos_mot();
	init_vitesse_mot();
}

/*
*	Pousse les objets dans l'arene dans un cercle de rayon
* 	RAYON_COLLECT en parcourant le tableau de mesures.
*/
void object_collect(void)
{
	init_vitesse_mot();
	/*
	*	Parcourt le tableau des mesures, en parcourant le cercle 
	*	en peripherie de l'arene. Et s'il detecte un objet, il plonge
	*	dans l'arene pour la pousser au centre.
	*/
	for(uint8_t i=0;i<NB_MESURES;i++)
	{
		if(get_mesure_i(i)!=0)
		{	
			turn_90(VITESSE_ROTA_ANGLES);
			deplacement_collect(i);
			turn_90(-VITESSE_ROTA_ANGLES);
			dec_compteur();
			check_compteur(get_compteur());
		}
		next_arc(VITESSE_ARC);
	}
	turn_90(VITESSE_ROTA_ANGLES);//finit en faisant face au tas
	playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
	chThdSleepMilliseconds(1000);
}

/*
*	Fait un mouvement rectiligne vers le centre de l'arene jusqu'a detection
* 	d'un objet, adapte son angle de poussee, et pousse l'objet au centre de l'arene.
*	Puis fait les étapes dans le sens inverse pour retourner a son point de depart. 
*/
void deplacement_collect(uint8_t indice){
	//etape 1: initialisation et avancer jusqu'a detection d'un objet vers le centre de l'arene
	bool avancer = MARCHE_AVANT;	
	bool charge = PAS_CHARGE;	
	uint16_t pos_detection =0;
	init_pos_mot();
	/*
	*	L'objectif est calculee en soustrayant au rayon de l'arene (et de la peripherie ou l'epuck 
	*	se deplace), la distance de l'objet pointe au centre de l'arene. 
	*	Si la mesure est faussee, les capteurs de proximite arreteront l'epuck devant l'objet. 
	*/
	float objectif = MmToCm(RAYON_ARENE+MARGE_PERIPH-get_mesure_i(indice));	
    ligne_droite_pi(objectif, avancer, charge);	

	//etape 2: ajustement de l'angle pour la meilleure poussee possible (face a la surface la plus proche)
	init_vitesse_mot();
	/*
	*	On garde dans pos_detection une memoire de la position de l'epuck avant la charge,
	*	pour la re-utiliser plus tard, afin de revenir à cette position apres charge. 
	*/
	pos_detection = StepsToCm(right_motor_get_pos());
	chThdSleepMilliseconds(100); //pour marquer un temps d'arret avant analyse
	ajustement_angle();
	chThdSleepMilliseconds(100); //pour marquer un temps d'arret avant analyse

	//etape 3:  charge de l'objet jusqu'a environ le centre de l'arene, puis recul.
	/*
	*	Jusqu'au centre de l'arene: dans le cercle de rayon RAYON_COLLECT.
	*	Si l'objet est deja dans ce cercle, on ne charge pas. 
	*	Le recul se fait jusqu'a pos_detection.
	*/
	if(objectif<(RAYON_ARENE+MARGE_PERIPH-RAYON_COLLECT))
	{
		avancer = MARCHE_AVANT; 	
		charge = CHARGE;
		ligne_droite_pi(MmToCm((uint16_t)RAYON_ARENE+MARGE_PERIPH-RAYON_COLLECT), avancer, charge);
		init_vitesse_mot();
		chThdSleepMilliseconds(100);


		avancer = MARCHE_ARRIERE; 	
		charge = PAS_CHARGE;
		ligne_droite_pi(pos_detection, avancer, charge);
		init_vitesse_mot();
		chThdSleepMilliseconds(100);
	}
	
	re_axage_angle();
	chThdSleepMilliseconds(100); //pour marquer un temps d'arret avant analyse

	//etape 5: Marche arriere jusqu'a la peripherie
	/*
	*	La position au debut de cette fonction correspond a la position
	*	ou le compteur est nul. D'ou la decrementation jusqu'a ce cas. 
	*/
	avancer = MARCHE_ARRIERE;  	
	charge = PAS_CHARGE; 	
	ligne_droite_pi(COMPTEUR_BASE, avancer, charge);

	//etape 6: initialisation et arret pour la prochaine fonction
	init_vitesse_mot();
	init_pos_mot();
}

/*
*		Sort l'epuck du centre de l'arene en cherchant un chemin vide
*		Puis se place au point de demarrage de la prochaine fonction
*/
void get_out_arena(void)
{
	//rotation jusqu'a la position du indice_sortie-eme angle
	for(uint8_t i=0;i<indice_sortie;i++)
	{
		next_angle(VITESSE_ROTA_ANGLES);
	}
	init_vitesse_mot();
	init_pos_mot();
	chThdSleepMilliseconds(100);
	//ligne droite vers la peripherie de l'arene
	ligne_droite_pi(MmToCm(RAYON_ARENE+MARGE_PERIPH),MARCHE_AVANT,CHARGE);
    init_pos_mot();
    init_vitesse_mot();
	chThdSleepMilliseconds(100);
	//rotation de -90 degres  
	turn_90(VITESSE_ROTA_ANGLES);
	init_pos_mot();
	chThdSleepMilliseconds(100);
	//prend le periph jusqu'a la position de depart des collect en marche arriere
	for(uint8_t i=0;i<indice_sortie;i++)
	{
		next_arc(-VITESSE_ARC);
	}
	init_pos_mot();
	init_vitesse_mot();
	chThdSleepMilliseconds(100);
}

/*
*	Effectue une rotation de 90 degres a une vitesse donnee. 
*/
void turn_90(int16_t speed)
{
	static int16_t tics90 = TICS_90;
	while((left_motor_get_pos()<tics90)&&(right_motor_get_pos()<tics90))
	{
		left_motor_set_speed(speed);
		right_motor_set_speed(-speed); 
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

//initialise les compteurs de moteur.
void init_pos_mot(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
//met les vitesses de moteur a zero. 
void init_vitesse_mot(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
