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
static uint16_t tab_mesures[NB_MESURES];
static uint8_t compteur;


_Bool prox_distance(void)
{
	/* Si les capteurs de proximite situes a un angle de 17° ont atteint la limite de collision
	 * alors la fonction return true pour arreter le robot*/
	if((get_calibrated_prox(PROX_FRONT_R17)>LIMITE_COLLISION) ||
	  (get_calibrated_prox(PROX_FRONT_L17)>LIMITE_COLLISION))
		return false;
	else
		return true;
}


/*1ere etape: remplir le tableau avec le TOF
 * en effectuant un tour de 360° autour de lui-meme*/
void tour_mesures(void)
{
	//initialiser les compteurs des moteurs
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	palSetPad(GPIOB, GPIOB_LED_BODY);

	/*le robot range dans un tableau la distance des objets
	 * dans l'arene avec un  offset qui permet d'etre plus precis d
	 * dans les distances
	 */
	for(uint8_t i=0; i<NB_MESURES; i++)
	{
		tab_mesures[i]=(uint16_t)VL53L0X_get_dist_mm() - (uint16_t)OFFSET;
		next_angle(200);
	}
	signal_fin();
}

//2eme etape: detecter les objets
void object_detec_proche(void){

	if(tab_mesures[0]<RAYON_ARENE && tab_mesures[NB_MESURES-1]<RAYON_ARENE)
	{
		uint16_t shortest_dist_g=tab_mesures[NB_MESURES-1];
		uint8_t pos_shortest_g = NB_MESURES-1;
		uint16_t shortest_dist_d=tab_mesures[0];
		uint8_t pos_shortest_d = 0;
		//Compter la taille de l'objet : trouver un debut et une fin
		uint8_t j=0;
		while((tab_mesures[j+1]<RAYON_ARENE)&&((j+1)<NB_MESURES))
			j++;
		uint8_t k=0;
		while((tab_mesures[NB_MESURES-1-k-1]<RAYON_ARENE)&& (k<NB_MESURES))
			k++;


		for(uint8_t l=1; l<=j;l++)
		{
			if(tab_mesures[l]<shortest_dist_d)
			{
				shortest_dist_d = tab_mesures[l];
				tab_mesures[pos_shortest_d]=0;
				pos_shortest_d = l;
				tab_mesures[l]=1;
			}
			else
				tab_mesures[l]=0;
		}

		for(uint8_t t=1; t<=k;t++)
		{
			if(tab_mesures[NB_MESURES-1-t]<shortest_dist_g)
			{

		    	tab_mesures[pos_shortest_g]=0;
				shortest_dist_g = tab_mesures[NB_MESURES-1-t];

				pos_shortest_g = NB_MESURES-1-t;
				tab_mesures[NB_MESURES-1-t]=1;

			}
			else
				tab_mesures[NB_MESURES-1-t]=0;
		}
 	//On compare les deux minimas des deux cotés du cercle
		if(shortest_dist_d<shortest_dist_g)
		{
			tab_mesures[pos_shortest_d]=shortest_dist_d;
			tab_mesures[pos_shortest_g]=0;

		}
		else
		{
			tab_mesures[pos_shortest_d]=0;
			tab_mesures[pos_shortest_g]=shortest_dist_g;
		}
	}

	for(uint8_t i=0; i<NB_MESURES; i++)
	{
		if(tab_mesures[i]<RAYON_ARENE)
		{
			uint8_t j = i;
			while((tab_mesures[j+1]<RAYON_ARENE)&&((j+1)<NB_MESURES))
				j++;
			if(j==i){}
			else
			{
				uint16_t shortest_dist=RAYON_ARENE;
				uint8_t pos_shortest = i;
				for(uint8_t k=0; k+i<=j; k++)
				{
					if(tab_mesures[k+i]<shortest_dist)
					{
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
	for(uint8_t i=0; i<NB_MESURES; i++)
	{
		if(tab_mesures[i]!=0)
			compteur++;
	}
	/*Cette fonction ecrite dans lumiere.c permet d'indiquer
	 * combien de d'objet il reste sur l'arene et de l'indiquer avec les LEDs */
	check_compteur(compteur);

}

void dec_compteur(void)
{
	compteur--; //decremente le compteur
}



