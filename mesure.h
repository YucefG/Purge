#ifndef MESURE_H
#define MESURE_H

//Defines du hardware
#define ANGLE360				360
#define D_ENTRE_ROUES			53			//en mm
#define PERIM_CERC_PARC			D_ENTRE_ROUES*PI		//Perimetre du cercle parcouru par les roues en 360∞
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

// Defines que l'on peut modifier
#define NB_MESURES				20     //taille max limit√© par uint8
#define TICS_360				(PERIM_CERC_PARC*TICS_1_TOUR)/DISTANCE_1_TOUR   //nb de tics pour faire un 360
#define TICS_90					TICS_360/4
#define TICS_1_MESURE			TICS_360/NB_MESURES
#define MARGE_PERIPH			40u
#define RAYON_ARENE				200u
#define LIMITE_COLLISION		1000	//pour pas de chocs via capt de prox
#define RAYON_ROUE_INT			RAYON_ARENE+MARGE_PERIPH-(D_ENTRE_ROUES/2)		
#define RAYON_ROUE_EXT			RAYON_ARENE+MARGE_PERIPH+(D_ENTRE_ROUES/2)	
#define ARC_1_MESURE_EPUCK		(ANGLE360/NB_MESURES)*(RAYON_ARENE+MARGE_PERIPH)*(2*PI/ANGLE360)
#define ARC_1_MESURE_INT		(ANGLE360/NB_MESURES)*RAYON_ROUE_INT*(2*PI/ANGLE360)		//en mm
#define ARC_1_MESURE_EXT		(ANGLE360/NB_MESURES)*RAYON_ROUE_EXT*(2*PI/ANGLE360)		//en mm
#define TICS_ARC_INT			ARC_1_MESURE_INT*(TICS_1_TOUR/DISTANCE_1_TOUR) 
#define TICS_ARC_EXT			ARC_1_MESURE_EXT*(TICS_1_TOUR/DISTANCE_1_TOUR) 
#define PERIMETRE_ARENE			2*(RAYON_ARENE+MARGE_PERIPH)*PI
#define RAYON_COLLECT			70			//en mm


#define KP						100.0f
#define KI 						0.3f	//must not be zero
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define MAX_SUM_ERROR 			300      //(MOTOR_SPEED_LIMIT/KI)

bool prox_distance(void);
void tour_mesures(void);
void object_detec_proche(void);
void dec_compteur(void);


#endif /* MESURE_H */
