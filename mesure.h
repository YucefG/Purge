#ifndef MESURE_H
#define MESURE_H

//Defines du hardware
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

// Defines que l'on peut modifier
#define NB_MESURES				30    //taille max limité par uint8
#define TICS_360				(PERIM_CERC_PARC*TICS_1_TOUR)/DISTANCE_1_TOUR   //nb de tics pour faire un 360
#define TICS_1_MESURE			TICS_360/NB_MESURES
#define DIAM_ARENE				200
#define TICS_1_ALLER			(DIAM_ARENE*TICS_1_TOUR)/DISTANCE_1_TOUR
#define LIMITE_COLLISION		1000							//pour pas de chocs via capt de prox

#define KP						150.0f
#define KI 						0.3f	//must not be zero
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define MAX_SUM_ERROR 			300 //(MOTOR_SPEED_LIMIT/KI)

bool prox_distance(void);
void next_angle(uint16_t speed);
void tour_mesures(void);
void object_detec_proche(void);

uint16_t get_mesure_i(uint8_t i);
void set_mesure_i(uint16_t distance_i, uint8_t i);
void show_mesures(void);

uint8_t get_compteur(void);
void dec_compteur(void);






#endif /* MESURE_H */
