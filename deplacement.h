#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

// Defines que l'on peut modifier
#define VITESSE_ARC						9
#define VITESSE_ROTA_ANGLES				200

//Defines optimises deconseille de modifier
#define MARGE_POUSSEE					80
#define MAX_VITESSE_PI					1000 
#define SEUIL_AJUSTEMENT 				20
#define SEUIL_VIT_NUL_PI				40
#define COEFF_MM_2_STEPS				100/13
#define COEFFSTEPSCM					100
#define PERIM_ROUE_CM					13

//Constantes arbitraires
#define MARCHE_AVANT 					true
#define MARCHE_ARRIERE 					false
#define CHARGE 							true
#define PAS_CHARGE						false
#define COMPTEUR_BASE					0
#define MM_2_CM							0.1f
#define CENT							100
#define ABSENCE_OBJET					0
#define PRESENCE_OBJET					1
#define EN_CHEMIN						true
#define ARRET							false
#define VITESSE_NULLE					0




int16_t pi_regulator(float distance, float goal);
void next_angle(uint16_t speed);
void ligne_droite_pi(float objectif, bool avancer, bool charge);
void ajustement_angle(void);
void re_axage_angle(void);
void object_push(void);
void deplacement_push(uint8_t indice);
void marche_avant(int16_t speed);
void marche_arriere(uint16_t speed);
void next_arc(float speed);
void object_collect(void);
void deplacement_collect(uint8_t indice);
void get_out_arena(void);
void turn_90(int16_t speed);
void init_pos_mot(void);
void init_vitesse_mot(void);

#endif /* DEPLACEMENT_H */
