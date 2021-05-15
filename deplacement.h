#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

#define COEFF_MM_2_STEPS				100/13
#define MARGE_POUSSEE					80
#define MAX_VITESSE_PI					1000
//#define MAX_TICS						((RAYON_ARENE+MARGE_PERIPH+MARGE_POUSSEE)*COEFF_MM_2_STEPS


int16_t pi_regulator(float distance, float goal);
void next_angle(uint16_t speed);
void ligne_droite_pi(float objectif, bool avancer, bool charge);
void ajustement_angle(void);
void object_push(void);
void deplacement_push(uint8_t indice);
void marche_avant(int16_t speed);
void marche_arriere(uint16_t speed);
void next_arc(float speed);
void object_collect(void);
void deplacement_collect(uint8_t indice);
void get_out_arena(void);
void turn_90(int16_t speed);

#endif /* DEPLACEMENT_H */
