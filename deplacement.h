#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

int16_t pi_regulator(float distance, float goal);
void ligne_droite_pi(float objectif, bool avancer, bool charge);
void ajustement_angle(void);
void object_push(void);
void deplacement(uint8_t indice);
void deplacement_couleur(void);
void object_push_couleur(void);
void marche_avant(int16_t speed);
void marche_arriere(uint16_t speed);

#endif /* DEPLACEMENT_H */
