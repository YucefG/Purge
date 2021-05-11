#ifndef MESURE_H
#define MESURE_H

bool prox_distance(void);
void ajustement_angle(void);
void next_angle(uint16_t speed);
void tour_mesures(void);
void object_detec_proche(void);
void object_push(void);
void deplacement(void);
void deplacement_couleur(void);

void object_push_couleur(void);
uint8_t get_mesure_i(uint8_t i);
void set_mesure_i(uint8_t distance_i, uint8_t i);
void show_mesure(void);
void marche_avant(void);
void marche_arriere(void);
int16_t pi_regulator(float distance, float goal)

//contournement objet
void initialisation_moteur(void);
void object_push_contournement(void);
void deplacement_contournement(void);
void contournement_objet(void);




#endif /* MESURE_H */
