#ifndef ANALYSE_COULEUR_H
#define ANALYSE_COULEUR_H

uint32_t moyenne_ligne(uint8_t *buffer);
bool detec_rouge(void);
void process_image_start(void);

void affichage(uint8_t* buffer); // a supprimer


#endif /* ANALYSE_COULEUR_H */
