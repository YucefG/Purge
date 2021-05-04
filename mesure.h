#ifndef MESURE_H
#define MESURE_H

void next_angle(uint16_t speed);

void tour_mesures(void);

void object_push(void);

uint8_t get_mesure_i(uint8_t i);

void set_mesure_i(uint8_t distance_i, uint8_t i);

//bool proximity_distance(void);

void show_mesure(void);


#endif /* MESURE_H */
