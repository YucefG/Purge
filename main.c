#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <msgbus/messagebus.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <selector.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <mesure.h>
#include <lumiere.h>
#include <analyse_couleur.h>
#include <deplacement.h>
#include <fonctions_maths.h>


// On initialise ici le bus afin de pouvoir utiliser les capteurs de proximite "proximity"
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//Fonction qui envoie a l'odinateur ce que le robot voit a travers la camera (a supprimer)
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}


// Fonction qui nous a permis d'afficher sur REALterm  les chprintf du robot (a supprimer)
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
	//je sais pas ce que c'est les trois
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);


    //starts the serial communication (a supprimer)
    serial_start();
    //starts the USB communication (a supprimer)
    usb_start();

    //Initialsation des moteurs pas a pas du robot
    motors_init();
    //Start le capteur longue distance
	VL53L0X_start();
	//Start des capteurs de proximite
	proximity_start();
	//Start la camera du robot
	dcmi_start();
    po8030_start();
    //Mise du white balance a zero pour mieux distinguer les couleurs RGB
    po8030_set_awb(0);
    // Start de l'analyse d'image
	process_image_start();
	// Calibration du capteur de courte proximite
	calibrate_ir();
	//Start les melodies utilisees par le robot
	dac_start();
	playMelodyStart();


    /* Boucle infinie */
    while (1) {

    	//Selecteur a 0 : mode static + jeu de LEDs
    	if(get_selector()==0){
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		lumiere_demarrage();
    	}

    	//Selecteur a 1: premier mode lance || Selecteur 2: sort tous les objets
    	// Le robot au son  "purge" se met a analyser les objets dans son arene et ne sortira uniquement les rouges
    	if((get_selector()==1)||(get_selector()==2)){
    		mic_start(&processAudioData);
    		if(get_demarrage()==0){
    			left_motor_set_speed(0);
    		    right_motor_set_speed(0);
        		lumiere_demarrage();
    		}

    	//	if(get_demarrage()==1){
    			//Eteindre toutes les LEDs

    			lumiere_eteinte();

    			// Dans ces trois fonctions, ecrites dans mesure.c, le robot detecte les objets, s'approche des objets
    			// et pousse les objets
    			tour_mesures();
    			object_detec_proche();
           //     show_mesures();
    			object_push();
    	//	}
    	}
        if(get_selector()==3)
        {
            lumiere_eteinte();
            tour_mesures();
            object_detec_proche();
            object_push();
            get_out_arena();
            object_collect();
            turn_90(200);
            signal_fin();
        }
    	if(get_selector()==15){
                lumiere_eteinte();
                chprintf((BaseSequentialStream *)&SD3, " CONNECTE");

              /*  chprintf((BaseSequentialStream *)&SD3, " float1  %f ", MmToCm(RAYON_ARENE+MARGE_PERIPH));
                chprintf((BaseSequentialStream *)&SD3, " %u ", RAYON_ARENE);

                chprintf((BaseSequentialStream *)&SD3, " %u ", RAYON_ARENE);

                chprintf((BaseSequentialStream *)&SD3, " float : %f ", MmToCm(RAYON_ARENE)); */

             //   ligne_droite_pi(MmToCm(RAYON_ARENE+MARGE_PERIPH),true,true); //il va charger vers l'exterieur

               // tour_mesures();
               // object_detec_proche();
           //     show_mesures();
           //     get_out_arena();
            //    turn_90(-200);
            //    ligne_droite_pi(MmToCm(RAYON_ARENE),true,true); //se place dans le cercle a cote et refait la meme chose

    	}
    	if(get_selector()==14){
    	    //tester le pi en ligne droite avec consigne prédéfinie.
    		get_out_arena();

        }

    	else {
    		// Mode static sans jeu de lumiere 
    /*        static float calcul1=2*PI/NB_MESURES;
            static float calcul2=(float)RAYON_ROUE_INT;
            float calcul3=calcul1*calcul2;

                    chprintf((BaseSequentialStream *)&SD3, "calcul 1 : %f |", calcul1);
                    chprintf((BaseSequentialStream *)&SD3, "calcul 2 : %f |", calcul2);
                    chprintf((BaseSequentialStream *)&SD3, "calcul 2 : %f |", calcul3);*/
          //  next_arc(9);


    		lumiere_eteinte();
			left_motor_set_speed(0);
		    right_motor_set_speed(0);
   //         chThdSleepMilliseconds(500);
    	}
    }
 }

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
