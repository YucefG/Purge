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



#include <audio_processing.h>
#include <arm_math.h>
#include <mesure.h>
#include <lumiere.h>
#include <analyse_couleur.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);


    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //inits the motors
    motors_init();
    //start the long range sensor
	VL53L0X_start();
	//start the short range sensor
	proximity_start();			//origine du probleme


	//start camera
    dcmi_start();
	/*chThdSleepMilliseconds(1000);
	chprintf((BaseSequentialStream *)&SD3, "|| Moyennué	bfbizbfzie");*/

    po8030_start();
    //mettre la white balance a zero
    po8030_set_awb(0);
	process_image_start();
	calibrate_ir();

    /* Infinite loop. */
    while (1) {
    	//selecteur a 0 : repos
    	if(get_selector()==0){			//decommenter les lignes apres test sur caméra
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		lumiere_demarrage();
    	}
    	//selecteur à 1: actif si frequence détectée
    	else if(get_selector()==1){
    		 //eteindre toutes les LEDs
    		mic_start(&processAudioData);
    		if(get_demarrage==0){
    			left_motor_set_speed(0);
    		    right_motor_set_speed(0);
        		lumiere_demarrage();
    		}

    		if(get_demarrage()==1){
    			//eteindre toutes les LEDs
    			lumiere_eteinte();

    			tour_mesures();
    			object_detec_proche();
    			object_push();
    		}
    	}
    	//selecteur à toute autre valeur: actif au démarrage
    	else {
    	   	 //eteindre toutes les LEDs
    		lumiere_eteinte();
    		tour_mesures();
    	    object_detec_proche();
    	    object_push();
    	}

   	//tests sur sensors de courte distance
    //	chprintf((BaseSequentialStream *)&SD3, "%u,  ",get_calibrated_prox(0));
    //	chThdSleepMilliseconds(100);

    }
 }

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
