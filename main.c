#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <selector.h>


#include <audio_processing.h
#include <arm_math.h>
#include <mesure.h>
#include <lumiere.h>
#include <analyse_couleur.h>

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

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //inits the motors
    motors_init();
    //start the long range sensor
	VL53L0X_start();

	//start camera
    dcmi_start();
    po8030_start();

    /* Infinite loop. */
    while (1) {
    	//selecteur a 0 : repos
    	if(get_selector()==0){
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		lumiere_demarrage();
    	}
    	//selecteur � 1: actif si frequence d�tect�e
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
    	//selecteur � toute autre valeur: actif au d�marrage
    	else {
    	   	 //eteindre toutes les LEDs
    		lumiere_eteinte();
    		tour_mesures();
    	    object_detec_proche();
    	    object_push();
    	}
    }
 }

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
