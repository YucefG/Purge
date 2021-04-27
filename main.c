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


#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <mesure.h>
//#include <mesure.h>

//uncomment to send the FFTs results from the real microphones
//#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
//#define DOUBLE_BUFFERING

//uncomment to test the long distance sensors
#define TEST_TOF


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

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
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

    //starts timer 12
    //timer12_start(); //je pense pas necessaire

    //inits the motors
    motors_init();
    //start the long range sensor
	VL53L0X_start();

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    //static float send_tab[FFT_SIZE];

    /* Infinite loop. */
    while (1) {


    	mic_start(&processAudioData);

    	bool demarrage = get_demarrage();

    	if(get_demarrage==0){
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    	}

    	//Si get_demarrage vaut 0 : jeu de lumiere
    	lumiere_demarrage()

    	if(get_demarrage()==1){
    		//eteindre toutes les LEDs
    		lumiere_eteinte();

    	tour_mesures();
    	object_detec();
    	object_push();

    		//mic_start(&processAudioData);	//vraiment pas necessaire
        	//chThdSleepMilliseconds(3000);   //pause avant de relancer le programme
        	//mic_start(&processAudioData);	//vraiment pas necessaire
    	}
 }

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
