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
    timer12_start();
    //inits the motors
    motors_init();
    //init the long range sensor
	VL53L0X_start();


    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
    //	VL53L0X_Dev_t mesure;
    	//VL53L0X_startMeasure(mesure,VL53L0X_DEVICEMODE_SINGLE_RANGING);
    	chprintf((BaseSequentialStream *)&SD3, "L'objet est � une distance %d", VL53L0X_get_dist_mm());


#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();
	#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
  //      SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
    	chprintf((BaseSequentialStream *)&SDU1, "timeee  ");
    	chprintf((BaseSequentialStream *)&SD3, "\n\nyoooo\n\n");



	#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
/*#else

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){

            //doFFT_optimized(FFT_SIZE, bufferCmplxInput);

        	static complex_float complex_buffer[FFT_SIZE];
        	for (uint16_t i=0; i<2*size; i+=2){
        		complex_buffer[i/2].real = bufferCmplxInput[i];
        		complex_buffer[i/2].imag = bufferCmplxInput[i+1];
        	}

        	doFFT_c(FFT_SIZE, complex_buffer);

        	for (uint16_t i=0; i<2*size; i+=2){
        		bufferCmplxInput[i] = complex_buffer[i/2].real;
        		bufferCmplxInput[i+1] = complex_buffer[i/2].imag;
        	}

        	volatile uint16_t time = 0;
        	chSysLock();
        	//reset the timer counter
        	GPTD12.tim->CNT = 0;

        	arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

        	time = GPTD12.tim->CNT;
        	chSysUnlock();
        	chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", time);
   //     	SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);

        	chprintf((BaseSequentialStream *)&SD3, "\n\nyoooo\n\n");
        }*/
#endif  /* SEND_FROM_MIC */
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
