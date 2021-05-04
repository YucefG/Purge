#include "ch.h"
#include "hal.h"
#include "camera/dcmi_camera.h"

#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <analyse_couleur.h>

#define IMAGE_BUFFER_SIZE		640

static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*

 */
uint32_t moyenne_ligne(uint8_t *buffer){

	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	//	chprintf((BaseSequentialStream *)&SD3, " %u-eme valeur : %u ",i,buffer[i]);
	}
//	chThdSleepMilliseconds(5000);

	//comparaison avec moyenne de bleu, vert et rouge
	return mean /= IMAGE_BUFFER_SIZE;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0};	//tab pour la couleur rouge
	uint8_t image_b[IMAGE_BUFFER_SIZE] = {0};	//tab pour la couleur bleue


	bool send_to_computer = true;
	uint16_t moyenne_r = 0;
	uint16_t moyenne_b = 0;


    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels

/*		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image_r[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}
		chprintf((BaseSequentialStream *)&SD3, "apres rouge\n");
		chThdSleepMilliseconds(5000);*/

		for(uint16_t i=0; i<2*IMAGE_BUFFER_SIZE; i+=2)
		{
			image_b[i/2] = (uint8_t)img_buff_ptr[i+1]&0b00011111;		//en commentaire car epuck passe en panic mode si on decommente
			image_r[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;

		}

		moyenne_r = moyenne_ligne(image_r);
		moyenne_b = moyenne_ligne(image_b);


	//	chprintf((BaseSequentialStream *)&SD3, "|| Moyenne ROUGE: %u, moyenne BLEUE: %u ",moyenne_r,moyenne_b);
	//	chThdSleepMilliseconds(1000);


		//converts the width into a distance between the robot and the camera

/*			if(send_to_computer){			inutile si on communique pas par un script
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer; */
    }
}

bool analyse_couleur_image(uint16_t moyenne_bleu, uint16_t moyenne_rouge){

	// dans cette fonction le robot est a l'arret --> moteur a 0
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	//si objet couleur bleu ou autre --> retour a la base
	// les moyennes sont bien static?
	if(moyenne_bleu > moyenne_rouge)
		return false;
	else
		//si objet rouge -->continue tout droit jusqu'en dehors de l'arene
		return true;
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){ // a initialiser ou on va
	chprintf((BaseSequentialStream *)&SD3, " juste avant la thread de capture\n ");
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chprintf((BaseSequentialStream *)&SD3, " juste avant la thread de process\n ");
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void affichage(uint8_t* buffer){
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			chprintf((BaseSequentialStream *)&SD3, " %u-eme valeur : %u ",i,buffer[i]);
			chThdSleepMilliseconds(1000);
		}
}
