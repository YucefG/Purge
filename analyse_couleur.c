#include "ch.h"
#include "hal.h"
#include "camera/dcmi_camera.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include <camera/po8030.h>
#include <analyse_couleur.h>

#define IMAGE_BUFFER_SIZE		640
#define SEUIL					120


//static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t moyenne_r = 0;
static uint16_t moyenne_b = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

// Fonction qui fait la moyenne de la couleur obtenue
uint32_t moyenne_ligne(uint8_t *buffer){

	uint32_t mean = 0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
		}

	return mean /= IMAGE_BUFFER_SIZE;
}

// Thread deja code pour la capture d'une image
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

// Thread pour enreistrer les valeurs du rouge et bleu
static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0};	//tableau pour la couleur rouge
	uint8_t image_b[IMAGE_BUFFER_SIZE] = {0};	//tableau pour la couleur bleue

	bool send_to_computer = true; // a supprimer

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i=0; i<2*IMAGE_BUFFER_SIZE; i+=2)
		{
			image_b[i/2] = (uint8_t)img_buff_ptr[i+1]&0b00011111; //bleu
			image_r[i/2] = (uint8_t)img_buff_ptr[i]&0xF8; //rouge

		}

		moyenne_r = moyenne_ligne(image_r);
		moyenne_b = moyenne_ligne(image_b);


	//converts the width into a distance between the robot and the camera

	/*		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;*/
    }
}


//Fonction qui detecte le rouge
bool detec_rouge(void){
	//Si objet est de couleur bleu ou autre --> retour a la base

	if(moyenne_r < SEUIL )		//la valeur du seuil qui distingue le rouge des autres couleurd a ete choisie de maniere empirique
		return false;
	else
	//Si objet rouge -->continue tout droit jusqu'en dehors de l'arene
		return true;
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void affichage(uint8_t* buffer){ // a supprimer
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			chprintf((BaseSequentialStream *)&SD3, " %u-eme valeur : %u ",i,buffer[i]);
			chThdSleepMilliseconds(1000);
		}
}

/*uint16_t get_moyenne_b(void){
	return moyenne_b;
}

uint16_t get_moyenne_r(void){
	return moyenne_r;
}*/
