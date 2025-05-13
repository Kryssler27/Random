/*
 * Schalter.c
 *
 *  Created on: Jul 26, 2024
 *      Author: stefan
 */
#include "main.h"

void startup_check_fire_button(){
	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==0)){

		while(1){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
			HAL_Delay(1000);

		}
	}
}


