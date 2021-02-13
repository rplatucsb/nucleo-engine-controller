/*
 * actuator.c
 *
 *  Created on: Feb 13, 2021
 *      Author: neilb
 */
#include "actuator.h"

void Actuator_Open(ActuatorTypeDef* actuator) {
	// If the actuator is normally open, write logic low. Else write logic high.
	HAL_GPIO_WritePin(actuator->port,
					  actuator->pin_num,
					  actuator->normally_open == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Actuator_Close(ActuatorTypeDef* actuator) {
	// If the actuator is normally open, write logic high. Else write logic low.
	HAL_GPIO_WritePin(actuator->port,
					  actuator->pin_num,
					  actuator->normally_open == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
