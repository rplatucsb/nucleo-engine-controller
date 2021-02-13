/*
 * actuator.h
 *
 *  Created on: Feb 13, 2021
 *      Author: neilb
 */

#ifndef INC_ACTUATOR_H_
#define INC_ACTUATOR_H_
#include "stm32f1xx_hal.h"

// Structure to define a simple two-state actuator.
typedef struct _Actuator {
	const GPIO_TypeDef* port;
	const uint16_t pin_num; // Sized to match with HAL GPIO type definitions.
	const uint8_t normally_open; // 1=normally-open, 0=normally-closed
	uint8_t current_state; // 1=open, 0=closed
} ActuatorTypeDef;

void Actuator_Open(ActuatorTypeDef* actuator);
void Actuator_Close(ActuatorTypeDef* actuator);

#endif /* INC_ACTUATOR_H_ */
