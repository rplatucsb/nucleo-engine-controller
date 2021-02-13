/*
 * adcsensor.c
 *
 *  Created on: Feb 13, 2021
 *      Author: neilb
 */
#include "adcsensor.h"

void ADCSensor_Trigger_Read() {
	// TODO: Add more ADC triggers when more ADCs are implemented.
	HAL_ADC_Start_DMA(&hadc1, adc1_val, ADC1_VAL_SIZE);
}

uint16_t ADCSensor_Read_Counts(ADCSensorTypeDef* adc_sensor) {
	// TODO: Should there be a way to return an error code if there isn't a new value ready?

	// Reset ready flag for next read operation.
	adc_sensor->value_ready_for_read = 0;
	// Return the correct value from the ADC buffer.
	return adc1_val[adc_sensor->adc_index];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// For each defined sensor, check to see if it uses the same ADC which
	// triggered the callback. If so, update its struct with the new information.
	for(int i = 0; i < ADC_SENSORS_SIZE; i++) {
		if(adc_sensors[i]->adc->Instance == hadc->Instance) {
			adc_sensors[i]->value_ready_for_read = 1;
			// TODO: Implement call to control loop indicating new sensor information is available.
		}
	}
}
