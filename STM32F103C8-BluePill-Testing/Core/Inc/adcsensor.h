/*
 * adcsensor.h
 *
 *  Created on: Feb 13, 2021
 *      Author: neilb
 */

#ifndef INC_ADCSENSOR_H_
#define INC_ADCSENSOR_H_
#include "stm32f1xx_hal.h"

#define ADC1_VAL_SIZE 10
#define ADC_SENSORS_SIZE 1 // TODO: Change later once proper sensor requirements are in place.

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// Structure to define an ADC-dependent sensor.
typedef struct _ADCSensor {
	const uint8_t adc_index; // Defined by the ADC input index in STM32CubeMX.
	const ADC_HandleTypeDef* adc;
	const float counts_per_volt; // TODO: Should this be a smaller type?
	volatile uint8_t value_ready_for_read; // 1=ready, 0=incomplete
} ADCSensorTypeDef;

// DMA buffer containing the output of adc1.
volatile uint16_t adc1_val[ADC1_VAL_SIZE];
// List of all defined ADC-dependent sensors.
extern ADCSensorTypeDef* adc_sensors[ADC_SENSORS_SIZE];

void ADCSensor_Trigger_Read();
uint16_t ADCSensor_Read_Counts(ADCSensorTypeDef* adc_sensor);

#endif /* INC_ADCSENSOR_H_ */
