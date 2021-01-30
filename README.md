# nucleo-engine-controller

(Eventually) source code for the Nucleo STM32F303RE-based rocket engine controller designed by the UCSB RPL. Written in C using the [STM32Cube](https://www.st.com/content/st_com/en/stm32cube-ecosystem.html) HAL and built/flashed in the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).

## Directory Structure

- `.vscode/*`: VS Code workspace configuration files
- `Adarsh_old/*`: Old Nucleo STM32F303RE test project made by Adarsh
- `STM32F103C8-BluePill-Testing/*`: STM32F103C8 project testing the use of STM32Cube HAL GPIO, ADC read, and serial UART communication. Also prototypes the sensor/actuator control driver API sitting between the [QM](www.state-machine.com) control loop and STM32Cube HAL layers.
- `README.md`: This file. :)

## Sensor/Actuator Driver API Overview

The driver API currently consists of the following types/functions/etc. with the following signatures:

- `struct ADCSensorTypeDef`: Base type for every ADC-dependent sensor. Should be instantiated once for each unique sensor.
  - `const uint8_t adc_index`: Corresponding ADC input index. (*mapping defined in the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) software*)
  - `const ADC_HandleTypeDef* adc`: Corresponding ADC handle.
  - `const float counts_per_volt`: Conversion factor for optional human-readable output.
  - `volatile uint8_t value_ready_for_read`: State member indicating when the corresponding ADC has completed its measurement. (*`1`=ready, `0`=incomplete*)
- `struct ActuatorTypeDef`: Base type for every simple two-state actuator. Should be instantiated once for each unique actuator.
  - `const GPIO_TypeDef* port`: Corresponding GPIO port handle.
  - `const uint16_t pin_num`: Corresponding GPIO pin number. (*intended to be assigned using the HAL's `GPIO_PIN_XX` macros in the file `stm32f1xx_hal_gpio.h`*)
  - `const uint8_t normally_open`: Constant determining whether or not the actuator is configured to be normally-open or normally-closed (*`1`=normally-open, `0`=normally-closed*)
  - `uint8_t current_state`: State member indicating whether or not the actuator was last driven open or closed. Should be initialized with identical value to `normally_open`. (*`1`=open, `0`=closed*)
- `ADCSensorTypeDef* adc_sensors[ADC_SENSORS_SIZE]`: List containing pointers to all defined ADC-dependent sensors. Used as a map between sensors and corresponding ADCs.
- `void ADCSensor_Trigger_Read()`: Function called to trigger an immediate (*async DMA*) ADC measurement.
- `uint32_t ADCSensor_Read_Counts(ADCSensorTypeDef* adc_sensor)`: Function called to get the raw ADC  measurement value in counts. (***NOTE:** can be converted to a human-readable measurement in volts by dividing by `adc_sensor.counts_per_volt`*)
- `void Actuator_Open(ActuatorTypeDef* actuator)`: Function called to open an actuator.
- `void Actuator_Close(ActuatorTypeDef* actuator)`: Function called to close an actuator.
