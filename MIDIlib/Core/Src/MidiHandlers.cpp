/*
 * MidiHandlers.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: Spiros Makris
 *
 */

#include "MidiHandlers.h"
#include "stm32f0xx_hal.h"
int i;
void Handle_Clock() {
	i++;
	if (i / 12 == 1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		i = 0;
	}
}

void Handle_Start() {
	i = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PinState::GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PinState::GPIO_PIN_SET);
}
void Handle_Stop() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PinState::GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PinState::GPIO_PIN_RESET);
}

