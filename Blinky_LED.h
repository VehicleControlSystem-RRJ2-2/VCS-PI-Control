/*
 * Blinky_LED.h
 *
 *  Created on: 2023. 11. 22.
 *      Author: 현대오토22
 */

#ifndef BLINKY_LED_H_
#define BLINKY_LED_H_

#include "IfxPort_PinMap.h"

#define WAIT_TIME   500
#define LED1        &MODULE_P10, 1
#define LED2        &MODULE_P10, 2

void blinkLED();

void initLED();

void blinkLED1();

void blinkLED2();

#endif /* BLINKY_LED_H_ */
