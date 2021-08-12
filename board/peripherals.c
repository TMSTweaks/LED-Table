/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v1.0
* BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/**
 * @file    peripherals.c
 * @brief   Peripherals initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include "peripherals.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "board.h"




/**
 * @brief Set up and initialize all required blocks and functions related to the peripherals hardware.
 */

void BOARD_InitGPIO(void) {
	//GPIO Config for Pin A1
	gpio_pin_config_t pinA1Config = {
			kGPIO_DigitalOutput,
			0
	};

	//GPIO Config for Pin A2
	PORT_SetPinInterruptConfig(PORTA, 2,
	      kPORT_InterruptLogicZero);
	NVIC_EnableIRQ(PORTA_IRQn);
	gpio_pin_config_t pinA2Config = {
			kGPIO_DigitalInput,
			0
	};


	gpio_pin_config_t pinBConfig = {
			kGPIO_DigitalOutput,
			1
	};

	gpio_pin_config_t pinDConfig = {
				kGPIO_DigitalOutput,
				1
	};

	GPIO_PinInit(GPIOA, 1, &pinA1Config);
	GPIO_PinInit(GPIOA, 2, &pinA2Config);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, 19U, &pinBConfig);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, 18U, &pinBConfig);
	GPIO_PinInit(GPIOD, 1, &pinDConfig);


}


void BOARD_InitBootPeripherals(void) {
	/* The user initialization should be placed here */
	BOARD_InitGPIO();

}

