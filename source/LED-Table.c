/*
 * Copyright 2016-2021 NXP
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

/**
 * @file    LED-Table.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "LED-Table.h"

#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_PllFllSelClk)


#define DMA_CHANNEL_0 0
#define DMA_CHANNEL_1 1
#define DMA_CHANNEL_2 2
#define TPM2_OF 56 //34 - TPM2 Channel 0 / 56 - TPM2 Overflow / 35 - TPM2 Channel 1 //DMA SOURCE
#define TPM2_C0_OF 34
#define TPM2_C1_OF 35

#define NUM_BITS_PIXEL 24 //each pixel has 24 bits for GRB (8:8:8)
#define NUM_LEDS 16


static const uint32_t resetValue = (1<<0);




static uint32_t transmitBuffer[NUM_BITS_PIXEL*NUM_LEDS];

static int r[8];
static int g[8];
static int b[8];

static int r2[8];
static int g2[8];
static int b2[8];




/*
 * @brief   Application entry point.
 */

void BOARD_InitTPM(void) {

	tpm_config_t tpmInfo = {
			  .prescale = kTPM_Prescale_Divide_1,
			  .useGlobalTimeBase = false,
			  .triggerSelect = kTPM_Trigger_Select_0,
			  .enableDoze = false,
			  .enableDebugMode = true,
			  .enableReloadOnTrigger = false,
			  .enableStopOnOverflow = false,
			  .enableStartOnTrigger = false,
	};
	tpm_chnl_pwm_signal_param_t tpmParam[] = {
			{
					.chnlNumber = 0,
					.level = kTPM_NoPwmSignal,
					.dutyCyclePercent = 72U
			},
			{
					.chnlNumber = 1,
					.level = kTPM_NoPwmSignal,
					.dutyCyclePercent = 28U
			}
	};

	CLOCK_SetTpmClock(1U);

	TPM_Init(TPM2, &tpmInfo);
	TPM_SetupPwm(TPM2, tpmParam, 2U, kTPM_EdgeAlignedPwm, 820000U, TPM_SOURCE_CLOCK);
	TPM_EnableInterrupts(TPM2, kTPM_Chnl0InterruptEnable | kTPM_Chnl1InterruptEnable | kTPM_TimeOverflowInterruptEnable);

	TPM2->CONTROLS[0].CnV = 18;
	TPM2->CONTROLS[1].CnV = 36;

	TPM2->MOD = 60;

}

void BOARD_InitDMA(void) {


	DMAMUX_Init(DMAMUX0);
	DMA_Init(DMA0);


	/* Setup DMA Channel 0 */
	dma_transfer_config_t transferConfig0;
	memset(&transferConfig0, 0, sizeof(transferConfig0));
	transferConfig0.srcAddr = (uint32_t)&resetValue;
	transferConfig0.destAddr = (uint32_t)&GPIOC->PSOR;
	transferConfig0.enableSrcIncrement = false;
	transferConfig0.enableDestIncrement = false;
	transferConfig0.srcSize = kDMA_Transfersize32bits;
	transferConfig0.destSize = kDMA_Transfersize32bits;
	transferConfig0.transferSize = sizeof(resetValue);


	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_0, TPM2_C0_OF);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_0, &transferConfig0);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_0, true);

	/* Setup DMA Channel 1 */
	dma_transfer_config_t transferConfig1;
	memset(&transferConfig1, 0, sizeof(transferConfig1));
	transferConfig1.srcAddr = (uint32_t)&resetValue;
	transferConfig1.destAddr = (uint32_t)&GPIOC->PDOR;
	transferConfig1.enableSrcIncrement = true;
	transferConfig1.enableDestIncrement = false;
	transferConfig1.srcSize = kDMA_Transfersize32bits;
	transferConfig1.destSize = kDMA_Transfersize32bits;
	transferConfig1.transferSize = sizeof(resetValue);

	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_1, TPM2_C1_OF);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_1, &transferConfig1);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_1, true);


	/* Setup DMA Channel 2 */
	dma_transfer_config_t transferConfig;
	memset(&transferConfig, 0, sizeof(transferConfig));
	transferConfig.srcAddr = (uint32_t)&resetValue;
	transferConfig.destAddr = (uint32_t)&GPIOC->PCOR;
	transferConfig.enableSrcIncrement = false;
	transferConfig.enableDestIncrement = false;
	transferConfig.srcSize = kDMA_Transfersize32bits;
	transferConfig.destSize = kDMA_Transfersize32bits;
	transferConfig.transferSize = sizeof(resetValue);

	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_2, TPM2_OF);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_2, &transferConfig);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_2, true);

	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_0);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_1);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_2);


}

void Transfer(uint32_t address, size_t numOfBytes) {

	DisableInterrupts();

	uint32_t channelZeroDone, channelOneDone, channelTwoDone;

	BOARD_InitDMA();

	//Clear DMA Channel Done Flag
	DMA_ClearChannelStatusFlags(DMA0, DMA_CHANNEL_0, kDMA_TransactionsDoneFlag);
	DMA_ClearChannelStatusFlags(DMA0, DMA_CHANNEL_1, kDMA_TransactionsDoneFlag);
	DMA_ClearChannelStatusFlags(DMA0, DMA_CHANNEL_2, kDMA_TransactionsDoneFlag);


	//Set the Source Address for Channel 1
	DMA_SetSourceAddress(DMA0, DMA_CHANNEL_1, address);

	//Set the Byte Count for Each Channel
	DMA_SetTransferSize(DMA0, DMA_CHANNEL_0, numOfBytes);
	DMA_SetTransferSize(DMA0, DMA_CHANNEL_1, numOfBytes);
	DMA_SetTransferSize(DMA0, DMA_CHANNEL_2, numOfBytes);

	//Reset TPM Timer
	BOARD_InitTPM();
	TPM_ClearStatusFlags(TPM2, kTPM_Chnl0Flag | kTPM_Chnl1Flag | kTPM_TimeOverflowFlag);


	//Re-Mux DMA Channels (disabled at end of transfer)
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_0);
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_1);
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_2);


	//Enable TPM DMA
	TPM2->CONTROLS[0].CnSC |= 1UL << 0; //Enable DMA requests for Channel 0
	TPM2->CONTROLS[1].CnSC |= 1UL << 0; //Enable DMA requests for Channel 1
	TPM2->SC |= TPM_SC_DMA_MASK;

	StartTimer();

	for(;;) {
		channelZeroDone = DMA_GetChannelStatusFlags(DMA0, DMA_CHANNEL_0);
		channelOneDone = DMA_GetChannelStatusFlags(DMA0, DMA_CHANNEL_1);
		channelTwoDone = DMA_GetChannelStatusFlags(DMA0, DMA_CHANNEL_2);
		if (channelZeroDone == kDMA_TransactionsDoneFlag && channelOneDone == kDMA_TransactionsDoneFlag && channelTwoDone == kDMA_TransactionsDoneFlag) {
			break;
		}
	}

	//Un-Mux DMA Channels
	DMAMUX_DisableChannel(DMAMUX0, DMA_CHANNEL_0);
	DMAMUX_DisableChannel(DMAMUX0, DMA_CHANNEL_1);
	DMAMUX_DisableChannel(DMAMUX0, DMA_CHANNEL_2);

	//DMA_Deinit(DMA0);

	//Disable TPM DMA
	TPM2->CONTROLS[0].CnSC |= 0UL << 0; //Disable DMA requests for Channel 0
	TPM2->CONTROLS[1].CnSC |= 0UL << 0; //Disable DMA requests for Channel 1
	TPM2->SC |= TPM_SC_DMA_MASK;

	StopTimer();

	//Wait at least 50 microseconds
	Wait(10);

	EnableInterrupts();

}

void SetPrimaryColor(int rVAL, int gVAL, int bVAL) {
	//Convert rgb values to binary
	int i;
	for (i = 0; i < 8; i++) {
		if (rVAL > 0) {
			r[i] = rVAL%2;
			rVAL = rVAL/2;
		}
		else {
			r[i] = 0;
		}
	}

	for (i = 0; i < 8; i++) {
		if (gVAL > 0) {
			g[i] = gVAL%2;
			gVAL = gVAL/2;
		}
		else {
			g[i] = 0;
		}
	}

	for (i = 0; i < 8; i++) {
		if (bVAL > 0) {
			b[i] = bVAL%2;
			bVAL = bVAL/2;
		}
		else {
			b[i] = 0;
		}
	}
}

void SetSecondaryColor(int rVAL, int gVAL, int bVAL) {
	//Convert rgb values to binary
	int i;
	for (i = 0; i < 8; i++) {
		if (rVAL > 0) {
			r2[i] = rVAL%2;
			rVAL = rVAL/2;
		}
		else {
			r2[i] = 0;
		}
	}

	for (i = 0; i < 8; i++) {
		if (gVAL > 0) {
			g2[i] = gVAL%2;
			gVAL = gVAL/2;
		}
		else {
			g2[i] = 0;
		}
	}

	for (i = 0; i < 8; i++) {
		if (bVAL > 0) {
			b2[i] = bVAL%2;
			bVAL = bVAL/2;
		}
		else {
			b2[i] = 0;
		}
	}

}

void ClearLEDs(void) {

	int i;
	for (i = 0; i<NUM_BITS_PIXEL*NUM_LEDS; i++) {
		transmitBuffer[i] = (0<<0);
	}

}

void SetLEDs(bool primary) {


	int i;
	//Move the bits into the transmitBuffer
	for (i = 0; i < NUM_LEDS; i++) {
		SetLED(i, primary);
	}

	/*for (i = 0; i < 1000; i++) {
		//Do nothing
	} */


}

void SetLED(int led, bool primary) {

	//DisableInterrupts();

	int n, t;
	if (primary) {
		//Move bits into transmitBuffer
		t = 0;
		for (n = 0 + (led*NUM_BITS_PIXEL); n < 8 + (led*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (g[t] << 0);
			t++;
		}
		t = 0;
		for (n = 8 + (led*NUM_BITS_PIXEL); n < 16 + (led*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (r[t] << 0);
			t++;
		}
		t = 0;
		for (n = 16 + (led*NUM_BITS_PIXEL); n < 24 + (led*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (b[t] << 0);
			t++;
		}
	}
	else {
		//Move bits into transmitBuffer
		t = 0;
		for (n = 0 + (led*NUM_BITS_PIXEL); n < 8 + (led*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (g2[t] << 0);
			t++;
		}
		t = 0;
		for (n = 8 + (led*NUM_BITS_PIXEL); n < 16 + (led*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (r2[t] << 0);
			t++;
		}
		t = 0;
		for (n = 16 + (led*NUM_BITS_PIXEL); n < 24 + (led*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (b2[t] << 0);
			t++;
		}

	}

	//EnableInterrupts();

}


void PushLEDs(void) {

	Transfer((uint32_t)&transmitBuffer[0], sizeof(transmitBuffer));

}

void PORTA_IRQHandler(void) {

	PushLEDs();

	if (PORTD->PCR[2] & (1<<24)) {
		SetLED(9, false);
	}
	else {
		SetLED(9, true);
	}
	if (PORTA->PCR[2] & (1<<24)) {
		SetLED(1, false);
	}
	else {
		SetLED(1, true);
	}
	if (PORTA->PCR[1] & (1<<24)) {
		SetLED(0, false);
	}
	else {
		SetLED(0, true);
	}
	if (PORTA->PCR[12] & (1<<24)) {
		SetLED(3, false);
	}
	else {
		SetLED(3, true);
	}
	if (PORTD->PCR[4] & (1<<24)) {
		SetLED(2, false);
	}
	else {
		SetLED(2, true);
	}
	if (PORTA->PCR[5] & (1<<24)) {
		SetLED(6, false);
	}
	else {
		SetLED(6, true);
	}
	if (PORTD->PCR[3] & (1<<24)) {
		SetLED(10, false);
	}
	else {
		SetLED(10, true);
	}
	if (PORTD->PCR[1] & (1<<24)) {
		SetLED(11, false);
	}
	else {
		SetLED(11, true);
	}
	if (PORTA->PCR[16] & (1<<24)) {
		SetLED(15, false);
	}
	else {
		SetLED(15, true);
	}
	if (PORTA->PCR[17] & (1<<24)) {
		SetLED(14, false);
	}
	else {
		SetLED(14, true);
	}
	if (PORTD->PCR[6] & (1<<24)) {
		SetLED(13, false);
	}
	else {
		SetLED(13, true);
	}
	if (PORTD->PCR[7] & (1<<24)) {
		SetLED(12, false);
	}
	else {
		SetLED(12, true);
	}
	if (PORTD->PCR[5] & (1<<24)) {
		SetLED(4, false);
	}
	else {
		SetLED(4, true);
	}
	if (PORTD->PCR[2] & (1<<24)) {
		SetLED(9, false);
	}
	else {
		SetLED(9, true);
	}
	if (PORTA->PCR[13] & (1<<24)) {
		SetLED(5, false);
	}
	else {
		SetLED(5, true);
	}
	if (PORTD->PCR[0] & (1<<24)) {
		SetLED(8, false);
	}
	else {
		SetLED(8, true);
	}
	if (PORTA->PCR[4] & (1<<24)) {
		SetLED(7, false);
	}
	else {
		SetLED(7, true);
	}


	PORT_ClearPinsInterruptFlags(PORTA, true);
	PORT_ClearPinsInterruptFlags(PORTD, true);
	PORTA->PCR[1] |= (1<<24);
	PORTA->PCR[2] |= (1<<24);
	PORTA->PCR[12] |= (1<<24);
	PORTD->PCR[4] |= (1<<24);
	PORTA->PCR[5] |= (1<<24);
	PORTD->PCR[2] |= (1<<24);
	PORTD->PCR[3] |= (1<<24);
	PORTD->PCR[1] |= (1<<24);
	PORTA->PCR[16] |= (1<<24);
	PORTA->PCR[17] |= (1<<24);
	PORTD->PCR[6] |= (1<<24);
	PORTD->PCR[7] |= (1<<24);
	PORTD->PCR[5] |= (1<<24);
	PORTD->PCR[2] |= (1<<24);
	PORTA->PCR[13] |= (1<<24);
	PORTD->PCR[0] |= (1<<24);
	PORTA->PCR[4] |= (1<<24);

}

void PORTD_IRQHandler(void) {
	/*SetLED(1, 0, 0, 255);
	if (PORTA->PCR[2] & (1<<24)) {
		SetLED(1, 255, 0, 0);
		PushLEDs();
	}

	SetLED(0, 0, 0, 255);
	if (PORTA->PCR[1] & (1<<24)) {
		SetLED(0, 255, 0, 0);
		PushLEDs();
	}

	SetLED(3, 0, 0, 255);
	if (PORTA->PCR[12] & (1<<24)) {
		SetLED(3, 255, 0, 0);
		PushLEDs();
	}

	SetLED(2, 0, 0, 255);
	if (PORTD->PCR[4] & (1<<24)) {
		SetLED(2, 255, 0, 0);
		PushLEDs();
	}

	PORT_ClearPinsInterruptFlags(PORTD, true);
	PORT_ClearPinsInterruptFlags(PORTA, true);
	PORTA->PCR[1] |= (1<<24);
	PORTA->PCR[2] |= (1<<24);
	PORTA->PCR[12] |= (1<<24);
	PORTD->PCR[4] |= (1<<24); */

	PORTA_IRQHandler();
}


void InitLED(void) {
	BOARD_InitTPM();
	BOARD_InitDMA();
	ClearLEDs();
	SetPrimaryColor(0, 0, 255);
	SetSecondaryColor(255, 0, 0);
	SetLEDs(true);
	PushLEDs();
}

void InitSensors(void) {

	gpio_pin_config_t pinConfig = {
				kGPIO_DigitalInput,
				0
		};


		//GPIO Config for Pin A

		PORT_SetPinInterruptConfig(PORTA, 1, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 2, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 12, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 5, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 16, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 17, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 13, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptLogicZero);
		NVIC_EnableIRQ(PORTA_IRQn);

		PORT_SetPinInterruptConfig(PORTD, 4, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 2, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 3, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 1, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 6, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 7, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 5, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 2, kPORT_InterruptLogicZero);
		PORT_SetPinInterruptConfig(PORTD, 0, kPORT_InterruptLogicZero);

		NVIC_EnableIRQ(PORTD_IRQn);

		GPIO_PinInit(GPIOA, 1, &pinConfig);
		GPIO_PinInit(GPIOA, 2, &pinConfig);
		GPIO_PinInit(GPIOA, 12, &pinConfig);
		GPIO_PinInit(GPIOA, 5, &pinConfig);
		GPIO_PinInit(GPIOA, 16, &pinConfig);
		GPIO_PinInit(GPIOA, 17, &pinConfig);
		GPIO_PinInit(GPIOA, 13, &pinConfig);
		GPIO_PinInit(GPIOA, 4, &pinConfig);

		GPIO_PinInit(GPIOD, 4, &pinConfig);
		GPIO_PinInit(GPIOD, 2, &pinConfig);
		GPIO_PinInit(GPIOD, 3, &pinConfig);
		GPIO_PinInit(GPIOD, 1, &pinConfig);
		GPIO_PinInit(GPIOD, 6, &pinConfig);
		GPIO_PinInit(GPIOD, 7, &pinConfig);
		GPIO_PinInit(GPIOD, 5, &pinConfig);
		GPIO_PinInit(GPIOD, 2, &pinConfig);
		GPIO_PinInit(GPIOD, 0, &pinConfig);

}

void StartTimer(void) {
	TPM_StartTimer(TPM2, kTPM_SystemClock);
}

void StopTimer(void) {
	TPM_StopTimer(TPM2);
}

void EnableInterrupts(void) {
	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
}

void DisableInterrupts(void) {
	NVIC_DisableIRQ(PORTA_IRQn);
	NVIC_DisableIRQ(PORTD_IRQn);
}

void Wait(int i) {
	int v;
	for (v = 0; v < i; v++) {

	}
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    InitLED();
    InitSensors();
    while(1) {
    	DisableInterrupts();
    	Wait(1000);
    	SetLEDs(true);
    	PushLEDs();
    	EnableInterrupts();
    }
}
