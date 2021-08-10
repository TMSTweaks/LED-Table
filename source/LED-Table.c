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
#define NUM_LEDS 3


static const uint32_t resetValue = (1<<1);




static uint32_t transmitBuffer[NUM_BITS_PIXEL*NUM_LEDS];


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

	//g_DMA_Transfer_Complete = false;

	DMAMUX_Init(DMAMUX0);
	DMA_Init(DMA0);


	/* Setup DMA Channel 0 */
	dma_transfer_config_t transferConfig0;
	memset(&transferConfig0, 0, sizeof(transferConfig0));
	transferConfig0.srcAddr = (uint32_t)&resetValue;
	transferConfig0.destAddr = (uint32_t)&GPIOA->PSOR;
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
	transferConfig1.destAddr = (uint32_t)&GPIOA->PDOR;
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
	transferConfig.destAddr = (uint32_t)&GPIOA->PCOR;
	transferConfig.enableSrcIncrement = false;
	transferConfig.enableDestIncrement = false;
	transferConfig.srcSize = kDMA_Transfersize32bits;
	transferConfig.destSize = kDMA_Transfersize32bits;
	transferConfig.transferSize = sizeof(resetValue);

	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_2, TPM2_OF);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_2, &transferConfig);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_2, true);

}

void Transfer(uint32_t address, size_t numOfBytes) {

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

	//Enable DMA Peripheral Requests
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_0);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_1);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_2);

	//Enable TPM DMA
	TPM2->CONTROLS[0].CnSC |= 1UL << 0; //Enable DMA requests for Channel 0
	TPM2->CONTROLS[1].CnSC |= 1UL << 0; //Enable DMA requests for Channel 1
	TPM2->SC |= TPM_SC_DMA_MASK;

	StartTimer();
	//g_DMA_Transfer_Complete = false;

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

	DMA_Deinit(DMA0);

	//Disable TPM DMA
	TPM2->CONTROLS[0].CnSC |= 0UL << 0; //Disable DMA requests for Channel 0
	TPM2->CONTROLS[1].CnSC |= 0UL << 0; //Disable DMA requests for Channel 1
	TPM2->SC |= TPM_SC_DMA_MASK;

	StopTimer();

	//Wait at least 50 microseconds
	int i;
	for (i = 0; i < 10; i++) {
		//Do nothing
	}
}

void ClearLEDs(void) {
	int i;
	for (i = 0; i<NUM_BITS_PIXEL*NUM_LEDS; i++) {
		transmitBuffer[i] = (0<<1);
	}
	Transfer((uint32_t)&transmitBuffer[0], sizeof(transmitBuffer));
}

void SetLEDs(int rVAL, int gVAL, int bVAL) {

	int i, n, t;

	int r[8];
	int g[8];
	int b[8];

	//Convert rgb values to binary
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

	//Move the bits into the transmitBuffer
	for (i = 0; i < NUM_LEDS; i++) {
		t = 0;
		for (n = 0 + (i*NUM_BITS_PIXEL); n < 8 + (i*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (g[t] << 1);
			t++;
		}
		t = 0;
		for (n = 8 + (i*NUM_BITS_PIXEL); n < 16 + (i*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (r[t] << 1);
			t++;
		}
		t = 0;
		for (n = 16 + (i*NUM_BITS_PIXEL); n < 24 + (i*NUM_BITS_PIXEL); n++) {
			transmitBuffer[n] = (b[t] << 1);
			t++;
		}
	}


	Transfer((uint32_t)&transmitBuffer[0], sizeof(transmitBuffer));


}

void Test(void) {

	int i;

	//Purple First LED
	for (i = 0;i<8;i++) {
		transmitBuffer[i] = (0<<1);
	}
	for (i = 8;i<16;i++) {
		transmitBuffer[i] = (1<<1);
	}
	for (i = 16; i<24; i++) {
		transmitBuffer[i] = (1<<1);
	}


	//Green Second LED
	for (i = 24; i<32; i++) {
		transmitBuffer[i] = (1<<1);
	}
	for (i = 32; i<48; i++) {
		transmitBuffer[i] = (0<<1);
	}

	//Blue Third LED
	for (i = 48; i<64; i++) {
		transmitBuffer[i] = (0<<1);
	}
	for (i = 64; i<72; i++) {
		transmitBuffer[i]  = (1<<1);
	}


	Transfer((uint32_t)&transmitBuffer[0], sizeof(transmitBuffer));
}

void InitLED(void) {
	BOARD_InitTPM();
	BOARD_InitDMA();
	ClearLEDs();
	SetLEDs(255, 0, 146);
}

void StartTimer(void) {
	TPM_StartTimer(TPM2, kTPM_SystemClock);
}

void StopTimer(void) {
	TPM_StopTimer(TPM2);
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
    while(1);
}
