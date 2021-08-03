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


dma_handle_t g_DMA_Handle;
volatile bool g_DMA_Transfer_Complete;
static const uint32_t resetValue = (1<<19);
static const uint32_t resetDValue = (1<<1);
static const uint32_t reset18Value = (1<<18);

void DMA_Callback(dma_handle_t *handle, void *param)
{
    g_DMA_Transfer_Complete = true;

}

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
					.dutyCyclePercent = 18U
			},
			{
					.chnlNumber = 1,
					.level = kTPM_NoPwmSignal,
					.dutyCyclePercent = 37U
			}
	};

	CLOCK_SetTpmClock(1U);

	TPM_Init(TPM2, &tpmInfo);
	TPM_SetupPwm(TPM2, tpmParam, 2U, kTPM_EdgeAlignedPwm, 5000U, TPM_SOURCE_CLOCK);
	TPM_EnableInterrupts(TPM2, kTPM_Chnl0InterruptEnable | kTPM_Chnl1InterruptEnable | kTPM_TimeOverflowInterruptEnable);


	TPM2->CONTROLS[0].CnSC |= 1UL << 0; //Enable DMA requests for Channel 0
	TPM2->CONTROLS[1].CnSC |= 1UL << 0; //Enable DMA requests for Channel 1
	TPM2->SC |= TPM_SC_DMA_MASK;


}

void BOARD_InitDMA(void) {

	g_DMA_Transfer_Complete = false;

	DMAMUX_Init(DMAMUX0);
	DMA_Init(DMA0);


	/* Setup DMA Channel 0 */
	dma_transfer_config_t transferConfig0;
	memset(&transferConfig0, 0, sizeof(transferConfig0));
	transferConfig0.srcAddr = (uint32_t)&resetDValue;
	transferConfig0.destAddr = (uint32_t)&GPIOD->PCOR;
	transferConfig0.enableSrcIncrement = false;
	transferConfig0.enableDestIncrement = false;
	transferConfig0.srcSize = kDMA_Transfersize32bits;
	transferConfig0.destSize = kDMA_Transfersize32bits;
	transferConfig0.transferSize = sizeof(resetValue);

	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_0, TPM2_C0_OF);
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_0);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_0, &transferConfig0);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_0);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_0, true);

	/* Setup DMA Channel 1 */
	dma_transfer_config_t transferConfig1;
	memset(&transferConfig1, 0, sizeof(transferConfig1));
	transferConfig1.srcAddr = (uint32_t)&reset18Value;
	transferConfig1.destAddr = (uint32_t)&GPIOB->PCOR;
	transferConfig1.enableSrcIncrement = false;
	transferConfig1.enableDestIncrement = false;
	transferConfig1.srcSize = kDMA_Transfersize32bits;
	transferConfig1.destSize = kDMA_Transfersize32bits;
	transferConfig1.transferSize = sizeof(resetValue);

	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_1, TPM2_C1_OF);
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_1);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_1, &transferConfig1);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_1);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_1, true);


	/* Setup DMA Channel 2 */
	dma_transfer_config_t transferConfig;
	memset(&transferConfig, 0, sizeof(transferConfig));
	transferConfig.srcAddr = (uint32_t)&resetValue;
	transferConfig.destAddr = (uint32_t)&GPIOB->PCOR;
	transferConfig.enableSrcIncrement = false;
	transferConfig.enableDestIncrement = false;
	transferConfig.srcSize = kDMA_Transfersize32bits;
	transferConfig.destSize = kDMA_Transfersize32bits;
	transferConfig.transferSize = sizeof(resetValue);

	DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL_2, TPM2_OF);
	DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL_2);

	DMA_CreateHandle(&g_DMA_Handle, DMA0, DMA_CHANNEL_2);
	DMA_SetCallback(&g_DMA_Handle, DMA_Callback, NULL);

	DMA_SetTransferConfig(DMA0, DMA_CHANNEL_2, &transferConfig);
	DMA_EnableChannelRequest(DMA0, DMA_CHANNEL_2);
	DMA_EnableCycleSteal(DMA0, DMA_CHANNEL_2, true);
	DMA_EnableInterrupts(DMA0, DMA_CHANNEL_2);




}

void InitLED(void) {
	BOARD_InitTPM();
	BOARD_InitDMA();

	TPM_StartTimer(TPM2, kTPM_SystemClock);

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
