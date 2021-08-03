#include "fsl_tpm.h"
#include "fsl_dma.h"
#include "fsl_dmamux.h"

#ifndef _LEDABLE_H_
#define _LEDTABLE_H_

#define NUM_LED 1
#define FIRST_LED 0
#define LAST_LED (NUM_LED-1)

typedef uint32_t LEDBitsT;


void InitLED(void);
void BOARD_InitTPM(void);
void BOARD_InitDMA(void);

#endif


