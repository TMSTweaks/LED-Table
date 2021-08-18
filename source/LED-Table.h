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
void StartTimer(void);
void StopTimer(void);
void Transfer(uint32_t address, size_t numOfBytes);
void ClearLEDs(void);
void SetLEDs(bool primary);
void SetLED(int led, bool primary);
void PushLEDs(void);
void InitSensors(void);
void EnableInterrupts(void);
void DisableInterrupts(void);
void Wait(int i);
void SetPrimaryColor(int rVAL, int gVAL, int bVAL);
void SetSecondaryColor(int rVAL, int gVAL, int bVAL);

#endif


