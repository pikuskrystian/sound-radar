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
 * @file    sound_locator.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "lcd.h"
#include "locator_bitmap.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "fsl_powerquad.h"
#include "fsl_power.h"
#include <complex.h>

#define N 256
#define N2 (2*N)
#define NFFT (2*N2)
#define V 340
#define FP 48000.0
#define L 0.1
#define M 1

/* TODO: insert other definitions and declarations here. */
float x1[N2]={0} , x2[N2]={0};
uint16_t dataBuffer1[N]={0} , dataBuffer2[N]={0};
uint32_t wr=0;
bool dataReady = false;
lpadc_conv_result_t g_LpadcResultConfigStruct;

q31_t y[NFFT]={0};
float g1[NFFT]={0} , g2[NFFT]={0} , g12[NFFT]={0};
int result;
uint16_t counter = N2;
float complex z;

typedef struct element{
	uint32_t index;
	float value;
}element;

element sortTab[NFFT] = {0};

/* ADC0_IRQn interrupt handler */
void ADC0_IRQHANDLER(void)
{
	LPADC_GetConvResult(ADC0, &g_LpadcResultConfigStruct, 0U);
	if(!dataReady) {
		dataBuffer1[wr]=g_LpadcResultConfigStruct.convValue;
	}

	LPADC_GetConvResult(ADC0, &g_LpadcResultConfigStruct, 0U);
	if(!dataReady) {
		dataBuffer2[wr++]=g_LpadcResultConfigStruct.convValue;
	}

	if(wr>=N){
		wr=0;
		dataReady=true;
	}
}

void gcc_phat(float* x1, float* x2)
{
	arm_float_to_q31(x1, y, N2);
	arm_scale_q31 (y, 0x03FFFFFF, 0, y, NFFT); // max: 27 bit
	PQ_TransformCFFT(POWERQUAD, N2, y, y);
	PQ_WaitDone(POWERQUAD);
	arm_scale_q31 (y, 0x7FFFFFFF, 7, y, N2);
	arm_q31_to_float(y, g1, N2);

	arm_float_to_q31(x2, y, N2);
	arm_scale_q31 (y, 0x03FFFFFF, 0, y, NFFT); // max: 27 bit
	PQ_TransformCFFT(POWERQUAD, N2, y, y);
	PQ_WaitDone(POWERQUAD);
	arm_scale_q31 (y, 0x7FFFFFFF, 7, y, N2);
	arm_q31_to_float(y, g2, N2);

	arm_cmplx_conj_f32(g2,g2,N2);
	arm_cmplx_mult_cmplx_f32(g1,g2,g12,N2);
	arm_cmplx_mag_f32(g12,x1,N2);

	for(int i=0;i<N2;i++)
	{
		if(x1[i] < 0.000001)
			x1[i] = 0.000001;

		g12[2*i] /= x1[i];
		g12[2*i+1] /= x1[i];
	}

	arm_cfft_f32(&arm_cfft_sR_f32_len512, g12, 1, 1);
	arm_scale_f32 (g12, 1/256.0, g12, NFFT);

	for(int i=0;i<NFFT;i++)
	{
		g1[i] = g12[counter++];

		if(counter == NFFT)
			counter=0;
	}

	counter=N2;
}

int compare_elements(const void *p, const void *q)
{
	element x = *(const element *)p;
	element y = *(const element *)q;
	return (x.value < y.value) - (x.value > y.value);
}

static inline void draw_point(uint8_t x, uint8_t y, uint8_t radius_locator, uint8_t radius_point , float angle , uint16_t color)
{
	for(int i=0;i<radius_point;i++)
		LCD_Draw_Circle(x+radius_locator*sin(angle),y-radius_locator*cos(angle),radius_point-i,color);
}

/*
 * @brief   Application entry point.
 */
int main(void) {

	/* Disable LDOGPADC power down */
	POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PQ_Init(POWERQUAD);

    pq_config_t pq_cfg;
    pq_cfg.inputAFormat = kPQ_32Bit;
    pq_cfg.inputAPrescale = 0;
    pq_cfg.inputBFormat = kPQ_32Bit;
    pq_cfg.inputBPrescale = 0;
    pq_cfg.tmpFormat = kPQ_32Bit;
    pq_cfg.tmpPrescale = 0;
    pq_cfg.outputFormat = kPQ_32Bit;
    pq_cfg.outputPrescale = 0;
    pq_cfg.tmpBase = (uint32_t *)0xE0000000;
    pq_cfg.machineFormat = kPQ_32Bit;
    PQ_SetConfig(POWERQUAD, &pq_cfg);

    LCD_Init(FLEXCOMM3_PERIPHERAL);

    while(1) {
    	LCD_Set_Bitmap((uint16_t*)locator_bitmap_160x128);

    	if(dataReady)
    	{
    		for(int i=0;i<N;i++)
    		{
    			x1[2*i] = (dataBuffer1[i]/32768.0)-1; // uint16 to float
    			x1[2*i+1] = 0;

    			x2[2*i] = (dataBuffer2[i]/32768.0)-1; // uint16 to float
    			x2[2*i+1] = 0;
    		}

    		dataReady=false;

    		gcc_phat(x1,x2);

    		for(int i=0;i<NFFT;i++)
    		{
    			sortTab[i].index = i;
    			sortTab[i].value = g1[i];
    		}

    		qsort(sortTab, NFFT, sizeof(sortTab[0]), compare_elements);

    		for(int i=0;i<M;i++)
    		{
    			result = N2-sortTab[i].index;
    			z = casin((V*(result/FP))/L);
    			draw_point(80,128,60,6,creal(z), 0xF000);
    		}

    		LCD_GramRefresh();
    	}
    }
    return 0 ;
}
