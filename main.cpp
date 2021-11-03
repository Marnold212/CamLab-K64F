/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include "fsl_device_registers.h"
// #include "fsl_debug_console.h"
// #include "fsl_common.h"
// #include "fsl_adc16.h"
// #include "fsl_dmamux.h"
// #include "fsl_edma.h"
// #include "pin_mux.h"
// #include "clock_config.h"
// #include "board.h"

#include "mbed.h"
// #include "mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\drivers\fsl_edma.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_DMA    DMA0
#define EXAMPLE_DMAMUX DMAMUX0

#define BUFFER_LENGTH       8
#define TCD_QUEUE_SIZE      2U
#define DEMO_EDMA_CHANNEL_0 0


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Initialize the DMAMUX.
 */
static void DMAMUX_Configuration(void);

void edma_modulo_wrap(void);
static void EDMA_InstallTCD(DMA_Type *base, uint32_t channel, edma_tcd_t *tcd);


/*******************************************************************************
 * Variables
 ******************************************************************************/
edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;
/* must align with the modulo range */
uint32_t srcAddr[BUFFER_LENGTH] __attribute__((aligned(16))) = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U};
uint32_t destAddr[BUFFER_LENGTH] __attribute__((aligned(16))) = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U};
/* Allocate TCD memory poll */
edma_tcd_t tcdMemoryPoolPtr[TCD_QUEUE_SIZE + 1] __attribute__((aligned(sizeof(edma_tcd_t))));

/*******************************************************************************
 * Code
 ******************************************************************************/

/* User callback function for EDMA transfer. */
void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief Main function
 */
int main(void)
{

    DMAMUX_Configuration(); /* Initialize DMAMUX. */
    edma_modulo_wrap();

    
    while (1)
    {
       

        // printf("The contents of DMA0_CR = %lu\n", DMA0->CR);
        // printf("The contents of DMA0_ERQ = %lu\n", DMA0->ERQ);
        // printf("The contents of DMA0_EEI = %lu\n", DMA0->EEI);
        // printf("The contents of DMA0_SERQ = %u\n", DMA0->SERQ);
        // printf("The contents of DMA0_INT = %lu\n", DMA0->INT);
        // printf("The contents of DMA0_HRS = %lu\n", DMA0->HRS);
        // printf("The contents of DMA0_TCD0_SADDR = %lu\n", DMA0->TCD[0].SADDR);
        // printf("The contents of DMA0_TCD0_SOFF = %lu\n", DMA0->TCD[0].SOFF);
        // printf("The contents of DMA0_TCD0_ATTR = %lu\n", DMA0->TCD[0].ATTR);
        // printf("The contents of DMA0_TCD0_DADDR = %lu\n", DMA0->TCD[0].DADDR);
        // printf("The contents of DMA0_TCD0_CSR = %lu\n", DMA0->TCD[0].CSR);
        // printf("The contents of DMA0_TCD0_NBYTES_MLNO = %lu\n", DMA0->TCD[0].NBYTES_MLNO);
        // printf("The contents of DMA0_TCD0_NBYTES_MLOFFNO = %lu\n", DMA0->TCD[0].NBYTES_MLOFFNO);
        // printf("The contents of DMA0_TCD0_NBYTES_MLOFFYES = %lu\n", DMA0->TCD[0].NBYTES_MLOFFYES);
        // printf("The contents of DMA0_TCD0_SLAST = %lu\n", DMA0->TCD[0].SLAST);
        // printf("The contents of DMA0_TCD0_DOFF = %lu\n", DMA0->TCD[0].DOFF);
        // printf("The contents of DMA0_TCD0_CITER_ELINKNO = %lu\n", DMA0->TCD[0].CITER_ELINKNO);
        // printf("The contents of DMA0_TCD0_CITER_ELINKYES = %lu\n", DMA0->TCD[0].CITER_ELINKYES);
        // printf("The contents of DMA0_TCD0_DLAST_SGA = %lu\n", DMA0->TCD[0].DLAST_SGA);
        // printf("The contents of DMA0_TCD0_BITER_ELINKNO = %lu\n", DMA0->TCD[0].BITER_ELINKNO);
        // printf("The contents of DMA0_TCD0_BITER_ELINKYES = %lu\n", DMA0->TCD[0].BITER_ELINKYES);
    }
}

static void DMAMUX_Configuration(void)
{
    /* Configure DMAMUX */
    DMAMUX_Init(EXAMPLE_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, DEMO_EDMA_CHANNEL_0, kDmaRequestMux0AlwaysOn63); /* Map ADC source to channel 0 */
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, DEMO_EDMA_CHANNEL_0);
}

void edma_modulo_wrap(void)
{
    uint32_t i = 0;
    edma_transfer_config_t transferConfig;
    edma_config_t userConfig;

    printf("\r\nedma modulo wrap start\r\n");
    for (i = 0; i < BUFFER_LENGTH; i++)
    {
        printf("%lu\t", destAddr[i]);
    }
    /* Configure EDMA one shot transfer */
    /*
     * userConfig.enableRoundRobinArbitration = false;
     * userConfig.enableHaltOnError = true;
     * userConfig.enableContinuousLinkMode = false;
     * userConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&userConfig);
    EDMA_Init(EXAMPLE_DMA, &userConfig);
    EDMA_CreateHandle(&g_EDMA_Handle, EXAMPLE_DMA, DEMO_EDMA_CHANNEL_0);
    EDMA_SetCallback(&g_EDMA_Handle, EDMA_Callback, NULL);
    EDMA_ResetChannel(g_EDMA_Handle.base, g_EDMA_Handle.channel);
    /* Configure and submit transfer structure 1 */
    EDMA_PrepareTransfer(&transferConfig, srcAddr, sizeof(srcAddr[0]), destAddr, sizeof(destAddr[0]),
                         sizeof(srcAddr[0]),                 /* minor loop bytes : 4 */
                         sizeof(srcAddr[0]) * BUFFER_LENGTH, /* major loop counts : 8 */
                         kEDMA_MemoryToMemory);

    EDMA_TcdSetTransferConfig(tcdMemoryPoolPtr, &transferConfig, NULL);
    EDMA_TcdSetModulo(tcdMemoryPoolPtr, kEDMA_Modulo16bytes, kEDMA_ModuloDisable);
    EDMA_TcdEnableInterrupts(tcdMemoryPoolPtr, kEDMA_MajorInterruptEnable);
    EDMA_TcdEnableAutoStopRequest(tcdMemoryPoolPtr, true);
    EDMA_InstallTCD(EXAMPLE_DMA, DEMO_EDMA_CHANNEL_0, tcdMemoryPoolPtr);

    EDMA_EnableChannelRequest(EXAMPLE_DMA, DEMO_EDMA_CHANNEL_0);
    /* Wait for EDMA transfer finish */
    while (g_Transfer_Done != true)
    {
    }
    /* Print destination buffer */
    printf("\r\nEDMA modulo wrap finished.\r\n");
    for (i = 0; i < BUFFER_LENGTH; i++)
    {
        printf("%lu\t", destAddr[i]);
    }

    EDMA_Deinit(EXAMPLE_DMA);
}

void EDMA_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
        g_Transfer_Done = true;
    }
}





// Should be defined in fsl_edma.c file but not working for soem reason 
static void EDMA_InstallTCD(DMA_Type *base, uint32_t channel, edma_tcd_t *tcd)
{
    assert(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
    assert(tcd != NULL);
    assert(((uint32_t)tcd & 0x1FU) == 0);

    /* Push tcd into hardware TCD register */
    base->TCD[channel].SADDR = tcd->SADDR;
    base->TCD[channel].SOFF = tcd->SOFF;
    base->TCD[channel].ATTR = tcd->ATTR;
    base->TCD[channel].NBYTES_MLNO = tcd->NBYTES;
    base->TCD[channel].SLAST = tcd->SLAST;
    base->TCD[channel].DADDR = tcd->DADDR;
    base->TCD[channel].DOFF = tcd->DOFF;
    base->TCD[channel].CITER_ELINKNO = tcd->CITER;
    base->TCD[channel].DLAST_SGA = tcd->DLAST_SGA;
    /* Clear DONE bit first, otherwise ESG cannot be set */
    base->TCD[channel].CSR = 0;
    base->TCD[channel].CSR = tcd->CSR;
    base->TCD[channel].BITER_ELINKNO = tcd->BITER;
}