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
#include "mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\drivers\fsl_adc16.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_CHANNEL       12U
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_BASEADDR      ADC0
#define DEMO_DMAMUX_BASEADDR     DMAMUX0
#define DEMO_DMA_CHANNEL         0U
#define DEMO_DMA_ADC_SOURCE      40U
#define DEMO_DMA_BASEADDR        DMA0
#define ADC16_RESULT_REG_ADDR    0x4003b010U
#define DEMO_DMA_IRQ_ID          DMA0_IRQn
#define DEMO_ADC16_SAMPLE_COUNT 16U /* The ADC16 sample count. */

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);
// Application buffer to receive the data
// char buf[MAXIMUM_BUFFER_SIZE] = "00\r\n"; // Not sure which part of buffer this uses - prefer below implementation 
char buf[MAXIMUM_BUFFER_SIZE] = "{0}";
int num = (8*2) + 1;  // 8 * 2 bytes + EOL char   


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Initialize the EDMA.
 */
static void EDMA_Configuration(void);

/*!
 * @brief Initialize the DMAMUX.
 */
static void DMAMUX_Configuration(void);

/*!
 * @brief Initialize the ADC16.
 */
static void ADC16_Configuration(void);

/*!
 * @brief Callback function for EDMA.
 */
static void Edma_Callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_Transfer_Done = false; /* DMA transfer completion flag. */
uint16_t g_adc16SampleDataArray[DEMO_ADC16_SAMPLE_COUNT];
uint32_t g_avgADCValue = 0U; /* Average ADC value .*/
edma_handle_t g_EDMA_Handle; /* Edma handler. */
edma_transfer_config_t g_transferConfig;
const uint32_t g_Adc16_16bitFullRange = 65536U;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    adc16_channel_config_t adcChnConfig;

    /* Initialize board hardware. */
    // BOARD_InitBootPins();
    // BOARD_InitBootClocks();
    // BOARD_InitDebugConsole();

    printf("ADC16 CONTINUOUS EDMA DEMO\r\n");

    EDMA_Configuration();   /* Initialize EDMA. */
    DMAMUX_Configuration(); /* Initialize DMAMUX. */
    ADC16_Configuration();  /* Initialize ADC16. */

    /* Configure channel and SW trigger ADC16. */
    adcChnConfig.channelNumber = DEMO_ADC16_CHANNEL;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.enableDifferentialConversion = false;
#endif
    adcChnConfig.enableInterruptOnConversionCompleted = false;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);

    printf("ADC Full Range: %lu\r\n", g_Adc16_16bitFullRange);
    printf("Press any key to get user channel's ADC value ...\r\n");

    buf[num-1] = '\n';
    while (1)
    {
        // GETCHAR();
        // g_Transfer_Done = false;
        // while (!g_Transfer_Done)
        // {
        // }
        for(int x = 0; x < (num-1) ; x+=2){
            buf[x] = (g_adc16SampleDataArray[x] >> 8) & 0xff;
            buf[x+1] = g_adc16SampleDataArray[x] & 0xff;
        }
        serial_port.write(buf, num);
    }
}

static void DMAMUX_Configuration(void)
{
    /* Configure DMAMUX */
    DMAMUX_Init(DEMO_DMAMUX_BASEADDR);
    DMAMUX_SetSource(DEMO_DMAMUX_BASEADDR, DEMO_DMA_CHANNEL, DEMO_DMA_ADC_SOURCE); /* Map ADC source to channel 0 */
    DMAMUX_EnableChannel(DEMO_DMAMUX_BASEADDR, DEMO_DMA_CHANNEL);
}

static void EDMA_Configuration(void)
{
    edma_config_t userConfig;

    EDMA_GetDefaultConfig(&userConfig);
    EDMA_Init(DEMO_DMA_BASEADDR, &userConfig);
    EDMA_CreateHandle(&g_EDMA_Handle, DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL);
    EDMA_SetCallback(&g_EDMA_Handle, Edma_Callback, NULL);
    EDMA_PrepareTransfer(&g_transferConfig, (void *)ADC16_RESULT_REG_ADDR, sizeof(uint16_t),
                         (void *)g_adc16SampleDataArray, sizeof(uint16_t), sizeof(uint16_t),
                         sizeof(g_adc16SampleDataArray), kEDMA_PeripheralToMemory);
    EDMA_SubmitTransfer(&g_EDMA_Handle, &g_transferConfig);
    /* Enable interrupt when transfer is done. */
    EDMA_EnableChannelInterrupts(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, kEDMA_MajorInterruptEnable);
#if defined(FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT) && FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT
    /* Enable async DMA request. */
    // NOT SURE WHAT THIS DOES 
    EDMA_EnableAsyncRequest(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, true);
#endif /* FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT */
    /* Enable transfer. */
    EDMA_StartTransfer(&g_EDMA_Handle);
}

static void ADC16_Configuration(void)
{
    adc16_config_t adcUserConfig;

    /*
     * Initialization ADC for 16bit resolution, DMA mode, normal convert speed, VREFH/L as reference,
     * enable continuous convert mode.
     */
    ADC16_GetDefaultConfig(&adcUserConfig);
    adcUserConfig.resolution                 = kADC16_Resolution16Bit;
    adcUserConfig.enableContinuousConversion = true;
    adcUserConfig.clockSource                = kADC16_ClockSourceAsynchronousClock;

    adcUserConfig.enableLowPower = true;
#if ((defined BOARD_ADC_USE_ALT_VREF) && BOARD_ADC_USE_ALT_VREF)
    adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(DEMO_ADC16_BASEADDR, &adcUserConfig);

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR))
    {
        printf("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        printf("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif

    /* Enable software trigger.  */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, false);
    /* Enable DMA. */
    ADC16_EnableDMA(DEMO_ADC16_BASEADDR, true);
}

static void Edma_Callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    /* Clear Edma interrupt flag. */
    EDMA_ClearChannelStatusFlags(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, kEDMA_InterruptFlag);
    /* Setup transfer */
    EDMA_PrepareTransfer(&g_transferConfig, (void *)ADC16_RESULT_REG_ADDR, sizeof(uint16_t),
                         (void *)g_adc16SampleDataArray, sizeof(uint16_t), sizeof(uint16_t),
                         sizeof(g_adc16SampleDataArray), kEDMA_PeripheralToMemory);
    EDMA_SetTransferConfig(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, &g_transferConfig, NULL);
    /* Enable transfer. */
    EDMA_StartTransfer(&g_EDMA_Handle);
    g_Transfer_Done = true;
}
