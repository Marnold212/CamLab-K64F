
#include "mbed.h"
#include "mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\drivers\fsl_adc16.h"  // Needed for ADC-Init class? 
#include "mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\TARGET_FRDM\PeripheralPinMaps.h"  // Needed for ADC Read function 

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);

void pit_init();
void PIT_IRQHandler(void);
void Custom_K64F_ADC_Init(ADC_Type *base);
void Custom_K64F_ADC_Trigger_Read_Pin(PinName pin);
void DMA_Init(void);

static bool custom_pit_initied = false;
static void pit0_isr(void);
void Custom_PIT_Init(void);
void ADC_IRQHandler(void);

// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = "000";
uint32_t num = 3;

uint32_t result;

volatile bool val = 0;
volatile bool *ptrval = &val;

volatile bool val2 = 1;
volatile bool *ptrval2 = &val2;

int main(){
    static DigitalOut led(LED1);
    static DigitalOut led2(LED_BLUE);

    Custom_K64F_ADC_Init(ADC0);

    printf("\n\n");
    printf("ADC0_SC1A : %lu \n", *(volatile uint32_t *)(ADC0_BASE));   // ADC SC1A
    printf("ADC0_SC1B : %lu \n", *(volatile uint32_t *)(ADC0_BASE + 0x4)); // ADC SC1B
    printf("ADC0_CFG1 : %lu \n", *(volatile uint32_t *)(ADC0_BASE + 0x8)); // ADC Config Register 1 ADCx_CFG1 
    printf("ADC0_R0 : %lu \n", *(volatile uint32_t *)(ADC0_BASE + 0x10)); // Data Result Register 
    printf("ADC0_CFG2 : %lu \n", *(volatile uint32_t *)(ADC0_BASE + 0x0C)); // ADC Config Register 2 ADCx_CFG2 
    printf("ADC0_SC2 : %lu \n", *(volatile uint32_t *)(ADC0_BASE + 0x20)); // ADC Status Control Register 2 
    printf("ADC0_SC3 : %lu \n", *(volatile uint32_t *)(ADC0_BASE + 0x24)); // ADC Status Control Register 3
    printf("SIM_SOPT7 : %lu \n", *(volatile uint32_t *)(SIM_BASE + 0x1018)); // SIM_SOPT7
    printf("SIM_SCGC6 : %lu \n", *(volatile uint32_t *)(SIM_BASE + 0x103C)); // SIM_SCGC6
    // printf("PDB0_BASE : %lu \n", *(volatile uint32_t *)(PDB0_BASE));   // ADC PDB Bases register 

    // Set desired properties (9600-8-N-1).
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

    buf[2] = '\n';

    // // Enable clocks
	// SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;	// ADC 0 clock
	// SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;	// PTB0 clock
    ADC0->SC2 |= ADC_SC2_DMAEN_MASK;    // DMA Enable

    // pit_init();
    Custom_PIT_Init();
    DMA_Init();


    while(1)
    {
        while(!(*(volatile uint32_t *)(ADC0_BASE) & (1U << 7))){
            
        }
        led = *ptrval;
        // buf[0] = ((*(volatile uint32_t *)(0x4003B000 + 0x10) >> 8) & 0xFF); // Upper byte of 16 bit data 
        buf[0] = ((ADC0->R[0]) >> 8) & 0xFF;  // Lower byte of 16 bit data 
        buf[1] = ADC0->R[0];  // Lower byte of 16 bit data 
        serial_port.write(buf, num);
        led2 = *ptrval2;
        printf("\n The updated contents of results register after DMA = %lu \n", result);
        // ThisThread::sleep_for(1s);
    }
    
}


void ADC_IRQHandler(void)
{
    // NVIC_ClearPendingIRQ(ADC0_IRQn);
    NVIC_DisableIRQ(ADC0_IRQn);
    *ptrval2 = !(*ptrval2);
}

void Custom_K64F_ADC_Init(ADC_Type *base)
{
    // #define MAX_FADC 6000000
    #define MAX_FADC 12000000 // Datasheet specifies range of sampling frequency to be 2-12 MHz for 16-bit mode 

    adc16_config_t adc16_config;

    uint32_t bus_clock = CLOCK_GetFreq(kCLOCK_BusClk);
    uint32_t clkdiv;
    for (clkdiv = 0; clkdiv < 4; clkdiv++) {
        if ((bus_clock >> clkdiv) <= MAX_FADC) {
            break;
        }
    }
    if (clkdiv == 4) {
        clkdiv = 0x3; //Set max div
    }

    ADC16_GetDefaultConfig(&adc16_config);

    // Test these parameters for increasing speed of sampling 
    // adc16_config.longSampleMode = kADC16_LongSampleDisabled;
    // adc16_config.enableHighSpeed = false;

    adc16_config.clockSource = kADC16_ClockSourceAlt0;
    adc16_config.clockDivider = (adc16_clock_divider_t)clkdiv;
    adc16_config.resolution = kADC16_ResolutionSE16Bit;

    // ADC DMA Trigger Configuration 
    // ADC16_EnableDMA(base, true);

    ADC16_Init(base, &adc16_config);

    ADC16_EnableHardwareTrigger(base, false); 
    ADC16_SetHardwareAverage(base, kADC16_HardwareAverageCount4);

    // Check effect of following setting on accuracy + speed  
    // ADC16_SetHardwareAverage(base, kADC16_HardwareAverageDisabled);
    
    NVIC_SetVector(ADC0_IRQn, (uint32_t) ADC_IRQHandler);
    NVIC_EnableIRQ(ADC0_IRQn);
}

/*	Custom ADC Read if relevent ADC enabled
 * 
 * 	Starts conversion in selected ADC on channel if valid - allows custom configuration 
 * 
 * */
void Custom_K64F_ADC_Trigger_Read_Pin(PinName pin)
{
    /* Array of ADC peripheral base address. */
    static ADC_Type *const adc_addrs[] = ADC_BASE_PTRS;

    adc16_channel_config_t adc16_channel_config;

    int peripheral = (int)pinmap_peripheral(pin, PinMap_ADC);
    int function = (int)pinmap_find_function(pin, PinMap_ADC);
    const PinMap pinmap = {pin, peripheral, function};

    // Check the pin specified exists as an ADC input 
    MBED_ASSERT((ADCName)pinmap.peripheral != (ADCName)NC);
    pin_function(pinmap.pin, pinmap.function);
    pin_mode(pinmap.pin, PullNone);

    uint32_t instance = (ADCName)pinmap.peripheral >> ADC_INSTANCE_SHIFT;

    adc16_channel_config.channelNumber = (ADCName)pinmap.peripheral & 0xF;
    adc16_channel_config.enableInterruptOnConversionCompleted = false;

    ADC16_SetChannelMuxMode(adc_addrs[instance],
                            (ADCName)pinmap.peripheral & (1 << ADC_B_CHANNEL_SHIFT) ? kADC16_ChannelMuxB : kADC16_ChannelMuxA);

    /*
     * When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
     * function, which works like writing a conversion command and executing it.
     */
    ADC16_SetChannelConfig(adc_addrs[instance], 0, &adc16_channel_config);
}






static void pit0_isr(void)
{
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
    // PIT_StopTimer(PIT, kPIT_Chnl_0);

    // Write to SC1A to start conversion with channel 0
	// *(volatile uint32_t *)(ADC0_BASE) = 0x0C;  // Pin A0 
    uint32_t sc1 = ADC_SC1_ADCH(12); /* Set the channel number. */
	ADC0->SC1[0] = sc1 |= ADC_SC1_AIEN_MASK;  // Enable interupt on conversion complete 
    // Custom_K64F_ADC_Trigger_Read_Pin(PTB2);

	*ptrval = !(*ptrval);
    NVIC_EnableIRQ(ADC0_IRQn);
    DMA0->ERQ = DMA_ERQ_ERQ0_MASK;
}

/** Initialize the high frequency ticker
 *   mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\drivers\fsl_pit.c
 */
void Custom_PIT_Init(void)
{
    /* Common for ticker/timer. */
    uint32_t busClock;
    /* Structure to initialize PIT. */
    pit_config_t pitConfig;

    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);

    busClock = CLOCK_GetFreq(kCLOCK_BusClk);

    /* Let the timer to count if re-init. */
    if (!custom_pit_initied) {

        // PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, busClock / 1 - 1);
        PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 0x2FAF079);

        // PIT_SetTimerPeriod(PIT, kPIT_Chnl_1, 0xFFFFFFFF);
        // PIT_SetTimerChainMode(PIT, kPIT_Chnl_1, true);
        PIT_StartTimer(PIT, kPIT_Chnl_0);
        // PIT_StartTimer(PIT, kPIT_Chnl_1);
    }
	// PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // Enable interrupt and enable timer
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    
    // PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
    NVIC_SetVector(PIT0_IRQn, (uint32_t) pit0_isr);
    NVIC_EnableIRQ(PIT0_IRQn);

    custom_pit_initied = true;
}


void DMA_Init()
{
    uint32_t ADC0_DMA_CHANNEL = 0;
 
    // Enable clock for DMAMUX and DMA
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
    // Enable Channel 0 and set ADC0 as DMA request source 
	DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40);
	// Enable request signal for channel 0 
	DMA0->ERQ = DMA_ERQ_ERQ0_MASK;
    // DMA0->TCD[ADC0_DMA_CHANNEL].ERQ |= DMA_ERQ_ERQ0_MASK;
    // DMA0->ERQ
    // Setup control and status register
	DMA0->TCD[ADC0_DMA_CHANNEL].CSR = 0;

    
    // CLOCK_EnableClock(s_dmamuxClockName[DMAMUX_GetInstance(DMAMUX)]);
    DMAMUX_SetSource(DMAMUX, ADC0_DMA_CHANNEL, kDmaRequestMux0ADC0);
    DMAMUX_EnableChannel(DMAMUX, ADC0_DMA_CHANNEL);
    // DMAMUX_EnablePeriodTrigger(DMAMUX, ADC0_DMA_CHANNEL);

    edma_config_t edma_ADC0_config;
    EDMA_GetDefaultConfig(&edma_ADC0_config);
    EDMA_Init(DMA0, &edma_ADC0_config);

    edma_tcd_t edma_ADC0_TCD;

    edma_transfer_config_t edma_ADC0_transfer_config;
    EDMA_PrepareTransfer(&edma_ADC0_transfer_config, (void *)(ADC0_BASE + 0x10), kEDMA_TransferSize4Bytes, &result, kEDMA_TransferSize4Bytes, kEDMA_TransferSize4Bytes, kEDMA_TransferSize4Bytes, kEDMA_PeripheralToMemory);
    EDMA_SetTransferConfig(DMA0, ADC0_DMA_CHANNEL, &edma_ADC0_transfer_config, NULL);

/*
    uint32_t ADC0_DMA_CHANNEL = 0;
    // CLOCK_EnableClock(s_dmamuxClockName[DMAMUX_GetInstance(DMAMUX)]);
    // DMAMUX_SetSource(DMAMUX0, ADC0_DMA_CHANNEL, kDmaRequestMux0ADC0);
    // DMAMUX_EnableChannel(kDmaRequestMux0ADC0, ADC0_DMA_CHANNEL);
    // DMAMUX_EnablePeriodTrigger(DMAMUX0, ADC0_DMA_CHANNEL);

    // // dma_channel_allocate(kDmaRequestMux0ADC0);   // Automatically finds 1st available DA chanel, however for our use, better to specify since we want to use 1-4 for auto trigger 

    edma_config_t edma_ADC0_config;
    EDMA_GetDefaultConfig(&edma_ADC0_config);
    EDMA_Init(DMA0, &edma_ADC0_config);


    edma_transfer_config_t edma_ADC0_transfer_config;
    EDMA_PrepareTransfer(&edma_ADC0_transfer_config, (void *)(ADC0_BASE + 0x12), kEDMA_TransferSize2Bytes, &result, kEDMA_TransferSize2Bytes, kEDMA_TransferSize2Bytes, kEDMA_TransferSize2Bytes, kEDMA_PeripheralToMemory);
    // edma_ADC0_transfer_config.srcAddr = ADC0_BASE + 0x12; // Note right 2 bytes of results register //   &(ADC0->R[0]);
    // edma_ADC0_transfer_config.destAddr = result;
    // edma_ADC0_transfer_config.srcTransferSize = kEDMA_TransferSize2Bytes;
    // edma_ADC0_transfer_config.destTransferSize = kEDMA_TransferSize2Bytes;
    // edma_ADC0_transfer_config.srcOffset = 0;
    // edma_ADC0_transfer_config.destOffset = 0;
    // edma_ADC0_transfer_config.minorLoopBytes = 1;
    // edma_ADC0_transfer_config.majorLoopCounts = 1;

    EDMA_SetTransferConfig(DMA0, ADC0_DMA_CHANNEL, &edma_ADC0_transfer_config, NULL);
    EDMA_EnableChannelInterrupts(DMA0, ADC0_DMA_CHANNEL, kEDMA_MajorInterruptEnable);
*/
}