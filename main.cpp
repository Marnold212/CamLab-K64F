#include "mbed.h"
#include "mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\drivers\fsl_adc16.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);


void pit_init();
void PIT_IRQHandler(void);
void Custom_K64F_ADC_Init(ADC_Type *base);

// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = "{0}";
uint32_t num = (8*2) + 1;

uint8_t ADC_0_Selection = 0;
uint8_t ADC_1_Selection = 0;
#define No_ADC_0_Channels     4
#define No_ADC_1_Channels     4
uint32_t ADC_1_Channels[] = {0x0C, 0x0D, 0x0E, 0x0F};
uint32_t ADC_0_Channels[] = {0x0C, 0x0D, 0x0E, 0x0F};

// volatile bool val = 0;
// volatile bool *ptrval = &val;

int main(){
    
    printf("System CLock frequency = %lu\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    printf("Bus CLock frequency = %lu\n", CLOCK_GetBusClkFreq());
    printf("Bus CLock frequency = %lu\n", CLOCK_GetFreq(kCLOCK_BusClk));

    Custom_K64F_ADC_Init(ADC0);
    Custom_K64F_ADC_Init(ADC1);

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

    // // Enable clocks
	// SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;	// ADC 0 clock
	// SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;	// PTB0 clock
    // ADC0->SC2 |= ADC_SC2_DMAEN_MASK;    // DMA Enable

    // printf("All UART registers are 32 bits wide - 4 bytes each!!\n");
    // printf("Contents of register SIM_BASE = %lu \n", *(uint32_t *)(SIM_BASE));
    // printf("Contents of register SIM_BASE + 0x04 = %lu \n", *(uint32_t *)(SIM_BASE + 0x04));
    // printf("Contents of register SIM_BASE + 0x1004 = %lu \n", *(uint32_t *)(SIM_BASE + 0x1004));
    // printf("Contents of register SIM_BASE + 0x100C = %lu \n", *(uint32_t *)(SIM_BASE + 0x100C));
    // printf("Contents of register SIM_BASE + 0x1010 = %lu \n", *(uint32_t *)(SIM_BASE + 0x1010));
    // printf("Contents of register SIM_BASE + 0x1018 = %lu \n", *(uint32_t *)(SIM_BASE + 0x1018));
    // printf("Contents of register SIM_BASE + 0x1024 = %lu \n", *(uint32_t *)(SIM_BASE + 0x1024));
    // printf("Contents of register SIM_BASE + 0x1028 = %lu \n", *(uint32_t *)(SIM_BASE + 0x1028));
    // printf("Contents of register SIM_BASE + 0x102C = %lu \n", *(uint32_t *)(SIM_BASE + 0x102C));


    pit_init();

    buf[num-1] = '\n';
    int count = 0;
    while(1)
    {
        while(!(*(volatile uint32_t *)(ADC0_BASE) & (1U << 7)) && !(*(volatile uint32_t *)(ADC1_BASE) & (1U << 7))){
            
        }
        // buf[0] = ((*(volatile uint32_t *)(0x4003B000 + 0x10) >> 8) & 0xFF); // Upper byte of 16 bit data 
        buf[(2 * ADC_0_Selection) + 0] = ((ADC0->R[0]) >> 8) & 0xFF;  // Lower byte of 16 bit data 
        buf[(2 * ADC_0_Selection) + 1] = ADC0->R[0];  // Lower byte of 16 bit data 
        buf[(2 * ADC_1_Selection) + 8] = ((ADC1->R[0]) >> 8) & 0xFF;  // Lower byte of 16 bit data 
        buf[(2 * ADC_1_Selection) + 9] = ADC1->R[0];  // Lower byte of 16 bit data 
        if(ADC_0_Selection == (No_ADC_0_Channels - 1)){
            serial_port.write(buf, num);
        }
        
    }
    
}

/*	Handles PIT interrupt if enabled
 * 
 * 	Starts conversion in ADC0 with single ended channel 8 (PTB0) as input
 * 
 * */
void PIT0_IRQHandler(void)
{	
	// Clear interrupt
	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
	
    ADC_0_Selection = (ADC_0_Selection + 1) % No_ADC_0_Channels;
    ADC_1_Selection = (ADC_1_Selection + 1) % No_ADC_1_Channels;
    
    uint32_t ADC0sc1a = ADC_0_Channels[ADC_0_Selection];
    uint32_t ADC1sc1a = ADC_1_Channels[ADC_1_Selection];

	// Write to SC1A to start conversion with channel 0
    ADC0->SC1[0] = ADC0sc1a;
    ADC1->SC1[0] = ADC1sc1a;
	// *(volatile uint32_t *)(ADC0_BASE) = ADC0sc1a;  
    // *(volatile uint32_t *)(ADC1_BASE) = ADC1sc1a;  

	// *ptrval = !(*ptrval);
}

/* Initializes the PIT module to produce an interrupt every second
 * 
 * */
void pit_init(void)
{
	// Enable PIT clock
    CLOCK_EnableClock(kCLOCK_Pit0);
	// SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Turn on PIT
	PIT->MCR = 0;
	
	// Configure PIT to produce an interrupt every 1s
    // PIT Clock on K64F has frequency of 21MHz after RESET according to table 25-17 in datasheet XXXX
    // Definetely FASTER than 21MHz 
    // PIT Clock is MCG - Appararently uses the bus clock 

    // PIT->CHANNEL[0].LDVAL = 0x1406D9B;  // 1/21Mhz = 47.62ns   (1s/47.62ns)-1= 20,999,279 cycles or                                      
    // PIT->CHANNEL[0].LDVAL = 0x2FAF079;   // 1/50MHz = 
	PIT->CHANNEL[0].LDVAL = 0x0002000; 
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // Enable interrupt and enable timer
	
	// Enable interrupt registers ISER and ICPR
    NVIC_SetVector(PIT0_IRQn, (uint32_t)PIT0_IRQHandler);
    NVIC_EnableIRQ(PIT0_IRQn);

}



void Custom_K64F_ADC_Init(ADC_Type *base)
{
    // #define MAX_FADC 6000000
    #define MAX_FADC 12000000 // Datasheet specifies range of sampling frequency to be 2-12 MHz for 16-bit mode 

    adc16_config_t adc16_config;

    // Calculate fastest clock parameters which don't exceed defined max sample speed 
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

    // Following setting changes reference voltage to pins: 
    // adc16_config.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;

    // Test these parameters for increasing speed of sampling 
    // adc16_config.longSampleMode = kADC16_LongSampleDisabled;
    adc16_config.enableHighSpeed = true;

    adc16_config.clockSource = kADC16_ClockSourceAlt0;
    adc16_config.clockDivider = (adc16_clock_divider_t)clkdiv;
    adc16_config.resolution = kADC16_ResolutionSE16Bit;

    // adc16_config.enableInterruptOnConversionCompleted = false;

    ADC16_Init(base, &adc16_config);

    ADC16_EnableHardwareTrigger(base, false); 
    ADC16_SetHardwareAverage(base, kADC16_HardwareAverageCount4);

    // Check effect of following setting on accuracy + speed  
    // ADC16_SetHardwareAverage(base, kADC16_HardwareAverageDisabled);
}

