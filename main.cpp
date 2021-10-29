
#include "mbed.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);


void pit_init();
void PIT_IRQHandler(void);

// Application buffer to receive the data
// char buf[MAXIMUM_BUFFER_SIZE] = "00\r\n"; // Not sure which part of buffer this uses - prefer below implementation 
char buf[MAXIMUM_BUFFER_SIZE] = "0000";
uint32_t num = 4;

int main(){
    // Initialise ADC0 
    AnalogIn input(A0);
    uint16_t sample = input.read_u16();

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

    pit_init();

    buf[2] = '\r';
    buf[3] = '\n';
    while(1)
    {
        while(!(*(volatile uint32_t *)(ADC0_BASE) & (1U << 7))){
            
        }
        buf[0] = ((*(volatile uint32_t *)(0x4003B000 + 0x10) >> 8) & 0xFF); // Upper byte of 16 bit data 

        buf[1] = (*(volatile uint32_t *)(0x4003B000 + 0x10) & 0xFF);  // Lower byte of 16 bit data 
        // buf[1] = ADC0->R[0] & 0xFF;  // Lower byte of 16 bit data 
        serial_port.write(buf, num);
        // ThisThread::sleep_for(2s);
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
	
	// Write to SC1A to start conversion with channel 12 PTB2
	// ADC0_SC1A = (ADC_SC1_ADCH(ADC_CHANNEL) |
				//  (ADC0_SC1A & (ADC_SC1_AIEN_MASK | ADC_SC1_DIFF_MASK)));  
	
	// Write to SC1A to start conversion with channel 0
	*(volatile uint32_t *)(ADC0_BASE) = 0x0C;  // Pin A0 
}

/* Initializes the PIT module to produce an interrupt every second
 * 
 * */
void pit_init(void)
{
	// Enable PIT clock
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Turn on PIT
	PIT->MCR = 0;
	
	// Configure PIT to produce an interrupt every 1s
    // PIT Clock on K64F has frequency of 21MHz after RESET according to table 25-17 in datasheet ?? 

    PIT->CHANNEL[0].LDVAL = 0x2FAF080;  // 1/21Mhz = 20ns   (1s/20ns)-1= 49,999,999 cycles or 0x2FAF079
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // Enable interrupt and enable timer
	
	// Enable interrupt registers ISER and ICPR
    NVIC_SetVector(PIT0_IRQn, (uint32_t)PIT0_IRQHandler);
    NVIC_EnableIRQ(PIT0_IRQn);

}


