#include "mbed.h"

SPI spi(D11, D12, D13); // mosi, miso, sclk

/*
MCP4922 Dual Channel Through-Hole ADC 
14 Pin package, Pin 1 top left with semicircle at top, Pin 14 top right

1 - VDD (5V)
2 - NC
3 - CS (D15)
4 - SCK (D13) (CLK)
5 - SDI (D11) (MOSI)
6 - NC 
7 - NC

14 - VoutA (Measure Output compared to ground)
13 - VrefA (Sets Ref - Wire to pin 1 (5V))
12 - Vss (GND)
11 - VrefB
10 - VoutB 
9 - SHDN - 5V (Wired to pin 1)
8 - LDAC - GND (Wired to pin 12)

*/



DigitalOut cs(D15);

int main(){
    // Chip must be deselected
    cs = 1;

    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    // spi.format(8, 3);
    spi.frequency(1000000);

    // Select the device by seting chip select low
    cs = 0;

    // Send 0x8f, the command to read the WHOAMI register
    // spi.write(0x3F);
    spi.write(0x30);

    spi.write(0xFF);
    // spi.write(0x00);

    // Deselect the device
    cs = 1;
        
}


