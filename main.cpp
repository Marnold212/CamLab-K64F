#include "mbed.h"
#include "CamLab_Mbed_Serial.h"

using namespace std;

BufferedSerial  serial_port(USBTX, USBRX);
SPI spi_port(D11, D12, D13); // mosi, miso, sclk

// BufferedSerial class is NonCopyable, therefore must pass pointer to Custom Class to use functionality 
char buff[32];
int main(){
    buff[0] = 'H';
    buff[1] = 'e';
    buff[2] = 'l';
    buff[3] = 'l';
    buff[4] = 'o';
    buff[5] = '\n';

    // printf("Testing Reg Read: uint32_t: %lx uint_8t: %c\n", *(uint32_t * )(0x40048024), *(uint *)(0x40048024));
    printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x40048054));
    printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x40048058));
    printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x4004805C));
    printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x40048060));
    
    CamLab_Mbed_Serial Test_Class(&serial_port, &spi_port);


    // Test_Class.Write_Serial_Message(buff, 6);
    while(1){
        Test_Class.Serial_Response();
        // Test_Class.Write_Serial_Message(buff, 5);
    }
}

// LED Green = PTE26  



// #include "mbed.h"

// SPI spi(D11, D12, D13); // mosi, miso, sclk

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



// DigitalOut cs(D15);

// int main(){
//     // Chip must be deselected
//     cs = 1;

//     printf("GPIOD PDDR Contents: %lu \n", (GPIOE->PDDR));
//     // Setup the spi for 8 bit data, high steady state clock,
//     // second edge capture, with a 1MHz clock rate
//     // spi.format(8, 3);
//     spi.frequency(1000000);

//     // Select the device by seting chip select low
//     // cs = 0;
//     GPIOE->PCOR = (1U << 24);

//     // Send 0x8f, the command to read the WHOAMI register
//     spi.write(0x3F);
//     // spi.write(0x30);

//     spi.write(0xFF);
//     // spi.write(0x00);

//     // Deselect the device
//     // cs = 1;
//     GPIOE->PSOR = (1U << 24); // /Set CS for pin corresponding to Device1 to 1
        
// }






//     while(1)
//     {

//         Serial_Response();

        
//     }
    
// }

// void Serial_Response(void){
//     uint32_t num_in = 0;
//     // ThisThread::sleep_for(1s);
//     while (serial_port.readable()) {
//         ThisThread::sleep_for(10ms); // Needed to get full input - 10ms just arbitrary value 
//         if ((num_in = serial_port.read(buf, sizeof(buf)))) { // Check there is something other than "\n" in the buffer AKA num_1 != 0
//             if(buf[0] == 0x30){
//                 if(num_in == 6){ // 1 Command byte (030), 4 address bytes (8 Hex Digits), 1 End of Line ("\n")
//                     Serial_Read_Register();
//                 }
//             }  
//         }

//     }
// }

// // Maximum number of element the application buffer can contain
// #define MAXIMUM_BUFFER_SIZE                                                  32
// // Create a BufferedSerial object with a default baud rate.
// static BufferedSerial serial_port(USBTX, USBRX); // May be able to have full-duplex if no arguments given to buffered Serial class 

// // Application buffer to receive & send serial data
// char buf[MAXIMUM_BUFFER_SIZE] = "{0}";

// void Serial_Response(void);
// void Serial_Read_Register(void);
// void Serial_Mirror_Request(void);

// int main(){
//     DigitalOut led(LED1);
//     // Set desired properties (9600-8-N-1).   Currently 9600 so I can easily use TeraTerm, however my want to increase to 115200 for performance 
//     serial_port.set_baud(9600);
//     serial_port.set_format(
//         /* bits */ 8,
//         /* parity */ BufferedSerial::None,
//         /* stop bit */ 1
//     );

    



// void Serial_Read_Register(void){
//     uint32_t num = 5;   // 4 bytes of data, "\n" char for endline

//     // Receiving 5 bytes - 1st 4 bytes are the hex address of register to read
//     // Cast this value to a uint32_t (Note to use __REV() to keep LSB at far right of 32 bits with 0 padding on the left) 
//     // (mbed/C++ is little endian and I like to think in big-endian as this is how we write numbers)
//     // Cast this uint_32_t to a pointer and use this to access uint32_t value in the requested register 
//     // Note that this value again needs to be reversed - THIS IS COMPLETELY SEPARATE OPERATION TO PREVIOUS __REV()

//     // uint32_t Address = __REV(*(uint32_t *)(buf))
//     // uint32_t val = *(uint32_t *)(Address) 
//     // *(uint32_t *)(buf) = __REV(val)
//     uint32_t offset = 0x1;   
//     *(uint32_t *)(buf) = __REV(*(uint32_t *)__REV(*(uint32_t *)(buf + offset)));  // Add the offset as 1st byte is instruction byte

    
//     buf[num-1] = '\n'; // Do this last so we are not overwriting data 
//     serial_port.write(buf, num);

//     // printf("Address = %p Contents = %lu\n", Address, *Address);

// }

// void Serial_Mirror_Request(uint32_t num_bytes){ // Need to know the number of bytes received in order to return - DOES include '\n' cahracter
//     // For testing purposes, return the command sent over serial in same order it was received 
//     serial_port.write(buf, num_bytes);
// }



