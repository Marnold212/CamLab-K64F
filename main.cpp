#include "mbed.h"
#include "CamLab_Mbed_Serial.h"
#include "CamLab_Mbed_AD7606.h"

using namespace std;

BufferedSerial  serial_port(USBTX, USBRX);
SPI spi_port(D11, D12, D13); // mosi, miso, sclk


// BufferedSerial class is NonCopyable, therefore must pass pointer to Custom Class to use functionality 
int main(){
    // Within SRAM of K64F : 0x1FFF_0000 to 0x2002_FFFF 
    uint32_t *Dummy_Reg_Ptr_0 = (uint32_t *)0x1FFF0000U;
    uint32_t *Dummy_Reg_Ptr_1 = (uint32_t *)0x1FFF0004U;
    uint32_t *Dummy_Reg_Ptr_2 = (uint32_t *)0x1FFF0008U;
    uint32_t *Dummy_Reg_Ptr_3 = (uint32_t *)0x1FFF000CU;
    *(Dummy_Reg_Ptr_0) = 60000;
    *(Dummy_Reg_Ptr_1) = 6000000;
    *(Dummy_Reg_Ptr_2) = 1234567;
    *(Dummy_Reg_Ptr_3) = 0;

    // printf("Testing Reg Read: uint32_t: %lx uint_8t: %c\n", *(uint32_t * )(0x40048024), *(uint *)(0x40048024));
    
    // printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x40048054));
    // printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x40048058));
    // printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x4004805C));
    // printf("Testing Reg Read: uint32_t: %lx \n", *(uint32_t * )(0x40048060));
    
    CamLab_Mbed_AD7606 Test_ADC(NC, A4, A5, NC, A3, A0, A1, NC, NC, NC);
    CamLab_Mbed_Serial Test_Class(&serial_port, &spi_port);


    // Test_Class.Write_Serial_Message(buff, 6);
    while(1){
        Test_ADC.Read_Raw((uint16_t *)(0x1FFF0000U), 8);
        Test_Class.Serial_Response();
        // Test_Class.Write_Serial_Message(buff, 5);
    }
}

// #include "mbed.h"
// #include "CamLab_Mbed_Serial.h"

// using namespace std;

// BufferedSerial  serial_port(USBTX, USBRX);
// DigitalOut RD(A4);
// DigitalOut CONV(A5);
// DigitalIn Busy(A3);
// DigitalOut VIO(A2);   // Might need to be 5v not 3.3v output
// DigitalIn FRST(A0);
// DigitalOut led(LED_RED);
// DigitalOut RST(A1);

// BusIn DB(D0, PTB18, D1, PTB19, D2, PTC1, D3, PTC8, PTC12, PTC9, PTC4, PTC0, PTD0, PTC7, PTD2, PTC5);
// char buf[32] = "{0}";



// // BufferedSerial class is NonCopyable, therefore must pass pointer to Custom Class to use functionality 
// int main(){
//     RST = 0;
//     RD = 1;
//     CONV = 1;
//     VIO = 1;
//     buf[16] = '\n';
//     uint16_t results[8];
//     RST = 1; // Pulse Reset Output Pin 
//     RST = 0;
//     while(1){
//         CONV = 0;
//         led = 0;
//         CONV = 1;
//         wait_us(5000);
//         // DB.mode(PullNone);
//         for (size_t i = 0; i < 8; i++)
//         {
            
//             RD = 0;
//             // *(uint16_t *)(buf + 2*i) = DB.read();
//             // printf("i:%i   Val = %u \n", i, DB.read());
//             results[i] = (uint16_t)(DB.read());
//             RD = 1;
//             // if(i == 7){
//             //     printf("1st value = %u; 7th value = %u\n", results[0], results[7]);
//             // }
//         }
//         led = 1;
//         printf("V1: %u   V2: %u    V3: %u  V4:%u    V5:%u   V6:%u   V7:%u   V8:%u\n", results[0], results[1], results[2], results[3], results[4], results[5], results[6], results[7]);
//         // serial_port.write(buf, 16);
//     }


// }



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


