#include "mbed.h"
#include "CamLab_Mbed_Serial.h"

using namespace std;

// BufferedSerial  serial_port(USBTX, USBRX);

// class CamLab
// {
// private:
//     BufferedSerial *serial_handle;
//     /* data */
// public:
//     CamLab(BufferedSerial *serial_pointer);
//     ~CamLab();

//     void Write_Reg(char *reg, int num){
//         serial_handle->write(reg, num);
//     }
// };

// // Use an Initialisation List
// CamLab::CamLab(BufferedSerial *serial_pointer) : serial_handle(serial_pointer)
// {
// }

// CamLab::~CamLab()
// {
// }


BufferedSerial  serial_port(USBTX, USBRX);
// BufferedSerial class is NonCopyable, therefore must pass pointer to Custom Class to use functionality 
char buff[32];
int main(){
    buff[0] = 'H';
    buff[1] = 'e';
    buff[2] = 'l';
    buff[3] = 'l';
    buff[4] = 'o';
    buff[5] = '\n';

    CamLab_Mbed_Serial Test_Class(&serial_port);
    // Test_Class.Write_Serial_Message(buff, 6);
    while(1){
        Test_Class.Serial_Response();
        // Test_Class.Write_Serial_Message(buff, 5);
    }


}

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



