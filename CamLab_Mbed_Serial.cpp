#include "CamLab_Mbed_Serial.h"

void CamLab_Mbed_Serial::Init_Serial(void){
    // Create a BufferedSerial object with a default baud rate.
    static BufferedSerial serial_port(USBTX, USBRX); // May be able to have full-duplex if no arguments given to buffered Serial class 
    // Set desired properties (9600-8-N-1).   Currently 9600 so I can easily use TeraTerm, however my want to increase to 115200 for performance 
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    )
}

void CamLabMbed_Serial::Receive_Serial_Data(void){
    while(serial_port.read)
}

 uint32_t CamLabMbed_Serial::Read_Serial_Buffer(void){
    // Reads Serial buffer to buf[], while returning the number of bytes received 
    return serial_port.read(buf, sizeof(buf))
 }

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