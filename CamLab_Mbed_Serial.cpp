#include "CamLab_Mbed_Serial.h"
#include "mbed.h"

void CamLab_Mbed_Serial::Init_Serial(void){
    // Create a BufferedSerial object with a default baud rate.
    // BufferedSerial serial_port(USBTX, USBRX); // May be able to have full-duplex if no arguments given to buffered Serial class 
    // Set desired properties (9600-8-N-1).   Currently 9600 so I can easily use TeraTerm, however my want to increase to 115200 for performance 
    serial_handle->set_baud(9600);
    serial_handle->set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
}

void CamLab_Mbed_Serial::Receive_Serial_Data(void){ // Look for more efficient implemmentation 
    ThisThread::sleep_for(10ms); // Needed to get full input - 10ms just arbitrary value - but errors if only 1ms
    
}

int CamLab_Mbed_Serial::Read_Serial_Buffer(void){ 
// Reads Serial buffer to buf[], while returning the number of bytes received 
    return serial_handle->read(buf_serial, sizeof(buf_serial));
}

void CamLab_Mbed_Serial::Write_Serial_Message(char *reg, int num){
    serial_handle->write(reg, num);
}

void CamLab_Mbed_Serial::Append_EOL_Char(int Num_Reply_Bytes)
{
    buf_serial[Num_Reply_Bytes - 1] = '\n';
}

void CamLab_Mbed_Serial::Serial_Response(void){
    int num_in = 0;
    // ThisThread::sleep_for(1s);
    while (serial_handle->readable()) {
        Receive_Serial_Data(); // Wait for a given time to receive all data ?? Look for more efficient implemmentation  
        if ((num_in = serial_handle->read(buf_serial, sizeof(buf_serial)))) { // Check there is something other than "\n" in the buffer AKA num_1 != 0
            // if(buf_serial[0] == 0x30){
            //     if(num_in == 6){ // 1 Command byte (030), 4 address bytes (8 Hex Digits), 1 End of Line ("\n")
            //         Serial_Read_Register();
            //     }
            // }  
            if(buf_serial[0] == Read_32_Reg_Instr && num_in == Read_32_Expected_Bytes){ //  
                Read_32_Reg_Response();
            }
            else if(buf_serial[0] == Read_16_Reg_Instr && num_in == Read_16_Expected_Bytes){
                Read_16_Reg_Response();
            }
            else if(buf_serial[0] == Read_8_Reg_Instr && num_in == Read_8_Expected_Bytes){
                Read_8_Reg_Response();
            }

            else if (buf_serial[0] == SPI_Message_Instr && num_in == (Num_Bytes_Instruction + 2 + buf_serial[2] + Num_Bytes_EOL)){
                SPI_Message_Response(spi_handle, buf_serial[1], buf_serial[2]);
            }
        }
    }
}


void CamLab_Mbed_Serial::Read_32_Reg_Response(void)
{
    uint32_t Address = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    Serial_Register_Read(Address, Num_Bytes_32_Reg);  // Read 4 bytes from Address defined in Serial Command into buf_serial
    Append_EOL_Char(Read_32_Response_Bytes);
    Write_Serial_Message(buf_serial, Read_32_Response_Bytes);
}

void CamLab_Mbed_Serial::Read_16_Reg_Response(void)
{
    uint32_t Address = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    Serial_Register_Read(Address, Num_Bytes_16_Reg);  // Read 4 bytes from Address defined in Serial Command into buf_serial
    Append_EOL_Char(Read_16_Response_Bytes);
    Write_Serial_Message(buf_serial, Read_16_Response_Bytes);
}

void CamLab_Mbed_Serial::Read_8_Reg_Response(void)
{
    uint32_t Address = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    Serial_Register_Read(Address, Num_Bytes_8_Reg);  // Read 4 bytes from Address defined in Serial Command into buf_serial
    Append_EOL_Char(Read_8_Response_Bytes);
    Write_Serial_Message(buf_serial, Read_8_Response_Bytes);
}




void CamLab_Mbed_Serial::Serial_Register_Read(uint32_t Addr, int size = Num_Bytes_32_Reg)
{
    if(size == Num_Bytes_32_Reg){
        *(uint32_t *)(buf_serial) = __REV(*(uint32_t *)(Addr));
    }
    else if(size == Num_Bytes_16_Reg){
        *(uint16_t *)(buf_serial) = __REVSH(*(uint16_t *)(Addr));  // Use different function to reverse 16 bit value 
    }
    else if(size == Num_Bytes_8_Reg){
        *(uint8_t *)(buf_serial) = *(uint8_t *)(Addr);  // Don't need to reverse byte order for a single byte 
    }    
}

void CamLab_Mbed_Serial::SPI_Message_Response(SPI *spi_handle, int Device, int Len_Message)
{
    if(Device == 1){
        //SPIDeviceCS CS  = Device1;
        GPIOD->PCOR |= (1U << SPI_Device_1); // Set CS for pin corresponding to Device1 to 0  
        for (int i = 0; i < Len_Message; i++) // Write contents of SPI message, depending on length of message
        {
            spi_handle->write(buf_serial[SPI_Message_Offset + i]);
            buf_serial[0 + i] = buf_serial[SPI_Message_Offset + i]; // Write SPI message back to PC for error checking 
        }
        GPIOD->PSOR |= (1U << SPI_Device_1); // Set CS for pin corresponding to Device1 to 1
    }
    int Serial_Return_Len = Len_Message + 1;
    Append_EOL_Char(Serial_Return_Len);
    Write_Serial_Message(buf_serial, Serial_Return_Len);
}

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