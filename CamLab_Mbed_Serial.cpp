#include "CamLab_Mbed_Serial.h"
#include "mbed.h"

void CamLab_Mbed_Serial::Init_Serial(void){
    // Create a BufferedSerial object with a default baud rate.
    // BufferedSerial serial_port(USBTX, USBRX); // May be able to have full-duplex if no arguments given to buffered Serial class 
    // Set desired properties (115200-8-N-1).    
    serial_handle->set_baud(Serial_baudrate);
    serial_handle->set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
}

void CamLab_Mbed_Serial::Init_SPI(void){
    // For now use mbed code to initialise all clocks/registers 
    DigitalOut Device_1_CS(D15);
    Device_1_CS = 1;
}

void CamLab_Mbed_Serial::Receive_Serial_Data(void){ // Look for more efficient implemmentation 
    // ThisThread::sleep_for(4ms); // Needed to get full input - 10ms just arbitrary value - but errors if only 1ms
    wait_us(3500);  // Wait time in us for 8*32 bits to arrive at a rate of 115200 
    // wait_us(us_wait_32_bytes_Serial); 
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
    if (serial_handle->readable()) {
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

            else if(buf_serial[0] == Write_32_Reg_Instr && num_in == Write_32_Expected_Bytes){
                Write_32_Reg_Response();
            }
            else if(buf_serial[0] == Write_16_Reg_Instr && num_in == Write_16_Expected_Bytes){
                Write_16_Reg_Response();
            }
            else if(buf_serial[0] == Write_8_Reg_Instr && num_in == Write_8_Expected_Bytes){
                Write_8_Reg_Response();
            }
            
            else if (buf_serial[0] == SPI_Message_Instr && num_in == (Num_Bytes_Instruction + 2 + buf_serial[2] + Num_Bytes_EOL)){
                SPI_Message_Response(spi_handle, buf_serial[1], buf_serial[2]);
            }
            
            else if(buf_serial[0] == Read_128_Reg_Instr && num_in == Read_128_Expected_Bytes){
                Read_128_Reg_Response();
            }
            
        }
    }
}

void CamLab_Mbed_Serial::Read_128_Reg_Response(void)
{
    uint32_t Addr = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    for (int i = 0; i < 4; i++)
    {
        *(uint32_t *)(buf_serial + (4*i)) = __REV(*(uint32_t *)(Addr));
        Addr += 0x4; 
    }
    
    Append_EOL_Char(Read_128_Response_Bytes);
    Write_Serial_Message(buf_serial, Read_128_Response_Bytes);
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


void CamLab_Mbed_Serial::Write_32_Reg_Response(void)
{
    uint32_t _Address = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    uint32_t New_Value = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset + Num_Bytes_Reg_Addr));
    Serial_Register_Write(_Address, Num_Bytes_32_Reg, New_Value);  // Read 4 bytes from Address defined in Serial Command into buf_serial
    Append_EOL_Char(Write_32_Response_Bytes);
    Write_Serial_Message(buf_serial, Write_32_Response_Bytes);
}

void CamLab_Mbed_Serial::Write_16_Reg_Response(void)
{
    uint32_t Address = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    uint16_t New_Value = __REVSH(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset + Num_Bytes_Reg_Addr));
    Serial_Register_Write(Address, Num_Bytes_16_Reg, New_Value);  // Read 4 bytes from Address defined in Serial Command into buf_serial
    Append_EOL_Char(Write_16_Response_Bytes);
    Write_Serial_Message(buf_serial, Write_16_Response_Bytes);
}

void CamLab_Mbed_Serial::Write_8_Reg_Response(void)
{
    uint32_t Address = __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset));
    uint8_t New_Value = *(uint32_t *)(buf_serial + Read_Reg_Addr_Offset + Num_Bytes_Reg_Addr);
    Serial_Register_Write(Address, Num_Bytes_8_Reg, New_Value);  // Read 4 bytes from Address defined in Serial Command into buf_serial
    Append_EOL_Char(Write_8_Response_Bytes);
    Write_Serial_Message(buf_serial, Write_8_Response_Bytes);
}

void CamLab_Mbed_Serial::Serial_Register_Write(uint32_t Addr, int size, uint32_t value)
{ // Same as Read, but first write the value to the requested address   
    if(size == Num_Bytes_32_Reg){
        *(uint32_t *)(Addr) = value; 
        *(uint32_t *)(buf_serial) = __REV(*(uint32_t *)(Addr));
    }
    else if(size == Num_Bytes_16_Reg){
        *(uint16_t *)(Addr) = value; 
        *(uint16_t *)(buf_serial) = __REVSH(*(uint16_t *)(Addr));  // Use different function to reverse 16 bit value 
    }
    else if(size == Num_Bytes_8_Reg){
        *(uint8_t *)(Addr) = value; 
        *(uint8_t *)(buf_serial) = *(uint8_t *)(Addr);  // Don't need to reverse byte order for a single byte 
    }    

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
        GPIOE->PCOR = (1U << SPI_Device_1); // Set CS for pin corresponding to Device1 to 0  
        for (int i = 0; i < Len_Message; i++) // Write contents of SPI message, depending on length of message
        {
            buf_serial[0 + i] = buf_serial[SPI_Message_Offset + i]; // Write SPI message back to PC for error checking 
            spi_handle->write(buf_serial[SPI_Message_Offset + i]);
        }
        GPIOE->PSOR = (1U << SPI_Device_1); // Set CS for pin corresponding to Device1 to 1
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