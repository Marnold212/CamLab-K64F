#ifndef CAMLAB_MBED_SERIAL_H
#define CAMLAB_MBED_SERIAL_H

#include "mbed.h"

/* Maximum number of element the application buffer can contain */
/* Limited to 32 in hardware Serial buffer ? */
#define MAXIMUM_BUFFER_SIZE                                                   32


/** Expected Format of Command */ 
/* Length of various parts of Command */
/* Could convert some of these definitions into enum for neater code */
#define Num_Bytes_EOL                           1  /* End of Line character occupies 1 byte */
#define Num_Bytes_Instruction                   1  /* Instruction character occupies 1 byte */
#define Num_Bytes_32_Reg                        4  /* The value stored in a 32-bit Register occupies 4 byte */
#define Num_Bytes_16_Reg                        2  /* The value stored in a 16-bit Register occupies 4 byte */
#define Num_Bytes_8_Reg                         1  /* The value stored in a 8-bit Register occupies 4 byte */
#define Num_Bytes_Reg_Addr                      4  /* Number of bytes required to store a register address */


/* ----------------------------------------------------------------------------
   -- Serial Instruction Definitions 
   ---------------------------------------------------------------------------- */

/** Instructions - Instruction Bytes */ 
/* 1st byte of command received from PC Serial channel */
#define Read_32_Reg_Instr                           0x30
#define Read_16_Reg_Instr                           0x31
#define Read_8_Reg_Instr                            0x32
#define Write_32_Reg_Instr                          0x33
#define Write_16_Reg_Instr                          0x34
#define Write_8_Reg_Instr                           0x35
#define Read_128_Reg_Instr                          0x36
#define SPI_Message_Instr                           0x40

/** Expected Received Bytes */
/* Expected number of received bytes expected for a given instruction including End of Line char */
#define Read_128_Expected_Bytes                 Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL  
#define Read_32_Expected_Bytes                  Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL     
#define Read_16_Expected_Bytes                  Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL  
#define Read_8_Expected_Bytes                   Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL  
#define Write_32_Expected_Bytes                 Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_32_Reg + Num_Bytes_EOL  
#define Write_16_Expected_Bytes                 Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_16_Reg + Num_Bytes_EOL  
#define Write_8_Expected_Bytes                  Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_8_Reg + Num_Bytes_EOL  

/** Expected Replied Bytes */
/* Expected number of bytes sent as reply to a given instruction including End of Line char */

#define Read_128_Response_Bytes                 (4 * Num_Bytes_32_Reg) + Num_Bytes_EOL
#define Read_32_Response_Bytes                  Num_Bytes_32_Reg + Num_Bytes_EOL
#define Read_16_Response_Bytes                  Num_Bytes_16_Reg + Num_Bytes_EOL
#define Read_8_Response_Bytes                   Num_Bytes_8_Reg + Num_Bytes_EOL
#define Write_32_Response_Bytes                 Num_Bytes_32_Reg + Num_Bytes_EOL
#define Write_16_Response_Bytes                 Num_Bytes_16_Reg + Num_Bytes_EOL
#define Write_8_Response_Bytes                  Num_Bytes_8_Reg + Num_Bytes_EOL

/** Address Offsets */
/* Define the Offset of serial buffer for Register Address for various commands */

#define Read_Reg_Addr_Offset                    Num_Bytes_Instruction
#define Write_Reg_Addr_Offset                   Num_Bytes_Instruction

#define SPI_Message_Offset                      Num_Bytes_Instruction + 2 // 1 Byte for instruction + 1 byte for length of SPI Message + 1 byte for Device selection  



/* ----------------------------------------------------------------------------
   -- CamLab_Mbed_Serial Communication Class 
   ---------------------------------------------------------------------------- */

/* Note that Byte order changes occurs due to casting of char[] to uint and vice-versa */


class CamLab_Mbed_Serial
{
    public:

    enum SPIDeviceCS{ 
        SPI_Device_1 = 24  /* D15 = PTE24 so GPIOE pin 24 */             
    };

    // Consider using mbed ByteBuffer class ?
    char buf_serial[MAXIMUM_BUFFER_SIZE] = "{0}"; /* Buffer for Serial Communication - limited to 32 bytes */
    BufferedSerial *serial_handle;
    SPI *spi_handle;

    // static BufferedSerial serial_port(USBTX, USBRX); /* Stores object for Serial Communication */
    // Use a list iniitialization
    // BufferedSerial is a NonCopyable Class 
    CamLab_Mbed_Serial(BufferedSerial *serial_pointer, SPI *spi_pointer) : serial_handle(serial_pointer), spi_handle(spi_pointer) {
        
        Init_Serial(); 
        Init_SPI();
    
    }; /* Upon construction initialise serial channel */

    void Init_Serial(void); /* Initialises the Serial Communication object */

    /**
     * @brief Device 1 - Pin D15 - High by default (Set high at initialisation)
     * 
     */
    void Init_SPI(void); /* Initialise the CS pins required for SPI communication */


    /**
     * @brief Public method used for responding to serial commands. Check if there any serial bytes availble to read, if not return nothing. 
     * If there are bytes available, wait short period to ensure full message has arrived, then read into intermediate char ser_buff[32]. 
     * Check the instruction (1st) byte of the received message, and compare to instructions defined in header file. As an error checking 
     * step check that the message has an End of Line character ('\n') at the end of the expected length of message for given instruction. 
     * Respond appropriately for the given instruction using same ser_buff array for storing the reply, and adding an '\n' to EOL. 
     * 
     * Command Layout: 
     * ------------------------------------------------------------------
     * | Instruction | Address | Extra Information (e.g. values) | '\n' | 
     * ------------------------------------------------------------------
     * 
     * Response Layout: 
     * ----------------
     * | Value | '\n' | 
     * ----------------
     * 
     */
    void Serial_Response(void);


    // Doesn't read any of the data, simply waits for it to arrive - must be better way of ensuring we get all data 
    void Receive_Serial_Data(void); 
    

    /**
     * @brief Once all serial data has arrived, read to buffer, and return number of bytes received 
     * 
     * @return int Number of bytes received, intcluding '\n' char
     */
    int Read_Serial_Buffer(void);

    /**
     * @brief Sends the Data in the register specified over Serial Channel. Note that the data is stored in an intermediate buffer, 
     * and the number of bytes to send must be specified. ? Need to ensure message sent has a '\n' as final character sent ? 
     * Expected to send unsigned integer values (char). No return value 
     * 
     * @param reg Pointer to 1st character of byte register storing message to send
     * @param num Length of message to send (Manual handling of Endline characters '\n')
     */
    void Write_Serial_Message(char *reg, int num); 


    /**
     * @brief Sets the last byte (Num_Reply_Bytes - 1) of the message to '\n' as an added error checking step for the PC when receiving serial response 
     * Uses the ser_buff[] character buffer as the basis for storing the messge sent as a response. Number of bytes sent in 
     * response to a given instruction is defined in header file. 
     * 
     * @param Num_Reply_Bytes Number of bytes sent in response for a given instruction, defined in header file.  
     */
    void Append_EOL_Char(int Num_Reply_Bytes);

 

  
    //  * first determine the 4-byte address to register of interest, which is read as 4 consecutive bytes of command in ser_buff[] offset by an 
    //  * amount depending on defined length of instruction which comes at start of command
     

    /**
     * @brief PRIVATE If the received command instruction (1st) byte indicates a request to read a device register of a specified size, 
     * and the value in the address is entered into the start of the ser_buff[] array. Note that the byte order is reversed from 
     * the format used by mbed device (Little Endian) so that the returned value has MSB at left and LSB at right. 
     * ## ONLY USE 16 OR 8 BIT READS IF SECTIONS OF REGISTER ARE READ PROTECTED, otherwise you can easily run into issues where you 
     * are not reading the correct bytes due to the reversing of orders that are occuring. Try and always read a 32 bit aligned array 
     * as specified in datasheet for K64F where possible.
     *  
     * 
     * @param Addr Address of LEFTMOST byte of register in question - must be a uint32_t (Obtain from command via __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset)))
     * @param size Size of address being read: 8;16;32 bits (Use definitions from header file) - Use 32 unless sections of target are read protected to ensure correct byte order. 
     */
    void Serial_Register_Read(uint32_t Addr, int size);
    

    /**
     * @brief Read a 32 byte address. First extract address from ser_buff[] to uint32_t at offset defined in header file, then 
     * read the value back into ser_buff[] and append '\n' at last byte of expected length of response message. Then write this 
     * value back to the PC. 
     * If read is taken from Left address of 32-bit aligned resigster: 
     * Response Bit Order: | 31-24 | 23-16 | 15-8 | 7-0 | '\n' |
     * 
     */
    void Read_32_Reg_Response(void);

    /**
     * @brief Read a 32 byte address. First extract address from ser_buff[] to uint32_t at offset defined in header file, then 
     * read the value back into ser_buff[] and append '\n' at last byte of expected length of response message. Then write this 
     * value back to the PC. 
     * If read is taken from Left address of 32-bit aligned resigster: 
     * Response Bit Order: | 15-8 | 7-0 | '\n' |
     * 
     */
    void Read_16_Reg_Response(void);

    void Read_8_Reg_Response(void);

    /**
     * @brief Same as previous Reads, but reads 4 consecutive 32 bit values. Intended for reading the buffered ADC readings 8x16bit = 16 bytes total 
     * 
     */
    void Read_128_Reg_Response(void);

    /**
     * @brief PRIVATE If the received command instruction (1st) byte indicates a request to write to a device register of a specified size, 
     * and the value in the address is entered into the start of the ser_buff[] array. Note that the byte order is reversed from 
     * the format used by mbed device (Little Endian) so that the returned value has MSB at left and LSB at right. 
     * Also writes the new value back to the PC as an error checking step 
     * 
     * ## ONLY USE 16 OR 8 BIT Writes IF SECTIONS OF REGISTER ARE READ PROTECTED, otherwise you can easily run into issues where you 
     * are not reading the correct bytes due to the reversing of orders that are occuring. Try and always read a 32 bit aligned array 
     * as specified in datasheet for K64F where possible.
     * 
     * @param Addr Address of LEFTMOST byte of register in question - must be a uint32_t (Obtain from command via __REV(*(uint32_t *)(buf_serial + Read_Reg_Addr_Offset)))
     * @param size Size of address being read: 8;16;32 bits (Use definitions from header file) - Use 32 unless sections of target are read protected to ensure correct byte order. 
     * @param value New value to be stored in address 
     */
    void Serial_Register_Write(uint32_t Addr, int size, uint32_t value);

    void Write_32_Reg_Response(void);

    void Write_16_Reg_Response(void);

    void Write_8_Reg_Response(void);

    /**
     * @brief Method for manually sending custom messages over SPI 
     * Form of Instruction: | Instruction Byte | Device | Len_Message | Message Bytes | '\n' | 
     * 
     * @param Device Each Channel will have a corresponding Chip Select (CS) pin predefined e.g. 1 = D15
     * @param Len_Message Number of bytes to send as SPI message 
     */
    void SPI_Message_Response(SPI *spi_handle, int Device, int Len_Message);
    

};

#endif