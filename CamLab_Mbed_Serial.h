#ifndef CAMLAB_MBED_SERIAL_H
#define CAMLAB_MBED_SERIAL_H

#include "mbed.h"

/* Maximum number of element the application buffer can contain */
/* Limited to 32 in hardware Serial buffer ? */
#define MAXIMUM_BUFFER_SIZE                                                   32

/** Expected Format of Command */ 
/* Length of various parts of Command */
#define Num_Bytes__EOL                           1  /* End of Line character occupies 1 byte */
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
#define Read_32_Reg                             0x30
#define Read_16_Reg                             0x31
#define Read_8_Reg                              0x32
#define Write_32_Reg                            0x33
#define Write_16_Reg                            0x34
#define Write_8_Reg                             0x35

/** Expected Received Bytes */
/* Expected number of received bytes expected for a given instruction including End of Line char */
#define Read_32_Expected_Bytes                  Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL     
#define Read_16_Expected_Bytes                  Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL  
#define Read_8_Expected_Bytes                   Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_EOL  
#define Write_32_Expected_Bytes                 Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_32_Reg + Num_Bytes_EOL  
#define Write_16_Expected_Bytes                 Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_16_Reg + Num_Bytes_EOL  
#define Write_8_Expected_Bytes                  Num_Bytes_Instruction + Num_Bytes_Reg_Addr + Num_Bytes_8_Reg + Num_Bytes_EOL  

/** Expected Replied Bytes */
/* Expected number of bytes sent as reply to a given instruction including End of Line char */

#define Read_32_Response_Bytes                  Num_Bytes_32_Reg + Num_Bytes_EOL
#define Read_16_Response_Bytes                  Num_Bytes_16_Reg + Num_Bytes_EOL
#define Read_8_Response_Bytes                   Num_Bytes_8_Reg + Num_Bytes_EOL
#define Write_32_Response_Bytes                 Num_Bytes_32_Reg + Num_Bytes_EOL
#define Write_16_Response_Bytes                 Num_Bytes_16_Reg + Num_Bytes_EOL
#define Write_8_Response_Bytes                  Num_Bytes_8_Reg + Num_Bytes_EOL




/* ----------------------------------------------------------------------------
   -- CamLab_Mbed_Serial Communication Class 
   ---------------------------------------------------------------------------- */
class CamLab_Mbed_Serial
{
    public:

    char buf[MAXIMUM_BUFFER_SIZE] = "{0}"; /* Buffer for Serial Communication - limited to 32 bytes */
    BufferedSerial serial_port;

    // static BufferedSerial serial_port(USBTX, USBRX); /* Stores object for Serial Communication */
    // Use a list iniitialization
    // BufferedSerial is a NonCopyable Class 
    CamLab_Mbed_Serial(BufferedSerial buffered_serial_port) : serial_port(buffered_serial_port){
        
        Init_Serial(); 
    
    }; /* Upon construction initialise serial channel */

    void Init_Serial(void); /* Initialises the Serial Communication object */




    // Doesn't read any of the data, simply waits for it to arrive - must be better way of ensuring we get all data 
    void Receive_Serial_Data(void); 
    
    // After waiting for all data to arrive, read buf, and return number of bytes received 
    int Read_Serial_Buffer(void);

    /**
     * @brief Sends the Data in the register specified over Serial Channel. Note that the data is stored in an intermediate buffer, 
     * and the number of bytes to send must be specified. ? Need to ensure message sent has a '\n' as final character sent ? 
     * Expected to send unsigned integer values (char). No return value 
     * 
     * @param reg Pointer to 1st character of byte register storing message to send
     * @param num Length of message to send (Manual handling of Endline characters '\n')
     */
    void Write_Serial_Register(char *reg, int num); 

    void print_Test();


};

#endif