#ifndef CAMLAB_MBED_SERIAL_H
#define CAMLAB_MBED_SERIAL_H

#include "mbed.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Application buffer to receive & send serial data
char buf[MAXIMUM_BUFFER_SIZE] = "{0}";

class CamLab_Mbed_Serial
{
public:
    void Init_Serial(void);


public:
    // Doesn't read any of the data, simply waits for it to arrive - must be better way of ensuring we get all data 
    uint32_t Receive_Serial_Data(void); // Returns the number of bytes received 
    
    // After waiting for all data to arrive, read buf, and return number of bytes received 
    uint32_t Read_Serial_Buffer(void);
};

#endif