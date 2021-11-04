#include "mbed.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);

// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = "{0}";
uint32_t num = (8*2) + 1;

int main(){
    // Set desired properties (9600-8-N-1).
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );


    buf[num-1] = '\n';
    int count = 0;
    while(1)
    {
        
        
        
    }
    
}

