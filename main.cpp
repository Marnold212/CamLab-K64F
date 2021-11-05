#include "mbed.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);

// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = "{0}";

int main(){
    DigitalOut led(LED1);
    // Set desired properties (9600-8-N-1).
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

    uint32_t *Test_Addr = (uint32_t *)0x40048024U;  // 0x40047000U + 0x1024U
    uint32_t num = 0;
    uint32_t num_in = 0;
    printf("Contents of Register with address %p = %lu\n", Test_Addr, *Test_Addr);
    while(1)
    {

        led = 0;
        num = 0;
        num_in = 0;
        // ThisThread::sleep_for(1s);
        while (serial_port.readable()) {
            led = 1;
            ThisThread::sleep_for(1s); // Needed to get full input 
            if ((num_in = serial_port.read(buf, sizeof(buf)))) {
                num = num_in + 1;

            }
        }
        // uint32_t *Address;
        // for(int x = 0; x < 4; x++){
        //     uint32_t byte = buf[x];
        //     Address |= (uint32_t *)(byte << (8*(3-x)));
        // }

        if(num > 1){
            buf[num-1] = '\n';
            buf[num-2] = num_in;
            // (*buf) = *Address;
            serial_port.write(buf, num);
        }
        // printf("Address = %p Contents = %lu\n", Address, *Address);
    }
    
}

