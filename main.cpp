#include "mbed.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE                                                  32
// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(USBTX, USBRX);

// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = "{0}";

char test_buf[4] = {"0"};
uint32_t test_val = 0x12345;

int main(){
    *(uint32_t *)(test_buf) = __REV(test_val);
    for(int x = 0; x < 4; x++){    
        printf("[%i] = %x ; ", x, test_buf[x]);
    }
    DigitalOut led(LED1);
    // Set desired properties (9600-8-N-1).
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

    // uint32_t *Test_Addr = (uint32_t *)0x40048024U;  // 0x40047000U + 0x1024U
    uint32_t *Test_Addr = (uint32_t *)0x40048024U;  // 0x40047000U + 0x1024U
    uint32_t num = 0;
    uint32_t num_in = 0;

    char address_arr[4] = {0}; // Register to store up to a 32 bit value

    printf("\nContents of Register with address %p = %lu\n", Test_Addr, *Test_Addr);
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
                num = num_in + 1; // 0 index num_in to num 

            }
        }
        
        if(num > 1){
            /*
            for(int x = 0; x < 4; x++){
                address_arr[x] = buf[x];
            }
            uint32_t reg_data = (*(uint32_t *)(address_arr)).format(16);   // Can skip this step for speed 
            // reg_data = strtol(&reg_data, reg_data, 16);
            // // NEED to Cast reg_data into 0x hex form before next step 
            // // uint32_t *pointr = (uint32_t *)(reg_data);
            *(uint32_t *)(buf) = *(uint32_t *)reg_data;

            // *(uint32_t *)(buf) = *(uint32_t *)*(uint32_t *)(address_arr);
            */

           // uint32_t Address = __REV(*(uint32_t *)(buf))
           // uint32_t val = *(uint32_t *)(Address) 
           // *(uint32_t *)(buf) = __REV(val)

            *(uint32_t *)(buf) = __REV(*(uint32_t *)__REV(*(uint32_t *)(buf)))


            buf[num-1] = '\n';
            buf[num-2] = num_in;
            // (*buf) = *Address;
            serial_port.write(buf, num);
            // while(1){
            //     printf("Requested = 40048024(hex)   Contecnts =%lu \n", *(uint32_t *)(40048024));
            //     // printf("Address = %p Contents = %lu\n", *(uint32_t *)(address_arr), reg_data);
            //     printf("Address = %lu Contents ", *(uint32_t *)(address_arr));
            // }
        }
        // printf("Address = %p Contents = %lu\n", Address, *Address);
        
    }
    
}


/*  
#include <iostream>
#include <string>
#include <typeinfo>

using std::cout;

// Testing Using things such as 3 or 2 to shift 11 or 10 to specific places 

# define MAX_BUF_LENGTH       32
char buf[32] = {0};

int main(){
    // uint32_t uint_32_input = 12345;
    uint32_t uint_32_input = 0x12345;
    uint32_t offset = 0x0;

    *(uint32_t *)(buf + offset) = uint_32_input;
    printf("unisigned input in decimal: %u ; hex: %x \n", uint_32_input, uint_32_input);

    uint_32_input = 0x67890;
    offset = 0x04;
    *(uint32_t *)(buf + offset) = uint_32_input;
    
    for(int x = 0; x < 8; x++){    
        
        printf("[%i] = %x ; ", x, buf[x]);
    }
    uint32_t uint_32_output = *(uint32_t *)(buf);
    printf("\nunisigned input in decimal: %u ; hex: %x \n", uint_32_output, uint_32_output);
    return 0;
}

*/