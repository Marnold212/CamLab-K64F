#ifndef CAMLAB_MBED_LOOP_FUNCTIONS_H
#define CAMLAB_MBED_LOOP_FUNCTIONS_H

#include "mbed.h"

volatile bool Experiment_Complete = false; 
volatile bool Sample_Ready = false;
volatile bool Sample_Running = false; 
volatile bool Sample_error = false;

int ADC_Resolution = 16;
int ADC_max_voltage = 5;
int ADC_min_voltage = -5;
float ADC_q_step = (ADC_max_voltage - ADC_min_voltage) / pow(2, ADC_Resolution);

static void pit0_isr(void)
{
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
    // PIT_StopTimer(PIT, kPIT_Chnl_0);
    if(!Sample_Running){
        Sample_Ready = true; 
    }else{
        Sample_error = true;
    }
	// *ptrval = !(*ptrval);
}

/** Initialize the high frequency ticker
 *   mbed-os\targets\TARGET_Freescale\TARGET_MCUXpresso_MCUS\TARGET_MCU_K64F\drivers\fsl_pit.c
 */
void Custom_PIT_Init(void)
{
    /* Structure to initialize PIT. */
    pit_config_t pitConfig;

    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    
	// PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // Enable interrupt and enable timer
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    
    // PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
    NVIC_SetVector(PIT0_IRQn, (uint32_t) pit0_isr);
    NVIC_EnableIRQ(PIT0_IRQn);
}

void Custom_PIT_Start(uint32_t frequency_hz){
    /* Common for ticker/timer. */
    uint32_t busClock;
    busClock = CLOCK_GetFreq(kCLOCK_BusClk);

    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, busClock/frequency_hz - 1);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

void Custom_PIT_Close(){
    PIT_StopTimer(PIT, kPIT_Chnl_0);
    PIT_Deinit(PIT); 
}

void Configure_Loop_Serial(BufferedSerial *serial_handle, uint32_t baudrate){
    serial_handle->set_baud(baudrate); 
    serial_handle->set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
}



/**
 * @brief Convert a float Duty Cycle (0.0 - 1.0) to a uint percentage (000 - 100) to reduce size. Conversion always rounds down. 
 * 
 * @param decimal Input 32-bit float value between 0.0 and 1.0 inclusive
 * @return uint8_t Equivalent value represented as 0 - 100 inclusive 
 */
uint8_t Duty_Float_To_Dec(float decimal){
    return (uint8_t)(decimal * 100.0f);   
}

void Raw_ADC_To_Voltage(uint16_t *source_arr, float *dest_arr, int Num_Channels){
    for(int x = 0; x < Num_Channels; x++){
        // *(dest_addr + x)
        dest_arr[x] = (int16_t)(source_arr[x]) *  ADC_q_step;
    }
    // Voltages[x] = (int16_t)(Dummy_ADC_Results[x]) * q_step; 
}

void Compress_PWM_Duty(float *source_arr, uint8_t *dest_arr, int Num_Channels){
    for(int x = 0; x < Num_Channels; x++){
        dest_arr[x] = Duty_Float_To_Dec(source_arr[x]); 
    }
}


void Copy_Data_To_Buffer(void *dest_buf_base, uint32_t &dest_buf_offset, void *source_buf, uint32_t Num_Bytes){
    memcpy(dest_buf_base + dest_buf_offset, source_buf, Num_Bytes);
    dest_buf_offset += Num_Bytes;
} 


#endif








// #include "mbed.h"
// #include "CamLab_Mbed_Serial.h"
// #include "CamLab_Mbed_AD7606.h"

// #include "CamLab_Mbed_Loop_Functions.h"

// using namespace std;

// /****************************************************/

// /* Communication Definitions and Setup */

// BufferedSerial  serial_port(USBTX, USBRX);

// #define COMM_BUFFER_SIZE                32
// uint32_t Comm_Write_Bytes_Per_Sample = 4 + 16; 

// uint8_t Comm_Buffer[COMM_BUFFER_SIZE];
// uint32_t Comm_Buffer_Offset = 0;

// /*****************************************/


// static DigitalOut led(LED_RED);


// /* No Longer Needed 
// void Save_Secs_Elapsed_to_Mem(void *buf_base, uint32_t &buffer_offset){
//     *(uint32_t *)(buf_base + buffer_offset) = *(uint32_t *)(RTC_BASE);
//     buffer_offset += 4; 
//     // *buf = RTC->TSR; // Equivalent Statement
// }
// */

// uint16_t Dummy_ADC_Results[8] = {0, 12567, 56000, 123, 3456, 409, 167, 65432};
// float Voltages[8];


// float Dummy_PWM_Duty[6] = {0.0f, 0.1234, 0.87, 1.0, 0.5, 0.879};
// uint8_t Compressed_PWM[6] = {0}; 


// int main(){

//     // serial_port.set_baud(921600); 
//     // serial_port.set_baud(576000); 
//     Configure_Loop_Serial(&serial_port, 230400); 

//     set_time(0);  // Enable RTC

//     Custom_PIT_Init();
//     Custom_PIT_Start(100);

//     led = 1;
//     while(!Experiment_Complete && !Sample_error){
//         if(Sample_Ready){
//             Sample_Ready = false; 
//             Sample_Running = true;

//             /* Start ADC Conversion */
//             //////////////////////////
            
//             /* Read/Calculate Actuator Inputs from SD */
//             ////////////////////

//             /* Read ADC Results */ 
//             /////////////////////////

//             /* Conversion/Control */
            
//             // Convert ADC Readings in uint16_t Source Array and save in Float destination Array
//             Raw_ADC_To_Voltage(Dummy_ADC_Results, Voltages, 8); 

//             // Control + Output Stuff ///////////////////////////////////

//             // Convert Calculated PWM Duty Cycles in Float Source Array and save in uint8_t Destination Array 
//             Compress_PWM_Duty(Dummy_PWM_Duty, Compressed_PWM, 6); 
            
            
//             /////////////////////////

            
//             /* Handle Communication */ 
//             // Save_Secs_Elapsed_to_Mem(Comm_Buffer, Comm_Buffer_Offset);
//             Copy_Data_To_Buffer(Comm_Buffer, Comm_Buffer_Offset, (uint32_t *)(RTC_BASE), sizeof(uint32_t)); 
            
//             // memcpy(Comm_Buffer+Comm_Buffer_Offset, Dummy_ADC_Results, 16);
//             // Comm_Buffer_Offset += 16; 
//             Copy_Data_To_Buffer(Comm_Buffer, Comm_Buffer_Offset, Dummy_ADC_Results, 8*sizeof(uint16_t)); 

//             // memcpy(Comm_Buffer+Comm_Buffer_Offset, Compressed_PWM, 6);
//             // Comm_Buffer_Offset += 6; 
//             Copy_Data_To_Buffer(Comm_Buffer, Comm_Buffer_Offset, Compressed_PWM, 6*sizeof(uint8_t));

//             /* Check space available in Serial Buffer */ 
//             if(Comm_Buffer_Offset > COMM_BUFFER_SIZE){
//                 printf("Error with Serial Data Buffer \n");
//                 break;
//             }else if(Comm_Buffer_Offset > (COMM_BUFFER_SIZE - Comm_Write_Bytes_Per_Sample)){
//                 // *(Comm_Buffer + Comm_Buffer_Offset) = '\n';
//                 serial_port.write(Comm_Buffer, COMM_BUFFER_SIZE);
//                 Comm_Buffer_Offset = 0;
//             }

//             // wait_us(50); 
//             /////////////////////////////////////////////

//             // printf("Testing\n");

//             Sample_Running = false;
//         }
//     }
//     Custom_PIT_Close();
//     printf("Error in Sample Structure!!\n");
//     led = 0; 
// }

