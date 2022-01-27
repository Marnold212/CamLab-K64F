#ifndef CAMLAB_MBED_AD7606_H
#define CAMLAB_MBED_AD7606_H

#include "mbed.h"

/**
 * @brief The OS[2:0] pins set the oversampling - hardware average. 
 * Newer versions of the AD7606 have a software mode which can be accessed by writing all 1s to OS pins. 
 * OS = 000 corresponds to full speed operation, single reading returned from AD7606.
 * If overampling is used, the value returned is more accurate, but takes longer time - see datasheet for details. 
 * 
 */
enum AD7606_Oversampling{
    ad7606_hw_avg_1 = 0,
    ad7606_hw_avg_2 = 1,
    ad7606_hw_avg_4 = 2,
    ad7606_hw_avg_8 = 3,
    ad7606_hw_avg_16 = 4,
    ad7606_hw_avg_32 = 5,
    ad7606_hw_avg_64 = 6,
    ad7606_software_mode = 7 // On AD7606B / AD7606C this config sets software mode 
};

/**
 * @brief Simple class for using the AD7606 ADC in 16-bit parallel mode. Note the 10k resistor must be in the R2 position on the breakout board.
 * Currently data pins defined within class, with control pins passed as arguments. Therefore only 1 instance of this class should exist in current form.
 * 
 */
class CamLab_Mbed_AD7606 // Parallel 16 bit mode
{
    private:

        DigitalOut _RAGE;
        DigitalOut _RD;
        DigitalOut _CVA;
        DigitalOut _CVB;
        DigitalIn _BUSY;
        DigitalIn _FRST;
        // DigitalOut _CS; // Wired to RD in Parallel mode
        DigitalOut _RST;
        // DigitalOut _OS0;
        // DigitalOut _OS1;
        // DigitalOut _OS2;

        // Could pass pins as argument but want to avoid too many arguamnets to constructor
        PinName AD7606_DB_pins[16] = {D0, PTB18, D1, PTB19, D2, PTC1, D3, PTC8, D4, PTC9, D5, PTC0, D6, PTC7, D7, PTC5}; // DB0 - DB15  (LSB on Left, MSB on Right)
        BusOut _OS;
        BusIn _DB;

        uint32_t max_Sample_Rate = 200000;  // SPS: True for current version of AD7606
        uint32_t sample_time_us = 1000000 / max_Sample_Rate;    // If no Oversampling - Sample duration in microseconds

    public: 

    // Constructor 
    CamLab_Mbed_AD7606(PinName Range, PinName rd, PinName ConvStA, PinName ConvStB, PinName Busy, PinName FirstData, PinName Reset, PinName OS0, PinName OS1, PinName OS2);

    void reset(void);

    // Note that AD7606 uses 2's compliment 
    /**
     * @brief Starts new ADC sample, then reads a given number of 16-bit channels into a block of memory starting at address specified. 
     * Size of memory written to is dependant on number of channels limited by AD7606 max resolution and number of channels. 
     * 
     * @param rawDataBuffer 
     * @param Channels 
     */
    void Read_Raw(uint16_t *rawDataBuffer, int Channels);

    /**
     * @brief Start new sample - currently assmues the two CONVST pins are physically wired together. Therefore samples all 8 channels simultaneously   
     * 
     */
    void NewSample(void);

    /**
     * @brief If the provided argument is valid, the oversampling pins OS[2:0] are set accordingly. 
     * If the requested configuration is invalid, the value is unchanged. 
     * The value returned is an integer corresponding to OS[2:0], so if the requested config was invalid, it will simply return the previous configuration. 
     * Need to be careful - OS[2:0] = 111 corresponds to software mode for the AD7606B/AD7606C, but is invali for AD7606-8 e.g. breakout board. 
     * Also updates the sample time to the correct value for this new oversampling configuration. 
     * 
     * @param OS_Configuration An integer corresponding to desired value of OS[2:0], therefore should be integer in range 0-7 (0-6 if AD7606 has no software mode)
     * @return int Returns final config of OS[2:0] in integer form (0-7) regardless of if the function changed anything or not. 
     */
    int Set_Oversampling(int OS_Configuration);
};




#endif