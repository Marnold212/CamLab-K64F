#ifndef CAMLAB_MBED_AD7606_H
#define CAMLAB_MBED_AD7606_H

#include "mbed.h"

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
        DigitalOut _OS0;
        DigitalOut _OS1;
        DigitalOut _OS2;

        // Could pass pins as argument but want to avoid too many arguamnets to constructor
        PinName AD7606_DB_pins[16] = {D0, PTB18, D1, PTB19, D2, PTC1, D3, PTC8, PTC12, PTC9, PTC4, PTC0, PTD0, PTC7, PTD2, PTC5}; // DB0 - DB15  (LSB on Left, MSB on Right)
        BusIn _DB;


        
    public: 

    CamLab_Mbed_AD7606(PinName Range, PinName rd, PinName ConvStA, PinName ConvStB, PinName Busy, PinName FirstData, PinName Reset, PinName OS0, PinName OS1, PinName OS2);

    void reset(void);

    // Note that AD7606 uses 2's compliment 
    void Read_Raw(uint16_t *rawDataBuffer, int Channels);

    void NewSample(void);
};




#endif