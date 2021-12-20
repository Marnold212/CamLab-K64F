#include "CamLab_Mbed_AD7606.h"
#include "mbed.h"

CamLab_Mbed_AD7606::CamLab_Mbed_AD7606(PinName Range, PinName rd, PinName ConvStA, PinName ConvStB, PinName Busy, PinName FirstData, PinName Reset, PinName OS0 = NC, PinName OS1 = NC, PinName OS2 = NC) : _RAGE(Range), _RD(rd), _CVA(ConvStA), _CVB(ConvStB), _BUSY(Busy), _FRST(FirstData), _RST(Reset), _OS(OS0, OS1, OS2), _DB(AD7606_DB_pins) {
    _RD = 1; // Wired to CS for 16 parallel operation
    _CVA = 1; // Could wire to CVB, but leave option of seperate sampling? 
    if(ConvStB != NC){
        _CVB = 1;
    }
    if(Reset != NC){
        _RST = 0; // Seems trying to write to DigitalOut(NC) crashes device? - shouldnt
    }
    if(Range != NC){
        _RAGE = 0; // Range: LOW = +/-5V ; High = +/-10V 
    }
    if(OS0 != NC && OS1 != NC && OS2 != NC){
        _OS = 0; // Set the Oversampling settings on AD7606
        // Note that OS = 111 is an invalid setting on breakout board - sets software mode on newer versinos of AD7606
        
    }
    if(Reset != NC){
        reset();
    }

    
};

void CamLab_Mbed_AD7606::reset(void){
    _RST = 1; // Pulse Reset Output Pin 
    _RST = 0;
}

void CamLab_Mbed_AD7606::NewSample(void){
    _CVA = 0;
    // _CVB = 0;
    _CVA = 1;
    // _CVB = 1;
}

void CamLab_Mbed_AD7606::Read_Raw(uint16_t *rawDataBuffer, int Channels = 8){
    NewSample();
    wait_us(10); // Depending on Oversampling settings - ideally use busy pin but forums claim it doesnt work 
    // while(_BUSY){
    //     // Wait for Conversion to Complete
    // }

    for (int i = 0; i < Channels; i++)
    {
        _RD = 0; // Command Reading onto DB[15:0] pins, 8 times for each channel in sequence
        *(uint16_t *)(rawDataBuffer + i) = (uint16_t)(_DB.read()); // Data is output using 2's compliment
        _RD = 1;
    }
    
}

// Couldnt think of a better implementation for checking for valid combinations
// Could instead use: & 0x7 to only take 3 least significant bits 
int CamLab_Mbed_AD7606::Set_Oversampling(int OS_Configuration){
    switch(OS_Configuration){
        case(ad7606_hw_avg_1):
            _OS = OS_Configuration; 
            break;
        case(ad7606_hw_avg_2):
            _OS = OS_Configuration; 
            break;
        case(ad7606_hw_avg_4):
            _OS = OS_Configuration; 
            break;
        case(ad7606_hw_avg_8):
            _OS = OS_Configuration; 
            break;
        case(ad7606_hw_avg_16):
            _OS = OS_Configuration; 
            break;
        case(ad7606_hw_avg_32):
            _OS = OS_Configuration; 
            break;
        case(ad7606_hw_avg_64):
            _OS = OS_Configuration; 
            break;
        case(ad7606_software_mode):
            _OS = OS_Configuration; 
            break;
    }
    return (_OS.read()); // Returns the value of OS regardless of if it changed or not 
}