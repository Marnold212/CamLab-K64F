#ifndef CAMLAB_MBED_SD_H
#define CAMLAB_MBED_SD_H

#include "mbed.h"
#include <errno.h>
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

using namespace std;

#define SD_WRITE_FILE_NAME                      "/fs/Raw_Data.txt"
#define SD_READ_FILE_NAME                       "/fs/Test.txt"

/**
 * @brief Implements the Mbed SD card interface - uses the SPI interface rather than the faster SDIO? The SDHC block of the K64F doesn't appear to be used at all, which is likely limiting performance. 
 * 
 */
class CamLab_Mbed_SD
{
    
public:

    SDBlockDevice *_sd; 
    FATFileSystem *_fileSystem;

    DIR*    dir_ptr = nullptr; // SD Card Directory labelled d 
    FILE*   f_sdWrite = nullptr;  // File Used for Writing Results form Experiment
    FILE*   f_sdRead = nullptr;  // File Used to Read in Data for Control
    bool SD_Status = false; // Check for any errors with SD operation
    uint32_t chunk_size;  

    CamLab_Mbed_SD(SDBlockDevice *sd_instance, FATFileSystem *fileSystem);
    // ~CamLab_Mbed_SD();

    void Check_Buffer_Size_Sufficient(uint32_t sd_buffer_size, uint32_t SD_Read_Bytes_Per_Sample, uint32_t SD_Write_Bytes_Per_Sample);

    void Print_SD_Card_Properties();

    /**
     * @brief Prints the contents of the file directory on the SD card to the default serial terminal. 
     * 
     */
    void Print_SD_Directory();

    /**
     * @brief Public method to open a file to read data for control, name of file to open is defined in header file. Checks SD status is valid, and if there is an error opening file such as file not being present, it sets SD status as invalid. Opens file in Read-only mode "r". 
     * 
     */
    void Open_SD_Read_File();

    /**
     * @brief Public method to write data to a file, name of file to open is defined in header file. Checks SD status is valid, and if there is an error opening file sets SD status as invalid. Opens file in "w+" mode, which creates a blank version of file, replacing any exsting data. 
     * 
     */
    void Open_SD_Write_File();

    /**
     * @brief Public method for writing data to SD card. Note the position of the file will update according to how many bytes are written, so next write operation will operate on the next chunk of data.
     * Be aware of memory alignment, e.g. blocks of 512 bytes, probably want to avoid writing over block boundaries. Requires the write file to be open. Checks sd valid, and any errors will mark sd as invalid for future operations. 
     * 
     * @param source_buf Source buffer which contains data to be written to SD card.
     * @param size_element Number of bytes of each element being read (e.g. 4 for a single precision float or 32 bit uint)
     * @param num_elements Number of elements of size: size_element to read to target buffer 
     */
    void Write_to_SD(void *source_buf, uint32_t size_element, uint32_t num_elements);

    /**
     * @brief This method needs to be called to actualy save any writes performed, otherwise no data is actually preserved until the SD card Close operation is called. 
     * 
     */
    void Flush_SD_Write_File();

    /**
     * @brief Public method for writing data to SD card. Note the position of the file will update according to how many bytes are written, so next write operation will operate on the next chunk of data.
     * Be aware of memory alignment, e.g. blocks of 512 bytes, probably want to avoid writing over block boundaries. Requires the Read file to be open. Checks sd valid, and any errors will mark sd as invalid for future operations.
     * 
     * @param target_buf Target buffer which data from SD card will be written to. 
     * @param size_element Number of bytes of each element being read (e.g. 4 for a single precision float or 32 bit uint)
     * @param num_elements Number of elements of size: size_element to read to target buffer 
     */   
    void Read_From_SD(void *target_buf, uint32_t size_element, uint32_t num_elements);

    /**
     * @brief If we wish to move the position indicator of an open file to a new position. Works relative to an origin: either start/end of file or current position of file
     * 
     * @param file Pointer to open FILE stream 
     * @param offset Number of bytes to offset from chosen origin 
     * @param origin Base Address of File: SEEK_SET=Beginning of File; SEEK_CUR=Current position; SEEK_END=End of File(Untested - may not be implemente)
     */
    void Seek_SD_File(FILE *file, long int offset_bytes, int origin);

    void Close_File(FILE *&file);
        
    void Dismount_SD_Card(FATFileSystem *file_system);                                          

    void Close_SD(){
        Close_File(f_sdWrite);
        Close_File(f_sdRead);
        Dismount_SD_Card(_fileSystem); 
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

    /**
     * @brief Converts a Decimal Duty Cycle percentage (0 - 100) to a float value. Output has a 2 dcimal place resolution. 
     * 
     * @param input Input uint8_t value between 0 - 100 inclusive representing the percentage of duty cycle spent ON. 
     * @return float Equivalent value represented as 0.00 - 1.00 inclusive (2 decimal place resolution) 
     */
    float Duty_Dec_To_Float(uint8_t input){
        return (float)(input) / 100.0f; 
    }

private:

    /**
     * @brief Attempts to initalise sd Card - if it is unsuccessful such as no SD card inserted return false. This can take some time (<5s but usually much less?) due to timeout defined in some SD library somewhere. 
     * 
     * @param sd_instance Reference to pointer of sd card object
     * @return true if sd card initiallised correctly
     * @return false if unable to init SD card - most likely there is no SD card inserted 
     */
    bool SD_Card_Init(SDBlockDevice *&sd_instance);

    /**
     * @brief Function to Mount the SD Card to the file System - uses the Mbed SDBlockDevice library to initialise - note we use lower level functions later to write/read to SD card for greater performance. Check if SD card is present prior to performing this function. 
     * If the SD card does not have a FATFileSystem present, the SD card will be reformatted so that it does. 
     * 
     * @param sd_instance Pointer to SDBlockDevice object - note this object should use the physical pins for SD card.  
     * @param file_system Pointer to file System used - we are using the FATFileSystem 
     */
    void _Mount_SD_Card(SDBlockDevice *sd_instance, FATFileSystem *file_system);

        /**
     * @brief Opens an SD file to write to - pointer to file stream passed in reference. File name to open defined in header file : SD_READ_WRITE_NAME. 
     * 
     * @param write_file Pointer to file stream used for writing data to a file on SD card. Pointer passed by reference so that the file stream is updated for future use. 
     */
    void _Create_SD_Raw_Data_File(FILE *&write_file);

    /**
     * @brief Opens an SD file to read from - pointer to file stream passed in reference. File name to open defined in header file : SD_READ_FILE_NAME. 
     * 
     * @param read_file Pointer to file stream used for reading data from a file on SD card. Pointer passed by reference so that the file stream is updated for future use. 
     */
    void _Open_SD_Read_File(FILE *&read_file);

    /**
     * @brief Function to Read a block of data from an open file stream on SD card. Note the position of the file will update according to how many bytes are read, so next read operation will
     * operate on the next chunk of data. Be aware of memory alignment, e.g. blocks of 512 bytes, probably want to avoid reading over block boundaries. Also reading full blocks is fast. 
     * 
     * @param buf Target buffer to recieve data 
     * @param size_element Number of bytes of each element being read (e.g. 4 for a single precision float or 32 bit uint)
     * @param num_elements Number of elements of size: size_element to read to target buffer   
     * @param read_file Open file stream used to read data from 
     */
    void _Read_SD_File(void *buf, uint32_t size_element, uint32_t num_elements, FILE *read_file);

    /**
     * @brief Function to Write a block of data from an open file stream on SD card. Note the position of the file will update according to how many bytes are written, so next write operation will
     * operate on the next chunk of data. Be aware of memory alignment, e.g. blocks of 512 bytes, probably want to avoid writing over block boundaries on single call. Also reading full blocks is fast. 
     * 
     * @param buf Target buffer to recieve data 
     * @param size_element Number of bytes of each element being read (e.g. 4 for a single precision float or 32 bit uint)
     * @param num_elements Number of elements of size: size_element to read to target buffer   
     * @param write_file Open file stream used to write data to 
     */
    void _Write_SD_Raw_Data_File(void *buf, uint32_t size_element, uint32_t num_elements, FILE *write_file);

    

};





#endif