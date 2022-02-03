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
void Mount_SD_Card(SDBlockDevice *sd_instance, FATFileSystem *file_system);

void Print_SD_Card_Properties(SDBlockDevice *sd_instance);

/**
 * @brief Prints the contents of the file directory on the SD card to the default serial terminal. 
 * 
 * @param dir Pointer to Directory used for printing contents - pointer passed by reference in case it is to be used later - may wish to destroy Dir object if not using later. 
 */
void Print_SD_Directory(DIR *&dir);

/**
 * @brief Opens an SD file to write to - pointer to file stream passed in reference. File name to open defined in header file : SD_READ_WRITE_NAME. 
 * 
 * @param write_file Pointer to file stream used for writing data to a file on SD card. Pointer passed by reference so that the file stream is updated for future use. 
 */
void Create_SD_Raw_Data_File(FILE *&write_file);

/**
 * @brief Opens an SD file to read from - pointer to file stream passed in reference. File name to open defined in header file : SD_READ_FILE_NAME. 
 * 
 * @param read_file Pointer to file stream used for reading data from a file on SD card. Pointer passed by reference so that the file stream is updated for future use. 
 */
void Open_SD_Read_File(FILE *&read_file);

/**
 * @brief If we wish to move the position indicator of an open file to a new position. Works relative to an origin: either start/end of file or current position of file
 * 
 * @param file Pointer to open FILE stream 
 * @param offset Number of bytes to offset from chosen origin 
 * @param origin Base Address of File: SEEK_SET=Beginning of File; SEEK_CUR=Current position; SEEK_END=End of File(Untested - may not be implemente)
 */
void Seek_SD_File(FILE *file, long int offset_bytes, int origin);


/**
 * @brief Function to Read a block of data from an open file stream on SD card. Note the position of the file will update according to how many bytes are read, so next read operation will
 * operate on the next chunk of data. Be aware of memory alignment, e.g. blocks of 512 bytes, probably want to avoid reading over block boundaries. Also reading full blocks is fast. 
 * 
 * @param buf Target buffer to recieve data 
 * @param size_element Number of bytes of each element being read (e.g. 4 for a single precision float or 32 bit uint)
 * @param num_elements Number of elements of size: size_element to read to target buffer   
 * @param read_file Open file stream used to read data from 
 */
void Read_SD_File(uint32_t *buf, uint32_t size_element, uint32_t num_elements, FILE *read_file);

/**
 * @brief Function to Write a block of data from an open file stream on SD card. Note the position of the file will update according to how many bytes are written, so next write operation will
 * operate on the next chunk of data. Be aware of memory alignment, e.g. blocks of 512 bytes, probably want to avoid writing over block boundaries on single call. Also reading full blocks is fast. 
 * 
 * @param buf Target buffer to recieve data 
 * @param size_element Number of bytes of each element being read (e.g. 4 for a single precision float or 32 bit uint)
 * @param num_elements Number of elements of size: size_element to read to target buffer   
 * @param write_file Open file stream used to write data to 
 */
void Write_SD_Raw_Data_File(uint32_t *buf, uint32_t size_element, uint32_t num_elements, FILE *write_file);

void Close_File(FILE *&file);
    
void Dismount_SD_Card(FATFileSystem *file_system);

#endif