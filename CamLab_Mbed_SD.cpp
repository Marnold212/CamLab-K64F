#include "CamLab_Mbed_SD.h"

CamLab_Mbed_SD::CamLab_Mbed_SD(SDBlockDevice *sd_instance, FATFileSystem *fileSystem) : _sd(sd_instance), _fileSystem(fileSystem){
    SD_Status = SD_Card_Init(_sd);
    if(SD_Status){
        _Mount_SD_Card(_sd, _fileSystem);
        chunk_size = _sd->get_read_size();
    }
};

bool CamLab_Mbed_SD::SD_Card_Init(SDBlockDevice *&sd_instance){
    // int SD_Status = SDHC->PROCTL |= SDHC_PROCTL_CDTL_MASK; // Try this after (initialising clocks) 
    int SD_Status = sd_instance->init();
    if(SD_Status == BD_ERROR_OK){
        return true;
    }else{
        printf("Error initialising SD card - check SD card inserted correctly\n");
        return false;
    }
}

void CamLab_Mbed_SD::_Mount_SD_Card(SDBlockDevice *sd_instance, FATFileSystem *file_system){
    // Mount SD file system 
    int err = file_system->mount(sd_instance);
    if (err) {
        // Reformat if we can't mount the filesystem
        // this should only happen on the first boot
        fflush(stdout);
        file_system->reformat(sd_instance);
        printf("SD Card Reformated\n");
    }
}

void CamLab_Mbed_SD::Check_Buffer_Size_Sufficient(uint32_t sd_buffer_size, uint32_t SD_Read_Bytes_Per_Sample = 0, uint32_t SD_Write_Bytes_Per_Sample = 0){
    if(SD_Status){ 
        if(chunk_size > sd_buffer_size){
            SD_Status = false;
            printf("%lu byte SD Buffer too small to contain reading a complete chunk of %lu bytes\n", sd_buffer_size, chunk_size); 
        }else if(SD_Read_Bytes_Per_Sample > sd_buffer_size){ // Reading 1 chunk cannot provide sufficient data required for sample
            SD_Status = false;
            printf("%lu read bytes per sample exceeds max of %lu bytes per chunk\n", SD_Read_Bytes_Per_Sample, chunk_size); 
        }else if(SD_Write_Bytes_Per_Sample > sd_buffer_size){ // Too much data from a sample to store in 1 write. 
            SD_Status = false;
            printf("%lu write bytes per sample exceeds max of %lu bytes per chunk\n", SD_Write_Bytes_Per_Sample, chunk_size); 
        }
    }
}

void CamLab_Mbed_SD::Print_SD_Card_Properties(){
    printf("sd size: %llu\n",         _sd->size());
    printf("sd read size: %llu\n",    _sd->get_read_size());
    printf("sd program size: %llu\n", _sd->get_program_size());
    printf("sd erase size: %llu\n",   _sd->get_erase_size());
}

void CamLab_Mbed_SD::Print_SD_Directory(){
    fflush(stdout);
 
    dir_ptr = opendir("/fs/");
 
    while (true) {
        struct dirent*  e = readdir(dir_ptr);
        if (!e) {
            break;
        }
 
        printf("    %s\n", e->d_name);
    }
}


void CamLab_Mbed_SD::Open_SD_Write_File(){
    if(SD_Status){ // Check Valid SD card initialised 
        _Create_SD_Raw_Data_File(f_sdWrite);
    }
}

void CamLab_Mbed_SD::_Create_SD_Raw_Data_File(FILE *&write_file){ // Reference to pointer - since we need to return updated pointer to file 
    fflush(stdout);
    write_file = fopen(SD_WRITE_FILE_NAME, "w+");  // w+ creates EMPTY file for both reading and writing
    if(write_file == nullptr){
        printf("Error Creating Raw Data File\n");
        SD_Status = false; 
    }
    // fprintf(file, "Test Document for Storing Raw Data:\n");
    // fprintf(f, "Format of Raw Data for Reading: 8 channels, uint16t, no EOL characters\n");
    // fprintf(f, "For reading results - Use the Raw_SD_Read.py program found in CamLab-K64F/K64FInterfacingTesting/ and select correct Number of channels and data type \n");
}

void CamLab_Mbed_SD::Open_SD_Read_File(){
    if(SD_Status){ // Check Valid SD card initialised 
        _Open_SD_Read_File(f_sdRead);
    }
}

void CamLab_Mbed_SD::_Open_SD_Read_File(FILE *&read_file){
    fflush(stdout);
    read_file = fopen(SD_READ_FILE_NAME, "r");  // w+ creates EMPTY file for both reading and writing 
    if(read_file == nullptr){
        printf("Error Opening Read Data File\n");
        SD_Status = false; 
    }
}

void CamLab_Mbed_SD::Seek_SD_File(FILE *file, long int offset_bytes, int origin){
    fseek(file, offset_bytes, origin);
}

void CamLab_Mbed_SD::Read_From_SD(void *target_buf, uint32_t size_element, uint32_t num_elements){
    if(SD_Status){
        _Read_SD_File(target_buf, size_element, num_elements, f_sdRead); 
    }
}

// Note that Read and Write operatinos move the pointer to position within file forward, so next read will read next chunkof data 
// Be aware of memory alignment - blocks of 512 - probably don't want to overlap chunks
void CamLab_Mbed_SD::_Read_SD_File(void *buf, uint32_t size_element, uint32_t num_elements, FILE *read_file){
    uint32_t check_return = fread(buf, size_element, num_elements, read_file);
    if(check_return != num_elements){ // Check Return value for any error 
        SD_Status = false;
    }
}

void CamLab_Mbed_SD::Write_to_SD(void *source_buf, uint32_t size_element, uint32_t num_elements){
    if(SD_Status){
        _Write_SD_Raw_Data_File(source_buf, size_element, num_elements, f_sdWrite);
    }
}

void CamLab_Mbed_SD::_Write_SD_Raw_Data_File(void *buf, uint32_t size_element, uint32_t num_elements, FILE *write_file){
    uint32_t check_return = fwrite(buf, size_element, num_elements, write_file);
    if(check_return != num_elements){ // Check Return value for any error 
        SD_Status = false;
    }
}

void CamLab_Mbed_SD::Flush_SD_Write_File(){
    fflush(f_sdWrite);
    fsync(fileno(f_sdWrite)); 
}

void CamLab_Mbed_SD::Close_File(FILE *&file){
    fflush(stdout);
    fclose(file);
}
    
void CamLab_Mbed_SD::Dismount_SD_Card(FATFileSystem *file_system){
    fflush(stdout);
    int err1 = file_system->unmount();
    if(err1){
        printf("Error Dismounting SD Card\n"); 
    }
}