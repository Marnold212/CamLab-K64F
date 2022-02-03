#include "CamLab_Mbed_SD.h"

bool SD_Card_Init(SDBlockDevice *&sd_instance){
    int SD_Status = sd->init();
    if(SD_Status == BD_ERROR_OK){
        return true;
    }else{
        printf("Error initialising SD card - check SD card inserted correctly\n");
        return false;
    }
}

void Mount_SD_Card(SDBlockDevice *sd_instance, FATFileSystem *file_system){
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

void Print_SD_Card_Properties(SDBlockDevice *sd_instance){
    printf("sd size: %llu\n",         sd_instance->size());
    printf("sd read size: %llu\n",    sd_instance->get_read_size());
    printf("sd program size: %llu\n", sd_instance->get_program_size());
    printf("sd erase size: %llu\n",   sd_instance->get_erase_size());
}

void Print_SD_Directory(DIR *&dir){ // Reference to pointer - since we need to return updated pointer to file 
    fflush(stdout);
 
    dir = opendir("/fs/");
 
    while (true) {
        struct dirent*  e = readdir(dir);
        if (!e) {
            break;
        }
 
        printf("    %s\n", e->d_name);
    }
}

void Create_SD_Raw_Data_File(FILE *&write_file){ // Reference to pointer - since we need to return updated pointer to file 
    fflush(stdout);
    write_file = fopen(SD_WRITE_FILE_NAME, "w+");  // w+ creates EMPTY file for both reading and writing 
    if(write_file == nullptr){
        printf("Error Creating Raw Data File\n");
    }
    // fprintf(file, "Test Document for Storing Raw Data:\n");
    // fprintf(f, "Format of Raw Data for Reading: 8 channels, uint16t, no EOL characters\n");
    // fprintf(f, "For reading results - Use the Raw_SD_Read.py program found in CamLab-K64F/K64FInterfacingTesting/ and select correct Number of channels and data type \n");
}

void Open_SD_Read_File(FILE *&read_file){
    fflush(stdout);
    read_file = fopen(SD_READ_FILE_NAME, "r");  // w+ creates EMPTY file for both reading and writing 
    if(read_file == nullptr){
        printf("Error Opening Read Data File\n");
    }
}

void Seek_SD_File(FILE *file, long int offset_bytes, int origin){
    fseek(file, offset_bytes, origin);
}

// Note that Read and Write operatinos move the pointer to position within file forward, so next read will read next chunkof data 
// Be aware of memory alignment - blocks of 512 - probably don't want to overlap chunks
void Read_SD_File(uint32_t *buf, uint32_t size_element, uint32_t num_elements, FILE *read_file){
    // Could check return value for errors     
    fread(buf, size_element, num_elements, read_file);
}

void Write_SD_Raw_Data_File(uint32_t *buf, uint32_t size_element, uint32_t num_elements, FILE *write_file){
    // Could check return value for errors 
    fwrite(buf, size_element, num_elements, write_file);
}

void Close_File(FILE *&file){
    fflush(stdout);
    fclose(file);
}
    
void Dismount_SD_Card(FATFileSystem *file_system){
    fflush(stdout);
    int err1 = file_system->unmount();
    if(err1){
        printf("Error Dismounting SD Card\n"); 
    }
}