#include "lfs.h"

// void fdc_init(uint8_t *buffer);
// uint8_t fdc_status(void);

// uint8_t fdc_command_read();

// uint8_t fdc_mount(uint8_t driveno,lfs_t lfs,lfs_file_t file);
// uint8_t fdc_unmount(uint8_t driveno);


void fdc_init(void);
void fdc_command_write(uint8_t data);

uint8_t fdc_read_status(void);
uint8_t fdc_read_track(void);
uint8_t fdc_read_sector(void); 
uint8_t fdc_read_control1(void); 
uint8_t fdc_read_control2(void);
uint8_t fdc_read_status_flag(void);
uint8_t fdc_read(void);

void fdc_write_track(uint8_t data);
void fdc_write_sector(uint8_t data);
void fdc_write_control1(uint8_t data);
void fdc_write_control2(uint8_t data);
void fdc_write(uint8_t data);

void fdc_check(uint8_t driveno);
uint8_t fdc_find_sector(void);

extern lfs_t lfs_handler;
extern lfs_file_t fd_drive[4];
extern uint8_t fd_drive_status[4];
extern uint32_t fdc_dma_datasize;