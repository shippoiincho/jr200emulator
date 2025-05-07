#include "fdc.h"

// MB8877 Emulation

uint8_t fdc_status;
uint8_t fd_track[4];
uint8_t fdc_command;

uint8_t fdc_track_register;     // 0x9
uint8_t fdc_sector_register;    // 0xa
uint8_t fdc_data_register;      // 0xb
uint8_t fdc_control1;           // 0xc
uint8_t fdc_control2;           // 0xd
uint8_t fdc_status_flag;        // 0xf

uint8_t fd_seek_dir;
uint32_t fd_sector_size;
uint32_t fd_sector_bytes;

lfs_t lfs_handler;
lfs_file_t fd_drive[4];
uint8_t fd_drive_status[4];
uint8_t fd_image_status[4];     // for MSX  0:2DD 720KB 1:1DD 360KB 2:2DD 640KB 3:1DD:320KB

//----------------------------------------------------------------------------------------------------

void fdc_command_write(uint8_t param) {

    uint8_t command,res,driveno;

    fdc_command=param;

    command=param>>4;

    // drive check ...

    driveno=fdc_control2&1;

//printf("[FDC:%x,%d]",command,driveno);

    if(fd_drive_status[driveno]==0) {
        fdc_status=0x80;
        fdc_status_flag=0x40;
        return;
    }

    if(driveno>2) {
        fdc_status=0x80;
        fdc_status_flag=0x40;
        return;
    }

    switch(command) {

        case 0:  // Restore

            fdc_track_register=0;
            fdc_data_register=0;

            fdc_status_flag=0x40;
            fd_track[driveno]=0;

            fdc_status=6;
            return;

        case 1:  // seek

            fdc_status_flag=0x40;

            fdc_track_register=fdc_data_register;

            fd_track[driveno]=fdc_track_register;

            if(fdc_track_register==0) {
                fdc_status=6;
            } else {
                fdc_status=0;
            }
            return;

        case 2:  // step
        case 4:  // step-in
        case 6:  // step-out

            fdc_status_flag=0x40;

            if(fdc_track_register==0) {
                fdc_status=6;
            } else {
                fdc_status=0;
            }
            fd_track[driveno]=fdc_track_register;
            return;

        case 3:  // step

            fdc_status_flag=0x40;

            if(fd_seek_dir==0) {

                res=fdc_track_register;

                if(res==80) {
                    fdc_status=0x10;
                } else { 
                    fdc_status=0;
                }
                fdc_track_register=res+1;
                fd_track[driveno]=fdc_track_register;
                return;

            } else {

               res=fdc_track_register;
        
                if(res==0) {
                    fdc_status=0x16;
                } else if(res==1) {
                    fdc_status=6;
                }

                fdc_track_register=res-1;
                fd_track[driveno]=fdc_track_register;

                fdc_status=0;
                return;

            }

            fdc_status=0;
            return;

        case 5:  // step-in

            fdc_status_flag=0x40;

            fd_seek_dir=0;
            res=fdc_track_register;

            if(res==80) {
                fdc_status=0x10;
            } else { 
                fdc_status=0;
            }
            fdc_track_register=res+1;
            fd_track[driveno]=fdc_track_register;
            return;

        case 7:  // step-out

            fdc_status_flag=0x40;

            fd_seek_dir=1;
            res=fdc_track_register;
        
            if(res==0) {
                fdc_status=0x16;
            } else if(res==1) {
                fdc_status=6;
            }

            fdc_track_register=res-1;
            fd_track[driveno]=fdc_track_register;
            fdc_status=0;
            return;

        case 8: // Read single sector

            fdc_status_flag=0x80;

            fdc_find_sector();
            fdc_status=3;

            return;

        case 9: // Read Multi sector

            fdc_status_flag=0x80;

            fdc_find_sector();
            fdc_status=3;
            return;

        case 0xa: // write single sector

            fdc_status_flag=0x80;
            fdc_find_sector();
            fdc_status=3;
            return;

        case 0xb: // write multi sector

            fdc_status_flag=0x80;
            fdc_find_sector();
            fdc_status=3;
            return;

        case 0xc: // Read address 

            // NOT SUPPORTED

            fdc_status_flag=0x40;

            fdc_status=0x80;
            return;

        case 0xd: // force reset

            if(fdc_command&0xf) {
                fdc_status_flag=0x40;
            } else {
                fdc_status_flag=0;
            }
            fdc_status=2;
            return;
        
        case 0xe:  // Read Track
        case 0xf:  // Write Track

            // NOT SUPPORTED
            fdc_status_flag=0x40;

            fdc_status=0x80;
            return;

        default:

            return;

    }

}

// check inserted disk

void fdc_check(uint8_t driveno) {

    uint8_t flags;
    uint32_t imagesize;

    if(lfs_file_seek(&lfs_handler,&fd_drive[driveno],0x1a,LFS_SEEK_SET)) {
        fd_drive_status[driveno]=0;
    }

    imagesize=lfs_file_size(&lfs_handler,&fd_drive[driveno]);

    switch(imagesize) {

        case 737280:    // 2DD 720KB
            fd_image_status[driveno]=0;
            break;

        case 368640:    // 1DD 360KB
            fd_image_status[driveno]=1;
            break;

        case 655360:    // 2DD 640KB
            fd_image_status[driveno]=2;


        case 327680:    // 1DD 320KB
            fd_image_status[driveno]=3;
            break;

        default:        // assume 2DD 720KB
            fd_image_status[driveno]=0;
    }

    fdc_status_flag=0;

//    lfs_file_read(&lfs_handler,&fd_drive[driveno],&flags,1);

    // if(flags==0) {
    //     fd_drive_status[driveno]=1;
    // } else {
        fd_drive_status[driveno]=3;        
    // }

}

// check first posision of sector

#if 0
// D88/D77 format image
uint8_t fdc_find_sector(void) {

    uint8_t driveno,track,sector,head,count;
    uint8_t sector_info[16];
    uint32_t sector_ptr;

    driveno=fdc_control2&1;
//    track=fdc_track_register;
    track=fd_track[driveno];
    sector=fdc_sector_register;
    head=fdc_control1&1;

    lfs_file_seek(&lfs_handler,&fd_drive[driveno],0x20,LFS_SEEK_SET);

    // find track top

    for(int i=0;i<=track*2;i++) {
        lfs_file_read(&lfs_handler,&fd_drive[driveno],&sector_ptr,4);
    }

    lfs_file_seek(&lfs_handler,&fd_drive[driveno],sector_ptr,LFS_SEEK_SET);

    while(1) {
        sector_ptr+=0x10;
        lfs_file_read(&lfs_handler,&fd_drive[driveno],sector_info,16);

        if((sector_info[2]==sector)&&(sector_info[1]==head)&&(sector_info[0]==track)) {
            if(sector_info[3]==0) {
                fd_sector_size=128;
            } else if(sector_info[3]==1) {
                fd_sector_size=256;
            } else if(sector_info[3]==2) {
                fd_sector_size=512;
            } else if(sector_info[3]==3) {
                fd_sector_size=1024;
            }
            break;
        }
        if(sector_info[3]==0) {
            lfs_file_seek(&lfs_handler,&fd_drive[driveno],128,LFS_SEEK_CUR);
            sector_ptr+=128;
        } else if(sector_info[3]==1) {
            lfs_file_seek(&lfs_handler,&fd_drive[driveno],256,LFS_SEEK_CUR);
            sector_ptr+=256;
        } else if(sector_info[3]==2) {
            lfs_file_seek(&lfs_handler,&fd_drive[driveno],512,LFS_SEEK_CUR);
            sector_ptr+=512;
        } else if(sector_info[3]==3) {
            lfs_file_seek(&lfs_handler,&fd_drive[driveno],1024,LFS_SEEK_CUR);
            sector_ptr+=1024;
        }

        count++;
        if(count>40) {

            return -1;

            break;
        } // error        
    }

    // fd_ptr=sector_ptr;
    fd_sector_bytes=0;

    return 0;

}
#endif

// MSX DSK image
uint8_t fdc_find_sector(void) {

    uint8_t driveno,track,sector,head,count;
    uint8_t sector_info[16];
    uint32_t sector_ptr;

    driveno=fdc_control2&1;
//    track=fdc_track_register;
    track=fd_track[driveno];
    sector=fdc_sector_register;
    head=fdc_control1&1;

    fd_sector_size=512;        // MSX use 512bytes/sector only

    switch(fd_image_status[driveno]) {
        case 0:
            sector_ptr=(track*18+head*9+sector-1)*512;    // 9 sectors/track
            break;
        case 1:
            sector_ptr=(track*9+sector-1)*512;
            break; 
        case 2:
            sector_ptr=(track*16+head*8+sector-1)*512;    // 8 sectors/track
            break;
        case 3:
            sector_ptr=(track*8+sector-1)*512;
            break; 
    }

//printf("[CHS:%d:%d:%d:%x]",track,head,sector,sector_ptr);

    lfs_file_seek(&lfs_handler,&fd_drive[driveno],sector_ptr,LFS_SEEK_SET);

    // // find track top

    // for(int i=0;i<=track*2;i++) {
    //     lfs_file_read(&lfs_handler,&fd_drive[driveno],&sector_ptr,4);
    // }

    // lfs_file_seek(&lfs_handler,&fd_drive[driveno],sector_ptr,LFS_SEEK_SET);

    // while(1) {
    //     sector_ptr+=0x10;
    //     lfs_file_read(&lfs_handler,&fd_drive[driveno],sector_info,16);

    //     if((sector_info[2]==sector)&&(sector_info[1]==head)&&(sector_info[0]==track)) {
    //         if(sector_info[3]==0) {
    //             fd_sector_size=128;
    //         } else if(sector_info[3]==1) {
    //             fd_sector_size=256;
    //         } else if(sector_info[3]==2) {
    //             fd_sector_size=512;
    //         } else if(sector_info[3]==3) {
    //             fd_sector_size=1024;
    //         }
    //         break;
    //     }
    //     if(sector_info[3]==0) {
    //         lfs_file_seek(&lfs_handler,&fd_drive[driveno],128,LFS_SEEK_CUR);
    //         sector_ptr+=128;
    //     } else if(sector_info[3]==1) {
    //         lfs_file_seek(&lfs_handler,&fd_drive[driveno],256,LFS_SEEK_CUR);
    //         sector_ptr+=256;
    //     } else if(sector_info[3]==2) {
    //         lfs_file_seek(&lfs_handler,&fd_drive[driveno],512,LFS_SEEK_CUR);
    //         sector_ptr+=512;
    //     } else if(sector_info[3]==3) {
    //         lfs_file_seek(&lfs_handler,&fd_drive[driveno],1024,LFS_SEEK_CUR);
    //         sector_ptr+=1024;
    //     }

    //     count++;
    //     if(count>40) {

    //         return -1;

    //         break;
    //     } // error        
    // }

    // fd_ptr=sector_ptr;
    fd_sector_bytes=0;

    return 0;

}


uint8_t fdc_read() {

    uint8_t data,driveno;

    driveno=fdc_control2&1;

    if(fd_drive_status[driveno]==0) return 0xff;
//    if(fdc_command==0) return 0xff;

    if((fdc_command&0xe0)!=0x80) {  // After Seek command
        return fd_track[driveno];
    }

    if(fd_sector_bytes==fd_sector_size) {
        fdc_status_flag=0x40;
        fdc_status=0;
    }

    lfs_file_read(&lfs_handler,&fd_drive[driveno],&data,1);

    fd_sector_bytes++;
    fdc_status_flag=0x80;

    // find next sector

    if(fd_sector_bytes==fd_sector_size) {

        if(fdc_command&0x10) {
            fdc_sector_register++;
            fdc_find_sector();

            fdc_status_flag=0x80;

        } else {

        fdc_status_flag=0x40;
        fdc_status=0;

        }
    }

    return data;

}

void fdc_write(uint8_t data) {

    uint8_t driveno;

    driveno=fdc_control2&1;

    if((fdc_command&0xe0)!=0xa0) {
        fdc_data_register=data;
        return;
    }

    if(fd_drive_status[driveno]!=1) { // Write protected
        fdc_status_flag=0x80;
        fdc_status=0x43;

        fd_sector_bytes++;
        
        if(fd_sector_bytes==fd_sector_size) {
            fdc_status_flag=0x40;
            fdc_status=0x40;
        }

        return;     
    }
    // if(fdc_command==0) return;

    if(fd_sector_bytes==fd_sector_size) {
        fdc_status_flag=0x40;
        fdc_status=0;
    }

    lfs_file_write(&lfs_handler,&fd_drive[driveno],&data,1);

    fd_sector_bytes++;
    fdc_status_flag=0x80;

    // find next sector

    if(fd_sector_bytes==fd_sector_size) {

        lfs_file_sync(&lfs_handler,&fd_drive[driveno]);

        if(fdc_command&0x10) {
            fdc_sector_register++;
            fdc_find_sector();

            fdc_status_flag=0x80;

        } else {

        fdc_status_flag=0x40;
        fdc_status=0;
        fdc_command=0;

        // mfd_irq=1;

        // if(mainioport[2]&0x40) {
        //     main_cpu.irq=true;
        // }

        }
    }

    return;

}

// Interface

void fdc_init(void) {
    fdc_status_flag=0;
    return;
}

uint8_t fdc_read_status(void) {
    uint8_t driveno;
    driveno=fdc_control1&1;
    if(fd_drive_status[driveno]!=1) {   // Write protected
        return fdc_status|0x40;
    }
    return fdc_status;
}

uint8_t fdc_read_track(void) {
    return fdc_track_register;
}

uint8_t fdc_read_sector(void) {
    return fdc_sector_register;
}

uint8_t fdc_read_control1(void) {
    return fdc_control1;
}

uint8_t fdc_read_control2(void) {
    return fdc_control2&0xfb;
}

uint8_t fdc_read_status_flag(void) {
    return (~fdc_status_flag)&0xc0;
}

void fdc_write_track(uint8_t data) {
    fdc_track_register=data;
    return;
}

void fdc_write_sector(uint8_t data) {
    fdc_sector_register=data;
    return;
}

void fdc_write_control1(uint8_t data) {
    fdc_control1=data;
    return;
}

void fdc_write_control2(uint8_t data) {
    fdc_control2=data;
    return;
}
