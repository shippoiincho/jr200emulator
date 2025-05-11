//  Matsushita JR-200 emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue0
//  GP3: Red0
//  GP4: Green0
//  GP6: Audio

#define HW_FLASH_STORAGE_MEGABYTES 2

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"

#include "tusb.h"
#include "bsp/board.h"
#include "hidparser/hidparser.h"

#include "vga16_graphics.h"

#include "cpuintrf.h"
#include "m6800.h"

#include "jr200keymap.h"

#include "jr200misc.h"
#include "jr200rom.h"

#include "lfs.h"

// VGAout configuration

#define DOTCLOCK 25000
#define CLOCKMUL 9

// Pico does not work at CLOCKMUL=7 (175MHz).

#define VGA_PIXELS_X 320
#define VGA_PIXELS_Y 200

#define VGA_CHARS_X 40
#define VGA_CHARS_Y 24

#define VRAM_PAGE_SIZE (VGA_PIXELS_X*VGA_PIXELS_Y/8)

extern unsigned char vga_data_array[];
volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline,vsync_scanline,total_scanline;

struct repeating_timer timer,timer2;

// PC configuration

uint32_t cpu_clocks=0;
uint32_t cpu_ei=0;
uint32_t cpu_cycles=0;
uint32_t cpu_hsync=0;

uint32_t cpu_trace=0;   // DEBUG
uint32_t cpu_boost=0;

uint32_t subcpu_ktest=0;

uint8_t mainram[0x10000];
//uint8_t ioport[0x100];
uint8_t via_reg[0x20];
uint16_t via_timercount[6];
uint16_t via_prescalecount[6];
uint16_t via_reload[6];

// Video
uint8_t menuram[0x1000];
uint8_t scandata[256];
uint8_t video_border;
uint8_t font[0x800];

uint32_t renderbuffer[81];  // 80x4

uint8_t timer_enable_irq=0;

volatile uint8_t redraw_flag=0;

uint8_t keymap[11];

volatile uint8_t keypressed=0;  //last pressed usbkeycode
uint32_t key_caps=0;            // Keyboard LED status
uint32_t key_kana=0;
uint32_t key_graph=0;        // Kana Keyboard select 

uint32_t key_irq=0;
uint32_t key_break_flag=0;
#define KEYBOARD_WAIT   32
uint32_t keyboard_cycles;
uint32_t joystick_count=0;      // SUBCPU command count
uint32_t key_repeat_flag=1;
uint32_t key_repeat_count=0;
uint32_t key_basic_flag=0;      // TYPE BASIC 
uint32_t key_basic_code=0;
uint32_t key_basic_bytes=0;

uint32_t lastmodifier=0;
uint32_t jr200keypressed=0;

// BEEP & SOUND

uint32_t beep_enable=0;
uint32_t pwm_slice_num;
volatile uint32_t sound_tick=0;

uint32_t psg_osc_interval[4];
uint32_t psg_osc_counter[4];
uint32_t psg_noteon[4];
//uint32_t psg_master_clock = 2000000;    // ???
uint16_t psg_master_volume = 0;

//#define SAMPLING_FREQ 44100    
#define SAMPLING_FREQ 22050
#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ) 

// Tape

uint32_t tape_ready=0;
uint32_t tape_ptr=0;
uint32_t tape_phase=0;
uint32_t tape_count=0;

uint32_t tape_read_wait=0;
uint32_t tape_leader=0;
uint32_t tape_autoclose=1;          // Default value of TAPE autoclose
uint32_t tape_skip=0;               // Default value of TAPE load accelaration
uint32_t tape_cycles;

const uint32_t tape_waits[] = { 270 ,1300 , 325 , 650 } ;  // Tape signal width

#define TAPE_WAIT_WIDTH 20

#define TAPE_THRESHOLD 2000000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

// UI

uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);
extern volatile uint8_t gamepad_info;
uint32_t gamepad_select=0;

uint32_t usbcheck_count=0;
uint32_t kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

// Define the flash sizes
// This is the setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
#define HW_SYSTEM_RESERVED  512     // 512KiB for System reserved
// Use the outcome of HW_FLASH_STORAGE_BYTES for creating the little file system
// Example calculation when using 2MB FLASH as defined in msxemulator.h ->  ((2 * 1024) - 512) * 1024 = 1572864
#define HW_FLASH_STORAGE_BYTES  (((HW_FLASH_STORAGE_MEGABYTES * 1024) - HW_SYSTEM_RESERVED) * 1024)
#define HW_FLASH_STORAGE_BASE   (1024*1024*HW_FLASH_STORAGE_MEGABYTES - HW_FLASH_STORAGE_BYTES) 

uint8_t __attribute__  ((aligned(sizeof(unsigned char *)*4096))) flash_buffer[4096];

lfs_t lfs;
lfs_file_t lfs_file,lfs_fd,lfs_cart1,lfs_cart2;

#define LFS_LS_FILES 12

volatile uint32_t load_enabled=0;
volatile uint32_t save_enabled=0;
//uint32_t file_cycle=0;

unsigned char filename[16];
unsigned char tape_filename[16];

static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);

void __not_in_flash_func(draw_framebuffer_menu)(uint32_t line);
void __not_in_flash_func(draw_framebuffer)(uint32_t line);

// *REAL* H-Sync for emulation
void __not_in_flash_func(hsync_handler)(void) {

    uint32_t vramindex;
    uint32_t tmsscan;
    uint8_t bgcolor;
    uint8_t *scandata;

    pio_interrupt_clear(pio0, 0);

    scandata=(uint8_t *)renderbuffer;

    total_scanline++;

    if((scanline!=0)&&(gpio_get(1)==0)) { // VSYNC
        scanline=0;
        video_vsync=1;
    } else {
        scanline++;
    }

    if((scanline%2)==0) {
        video_hsync=1;

        // VDP Draw on HSYNC

        // VGA Active starts scanline 35
        // TMS9918 Active scanline 75(0) to 474(199)

        if(scanline==78) {
            bgcolor=video_border & 0x7;
            memset(vga_data_array+160*4,bgcolor*0x11,160);
        }

//        if((scanline>=75)&&(scanline<=456)) {
        if((scanline>=81)&&(scanline<=464)) {

            tmsscan=(scanline-81)/2;
            if(menumode==0) {
                draw_framebuffer(tmsscan);
                bgcolor=video_border & 0x7;
                vramindex=(tmsscan%4)*160;

                memset(vga_data_array+(tmsscan%4)*160,bgcolor*0x11,16);
                memset(vga_data_array+(tmsscan%4)*160+16+128,bgcolor*0x11,16);
    
                for(int j=0;j<128;j++) {
                    vga_data_array[vramindex+j+16]=scandata[j];
                }

            } else {
                draw_framebuffer_menu(tmsscan);
                vramindex=(tmsscan%4)*160;

                for(int j=0;j<160;j++) {
                    vga_data_array[vramindex+j]=scandata[j];
                }      
            }
     
        }

    }

    return;

}

// BEEP and PSG emulation

bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {
#if 0
    uint16_t timer_diffs;
    uint32_t pon_count;
    uint16_t master_volume;

    uint8_t tone_output[3], noise_output[3], envelope_volume;

    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);

    // PSG

    master_volume = 0;

    // Run Noise generator

        psg_noise_counter += SAMPLING_INTERVAL;
        if (psg_noise_counter > psg_noise_interval) {
            psg_noise_seed = (psg_noise_seed >> 1)
                    | (((psg_noise_seed << 14) ^ (psg_noise_seed << 16))
                            & 0x10000);
            psg_noise_output = psg_noise_seed & 1;
            psg_noise_counter -= psg_noise_interval;
        }
        if (psg_noise_output != 0) {
            noise_output[0] = psg_noise_on[0];
            noise_output[1] = psg_noise_on[1];
            noise_output[2] = psg_noise_on[2];
        } else {
            noise_output[0] = 0;
            noise_output[1] = 0;
            noise_output[2] = 0;
        }

    // Run Envelope

        envelope_volume = 0;

        switch (psg_register[13] & 0xf) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 9:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 0;
            }
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 15:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 0;
            }
            break;
        case 8:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 31;
            }
            break;
        case 10:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter
                    < psg_envelope_interval * 64) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval - 32;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 31;
            }
            break;
        case 11:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 31;
            }
            break;
        case 12:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 0;
            }
            break;
        case 13:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 31;
            }
            break;
        case 14:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter
                    < psg_envelope_interval * 64) {
                envelope_volume = 63
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 0;
            }
            break;
        }


    // Run Oscillator

    for (int i = 0; i < 4 ; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) {
            tone_output[i] = psg_tone_on[i];
        } else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
            tone_output[i] = psg_tone_on[i];
        } else {
            tone_output[i] = 0;
        }
    }

    // Mixer

    master_volume = 0;

        for (int j = 0; j < 3; j++) {
            if ((tone_output[j] + noise_output[j]) > 0) {
                if ((psg_register[j + 8] & 0x10) == 0) {
                    master_volume += psg_volume[(psg_register[j + 8 ]
                            & 0xf) * 2 + 1];
                } else {
                    master_volume += psg_volume[envelope_volume];
                }
            }
        }

    psg_master_volume = master_volume / 4 + beep_enable*63 ;    // Add beep

    if (psg_master_volume > 255)
        psg_master_volume = 255;
#endif
    return true;
}

// PSG virtual registers

#if 0
void psg_write(uint32_t data) {

    uint32_t channel,freqdiv,freq;

    psg_register_number=ioport[0xa0];

    if(psg_register_number>15) return;

    psg_register[psg_register_number]=data;


    // printf("[PSG:%x,%x]",psg_register_number,data);

    switch(psg_register_number&0xf) {
        case 0:
        case 1:
            if((psg_register[0]==0)&&(psg_register[1]==0)) {
                psg_osc_interval[0]=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[0] + ((psg_register[1]&0x0f)<<8) );
            freq >>= 4;
            if(freq!=0) {
                psg_osc_interval[0] = TIME_UNIT / freq;
                psg_osc_counter[0]=0;
            } else {
                psg_osc_interval[0]=UINT32_MAX;
            }
            break;
        case 2:
        case 3:
            if((psg_register[2]==0)&&(psg_register[3]==0)) {
                psg_osc_interval[1]=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[2] + ((psg_register[3]&0x0f)<<8) );
            freq >>= 4;
            if(freq!=0) {
                psg_osc_interval[1] = TIME_UNIT / freq;
                psg_osc_counter[1]=0;
            } else {
                psg_osc_interval[1]=UINT32_MAX;
            }
            break;
        case 4:
        case 5:
            if((psg_register[4]==0)&&(psg_register[5]==0)) {
                psg_osc_interval[2]=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[4] + ((psg_register[5]&0x0f)<<8) );
            freq >>= 4;
            if(freq!=0) {
                psg_osc_interval[2] = TIME_UNIT / freq;
                psg_osc_counter[2]=0;
                } else {
                    psg_osc_interval[2]=UINT32_MAX;
                }
            break;
        case 6:
            if((psg_register[6]==0)&&(psg_register[7]==0)) {
                psg_noise_interval=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[6] & 0x1f );
            freq >>= 4;
            if(freq!=0) {
                psg_noise_interval = TIME_UNIT / freq;
                psg_noise_counter = 0;
            } else {
                psg_noise_interval=UINT32_MAX;
            }
            break;
        case 7:
            psg_tone_on[0]=((psg_register[7]&1)==0?1:0);
            psg_tone_on[1]=((psg_register[7]&2)==0?1:0);
            psg_tone_on[2]=((psg_register[7]&4)==0?1:0);
            psg_noise_on[0]=((psg_register[7]&8)==0?1:0);
            psg_noise_on[1]=((psg_register[7]&16)==0?1:0);
            psg_noise_on[2]=((psg_register[7]&32)==0?1:0);
            break;
        case 0xb:
        case 0xc:
            freq = psg_master_clock / ( psg_register[0xb] + (psg_register[0xc]<<8) );
            if(freq!=0) {
                psg_envelope_interval= TIME_UNIT / freq;
                psg_envelope_interval<<=5;
            } else {
                psg_envelope_interval=UINT32_MAX/2-1;
            }
            break;
        case 0xd:
            psg_envelope_counter=0;
            break;
//                        case 0xf:
//                        psg_reset(1,psg_no);
    }
}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}
#endif


#if 0

uint8_t tapein() {

static uint32_t tape_diff_cycles;
static uint8_t tape_bits,tape_file_data;
static uint8_t tape_half_bit,tape_signal;
static uint16_t tape_data;
static uint16_t tape_byte;
static uint32_t tape_header_bits;
static uint8_t tape_baud;
static uint8_t tape_last_bits;

    if(tape_ready==0) {
        return 0;
    }

    if(load_enabled==0) {
        return 0;
    }

    load_enabled=2;

    tape_diff_cycles=cpu_cycles-tape_cycles;
//    tape_cycles=cpu_cycles;
//    tape_last_bits=data;

    if(tape_phase%2) {

        if(tape_diff_cycles<tape_waits[(tape_last_bits+1)%2]) {
            return tape_signal;
        }

//    printf("[D:%d,%d,%d,%d,%x]",tape_diff_cycles,tape_signal,tape_last_bits,tape_bits,tape_file_data);

        tape_cycles=cpu_cycles;

        if(tape_bits==0) { // start bit
            if(tape_signal) {   // 1 -> 0
                tape_signal=0;
                tape_bits++;
                tape_half_bit=0;
                tape_last_bits=(tape_file_data&1);
                return 0;
            } else {            // 0 -> 1
                tape_signal=1;
                return 1;
            }
        }
        if(tape_bits<9) {
            if(tape_signal) {   // 1 -> 0
                tape_signal=0;
                if(tape_last_bits) {
                    if(tape_half_bit==0) {
                        tape_half_bit++;
                        return 0;
                    }
                }
                if(tape_bits<8) {
                    tape_last_bits=tape_file_data>>tape_bits;
                    tape_last_bits&=1;
                    tape_bits++;
                    tape_half_bit=0;
                    return 0;
                } else {
                    tape_last_bits=1;
                    tape_bits++;
                    tape_half_bit=0;
                    return 0;                    
                }
                return 0;
            } else {            // 0 -> 1
                tape_signal=1;
                return 1;
            }
        }

            if(tape_signal) {   // 1 -> 0

                if(tape_last_bits) {
                    if(tape_half_bit==0) {
                        tape_half_bit++;
                        tape_signal=0;
                        return 0;
                    }
                }
                if(tape_bits==9) {
                    tape_last_bits=1;
                    tape_bits++;
                    tape_signal=0;
                    tape_half_bit=0;
                    return 0;
                } else {
                    tape_last_bits=0;
                    tape_bits=0;
                    tape_signal=0;
                    tape_half_bit=0;
                    lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
                    tape_data=tape_file_data;
                    tape_ptr++;

                    return 0;                    
                }
            } else {            // 0 -> 1
                tape_signal=1;
                return 1;
            }
        
    } else {
        // Header 
        // Return '1' 2400baud

        if((tape_diff_cycles)>10000L) { // First 'h'
            tape_cycles=cpu_cycles;
            tape_signal=1;
            tape_header_bits=0;
            return 1;
        }
        if(tape_diff_cycles>tape_waits[0]) {

//    printf("[H:%d,%d,%d,%d]",tape_diff_cycles,tape_signal,tape_header_bits,tape_phase);

            tape_cycles=cpu_cycles;
            if(tape_signal) {
                tape_signal=0;
                tape_header_bits++;
                if(tape_header_bits>8000) {
                    // Skip CAS Header
                    for(int i=0;i<9;i++) {
                        lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
                    }
                    tape_bits=0;
                    tape_ptr++;
                    tape_last_bits=0;
                    tape_phase++;
                    tape_header_bits=0;
                }
                return 0;
            } else {
                tape_signal=1;
                return 1;
            }
        } else {
            return tape_signal;
        }
    }


#if 0
    static uint8_t tapebyte;



    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
    tape_ptr++;

    if(tapebyte==0xd3) {
        tape_leader++;
    } else if (tape_leader) {
        tape_leader++;
        if(tape_leader>0x20) {
            tape_leader=0;
        }
    }

//    printf("(%02x)",tapebyte);

    return tapebyte;
#endif
    return 0;

}

void tapeout(uint8_t data) {

static uint32_t tape_diff_cycles;
static uint8_t tape_bits,tape_file_data;
static uint8_t tape_half_bit;
static uint16_t tape_data;
static uint16_t tape_byte;
static uint8_t tape_baud;
static uint8_t tape_last_bits;

    if(tape_ready) {

        if(tape_last_bits!=data) {

            tape_diff_cycles=cpu_cycles-tape_cycles;
            tape_cycles=cpu_cycles;
            tape_last_bits=data;

//            printf("[%d:%d]",data,tape_diff_cycles);

            // Skip headers

            if(tape_phase%2) {
                if(data==0) {
                    if((tape_diff_cycles>tape_waits[tape_baud]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[tape_baud]+TAPE_WAIT_WIDTH)) {
                        tape_data=0;
                        tape_half_bit=0; 
  //                      printf("0");
                    } else if((tape_diff_cycles>tape_waits[tape_baud-1]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[tape_baud-1]+TAPE_WAIT_WIDTH)) {
                        if(tape_half_bit) {
                            tape_data=0x8000;
                            tape_half_bit=0;
   //                         printf("1");
                        } else {
                            tape_half_bit=1;
                            return;
                        }
                    } 
                    tape_byte=(tape_byte>>1)|tape_data;
                    tape_bits++;
                    if(tape_bits==11) {
                        if(save_enabled) {
                            save_enabled=2;
                            tape_file_data=(tape_byte>>6)&0xff;
                            tape_ptr++;
                            lfs_file_write(&lfs,&lfs_file,&tape_file_data,1);
                        } else {
                            printf("[%02x]",(tape_byte>>6)&0xff);
                        }
                        tape_bits=0;
                        tape_byte=0;
                    }
                }                
            } else {

                if(data!=0) {
                    if(tape_diff_cycles>10000L) {  // It is first H bit
                        tape_baud=0;
                        tape_bits=0;
                        tape_byte=0;
                    }
                } 
                if(data==0) {
                    if(tape_baud==0) {  // Baud rate check  (header bit is always '1')
                        if((tape_diff_cycles>tape_waits[0]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[0]+TAPE_WAIT_WIDTH)) {
                            tape_baud=1;
//printf("[1200]\n");
                        } else {
                            tape_baud=3;
//printf("[2400]\n");                            
                        }
                    } else {
                        if((tape_diff_cycles>tape_waits[tape_baud]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[tape_baud]+TAPE_WAIT_WIDTH)) {
                        // first '0' bit = Statbit of Data section
                            tape_bits=1;
                            tape_byte=0;
                            tape_phase++;
                        // Write CAS Header

                            lfs_file_write(&lfs,&lfs_file,tape_cas_header,8);

                        }
                    } 
                }
            }
        }

//        if(save_enabled) {
//
//        }
    } 
    
}

#endif

uint8_t tapein() {

    static uint8_t cmt_bit;
    static uint8_t cmt_buff;
    static uint8_t tape_phase,tape_bit_phase;

    if(!tape_ready) return 0;
    if(load_enabled==0) return 0;

    // Start

    if ((cpu_cycles - tape_cycles) > TAPE_THRESHOLD ) {
        cmt_buff = 0;
        cmt_bit = 0;
        tape_phase=1;
        tape_bit_phase=0;
        lfs_file_read(&lfs,&lfs_file,&cmt_buff,1);

//printf(" {%02x}",cmt_buff);

        tape_ptr++;
        load_enabled=2;
        tape_cycles=cpu_cycles;
        return tape_phase;
    }

    //

//    printf("[%d:%d]",tape_phase,cpu_cycles-tape_cycles);


    if ((cpu_cycles - tape_cycles) < tape_waits[0] ) {
        return tape_phase;
    } else if  ((cpu_cycles - tape_cycles) < 2*tape_waits[0] ) {

        // change phase if signal is 1

        if(tape_bit_phase==0) {

//            printf("[%d:%d]",tape_phase,cpu_cycles-tape_cycles);

            if(cmt_buff&0x80) {
                tape_phase++;
                tape_phase%=2;
            }
            tape_bit_phase=1;
        }

        return tape_phase;
    }

    tape_bit_phase=0;

//    printf("[%d:%d]",tape_phase,cpu_cycles-tape_cycles);

    tape_cycles=cpu_cycles;

    // load next bit

    cmt_bit++;
    if(cmt_bit==8) {
        lfs_file_read(&lfs,&lfs_file,&cmt_buff,1);
//        printf(" {%02x}",cmt_buff);
        cmt_bit=0;
        tape_ptr++;
    } else {
        cmt_buff<<=1;
    }

    tape_phase++;
    tape_phase%=2;

    return tape_phase;

}

void tapeout(uint8_t data) {

    static uint8_t cmt_bit;
    static uint8_t cmt_buff;

    if(!tape_ready) return;

    if(via_reg[7]&0x20) return;

    if ((cpu_cycles - tape_cycles) > TAPE_THRESHOLD ) {
        cmt_buff = 0;
        cmt_bit = 0;
    }

    tape_cycles=cpu_cycles;

    // CMT IN OUT WORKS ONLY 2400 BAUD MODE

#if 0    
    // 600 Baud

    cmt_buff <<= 1;
    if (data == 0xaa) {
        cmt_buff++;
    }
    cmt_bit++;
    if (cmt_bit == 8) {
        printf("%02x", cmt_buff);
        cmt_bit = 0;
        cmt_buff = 0;
    }
#endif

    // 2400 Baud

    for(int i=0;i<4;i++) {
        cmt_buff<<=1;
        switch(data&3) {
            case 0:
            case 3:
                break;
    
            case 1:
            case 2:
                cmt_buff++;
                break;
        }
        data>>=2;
        cmt_bit++;
    }
    if (cmt_bit == 8) {
        if(save_enabled) {
            save_enabled=2;
            tape_ptr++;
            lfs_file_write(&lfs,&lfs_file,&cmt_buff,1);
        } else {
            printf("%02x", cmt_buff);
        }
        cmt_bit = 0;
        cmt_buff = 0;
    }

    return;

}

void menuinit(void) {

    memcpy(font,fontrom,0x800);
    memset(menuram,0,0x800);
    memset(menuram+0x800,7,0x800);

    return;

}

static inline void video_cls() {

}

// static inline void video_scroll() {

//     memmove(vga_data_array, vga_data_array + VGA_PIXELS_X*10, (VGA_PIXELS_X*(VGA_PIXELS_X-10)));
//     memset(vga_data_array + (VGA_CHARS_X*(VGA_PIXELS_X-10)), 0, VGA_PIXELS_X*10);

// }

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;
    uint32_t vramindex;

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        menuram[cursor_x+cursor_y*VGA_CHARS_X]=string[i];
        menuram[cursor_x+cursor_y*VGA_CHARS_X+0x800]=fbcolor;        

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
//                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=2;
    cursor_y=2;
    fbcolor=7;
      video_print("                                    ");
    for(int i=3;i<19;i++) {
        cursor_x=2;
        cursor_y=i;
        video_print("                                    ");
    }

    cursor_x=2;
    cursor_y=19;
    fbcolor=7;
    video_print("                                    ");

}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    for(int i=0;i<LFS_LS_FILES;i++) {
        cursor_x=20;
        cursor_y=i+3;
        fbcolor=7;
//        video_print("                    ");
                video_print("  ");
    }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {
            
            if(num_entry>=LFS_LS_FILES*(page+1)) {
                break;
            }

            if((num_entry%LFS_LS_FILES)!=(LFS_LS_FILES-1)) {
                for(int i=num_entry%LFS_LS_FILES;i<LFS_LS_FILES;i++) {
                    cursor_x=22;
                    cursor_y=i+3;
                    fbcolor=7;
                    video_print("                  ");                    
                }
            }

            break;
        }

        cursor_x=28;
        cursor_y=23;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=23;
                    cursor_y=num_entry%LFS_LS_FILES+3;

                    if(num_entry==num_selected) {
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    snprintf(str,16,"%s            ",lfs_dir_info.name);
                    video_print(str);
//                    video_print(lfs_dir_info.name);

                }

                if(num_selected>=0) {
                    cursor_x=20;
                    cursor_y=(num_selected%LFS_LS_FILES)+3;
                    video_print("->");
                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x4b) { // Pageup
            keypressed=0;
            if(num_selected>=LFS_LS_FILES) {
                num_selected-=LFS_LS_FILES;
            }
        }

        if(keypressed==0x4e) { // Pagedown
            keypressed=0;
            if(num_selected<num_files-LFS_LS_FILES) {
                num_selected+=LFS_LS_FILES;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=3;
        cursor_y=18;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=3;
                cursor_y=18;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }


        }
    }

}

//----------------------------------------------------------------------------------------------

// Video OUT
 
// MENU mode
// output to render buffer
void __not_in_flash_func(draw_framebuffer_menu)(uint32_t line) {

    uint32_t scany,scanyy;
    uint32_t fontw1,fontw2;
    uint16_t attribute,vramaddr,ch;
    uint8_t screenwidth,scanx,bgcolor,fontdata,fgcolor;

    scany=line/8;
    scanyy=line%8;

    vramaddr=scany*VGA_CHARS_X;

    // check cursor position

    for(int i=0;i<VGA_CHARS_X;i++) {

        attribute=menuram[vramaddr+0x800];
        ch=menuram[vramaddr++];

        bgcolor=(attribute&0xf0)>>4;
        fgcolor=attribute&0xf;

        fontdata=font[ch*8+scanyy];
            
        fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
        renderbuffer[i]=fontw1;

    }
}

// Normal mode
// output to render buffer
void __not_in_flash_func(draw_framebuffer)(uint32_t line) {

    uint32_t scany,scanyy;
    uint32_t fontw1,fontw2;
    uint16_t attribute,vramaddr,ch;
    uint8_t screenwidth,scanx,bgcolor,fontdata,fgcolor;

    scany=line/8;
    scanyy=line%8;

    vramaddr=scany*32;

    // check cursor position

    for(int i=0;i<32;i++) {

        attribute=mainram[vramaddr+0xc500];
        ch=mainram[vramaddr+0xc100];

        vramaddr++;

        if((attribute&0xc0)==0x80) { // Semigraphics

            if(scanyy<4) {

                fgcolor=ch&7;
                bgcolor=(ch>>3)&7;

                renderbuffer[i]=0x11110000*bgcolor+0x1111*fgcolor;

            } else {

                fgcolor=attribute&7;
                bgcolor=(attribute>>3)&7;

                renderbuffer[i]=0x11110000*bgcolor+0x1111*fgcolor;

            }

        } else if(attribute&0x40) {  // PCG

            bgcolor=(attribute&0x38)>>3;
            fgcolor=attribute&0x7;
    
            fontdata=mainram[ch*8+scanyy+0xc000];
                
            fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
            renderbuffer[i]=fontw1;



        } else { // Normal

            bgcolor=(attribute&0x38)>>3;
            fgcolor=attribute&0x7;
    
            fontdata=mainram[ch*8+scanyy+0xd000];
                
            fontw1=bitexpand80[fontdata*2]*fgcolor+bitexpand80[fontdata*2+1]*bgcolor;
            renderbuffer[i]=fontw1;

        }

    }
}

//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();     // pause another core
    flash_range_program(addr, (const uint8_t *)buffer, size);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
        
    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{           
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);
    
//    printf("[FS] ERASE: %p, %d\n", offset, block);
        
    uint32_t ints = save_and_disable_interrupts();   
    multicore_lockout_start_blocking();     // pause another core
    flash_range_erase(offset, c->block_size);  
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256
    
    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?
    
    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256    
};

// Keyboard

static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

void process_kbd_leds(void) {

    hid_led=0;

    if(key_caps) hid_led+=KEYBOARD_LED_CAPSLOCK;          // CAPS Lock
    if(key_kana) hid_led+=KEYBOARD_LED_NUMLOCK;           // KANA -> Numlock
    if((hid_dev_addr!=255)&&(hid_instance!=255)) {
        tuh_hid_set_report(hid_dev_addr, hid_instance, 0, HID_REPORT_TYPE_OUTPUT, &hid_led, sizeof(hid_led));
    }

}

static inline int16_t getkeycode(uint8_t modifier,uint8_t keycode) {

    uint16_t jr200code;

    if(modifier&0x11) {  // Control

        if((key_basic_flag==0)&&(jr200basiccode[keycode][0]!=0)) {  // BASIC keyword input

            return 0x100;
        }

        jr200code=jr200usbcode[keycode*6];

        // Auto repeat control (SHIT+CTRL+0/1)

        // if(modifier&22) {
        //     if(keycode==0x1e) {
        //         key_repeat_flag=1;
        //         return -1;
        //     }
        //     if(keycode==0x27) {
        //         key_repeat_flag=0;
        //     }
        // }

        if(jr200code==0) return -1;

        if(jr200code<0x20) return jr200code;
        if(jr200code==0x40) return 0;
        if((jr200code>=0x61)&&(jr200code<=0x7a)) return jr200code-0x60;
        if((jr200code>=0x5b)&&(jr200code<=0x5f)) return jr200code-0x40;

        return -1;

    } else {

        if(key_graph) {
            if(modifier&0x22) {
                jr200code=jr200usbcode[keycode*6+5];
            } else {
                jr200code=jr200usbcode[keycode*6+4];
            }
            if(jr200code!=0) return jr200code;
            return -1;
        } else if(key_kana) {
            if(modifier&0x22) {
                jr200code=jr200usbcode[keycode*6+3];
            } else {
                jr200code=jr200usbcode[keycode*6+2];
            }
            if(jr200code!=0) return jr200code;
            return -1;
        } else {
            if(modifier&0x22) {
                jr200code=jr200usbcode[keycode*6+1];
            } else {
                jr200code=jr200usbcode[keycode*6];
            }

            if(jr200code==0) return -1;

            if(key_caps) {
                if((jr200code>=0x41)&&(jr200code<=0x5a)) {
                    jr200code+=0x20;
                } else  if((jr200code>=0x61)&&(jr200code<=0x7a)) {
                    jr200code-=0x20;
                }
            }

            return jr200code;

        }
    }

    return -1;
}


void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;
    int16_t jr200code;

    if(menumode==0) { // Emulator mode

        // unsigned char str[16];
        // sprintf(str,"%d",sub_cpu.cycles);
        // cursor_x=60;
        // cursor_y=24;
        // video_print(str);

//        printf("%d",sub_cpu.cycles);

        key_repeat_count=0;

        if(report->modifier!=lastmodifier) {  // stop auto repeat when modifier changed

            if(key_repeat_flag) {
                key_repeat_count=0;
            }
        }

        lastmodifier=report->modifier;

        key_break_flag=0;

        for(int i=0;i<6;i++) {

            if ( report->keycode[i] )
                {
                if ( find_key_in_report(&prev_report, report->keycode[i]) )
                {
                // exist in previous report means the current key is holding
                }else
                {


                keypressed=report->keycode[i];

                jr200code=getkeycode(report->modifier,report->keycode[i]);

                if(jr200code!=-1) {

                    jr200keypressed=jr200code;

                    key_irq=1;
                    via_reg[0x1c]|=1;
                    keyboard_cycles=cpu_cycles;
                    joystick_count=0xff;

                    if(key_repeat_flag) {
                        key_repeat_count=total_scanline;                    
                    }

                }

                if(jr200code==0x100) { // BASIC Keyword mode
                    key_irq=1;
                    key_basic_code=report->keycode[i];
                    key_basic_bytes=0;
                    via_reg[0x1c]|=1;
                    keyboard_cycles=cpu_cycles;
                    return;
                }
                key_basic_code=0;

                // CapsLock

//                 if(keypressed==0x39) {
//                     if(key_caps==0) {
//                         key_caps=1;
//                     } else {
//                         key_caps=0;
//                     }
// //                    process_kbd_leds();
//                 }

                // Kana

                if(keypressed==0x88) {
                        key_kana=1;
                        key_graph=0;
//                    process_kbd_leds();
                }

                // Henkan (Graph)
                if(keypressed==0x8a) {
                    key_kana=0;
                    key_graph=1;
//                    process_kbd_leds();
                }

                // Caps
                if(keypressed==0x39) {
                    key_kana=0;
                    key_graph=0;
//                    process_kbd_leds();
                }

                // Break

                if(keypressed==0x48) {
                    key_break_flag=1;             
                }


            // Enter Menu
            if(report->keycode[i]==0x45) {
                prev_report=*report;
                menumode=1;
                keypressed=0;
            }  

                }
            }   

        }

    prev_report=*report;

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}


// Emulator related

uint8_t subcpu_read() {

    uint8_t keycode;

//printf("[S:%04x]",m6800_get_reg(M6800_PC));

    // Clear IRQ signal
    key_irq=0;
    via_reg[0x1c]&=0xfe;

    if((via_reg[3]&2)&&(subcpu_ktest<=0x801)) {  // Read font ROM

        joystick_count=0xff;

        if(subcpu_ktest<=0x800) {
//            printf("[CR:%x]",subcpu_ktest);
            return fontrom[subcpu_ktest++];
        } else {        // Baudrate switch
            printf("[CR:%x]",subcpu_ktest);
            subcpu_ktest++;
            return 0;
        }

    } else {        
//        if((via_reg[3]&0x82)==2) {  // Joystick ?
                                    // Check key BASIC mode

        if(joystick_count<=2) {

        printf("[JS:%x]",joystick_count);
        joystick_count++;

            return 0xff;


        } else { // Keyboard

            if((key_basic_flag==0)&&(key_basic_code!=0)) {
                keycode=jr200basiccode[key_basic_code][key_basic_bytes++];
                if(jr200basiccode[key_basic_code][key_basic_bytes]!=0) {
                    key_irq=1;
                    via_reg[0x1c]|=1;
                    keyboard_cycles=cpu_cycles;
                }
                return keycode;
            }

            return jr200keypressed;

        }


    }

    return 0xff;
}

void settimer(uint16_t address,uint8_t data) {

    uint8_t number;

    switch(address) {

        case 0xe:
        case 0x10:
        case 0x12:
        case 0x14:

            number=(address-0xe)>>1;

            switch (data&0x18) {
            case 0:
                via_prescalecount[number]=1;
                break;

            case 8:
                via_prescalecount[number]=8;
                break;

            case 0x10:
                via_prescalecount[number]=64;
                break;

            case 0x18:
                via_prescalecount[number]=256;
                break;

            default:
                via_prescalecount[number]=1;
                break;
            }

            via_timercount[number]=via_prescalecount[number]*via_reload[number];

            // check beep on 
        
            return;

        case 0x16:
        case 0x19:
    
            if(address==0x16) {
                number=4;
            } else {
                number=5;
            }

            if(data&8) {
                via_prescalecount[number]=8;
            } else {
                via_prescalecount[number]=1;
            }

            via_timercount[number]=via_prescalecount[number]*via_reload[number];

            return;        

        case 0xf:
        case 0x11:
        case 0x13:
        case 0x15:

            number=(address-0xf)>>1;
            via_reload[number]=data;
            return;

        case 0x17:
        case 0x1a:

            number=(address-0x17)/3+4;
            via_reload[number]=data*256+via_reg[address+1];
            return;

        case 0x18:
        case 0x1b:

            number=(address-0x18)/3+4;
            via_reload[number]=via_reg[address-1]*256+data;
            return;

        default:
            return;

    }
}


// RUN timers
void exectimer(uint16_t cycles) {

    uint8_t timer_status;

    for(int i=0;i<6;i++) {

        if(i<5) {
            timer_status=via_reg[i*2+0xe];
        } else {
            timer_status=via_reg[i*2+0xf];            
        }


        if((i<2)&&(timer_status&1)==0) {    // Counter disabled
            continue;
        }
        if((i>=2)&(timer_status&0x7)!=6) {
            continue;
        }

        // check prescale

        if(via_timercount[i]>=cycles) {
            via_timercount[i]-=cycles;
            continue;
        }

        // IRQ
        if(timer_status&0x40) {
            if(via_reg[0x1f]&(1>>(i-1))) {

                timer_enable_irq=1;
                via_reg[0x1d]|=(1>>(i-1));

            }
        }    

        // Reload

        via_timercount[i]+=via_prescalecount[i]*via_reload[i];
        via_timercount[i]-=cycles;
        
        // Set BORROW Flag
        
        if(i<5) {
            via_reg[i*2+0xe]|=0x20;
        } else {
            via_reg[i*2+0xf]|=0x20;            
        }

        

    }

    return;

}

void cpu_writemem16(unsigned short addr, unsigned char bdat) { // RAM access is managed here to allow memory-mapped I/O access via traps

    uint16_t timer_control;

    if(addr<0xa000) {  // Main ram
        mainram[addr]=bdat;
        return;
    } else if(addr<0xc000) {    // BASIC ROM
        return;
    } else if(addr<0xc800) {    // Video RAM
        mainram[addr]=bdat;
        return;
    } else if(addr<0xd000) {    // IO

        if(addr<0xca00) {   // MN1271

            switch(addr&0x1f) {

                case 3:     // PIOB

//printf("[PB:%x:%x]",bdat,m6800_get_reg(M6800_PC));

                    if(bdat&2) {

                        if((via_reg[3]&2)==0) { // Command
                            if(via_reg[0x1e]&1) {
                                key_irq=1;
                                via_reg[0x1c]|=1;
                                keyboard_cycles=cpu_cycles;
                                joystick_count=0;
                            }
                        } else if((bdat&1)&&((via_reg[3]&1)==0)) { // continure
                            if((subcpu_ktest>=0x801)&&(joystick_count>2)) {  // JOYSTICK irq
                                return;
                            }
                            if(via_reg[0x1e]&1) {
//                                printf("[I:%x]",joystick_count);
                                key_irq=1;
                                via_reg[0x1c]|=1;
                                keyboard_cycles=cpu_cycles;
                            }
                        }
                    }

                    if(bdat&0x80) {
                        key_basic_flag=1;
                    } else {
                        key_basic_flag=0;
                    }

                    via_reg[3]=bdat;
                    return;

                case 7: // CMT/SIO control

                    if(bdat&0x40) {
                        tape_ready=1;
                        printf("[CMT:ON]");
                    } else {
                        tape_ready=0;
                        printf("[CMT:OFF]");
                    }

                    via_reg[7]=bdat;
                    return;

                case 0xd:   // CMT data

                    printf("[%02x]",bdat);
                    tapeout(bdat);



                    return;

                case 0xe:   // Timer control
                case 0x10:
                case 0x12:
                case 0x14:
                case 0x16:
                case 0x19:

//                printf("[TCW:%x:%x]",addr&0x1f,bdat);

                    via_reg[addr&0x1f]=bdat;
                    settimer(addr&0x1f,bdat);
                    return;

                case 0xf:
                case 0x11:
                case 0x13:
                case 0x15:
                case 0x17:
                case 0x18:
                case 0x1a:
                case 0x1b:

                printf("[TRW:%x:%x]",addr&0x1f,bdat);

                    via_reg[addr&0x1f]=bdat;
                    settimer(addr&0x1f,bdat);
                    return;

                default:

//                printf("[PIAW:%x/%x:%x]",addr&0x1f,bdat,m6800_get_reg(M6800_PC));

                    via_reg[addr&0x1f]=bdat;
                    return;

            }
        }

        if(addr<0xcc00) {   // CRTC

            video_border=bdat;
            return;

        }

        return;

    } else if(addr<0xd800) {
        mainram[addr]=bdat;
        return;
    }

    return;

}
//
unsigned char cpu_readmem16(unsigned short addr) { // to allow for memory-mapped I/O access

    uint8_t bdat,timer_ch;

    if(addr<0xa000) {   // mainram
        return mainram[addr];
    } else if(addr<0xc000) { // basic
        return basicrom[addr&0x1fff];
    } else if(addr<0xc800) { // VRAM
        return mainram[addr];
    } else if(addr<0xd000) { // IO

        if(addr<0xca00) {   // MN1271

            switch(addr&0x1f) {

                case 1:     // Sub CPU Read
                    return subcpu_read();

                case 7:

                    if(tapein()) {
                        return 0x80;
                    } else {
                        return 0;
                    }

                case 0xc:   // Serial status
                    return 0x20;

                case 0xe:   // Timer control
                case 0x10:
                case 0x12:
                case 0x14:
                case 0x16:
                case 0x19:

                    timer_ch=addr&0x1f;

//                    printf("[TC:%x:%x:%x]",timer_ch,via_reg[timer_ch],via_timercount[(timer_ch-0x1f)>>2]);
                    bdat=via_reg[timer_ch];
                    via_reg[timer_ch]&=0xdf;
                    return bdat;

                case 0xf:   // Timer count (8bits)
                case 0x11:
                case 0x13:
                case 0x15:

                    timer_ch=(addr&0x1f)-0xf;
                    timer_ch>>1;

//printf("[TR:%x,%x,%x",timer_ch,via_timercount[timer_ch],via_prescalecount[timer_ch]);

                    return via_timercount[timer_ch]/via_prescalecount[timer_ch] ;

                case 0x17:
                case 0x1a:
                    timer_ch=(addr&0x1f)-0x17;
                    timer_ch/=3;
                    timer_ch+=4;
                    return (via_timercount[timer_ch]/via_prescalecount[timer_ch])>>8 ;

                case 0x18:
                case 0x1b:
                    timer_ch=(addr&0x1f)-0x17;
                    timer_ch/=3;
                    timer_ch+=4;
                    return (via_timercount[timer_ch]/via_prescalecount[timer_ch])&0xff ;
    
                case 0x1c:  // IRQ status 1

                    if((key_irq)||(timer_enable_irq)) {           // TODO Timer irq
                        return via_reg[0x1c]|0x80;
                    } else {
                        return via_reg[0x1c];
                    }

                case 0x1d:  // IRQ status 2

                    if((key_irq)||(timer_enable_irq)) {
                        bdat=via_reg[0x1d]|0x80;

                        via_reg[0x1d]=0;        // ??

                        return bdat;
                    } else {
                        return via_reg[0x1d];
                    }

                default:

//printf("[PIA:%x/%x:%x]",addr&0x1f,via_reg[addr&0x1f],m6800_get_reg(M6800_PC));

                    return via_reg[addr&0x1f];

            }
        }

        if(addr<0xcc00) {   // CRTC

            return 0xff;

        }

        return 0xff;

    } else if(addr<0xd800) {    // CGRAM
        return mainram[addr];
    } else if(addr<0xe000) {    // EXT ROM
        return extrom[addr&0x7ff];
    } else {
        return monitorrom[addr&0x1fff];
    }

    return 0xff;

}

void init_emulator(void) {
//  setup emulator 

    for(int i=0;i<11;i++) {
        keymap[i]=0xff;
    }

    key_kana=0;
    key_caps=0;
    key_graph=0;

    tape_ready=0;
    tape_leader=0;

//    fdc_init();

    subcpu_ktest=0;

    gamepad_info=0x3f;


}

void main_core1(void) {

    uint8_t bgcolor;
    uint32_t vramindex;

    multicore_lockout_victim_init();

    scanline=0;

    // set Hsync timer
    irq_set_exclusive_handler (PIO0_IRQ_0, hsync_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled (pio0, pis_interrupt0 , true);

    // set PSG timer
    // Use polling insted for I2S mode

    // add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);

    while(1) { 

#ifdef USE_I2S
        i2s_process();
#endif

    }
}

int main() {

    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

    static uint32_t hsync_wait,vsync_wait,cycles;

    set_sys_clock_khz(DOTCLOCK * CLOCKMUL ,true);

    stdio_init_all();

    uart_init(uart0, 115200);

    initVGA();

    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // gpio_set_slew_rate(0,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(1,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(2,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(3,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(4,GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(2,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(3,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(4,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(5,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(6,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(7,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(8,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(9,GPIO_DRIVE_STRENGTH_2MA);


    // Beep & PSG

    gpio_set_function(10,GPIO_FUNC_PWM);
 //   gpio_set_function(11,GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(10);

    pwm_set_wrap(pwm_slice_num, 256);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
//    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_B, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // set PSG timer

    add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);

    tuh_init(BOARD_TUH_RHPORT);


//    video_cls();

    video_hsync=0;
    video_vsync=0;

    // video_mode=0;
    // fbcolor=0x7;

// uart handler

    // irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    // irq_set_enabled(UART0_IRQ,true);
    // uart_set_irq_enables(uart0,true,false);

    multicore_launch_core1(main_core1);
    multicore_lockout_victim_init();

    sleep_ms(1);

#ifdef USE_OPLL
    // set Hsync timer

    irq_set_exclusive_handler (PIO0_IRQ_0, hsync_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled (pio0, pis_interrupt0 , true);
#endif

// mount littlefs
    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
       cursor_x=0;
       cursor_y=0;
       fbcolor=7;
       video_print("Initializing LittleFS...");
       // format
       lfs_format(&lfs,&PICO_FLASH_CFG);
       lfs_mount(&lfs,&PICO_FLASH_CFG);
   }

    menuinit();

    menumode=1;  // Pause emulator

    init_emulator();

    m6800_init();
    m6800_reset();

    cpu_hsync=0;
    cpu_cycles=0;

    // start emulator
    
    menumode=0;

    while(1) {

        if(menumode==0) { // Emulator mode

// if(m6800_get_reg(M6800_PC)==0xe7e2) {
//     printf("[IRQ]");
// }

            cycles=m6800_execute(1);
            cpu_cycles+=cycles;
            exectimer(cycles);

            cpu_clocks++;

        // Wait

//        if((cpu_cycles-cpu_hsync)>1 ) { // 63us * 3.58MHz = 227

        // if((!cpu_boost)&&(cpu_cycles-cpu_hsync)>198 ) { // 63us * 3.58MHz = 227

        //     while(video_hsync==0) ;
        //     cpu_hsync=cpu_cycles;
        //     video_hsync=0;
        // }

        // Break key
        if(key_break_flag==1) {
                    key_break_flag=0;
                    ENTER_INTERRUPT("", 0xfffc);
        }

        // Keyboard IRQ
        if((key_irq==1)&&((cpu_cycles-keyboard_cycles)>KEYBOARD_WAIT)) {
                if((m6800_get_reg(M6800_CC)&0x10)==0) {
                    key_irq=2;
                    ENTER_INTERRUPT2("", 0xfff8);
                }
        }

        // Timer IRQ
        if((timer_enable_irq==1)) {
                if((m6800_get_reg(M6800_CC)&0x10)==0) {
                    timer_enable_irq=0;
//                    printf("[TON]");
                    ENTER_INTERRUPT2("", 0xfff8);
                }
        }


        if(video_vsync==1) { // Timer
            tuh_task();
            process_kbd_leds();

            // Process Keyrepeat

            if((key_repeat_flag)&&(key_repeat_count!=0)) {
                if((total_scanline-key_repeat_count)==40*525) {
                    key_irq=1;
                    via_reg[0x1c]|=1;
                    keyboard_cycles=cpu_cycles;
                } else if (((total_scanline-key_repeat_count)>40*525)&&((total_scanline-key_repeat_count)%(4*525)==0)) {
                    key_irq=1;
                    via_reg[0x1c]|=1;
                    keyboard_cycles=cpu_cycles;
                }
            }


            video_vsync=2;
            vsync_scanline=total_scanline;
      
            if((tape_autoclose)&&(save_enabled==2)) {
                if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                    save_enabled=0;
                    lfs_file_close(&lfs,&lfs_file);
                }
            }

            if((tape_autoclose)&&(load_enabled==2)) {
                if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                    load_enabled=0;
                    lfs_file_close(&lfs,&lfs_file);
                }
            }


        }

        } else { // Menu Mode


            unsigned char str[80];
            
            if(menuprint==0) {

                video_cls();
//                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=0;
            cursor_y=0;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=0;
            cursor_y=1;
            video_print(str);

            sprintf(str,"TAPE:%x",tape_ptr);
            cursor_x=0;
            cursor_y=2;
            video_print(str);


            cursor_x=3;            
            cursor_y=6;
            if(save_enabled==0) {
                video_print("SAVE: empty");
            } else {
                sprintf(str,"SAVE: %8s",tape_filename);
                video_print(str);
            }
            cursor_x=3;
            cursor_y=7;

            if(load_enabled==0) {
                video_print("LOAD: empty");
            } else {
                sprintf(str,"LOAD: %8s",tape_filename);
                video_print(str);
            }



            cursor_x=3;
            cursor_y=8;

            video_print("DELETE File");

            cursor_x=3;
            cursor_y=9;

            if(cpu_boost) {
                 video_print("CPU:Fast");
            } else {
                 video_print("CPU:Normal");
            }

            cursor_x=3;
            cursor_y=10;

            video_print("Reset");

            cursor_x=3;
            cursor_y=11;

            video_print("PowerCycle");

            cursor_x=0;
            cursor_y=menuitem+6;
            video_print("->");

   // for DEBUG ...

           cursor_x=0;
            cursor_y=23;
                 sprintf(str,"%04x %04x %04x %04x %04x",m6800_get_reg(M6800_PC),m6800_get_reg(M6800_A),m6800_get_reg(M6800_B),m6800_get_reg(M6800_X),m6800_get_reg(M6800_CC));
//                 sprintf(str,"%04x",Z80_PC(cpu));
                 video_print(str);

            if(filelist==0) {
                draw_files(-1,0);
                filelist=1;
            }
     
            while(video_vsync==0);

            video_vsync=0;

                tuh_task();

                if(keypressed==0x52) { // Up
                    cursor_x=0;
                    cursor_y=menuitem+6;
                    video_print("  ");
                    keypressed=0;
                    if(menuitem>0) menuitem--;
                }

                if(keypressed==0x51) { // Down
                    cursor_x=0;
                    cursor_y=menuitem+6;
                    video_print("  ");
                    keypressed=0;
                    if(menuitem<13) menuitem++; 
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE
                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                // tape_phase=0;
                                tape_ptr=0;
                                tape_cycles=0;
                                // tape_count=0;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD
                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDONLY);
                                load_enabled=1;
                                // tape_phase=0;
                                tape_ptr=0;
                                tape_cycles=0;
                                // tape_count=0;
//                                file_cycle=cpu.PC;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuprint=0;

                    }

                    if(menuitem==3) { 
                        cpu_boost++;
                        if(cpu_boost>1) cpu_boost=0;
                        menuprint=0;
                    }

                    if(menuitem==4) { // Reset
                        menumode=0;
                        menuprint=0;
                    
                        init_emulator();

                        m6800_reset();

                    }

                    if(menuitem==5) { // PowerCycle
                        menumode=0;
                        menuprint=0;

                        memset(mainram,0,0x10000);
//                        memset(ioport,0,0x100);

                        init_emulator();

                        m6800_reset();

                    }

                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                //  break;     // escape from menu
                }

        }


    }

}
