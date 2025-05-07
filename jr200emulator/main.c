/*

 Matshushita JR-100 Emulator for CH32V203
 You must use HSE 96MHz clock.
 (May be work on 48MHz by change SPI/Timer prescaler value)

 TIM1CH4:  (PA11) Sync signal
 TIM1CH3:  (NC)  Video out timing interrupt
 SPI1MOSI: (PA7) Video signal

 PA11 -- R1 --+
 PA7 -- R2 --+---> Video

 R1: 560 ohm
 R2: 240 ohm

 TIM4CH4: (PB9)  Sound
 Key input: USART2_RX £¨PA3£© or PS/2 PA8(Clock)/PA9(DATA)

 */

#include "debug.h"
#include "jr100rom.h"
//#include "jr100guldus.h"
#include "cpuintrf.h"
#include "m6800.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include "PS2Keyboard.h"

/* Global define */

#define NTSC_COUNT 3050 // = 63.56 us / 48MHz
#define NTSC_HSYNC 225  // =  4.7  us / 48MHz
#define NTSC_VSYNC 2825 // = NTSC_COUNT - NTSC_HSYNC
#define NTSC_SCAN_DELAY 0 // Delay for video signal generation
#define NTSC_SCAN_START 40 // Display start line

#define NTSC_X_PIXELS 256
#define NTSC_Y_PIXELS 192

#define NTSC_PRESCALER SPI_BaudRatePrescaler_16

#define NTSC_X_CHARS (NTSC_X_PIXELS/8)
#define NTSC_Y_CHARS (NTSC_Y_PIXELS/8)

#define USE_PS2_KEYBOARD

#define RX_BUFFER_LEN 64

/* Global Variable */

volatile uint16_t ntsc_line;
volatile uint8_t ntsc_blank = 0;
volatile uint8_t run_emulation = 0;

uint8_t *vram;
uint8_t *scandata[2];
uint8_t keymatrix[9];
uint8_t via_reg[16];
volatile uint8_t cmt_buff;
volatile uint8_t cmt_bit = 0;

uint64_t last_usart_tx_systick = 0;

volatile uint8_t rxbuff[RX_BUFFER_LEN];
uint8_t rxptr = 0;
uint32_t lastptr = RX_BUFFER_LEN;

// 700 bytes left
//16KB
#define MEM_SIZE 16384
const uint8_t memmap[] = { 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 2, 4, 0, 0, 3, 3, 3, 3 }; // Memory map 1=RAM,2=VRAM,3=ROM,4=IO

//14KB
//#define MEM_SIZE (14336+768)
//const uint8_t memmap[] = { 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//        0, 0, 0, 0, 0, 0, 2, 4, 0, 0, 3, 3, 3, 3 }; // Memory map 1=RAM,2=VRAM,3=ROM,4=IO

// keymap
const uint8_t keymap[] = { 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0,
        0, // scanline , key ,modifyer(shift/ctrl)
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 8, 16, 2, 0xff, 0, 0,
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 8, 8, 0, 0xff, 0, 0, 0xff, 0,
        0, // 0x0D = CR
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0,
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0, 16, 2,
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 8, 2, 0, 3, 1, 1, 3,
        2, // 0x1B (ESC = break (Ctrl+C)
        1, 3, 4,
        1, // 0x20
        3, 8, 1, 3, 16, 1, 4, 1, 1, 4, 2, 1, 4, 4, 1, 4, 8, 1, 8, 4, 1, 6, 16,
        1, 7, 16, 0, 8, 16, 0, 8, 1, 0, 6, 8, 1, 4, 16, 0, 3, 1, 0, 3, 2, 0, 3,
        4,
        0, // 0x30
        3, 8, 0, 3, 16, 0, 4, 1, 0, 4, 2, 0, 4, 4, 0, 4, 8, 0, 8, 4, 0, 6, 16,
        0, 7, 16, 1, 8, 16, 1, 8, 1, 1, 6, 4, 1, 5, 1, 1, 1, 1, 0, 7, 2, 0, 0,
        16,
        0, // 0x40
        1, 4, 0, 2, 4, 0, 1, 8, 0, 1, 16, 0, 6, 1, 0, 5, 4, 0, 6, 2, 0, 6, 4, 0,
        6, 8, 0, 7, 8, 0, 7, 4, 0, 5, 8, 0, 5, 16, 0, 2, 1, 0, 2, 8, 0, 1, 2,
        0, // 0x50
        2, 16, 0, 5, 2, 0, 7, 1, 0, 2, 2, 0, 0, 8, 0, 5, 1, 0, 0, 4, 0, 5, 8, 1,
        5, 4, 1, 5, 16, 1, 4, 16, 1, 7, 8, 1 };

// PS/2 Keymap
const uint8_t ps2_keymap[] =
        { 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0,
                0x00,   //0x00
                0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, 0, 0x00, 0, 0x00, 0, 0x02, 0, 0x00, 0, 0x01, 2, 0x01,
                3, 0x01, 0,
                0x00,   //0x10
                0, 0x00, 0, 0x00, 0, 0x04, 1, 0x02, 1, 0x01, 2, 0x02, 3, 0x02,
                0, 0x00, 0, 0x00, 0, 0x10, 0, 0x08, 1, 0x04, 2, 0x04, 3, 0x08,
                3, 0x04, 0,
                0x00,   //0x20
                0, 0x00, 8, 0x02, 7, 0x01, 1, 0x08, 2, 0x10, 2, 0x08, 3, 0x10,
                0, 0x00, 0, 0x00, 7, 0x04, 7, 0x02, 6, 0x01, 1, 0x10, 5, 0x01,
                4, 0x01, 0,
                0x00,   //0x30
                0, 0x00, 0, 0x00, 7, 0x08, 6, 0x02, 5, 0x02, 4, 0x02, 4, 0x04,
                0, 0x00, 0, 0x00, 7, 0x10, 6, 0x04, 5, 0x04, 5, 0x08, 4, 0x10,
                4, 0x08, 0,
                0x00,   //0x40
                0, 0x00, 8, 0x01, 0, 0x00, 6, 0x08, 6, 0x10, 5, 0x10, 8, 0x10,
                0, 0x00, 0, 0x00, 0, 0x00, 8, 0x04, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, 0,
                0x00,   //0x50
                0, 0x00, 0, 0x00, 8, 0x08, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, 0,
                0x00,   //0x60
                0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, 0,
                0x00,   //0x70
                0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,
                0, 0x00, };

// general global variables
//extern uint8_t RAM[];            // any reads or writes to ports or vectors are trapped in SW
uint8_t *RAM; // any reads or writes to ports or vectors are trapped in SW

//uint8_t cursor_x, cursor_y = 0;

// TVout for CH32V203

void video_init() {

    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    SPI_InitTypeDef SPI_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(
    RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1, ENABLE);

    // PC4:Sync

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure);

    // PC6: Video (SPI1)

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure);

    // Initalize TIM1

    TIM_TimeBaseInitStructure.TIM_Period = NTSC_COUNT;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1;                // Presclaer = 0
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = NTSC_HSYNC;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM1, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = NTSC_HSYNC * 2 - NTSC_SCAN_DELAY; // 9.4usec - delay
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init( TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC3PreloadConfig( TIM1, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM1, ENABLE);
    TIM_Cmd( TIM1, ENABLE);

    // Initialize SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = NTSC_PRESCALER; // 6MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_Cmd(SPI1, ENABLE);

    // NVIC

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);

    // Init VRAM

//    vram = malloc(NTSC_X_CHARS * NTSC_Y_CHARS);
    vram = malloc(1024);

    scandata[0] = malloc(NTSC_X_CHARS + 1);
    scandata[1] = malloc(NTSC_X_CHARS + 1);

    scandata[0][NTSC_X_CHARS] = 0;
    scandata[1][NTSC_X_CHARS] = 0;

    //

}

static inline void video_cls() {
    memset(vram + 0x100, 0, (NTSC_X_CHARS * NTSC_Y_CHARS));
}
/*
 static inline void video_scroll() {

 memmove(vram + 0x100, vram + 0x100 + NTSC_X_CHARS,
 NTSC_X_CHARS * (NTSC_Y_CHARS - 1));

 memset(vram + 0x100 + NTSC_X_CHARS * (NTSC_Y_CHARS - 1), 0, NTSC_X_CHARS);

 }

 static inline void video_print(uint8_t *string) {

 int len;

 len = strlen(string);

 for (int i = 0; i < len; i++) {
 vram[cursor_x + cursor_y * NTSC_X_CHARS + 0x100] = string[i];
 cursor_x++;
 if (cursor_x >= NTSC_X_CHARS) {
 cursor_x = 0;
 cursor_y++;
 if (cursor_y >= NTSC_Y_CHARS) {
 video_scroll();
 cursor_y = NTSC_Y_CHARS - 1;
 }
 }
 }

 }
 */
void video_wait_vsync() {

    while(ntsc_blank==1);
    while(ntsc_blank==0);

}
/*
 void video_debug(uint16_t addr, uint8_t op) {

 vram[0x100] = ((addr >> 12) & 0xf) + 0x10;
 vram[0x101] = ((addr >> 8) & 0xf) + 0x10;
 vram[0x102] = ((addr >> 4) & 0xf) + 0x10;
 vram[0x103] = ((addr & 0x0f)) + 0x10;

 vram[0x105] = ((op >> 4) & 0xf) + 0x10;
 vram[0x106] = ((op & 0x0f)) + 0x10;

 if (vram[0x100] > 0x19)
 vram[0x100] += 7;
 if (vram[0x101] > 0x19)
 vram[0x101] += 7;
 if (vram[0x102] > 0x19)
 vram[0x102] += 7;
 if (vram[0x103] > 0x19)
 vram[0x103] += 7;
 if (vram[0x105] > 0x19)
 vram[0x105] += 7;
 if (vram[0x106] > 0x19)
 vram[0x106] += 7;

 }
 */

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr,
        u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

}

void TIM4_PWMOut_Init(u16 arr, u16 psc, u16 ccp) {
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM4, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_OC4PreloadConfig( TIM4, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM4, ENABLE);
    TIM_Cmd( TIM4, ENABLE);
}

// Emulator related

void exec6522(uint16_t cycles) {

    uint16_t timer1_count, timer2_count, timer1_control;

    // run Timer 1

    timer1_count = via_reg[4] + (via_reg[5] << 8);

    //   if( timer1_count !=0 ) {          // check timer is running
    if (timer1_count > cycles) {
        timer1_count -= cycles;
    } else {
        //         if((via_reg[0xb]&0x40)==0) {  // One shot
        timer1_count = 0;
        //         } else {                 // Continuous
        timer1_control = via_reg[6] + (via_reg[7] << 8);
        if (timer1_control > cycles) {
            timer1_count = timer1_control - timer1_count - cycles;
        } else {
            timer1_count = 0;
        }
        //         }

        if ((via_reg[0xb] & 0x80) != 0) {  // check to toggle PB7
            if ((via_reg[0] & 0x80) == 0) {
                via_reg[0] |= 0xc0;
//                    GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
            } else {
                via_reg[0] &= 0x3f;
//                    GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);

                // run timer 2 (PB7 count mode)

                timer2_count = via_reg[8] + (via_reg[9] << 8);
                if (timer2_count > 0) {
                    timer2_count--;
                    if (timer2_count == 0) {
                        via_reg[0xd] |= 0x20;     // set T2IL
                    }
                }
                via_reg[8] = timer2_count & 0xff;
                via_reg[9] = (timer2_count >> 8) & 0xff;
            }
        }
        via_reg[0xd] |= 0x40;      // set T1IL
    }
//    }

    via_reg[4] = timer1_count & 0xff;
    via_reg[5] = (timer1_count >> 8) & 0xff;

    // Run timer2

    if ((via_reg[0xb] & 0xc0) == 0) {

        // run timer 2 (independent mode)

        timer2_count = via_reg[8] + (via_reg[9] << 8);

        if (timer2_count > cycles) {
            timer2_count -= cycles;
        } else {
            timer2_count = 0;
            via_reg[0xd] |= 0x20;     // set T2IL
        }
        via_reg[8] = timer2_count & 0xff;
        via_reg[9] = (timer2_count >> 8) & 0xff;
    }

    // CMT input

}

void cpu_writemem16(unsigned short addr, unsigned char bdat) { // RAM access is managed here to allow memory-mapped I/O access via traps

    uint16_t timer_control;

    switch (memmap[addr >> 11]) {

    case 1: // RAM
        RAM[addr & 0x3fff] = bdat;
        break;
    case 2: // VRAM
        vram[addr & 0x3ff] = bdat;
        break;
    case 4: // IO
        switch (addr & 0xf) {
        case 4:
        case 6:
            via_reg[4] = bdat;
            via_reg[6] = bdat;
            break;
        case 5:
        case 7:
            via_reg[5] = bdat;
            via_reg[7] = bdat;
            via_reg[0xd] &= 0xbf; // clear T1IL
            if ((via_reg[0xb] & 0xc0) == 0xc0) {
                timer_control = via_reg[6] + (via_reg[7] << 8);
                TIM4_PWMOut_Init(timer_control * 2, 107, timer_control);
            }
            break;
        case 8:
            via_reg[8] = bdat;
            break;
        case 9:
            via_reg[9] = bdat;
            via_reg[0xd] &= 0xdf; // clear T2IL
            break;
        case 0xa:    // CMT output to USART
            if ((SysTick->CNT - last_usart_tx_systick)
                    > SystemCoreClock / 100) {
                cmt_buff = 0;
                cmt_bit = 0;
            }
            last_usart_tx_systick = SysTick->CNT;
            cmt_buff <<= 1;
            if (bdat == 0xaa) {
                cmt_buff++;
            }
            cmt_bit++;
            if (cmt_bit == 8) {
                printf("%02x", cmt_buff);
                cmt_bit = 0;
                cmt_buff = 0;
            }

            via_reg[0xd] |= 0x84; // set SR flag
            break;
        case 0xb:
            if ((bdat & 0xc0) != 0xc0) {
                TIM4_PWMOut_Init(0, 107, 0);
            }
            via_reg[0xb] = bdat;
            break;
        default:
            via_reg[addr & 0xf] = bdat;
        }
    }

}
//
unsigned char cpu_readmem16(unsigned short addr) { // to allow for memory-mapped I/O access

    switch (memmap[addr >> 11]) {

    case 1: // RAM
        return (RAM[addr & 0x3fff]);

    case 2: // VRAM
        return (vram[addr & 0x3ff]);

    case 3: // ROM
        return (rom[addr & 0x1fff]);
    case 4: // IO
        switch (addr & 0xf) {
        case 0:
            return ((keymatrix[via_reg[1] & 0xf]) | (via_reg[0] & 0xc0));
        case 4:
            via_reg[0xf] &= 0xfd; // clear T1IL
            return via_reg[4];
        case 8:
            via_reg[0xf] &= 0xfb; // clear T2IL
            return via_reg[8];
        default:
            return via_reg[addr & 0xf];
        }

    }

    return 0xff;         // traps go above here

}

void beep_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    /*
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
     */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM4_PWMOut_Init(0, 107, 0);

}

//unsigned char cpu_readop(uint16_t addr) {return cpu_readmem16(addr); };
//unsigned char cpu_readop_arg(uint16_t addr) {return cpu_readmem16(addr); };

// init UART2 for key input

void USART_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    USART_InitTypeDef USART_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART2 TX-->A.2   RX-->A.3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART2, ENABLE);

}

void DMA_Rx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr,
        u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    DMA_Cmd(DMA_CHx, ENABLE);

}

static inline uint8_t usart_getch() {

    uint8_t ch;
    uint32_t currptr;

    currptr = DMA_GetCurrDataCounter(DMA1_Channel6);

    if (currptr == lastptr) {
        return 0;
    }

    /*
     while(1) {
     currptr=DMA_GetCurrDataCounter(DMA1_Channel6);
     if(currptr!=lastptr) break;

     }
     */

    ch = rxbuff[rxptr];
    lastptr--;
    if (lastptr == 0) {
        lastptr = RX_BUFFER_LEN;
    }

    rxptr++;
    if (rxptr >= RX_BUFFER_LEN)
        rxptr = 0;

    return ch;

}

void USART_Getkey() {

    static uint8_t ch, pressed;
    static uint8_t col, row;

    if (pressed == 0) { // no modifier keys pressed
        for (int i = 0; i < 9; i++) {
            keymatrix[i] = 0xff;
        }

        ch = toupper(usart_getch());

        if (ch != 0) {

//            if (ch == '|') { // load test game to memory
//                memcpy(RAM + 0x1000, jr100guldus, 0x1600);
//            }

            if (keymap[ch * 3] < 9) {
                if (keymap[ch * 3 + 2] == 1) { // use modifier (Shift)
                    keymatrix[0] &= ~(2);
                    col = keymap[ch * 3];
                    row = keymap[ch * 3 + 1];
                    pressed = 1;
                } else if (keymap[ch * 3 + 2] == 2) { // Control
                    keymatrix[0] &= ~(1);
                    col = keymap[ch * 3];
                    row = keymap[ch * 3 + 1];
                    pressed = 1;
                } else {
                    keymatrix[keymap[ch * 3]] &= ~(keymap[ch * 3 + 1]);
                }
            }
        }
    } else {
        pressed++;
        if (pressed > 2) {
            keymatrix[col] &= ~(row);
            pressed = 0;
        }
    }

}

//

void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM1_CC_IRQHandler(void) {

    TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
    uint8_t char_x, char_y, slice_y, ch;
    uint8_t clocks, clocks2;

    //  uint8_t render_line;

    ntsc_line++;

    // VSYNC/HSYNC slection for next scanline

    if ((ntsc_line == 3) || (ntsc_line == 4) || (ntsc_line == 5)) { // VSYNC : ntsc_line : 4-6
        TIM_SetCompare4(TIM1, NTSC_VSYNC);
        //    TIM1->CH4CVR = NTSC_VSYNC;
    } else {
        TIM_SetCompare4(TIM1, NTSC_HSYNC);
        //    TIM1->CH4CVR = NTSC_HSYNC;
    }

    // Video Out

    if ((ntsc_line >= NTSC_SCAN_START)
            && (ntsc_line < (NTSC_SCAN_START + NTSC_Y_PIXELS))) { // video out
        ntsc_blank = 0;
        DMA_Tx_Init(DMA1_Channel3, (u32) (&SPI1->DATAR + 1),
                (u32) scandata[ntsc_line % 2], NTSC_X_CHARS + 1);
        DMA_Cmd(DMA1_Channel3, ENABLE);
    } else {
        ntsc_blank = 1;
    }

    // Redner fonts for next scanline

    if ((ntsc_line >= NTSC_SCAN_START - 1)
            && (ntsc_line < (NTSC_SCAN_START + NTSC_Y_PIXELS - 1))) {

        char_y = (ntsc_line + 1 - NTSC_SCAN_START) / 8;
        slice_y = (ntsc_line + 1 - NTSC_SCAN_START) % 8;

        if ((via_reg[0] & 0x20) == 0) {         // PCG not used
            for (char_x = 0; char_x < NTSC_X_CHARS; char_x++) {
                ch = vram[char_x + char_y * NTSC_X_CHARS + 0x100];
                if (ch > 0x7f) {
                    scandata[(ntsc_line + 1) % 2][char_x] = ~rom[(ch & 0x7f) * 8
                            + slice_y];
                } else {
                    scandata[(ntsc_line + 1) % 2][char_x] =
                            rom[ch * 8 + slice_y];
                }
            }
        } else {                          // use PCG
            for (char_x = 0; char_x < NTSC_X_CHARS; char_x++) {
                ch = vram[char_x + char_y * NTSC_X_CHARS + 0x100];
                if (ch > 0x7f) {
                    scandata[(ntsc_line + 1) % 2][char_x] = vram[(ch & 0x7f) * 8
                            + slice_y];
                } else {
                    scandata[(ntsc_line + 1) % 2][char_x] =
                            rom[ch * 8 + slice_y];
                }
            }
        }
    }

    if (ntsc_line > 262) {
        ntsc_line = 0;
    }

    // Execute m6800
    // 63.5us/894KHz~57clocks

    if (run_emulation != 0) {

        clocks2 = 0;
        while(clocks2<57) {
            clocks=m6800_execute(1);
            clocks2+=clocks;
            exec6522(clocks);
        }
    }

}

void cmt_load() {
    uint8_t ch, cmt_bit;
    uint8_t cmt_stage;
    uint16_t bit_count, cmt_ch;
    uint8_t cmt_param[17];
    uint16_t byte_count,byte_ptr;

    // flush rx buffer;

    while(usart_getch()!=0);

    ch = 0;
    cmt_stage = 0;
    cmt_ch = 0;
    bit_count = 0;

    while(1) {   // extract bit
        ch=toupper(usart_getch());
        if(ch!=0) {
//            printf("%x ",ch);

            if(ch=='!') { return; }

            ch-='0';
            if(ch>10) ch-=7;

  //          if(cmt_stage>=3) printf("%x %d \n\r",ch,bit_count);

            for(int i=0;i<4;i++) {
                cmt_bit=(ch>>(3-i))&1;

                switch(cmt_stage) {
                    case 0:            // Preample 4081 bits
                    bit_count++;
                    if(bit_count==4081) {
                        bit_count=0;
                        cmt_stage=1;
                    }
                    break;
                    case 1:                // Filename
                    cmt_ch>>=1;
                    cmt_ch+= (cmt_bit==1)?0x400:0;
                    bit_count++;
                    if(bit_count%11==0) {
                        cmt_ch>>=1;
                        cmt_ch&=0xff;
                        printf("%c",cmt_ch);
                        cmt_ch=0;
                    }
                    if(bit_count==(11*16)) {
                        bit_count=0;
                        cmt_stage=2;
                        printf("\n\r");
                    }
                    break;
                    case 2:                // Parameters
                    cmt_ch>>=1;
                    cmt_ch+= (cmt_bit==1)?0x400:0;
                    bit_count++;
                    if(bit_count%11==0) {
                        cmt_ch>>=1;
                        cmt_ch&=0xff;
                        cmt_param[bit_count/11-1]=cmt_ch;
 //                       printf("%x ",cmt_ch);
                        cmt_ch=0;
                    }
                    if(bit_count==(11*17)) {
                        bit_count=0;
                        cmt_stage=3;
                        byte_ptr=cmt_param[0]*256+cmt_param[1];
                        byte_count=cmt_param[2]*256+cmt_param[3];
                        printf("Start addr %x / Total bytes %x\n\r",byte_ptr,byte_count);
                    }
                    break;
                    case 3:            // interim 255 bits
                    bit_count++;
                    if(bit_count==255) {
                        bit_count=0;
                        cmt_stage=4;
                    }
                    break;
                    case 4:                // Data
                    cmt_ch>>=1;
                    cmt_ch+= (cmt_bit==1)?0x400:0;
                    bit_count++;
                    if(bit_count%11==0) {
                        cmt_ch>>=1;
                        cmt_ch&=0xff;
                        cpu_writemem16(byte_ptr++, cmt_ch);
 //                       printf("%x ",cmt_ch);
                        cmt_ch=0;
                    }
                    if(bit_count==(11*byte_count)) {
                           if(cmt_param[4]==0) {  // set basic tail pointer
                               byte_ptr--;
                               cpu_writemem16(0x6, (byte_ptr>>8)&0xff);
                               cpu_writemem16(0x7, byte_ptr&0xff);
                           }
                           printf("Load end.\n\r");
                           return;
                    }
                    break;
                    default:
                    break;

                }
            }
        }
    }
}

#ifdef USE_PS2_KEYBOARD

void ps2_getkey() {
    uint8_t ps2_keycode, col, row;
    static uint8_t ps2_extra = 0;
    static uint8_t ps2_depressed = 0;

    ps2_keycode = get_scan_code();

    // if(ps2_keycode!=0) printf("%x ",ps2_keycode);

    if (ps2_keycode == 0xe0) {
        ps2_extra = 1;
    } else if (ps2_keycode == 0xf0) {
        ps2_depressed = 1;
    } else if (ps2_keycode != 0) {
        if (ps2_extra == 0) {      // skip extra modifier keys (eg. Right Shift)
            col = ps2_keymap[ps2_keycode * 2];
            row = ps2_keymap[ps2_keycode * 2 + 1];
            if (row != 0) {
                if (ps2_depressed == 0) {
                    keymatrix[col] &= ~row;
                } else {
                    keymatrix[col] |= row;
                }
            } else if (ps2_keycode == 0x11) {
                // ALT: reset keymatrix
                for (int i = 0; i < 9; i++) {
                    keymatrix[i] = 0xff;
                }
            } else if ((ps2_keycode == 0x05) && (ps2_depressed == 0)) {
                // F1: CMT LOAD
                cmt_load();
            } else if ((ps2_keycode == 0x07) && (ps2_depressed == 0)) {
                // F12: load test game to memory
//                memcpy(RAM + 0x1000, jr100guldus, 0x1600);
            }

        }
        ps2_depressed = 0;
        ps2_extra = 0;
    }

}

#endif

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

    // run Systick timer

    SysTick->CNT = 0;
    SysTick->CTLR |= (1 << 0);

//  Peripheral setup

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    USART_CFG();

    DMA_Rx_Init( DMA1_Channel6, (u32) &USART2->DATAR, (u32) &rxbuff,
    RX_BUFFER_LEN);

    beep_init();

    video_init();
    video_cls();

#ifdef USE_PS2_KEYBOARD

    kbd_init();

#endif

//  Emulator setup

    RAM = malloc(MEM_SIZE);
    memset(RAM, 0, MEM_SIZE); // zeroes execute as NOP (as do all undefined instructions)

    for (int i = 0; i < 9; i++) {
        keymatrix[i] = 0xff;
    }

    memset(rxbuff, 0, RX_BUFFER_LEN);

    m6800_init();
    m6800_reset();

    //   Delay_Init();
//
//     for (int xx = 0; xx < NTSC_X_CHARS; xx++) {
//     cpu_writemem16(xx, 0xaa);
//     if (cpu_readmem16(xx) != 0xaa)
//     vram[xx + 0x100] = 0x20;
//     Delay_Ms(10);
//     }

    run_emulation = 1;

    while(1)
    {

        video_wait_vsync();
#ifdef USE_PS2_KEYBOARD
        ps2_getkey();
#else
        USART_Getkey();
#endif

        if((cmt_bit!=0)&&(SysTick->CNT-last_usart_tx_systick>SystemCoreClock/100)) {
            printf("%0x\n\r\n\r",cmt_buff<<(8-cmt_bit));
            cmt_bit=0;
            cmt_buff=0;
        }

    }
}
