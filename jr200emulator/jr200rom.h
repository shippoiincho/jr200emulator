#define ROMBASE 0x10020000u

uint8_t *basicrom=(uint8_t *)(ROMBASE);            // BASIC   0x10020000 (8KiB)
uint8_t *monitorrom=(uint8_t *)(ROMBASE+0x2000);   // MONITOR 0x10022000 (8KiB)
uint8_t *extrom=(uint8_t *)(ROMBASE+0x4000);       // FD      0x10024000 (2KiB)
uint8_t *fontrom=(uint8_t *)(ROMBASE+0x6000);      // FONT    0x10026000 (2KiB)