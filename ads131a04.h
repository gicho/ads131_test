#ifndef ADS131A04_H
#define ADS131A04_H

#include <stdint.h>
#include <stdbool.h>

// Pin mapping (Pico GPIOs)
#define ADS_SPI_PORT   spi0
#define ADS_PIN_MISO   16   // ADC DOUT
#define ADS_PIN_CS     17   // ADC /CS
#define ADS_PIN_SCK    18   // ADC SCLK
#define ADS_PIN_MOSI   19   // ADC DIN
#define ADS_PIN_DRDY   20   // ADC /DRDY
#define ADS_PIN_RESET  21   // ADC /RESET

// Command opcodes
#define ADS131_CMD_NULL      0x0000
#define ADS131_CMD_RESET     0x0011
#define ADS131_CMD_STANDBY   0x0022
#define ADS131_CMD_WAKEUP    0x0033
#define ADS131_CMD_LOCK      0x0555
#define ADS131_CMD_UNLOCK    0x0655

// Registers
#define ADS131_REG_ID_MSB    0x00
#define ADS131_REG_ID_LSB    0x01
#define ADS131_REG_STAT_M2   0x07
#define ADS131_REG_D_SYS_CFG 0x0C
#define ADS131_REG_CLK1      0x0D
#define ADS131_REG_CLK2      0x0E
#define ADS131_REG_ADC_ENA   0x0F

static inline uint16_t ads131_cmd_rreg(uint8_t addr) {
    uint8_t msb = 0x20u | (addr & 0x1Fu);
    return (uint16_t)msb << 8;
}
static inline uint16_t ads131_cmd_wreg(uint8_t addr, uint8_t data) {
    uint8_t msb = 0x40u | (addr & 0x1Fu);
    return ((uint16_t)msb << 8) | data;
}

typedef enum {
    ADS131_OK = 0,
    ADS131_ERR_UNLOCK,
    ADS131_ERR_WAKEUP,
    ADS131_ERR_SPI
} ads131_status_t;

// PIO+DMA frame layout
#define ADS131_WORDS_PER_FRAME   5
#define ADS131_FRAMES_PER_BUFFER 1024
#define ADS131_NUM_BUFFERS       2

typedef struct {
    // Raw data: [buffer][frame][word]
    uint32_t frames[ADS131_NUM_BUFFERS]
                   [ADS131_FRAMES_PER_BUFFER]
                   [ADS131_WORDS_PER_FRAME];

    volatile bool     buffer_full[ADS131_NUM_BUFFERS];
    volatile uint64_t total_frames;   // total frames captured (all buffers)
    volatile uint64_t dma_errors;     // reserved for future use
} ads131_frame_buffers_t;

// SPI-based init: reset, unlock, wakeup, clock config, channel enable
ads131_status_t ads131_init(void);

// Access to global frame buffers
ads131_frame_buffers_t *ads131_get_frame_buffers(void);

// Start continuous PIO+DMA double-buffered capture
void ads131_start_pio_dma_capture(void);

// Debug helper: read remaining transfer counts for both DMA channels
void ads131_get_dma_counts(uint32_t *a_count, uint32_t *b_count);

#endif // ADS131A04_H