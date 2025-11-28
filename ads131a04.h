#ifndef ADS131A04_H
#define ADS131A04_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO / SPI mapping for Pico 2 (RP2350)
#define ADS_SPI_PORT   spi0
#define ADS_PIN_MISO   16
#define ADS_PIN_CS     17
#define ADS_PIN_SCK    18
#define ADS_PIN_MOSI   19
#define ADS_PIN_DRDY   20
#define ADS_PIN_RESET  21

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

// Build RREG / WREG command (single register)
static inline uint16_t ads131_cmd_rreg(uint8_t addr) {
    uint8_t msb = 0x20u | (addr & 0x1Fu);
    return (uint16_t)msb << 8;
}
static inline uint16_t ads131_cmd_wreg(uint8_t addr, uint8_t data) {
    uint8_t msb = 0x40u | (addr & 0x1Fu);
    return ((uint16_t)msb << 8) | data;
}

// Status codes for init
typedef enum {
    ADS131_OK = 0,
    ADS131_ERR_UNLOCK,
    ADS131_ERR_WAKEUP,
    ADS131_ERR_SPI
} ads131_status_t;

ads131_status_t ads131_init(void);
bool ads131_read_reg(uint8_t addr, uint8_t *value);
bool ads131_write_reg(uint8_t addr, uint8_t value);

// Read one conversion frame: status + 4x24-bit channels
// Returns true on success, false on DRDY timeout
bool ads131_read_frame(uint16_t *status_word, int32_t ch[4]);

#ifdef __cplusplus
}
#endif

#endif // ADS131A04_H
