#include "ads131a04.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>

static int32_t sign_extend24(uint32_t v) {
    if (v & 0x800000u) {
        v |= 0xFF000000u;
    }
    return (int32_t)v;
}

static void ads_spi_init(void) {
    // 16 MHz SPI clock (ADS131A04 supports up to 25 MHz)
    spi_init(ADS_SPI_PORT, 24 * 1000 * 1000);

    spi_set_format(ADS_SPI_PORT,
                   8,          // bits per transfer
                   SPI_CPOL_0, // mode 1
                   SPI_CPHA_1,
                   SPI_MSB_FIRST);

    gpio_set_function(ADS_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(ADS_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ADS_PIN_SCK,  GPIO_FUNC_SPI);

    gpio_init(ADS_PIN_CS);
    gpio_set_dir(ADS_PIN_CS, GPIO_OUT);
    gpio_put(ADS_PIN_CS, 1);

    gpio_init(ADS_PIN_RESET);
    gpio_set_dir(ADS_PIN_RESET, GPIO_OUT);
    gpio_put(ADS_PIN_RESET, 1);

    gpio_init(ADS_PIN_DRDY);
    gpio_set_dir(ADS_PIN_DRDY, GPIO_IN);
}

// Send 16-bit command framed as 3 bytes (as in the working version)
static void ads_spi_write_cmd16(uint16_t cmd) {
    uint8_t tx[3] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF),
        0x00
    };
    gpio_put(ADS_PIN_CS, 0);
    spi_write_blocking(ADS_SPI_PORT, tx, 3);
    gpio_put(ADS_PIN_CS, 1);
}

// Clock out 3 bytes of zeros and read back 3 bytes, return top 16 bits
static uint16_t ads_spi_null_and_read16(void) {
    uint8_t tx[3] = {0, 0, 0};
    uint8_t rx[3] = {0, 0, 0};
    gpio_put(ADS_PIN_CS, 0);
    spi_write_read_blocking(ADS_SPI_PORT, tx, rx, 3);
    gpio_put(ADS_PIN_CS, 1);
    return (uint16_t)((rx[0] << 8) | rx[1]);
}

bool ads131_read_reg(uint8_t addr, uint8_t *value) {
    if (!value) return false;
    ads_spi_write_cmd16(ads131_cmd_rreg(addr));
    uint16_t resp = ads_spi_null_and_read16();
    *value = (uint8_t)(resp & 0xFF);
    return true;
}

bool ads131_write_reg(uint8_t addr, uint8_t value) {
    ads_spi_write_cmd16(ads131_cmd_wreg(addr, value));
    (void)ads_spi_null_and_read16(); // ignore response for now
    return true;
}

ads131_status_t ads131_init(void) {
    ads_spi_init();

    // Reset pulse
    gpio_put(ADS_PIN_RESET, 0);
    sleep_ms(1);
    gpio_put(ADS_PIN_RESET, 1);
    sleep_ms(5);

    // READY word after reset
    uint16_t ready = ads_spi_null_and_read16();
    printf("READY word = 0x%04X\r\n", ready);

    // UNLOCK
    ads_spi_write_cmd16(ADS131_CMD_UNLOCK);
    uint16_t unlock_ack = ads_spi_null_and_read16();
    printf("UNLOCK ack = 0x%04X\r\n", unlock_ack);
    // The device echoes the last command word; check upper bits loosely
    if ((unlock_ack & 0xFF00u) != (ADS131_CMD_UNLOCK & 0xFF00u)) {
        return ADS131_ERR_UNLOCK;
    }

    // WAKEUP
    ads_spi_write_cmd16(ADS131_CMD_WAKEUP);
    uint16_t wake_ack = ads_spi_null_and_read16();
    printf("WAKEUP ack = 0x%04X\r\n", wake_ack);
    if ((wake_ack & 0xFF00u) != (ADS131_CMD_WAKEUP & 0xFF00u)) {
        return ADS131_ERR_WAKEUP;
    }

    // Configure clocking for high data rate using 16.384 MHz CLKIN.
    // Example: CLKSRC=0 (XTAL1/CLKIN), CLK_DIV=2 (0010b) => fICLK = fCLKIN/2
    // CLK2: ICLK_DIV=001b, OSR=1111b => max ODR ~128 kSPS
    if (!ads131_write_reg(ADS131_REG_CLK1, 0x02)) {
        return ADS131_ERR_SPI;
    }
    if (!ads131_write_reg(ADS131_REG_CLK2, 0x2F)) {
        return ADS131_ERR_SPI;
    }

    uint8_t clk1 = 0, clk2 = 0;
    ads131_read_reg(ADS131_REG_CLK1, &clk1);
    ads131_read_reg(ADS131_REG_CLK2, &clk2);
    printf("CLK1=0x%02X CLK2=0x%02X\r\n", clk1, clk2);

    // Enable all four channels
    if (!ads131_write_reg(ADS131_REG_ADC_ENA, 0x0F)) {
        return ADS131_ERR_SPI;
    }

    return ADS131_OK;
}

bool ads131_read_frame(uint16_t *status_word, int32_t ch[4]) {
    if (!status_word || !ch) return false;

    // Wait for DRDY to go low (new data)
    const uint32_t timeout_us = 100; // ~100 us timeout
    uint32_t waited = 0;
    while (gpio_get(ADS_PIN_DRDY)) {
        sleep_us(1);
        if (++waited >= timeout_us) {
            return false;
        }
    }

    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};

    gpio_put(ADS_PIN_CS, 0);
    spi_write_read_blocking(ADS_SPI_PORT, tx, rx, 15);
    gpio_put(ADS_PIN_CS, 1);

    *status_word = (uint16_t)((rx[0] << 8) | rx[1]);

    for (int i = 0; i < 4; ++i) {
        int base = (i + 1) * 3;
        uint32_t raw24 = ((uint32_t)rx[base] << 16) |
                         ((uint32_t)rx[base + 1] << 8) |
                         ((uint32_t)rx[base + 2]);
        ch[i] = sign_extend24(raw24);
    }

    return true;
}
