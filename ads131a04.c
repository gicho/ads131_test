#include "ads131a04.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <stdio.h>

#include "ads131a04_capture.pio.h"

// --- SPI helper functions for register access (used only during init) ---

static void ads_spi_init_for_config(void) {
    // Basic SPI0 config for ADS131A04 register access
    spi_init(ADS_SPI_PORT, 24 * 1000 * 1000); // 4 MHz

    spi_set_format(ADS_SPI_PORT,
                   8,          // bits per transfer
                   SPI_CPOL_0, // mode 1: CPOL=0, CPHA=1
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

static void ads_spi_deinit_after_config(void) {
    // Return pins to GPIO; PIO will reassign them later
    spi_deinit(ADS_SPI_PORT);

    gpio_set_function(ADS_PIN_MISO, GPIO_FUNC_SIO);
    gpio_set_function(ADS_PIN_MOSI, GPIO_FUNC_SIO);
    gpio_set_function(ADS_PIN_SCK,  GPIO_FUNC_SIO);

    // Make sure CS is high and under GPIO control before PIO grabs it
    gpio_put(ADS_PIN_CS, 1);
    gpio_set_function(ADS_PIN_CS, GPIO_FUNC_SIO);
}

static void ads_spi_write_cmd16(uint16_t cmd) {
    uint8_t tx[3] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF),
        0x00
    };
    uint8_t rx[3] = {0};
    gpio_put(ADS_PIN_CS, 0);
    spi_write_read_blocking(ADS_SPI_PORT, tx, rx, 3);
    gpio_put(ADS_PIN_CS, 1);
}

static uint16_t ads_spi_null_and_read16(void) {
    uint8_t tx[3] = {0, 0, 0};
    uint8_t rx[3] = {0, 0, 0};
    gpio_put(ADS_PIN_CS, 0);
    spi_write_read_blocking(ADS_SPI_PORT, tx, rx, 3);
    gpio_put(ADS_PIN_CS, 1);
    return (uint16_t)((rx[0] << 8) | rx[1]);
}

static bool ads131_read_reg_spi(uint8_t addr, uint8_t *value) {
    if (!value) return false;
    ads_spi_write_cmd16(ads131_cmd_rreg(addr));
    uint16_t resp = ads_spi_null_and_read16();
    *value = (uint8_t)(resp & 0xFF);
    return true;
}

static bool ads131_write_reg_spi(uint8_t addr, uint8_t value) {
    ads_spi_write_cmd16(ads131_cmd_wreg(addr, value));
    (void)ads_spi_null_and_read16(); // ignore response
    return true;
}

// --- Public init: reset, unlock, wakeup, configure CLK and enable channels ---

ads131_status_t ads131_init(void) {
    ads_spi_init_for_config();

    // Reset pulse
    gpio_put(ADS_PIN_RESET, 0);
    sleep_ms(1);
    gpio_put(ADS_PIN_RESET, 1);
    sleep_ms(5);

    uint16_t ready = ads_spi_null_and_read16();
    printf("READY word = 0x%04X\r\n", ready);

    // UNLOCK
    ads_spi_write_cmd16(ADS131_CMD_UNLOCK);
    uint16_t unlock_ack = ads_spi_null_and_read16();
    printf("UNLOCK ack = 0x%04X\r\n", unlock_ack);
    if ((unlock_ack & 0xFF00u) != (ADS131_CMD_UNLOCK & 0xFF00u)) {
        ads_spi_deinit_after_config();
        return ADS131_ERR_UNLOCK;
    }

    // WAKEUP
    ads_spi_write_cmd16(ADS131_CMD_WAKEUP);
    uint16_t wake_ack = ads_spi_null_and_read16();
    printf("WAKEUP ack = 0x%04X\r\n", wake_ack);
    if ((wake_ack & 0xFF00u) != (ADS131_CMD_WAKEUP & 0xFF00u)) {
        ads_spi_deinit_after_config();
        return ADS131_ERR_WAKEUP;
    }

    // Configure clocking for ~128 kSPS with 16.384 MHz CLKIN
    if (!ads131_write_reg_spi(ADS131_REG_CLK1, 0x02)) {
        ads_spi_deinit_after_config();
        return ADS131_ERR_SPI;
    }
    if (!ads131_write_reg_spi(ADS131_REG_CLK2, 0x2F)) {
        ads_spi_deinit_after_config();
        return ADS131_ERR_SPI;
    }

    uint8_t clk1 = 0, clk2 = 0;
    ads131_read_reg_spi(ADS131_REG_CLK1, &clk1);
    ads131_read_reg_spi(ADS131_REG_CLK2, &clk2);
    printf("CLK1=0x%02X CLK2=0x%02X\r\n", clk1, clk2);

    // Enable all four channels
    if (!ads131_write_reg_spi(ADS131_REG_ADC_ENA, 0x0F)) {
        ads_spi_deinit_after_config();
        return ADS131_ERR_SPI;
    }

    // Optional: ID and STAT_M2
    uint8_t id_msb = 0, id_lsb = 0, stat_m2 = 0;
    ads131_read_reg_spi(ADS131_REG_ID_MSB, &id_msb);
    ads131_read_reg_spi(ADS131_REG_ID_LSB, &id_lsb);
    ads131_read_reg_spi(ADS131_REG_STAT_M2, &stat_m2);
    printf("ID_MSB = 0x%02X  ID_LSB = 0x%02X  STAT_M2 = 0x%02X\r\n",
           id_msb, id_lsb, stat_m2);

    ads_spi_deinit_after_config();

    return ADS131_OK;
}

// --- Simple PIO-only capture (no DMA) for debugging ---

void ads131_start_pio_simple_capture(void) {
    PIO  pio = pio0;
    uint sm  = 0;

    // Load program
    uint offset = pio_add_program(pio, &ads131a04_capture_program);
    pio_sm_config c = ads131a04_capture_program_get_default_config(offset);

    // Map pins:
    // - SCK as sideset
    // - CS as SET pin (1 bit)
    // - MISO as IN base
    sm_config_set_sideset_pins(&c, ADS_PIN_SCK);
    sm_config_set_set_pins(&c, ADS_PIN_CS, 1);
    sm_config_set_in_pins(&c, ADS_PIN_MISO);

    // Shift config: right, autopush every 32 bits -> 4 words per 128-bit frame
    sm_config_set_in_shift(&c, true, true, 32);

    // Configure GPIOs for PIO
    pio_gpio_init(pio, ADS_PIN_SCK);
    pio_gpio_init(pio, ADS_PIN_CS);
    pio_gpio_init(pio, ADS_PIN_MISO);
    pio_gpio_init(pio, ADS_PIN_DRDY);

    pio_sm_set_consecutive_pindirs(pio, sm, ADS_PIN_SCK, 1, true);   // SCK out
    pio_sm_set_consecutive_pindirs(pio, sm, ADS_PIN_CS,  1, true);   // CS out
    pio_sm_set_consecutive_pindirs(pio, sm, ADS_PIN_MISO,1, false);  // MISO in
    pio_sm_set_consecutive_pindirs(pio, sm, ADS_PIN_DRDY,1, false);  // DRDY in

    // Init SM and set CS high initially (uses SET pin group)
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));

    // Compute clock divider for ~25 MHz SCK (2 instructions per bit)
    uint32_t sys_hz    = clock_get_hz(clk_sys);
    float    target_sck = 25e6f;
    float    div        = (float)sys_hz / (target_sck * 2.0f);
    if (div < 1.0f) div = 1.0f;
    sm_config_set_clkdiv(&c, div);

    // Re-init SM with final config and enable it
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    printf("PIO simple capture started (no DMA)...\r\n");

    uint64_t frame_count = 0;
    absolute_time_t last_time = get_absolute_time();

    uint32_t w[ADS131_WORDS_PER_FRAME];

    while (true) {
        // One ADS frame = 4 words (status + 4x24b packed across 4 words)
        for (int i = 0; i < ADS131_WORDS_PER_FRAME; ++i) {
            w[i] = pio_sm_get_blocking(pio, sm);
        }
        frame_count++;

        if ((frame_count & 0x3FFu) == 0) { // every 1024 frames
            absolute_time_t now = get_absolute_time();
            int64_t dt_us = absolute_time_diff_us(last_time, now);
            last_time = now;

            double rate = 0.0;
            if (dt_us > 0) {
                rate = (double)1024 * 1e6 / (double)dt_us;
            }

            printf("Frames=%llu  dt=%lld us  rate=%.1f SPS  "
                   "w0=0x%08lx w1=0x%08lx w2=0x%08lx w3=0x%08lx\r\n",
                   (unsigned long long)frame_count,
                   (long long)dt_us,
                   rate,
                   (unsigned long)w[0],
                   (unsigned long)w[1],
                   (unsigned long)w[2],
                   (unsigned long)w[3]);
        }
    }
}
