#include "ads131a04.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include <stdio.h>

static int32_t sign_extend24(uint32_t v) {
    if (v & 0x800000u) {
        v |= 0xFF000000u;
    }
    return (int32_t)v;
}

// DMA channels and buffers
static int dma_tx_chan = -1;
static int dma_rx_chan = -1;
static uint8_t tx_buf[15];
static uint8_t rx_buf[15];

static void ads_spi_init(void) {
    // 24 MHz SPI clock (ADS131A04 supports up to 25 MHz)
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

    // Prepare TX buffer with zeros (we only need clocks)
    for (int i = 0; i < 15; ++i) {
        tx_buf[i] = 0;
    }

    // Setup DMA channels
    dma_tx_chan = dma_claim_unused_channel(true);
    dma_rx_chan = dma_claim_unused_channel(true);

    dma_channel_config c_tx = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_tx, true);
    channel_config_set_write_increment(&c_tx, false);
    channel_config_set_dreq(&c_tx, spi_get_dreq(ADS_SPI_PORT, true)); // TX DREQ
    dma_channel_configure(
        dma_tx_chan,
        &c_tx,
        &spi_get_hw(ADS_SPI_PORT)->dr, // write to SPI DR
        tx_buf,                        // from TX buffer
        0,                             // count set per transfer
        false                          // don't start yet
    );

    dma_channel_config c_rx = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_rx, false);
    channel_config_set_write_increment(&c_rx, true);
    channel_config_set_dreq(&c_rx, spi_get_dreq(ADS_SPI_PORT, false)); // RX DREQ
    dma_channel_configure(
        dma_rx_chan,
        &c_rx,
        rx_buf,                        // write to RX buffer
        &spi_get_hw(ADS_SPI_PORT)->dr, // read from SPI DR
        0,                             // count set per transfer
        false                          // don't start yet
    );
}

// Send 16-bit command framed as 3 bytes (as in the working version)
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

bool ads131_read_reg(uint8_t addr, uint8_t *value) {
    if (!value) return false;
    ads_spi_write_cmd16(ads131_cmd_rreg(addr));
    uint16_t resp = ads_spi_null_and_read16();
    *value = (uint8_t)(resp & 0xFF);
    return true;
}

bool ads131_write_reg(uint8_t addr, uint8_t value) {
    ads_spi_write_cmd16(ads131_cmd_wreg(addr, value));
    (void)ads_spi_null_and_read16(); // ignore response
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

    // Configure clocking for max ODR with 16.384 MHz CLKIN:
    // CLK1=0x02, CLK2=0x2F -> ~128 kSPS
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

bool ads131_read_frame_dma(uint16_t *status_word, int32_t ch[4]) {
    if (!status_word || !ch) return false;

    // Wait for DRDY to go low (new data)
    const uint32_t timeout_us = 10000; // ~100 us timeout
    volatile uint32_t waited = 0;
    while (gpio_get(ADS_PIN_DRDY)) {
        //sleep_us(1);
        if (++waited >= timeout_us) {
            return false;
        }
    }

    // Configure DMA transfer count to 15 bytes
    dma_channel_set_trans_count(dma_tx_chan, 15, false);
    dma_channel_set_trans_count(dma_rx_chan, 15, false);

    // Reset RX buffer (not strictly necessary, but good for debugging)
    // for (int i = 0; i < 15; ++i) rx_buf[i] = 0;

    // Set up addresses (they remain the same, but this is cheap)
    dma_channel_set_read_addr(dma_tx_chan, tx_buf, false);
    dma_channel_set_write_addr(dma_tx_chan, &spi_get_hw(ADS_SPI_PORT)->dr, false);
    dma_channel_set_read_addr(dma_rx_chan, &spi_get_hw(ADS_SPI_PORT)->dr, false);
    dma_channel_set_write_addr(dma_rx_chan, rx_buf, false);

    // Start RX first, then TX
    gpio_put(ADS_PIN_CS, 0);
    dma_start_channel_mask((1u << dma_rx_chan) | (1u << dma_tx_chan));

    // Wait for RX to complete
    dma_channel_wait_for_finish_blocking(dma_rx_chan);
    gpio_put(ADS_PIN_CS, 1);

    // Parse frame: first 2 bytes status, then 4x24-bit channels
    *status_word = (uint16_t)((rx_buf[0] << 8) | rx_buf[1]);

    for (int i = 0; i < 4; ++i) {
        int base = 2 + 3 * i;
        uint32_t raw24 = ((uint32_t)rx_buf[base] << 16) |
                         ((uint32_t)rx_buf[base + 1] << 8) |
                         ((uint32_t)rx_buf[base + 2]);
        ch[i] = sign_extend24(raw24);
    }

    return true;
}
