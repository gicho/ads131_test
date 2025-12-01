#include "ads131a04.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include <stdio.h>

#include "ads131a04_capture.pio.h"

static int32_t sign_extend24(uint32_t v) {
    if (v & 0x800000u) {
        v |= 0xFF000000u;
    }
    return (int32_t)v;
}

// --- SPI helper functions for register access (used only during init) ---

static void ads_spi_init_for_config(void) {
    // Use SPI0 on the same pins as before, for config only
    spi_init(ADS_SPI_PORT, 4 * 1000 * 1000); // 4 MHz is enough for config

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

static void ads_spi_deinit_after_config(void) {
    // Return pins to GPIO; PIO init will reassign them later
    spi_deinit(ADS_SPI_PORT);

    gpio_set_function(ADS_PIN_MISO, GPIO_FUNC_SIO);
    gpio_set_function(ADS_PIN_MOSI, GPIO_FUNC_SIO);
    gpio_set_function(ADS_PIN_SCK,  GPIO_FUNC_SIO);

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

    // Configure clocking for max ODR with 16.384 MHz CLKIN:
    // CLK1=0x02, CLK2=0x2F -> ~128 kSPS (as before)
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

    // Optionally read ID and STAT_M2 for debug
    uint8_t id_msb = 0, id_lsb = 0, stat_m2 = 0;
    ads131_read_reg_spi(ADS131_REG_ID_MSB, &id_msb);
    ads131_read_reg_spi(ADS131_REG_ID_LSB, &id_lsb);
    ads131_read_reg_spi(ADS131_REG_STAT_M2, &stat_m2);
    printf("ID_MSB = 0x%02X  ID_LSB = 0x%02X  STAT_M2 = 0x%02X\r\n",
           id_msb, id_lsb, stat_m2);

    // Done with SPI config; free pins for PIO
    ads_spi_deinit_after_config();

    return ADS131_OK;
}

// --- PIO + DMA double-buffered capture ---

static ads131_frame_buffers_t g_buffers;

static PIO g_pio = pio0;
static uint g_sm = 0;
static uint g_pio_offset = 0;

static int g_dma_chan_a = -1;
static int g_dma_chan_b = -1;

ads131_frame_buffers_t *ads131_get_frame_buffers(void) {
    return &g_buffers;
}

static void __not_in_flash_func(ads131_dma_irq_handler)(void) {
    uint32_t ints = dma_hw->ints0;

    if (ints & (1u << g_dma_chan_a)) {
        dma_hw->ints0 = (1u << g_dma_chan_a);
        g_buffers.buffer_full[0] = true;
        g_buffers.total_frames += ADS131_FRAMES_PER_BUFFER;
    }
    if (ints & (1u << g_dma_chan_b)) {
        dma_hw->ints0 = (1u << g_dma_chan_b);
        g_buffers.buffer_full[1] = true;
        g_buffers.total_frames += ADS131_FRAMES_PER_BUFFER;
    }
}

// Initialize PIO SM and DMA ping-pong for continuous capture
void ads131_start_pio_dma_capture(void) {
    // Clear buffers state
    for (int b = 0; b < ADS131_NUM_BUFFERS; ++b) {
        g_buffers.buffer_full[b] = false;
    }
    g_buffers.total_frames = 0;
    g_buffers.dma_errors = 0;

    // Load PIO program
    g_sm = 0; // use SM 0
    g_pio_offset = pio_add_program(g_pio, &ads131a04_capture_program);

    pio_sm_config c = ads131a04_capture_program_get_default_config(g_pio_offset);

    // Map pins:
    // - sideset: SCK
    // - set pins: CS
    // - in pins: MISO
    // - jmp pin: DRDY
    sm_config_set_sideset_pins(&c, ADS_PIN_SCK);
    sm_config_set_set_pins(&c, ADS_PIN_CS, 1);
    sm_config_set_in_pins(&c, ADS_PIN_MISO);
    sm_config_set_jmp_pin(&c, ADS_PIN_DRDY);
    

    // Shift config: shift right, autopush every 32 bits
    sm_config_set_in_shift(&c, true, true, 32);

    // Set pin directions
    pio_gpio_init(g_pio, ADS_PIN_SCK);
    pio_gpio_init(g_pio, ADS_PIN_CS);
    pio_gpio_init(g_pio, ADS_PIN_MISO);
    pio_gpio_init(g_pio, ADS_PIN_DRDY);

    // SCK and CS are outputs, MISO and DRDY are inputs
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, ADS_PIN_SCK, 1, true);  // SCK out (sideset)
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, ADS_PIN_CS, 1, true);   // CS out (set)
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, ADS_PIN_MISO, 1, false);// MISO in (in_base)
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, ADS_PIN_DRDY, 1, false);// DRDY in (jmp_pin)

    // Configure initial pin states: CS high, SCK low
    pio_sm_exec(g_pio, g_sm, pio_encode_set(pio_pins, 1)); // CS=1
    //pio_sm_exec(g_pio, g_sm, pio_encode_sideset(0));       // SCK low (implicitly)

    // Compute clock divider to get ~25 MHz SCK:
    // SCK frequency = PIO_clk / (clkdiv * 2) because our loop uses two instructions per bit
    uint32_t sys_hz = clock_get_hz(clk_sys);
    float target_sck = 25e6f;
    float div = (float)sys_hz / (target_sck * 2.0f);
    if (div < 1.0f) div = 1.0f;
    sm_config_set_clkdiv(&c, div);

    // Initialize state machine
    pio_sm_init(g_pio, g_sm, g_pio_offset, &c);
    pio_sm_set_enabled(g_pio, g_sm, true);

    // --- DMA setup: two channels ping-pong between the two buffers ---

    // Each frame produces ADS131_WORDS_PER_FRAME 32-bit words
    const uint32_t words_per_buffer = ADS131_FRAMES_PER_BUFFER * ADS131_WORDS_PER_FRAME;

    g_dma_chan_a = dma_claim_unused_channel(true);
    g_dma_chan_b = dma_claim_unused_channel(true);

    dma_channel_config ca = dma_channel_get_default_config(g_dma_chan_a);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_dreq(&ca, pio_get_dreq(g_pio, g_sm, false)); // RX FIFO
    channel_config_set_chain_to(&ca, g_dma_chan_b);

    dma_channel_configure(
        g_dma_chan_a,
        &ca,
        g_buffers.frames[0],                 // write address (buffer 0)
        &g_pio->rxf[g_sm],                   // read from PIO RX FIFO
        words_per_buffer,
        false
    );

    dma_channel_config cb = dma_channel_get_default_config(g_dma_chan_b);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, true);
    channel_config_set_dreq(&cb, pio_get_dreq(g_pio, g_sm, false));
    channel_config_set_chain_to(&cb, g_dma_chan_a);

    dma_channel_configure(
        g_dma_chan_b,
        &cb,
        g_buffers.frames[1],                 // write address (buffer 1)
        &g_pio->rxf[g_sm],
        words_per_buffer,
        false
    );

    // Enable IRQ0 for both DMA channels
    dma_channel_set_irq0_enabled(g_dma_chan_a, true);
    dma_channel_set_irq0_enabled(g_dma_chan_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, ads131_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Start with channel A; it will chain to B, and B back to A, continuously
    dma_start_channel_mask(1u << g_dma_chan_a);

    printf("PIO+DMA continuous capture started (ping-pong buffers)...\r\n");
}
