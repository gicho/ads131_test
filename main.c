#include <stdio.h>
#include "pico/stdlib.h"
#include "ads131a04.h"

int main(void) {
    stdio_init_all();

    // Allow USB CDC to come up
    for (int i = 0; i < 10; ++i) {
        printf("Tick %d\r\n", i);
        sleep_ms(500);
    }

    printf("Phase 2: calling ads131_init()...\r\n");
    ads131_status_t st = ads131_init();
    if (st != ADS131_OK) {
        printf("ads131_init() failed: %d\r\n", (int)st);
        while (1) {
            sleep_ms(1000);
        }
    }
    printf("ads131_init() returned: OK\r\n");

    uint8_t id_msb = 0, id_lsb = 0, stat_m2 = 0;
    ads131_read_reg(ADS131_REG_ID_MSB, &id_msb);
    ads131_read_reg(ADS131_REG_ID_LSB, &id_lsb);
    ads131_read_reg(ADS131_REG_STAT_M2, &stat_m2);
    printf("ID_MSB = 0x%02X  ID_LSB = 0x%02X  STAT_M2 = 0x%02X\r\n",
           id_msb, id_lsb, stat_m2);

    // Start continuous capture using DRDY IRQ + SPI DMA into double buffer
    ads131_start_continuous_capture();
    printf("Continuous capture started (DRDY IRQ + DMA) ...\r\n");

    ads131_frame_buffers_t *fb = ads131_get_frame_buffers();

    uint64_t last_total_frames = 0;
    absolute_time_t last_time = get_absolute_time();

    while (true) {
        sleep_ms(1000);

        uint64_t current_total = fb->total_frames;
        uint64_t delta_frames = current_total - last_total_frames;
        last_total_frames = current_total;

        absolute_time_t now = get_absolute_time();
        int64_t dt_us = absolute_time_diff_us(last_time, now);
        last_time = now;

        double rate = 0.0;
        if (dt_us > 0) {
            rate = (double)delta_frames * 1e6 / (double)dt_us;
        }

        printf("Frames: total=%llu  overrun=%llu  delta=%llu  dt=%lld us  rate=%.1f SPS\r\n",
               (unsigned long long)fb->total_frames,
               (unsigned long long)fb->overrun_frames,
               (unsigned long long)delta_frames,
               (long long)dt_us,
               rate);

        // Optionally, check buffer_full flags and do something with data here.
        for (int b = 0; b < ADS131_NUM_BUFFERS; ++b) {
            if (fb->buffer_full[b]) {
                printf("Buffer %d full (contains %d frames)\r\n",
                       b, ADS131_FRAMES_PER_BUFFER);
                // TODO: process or stream buffer fb->frames[b][...]
                fb->buffer_full[b] = false;
            }
        }
    }

    return 0;
}
