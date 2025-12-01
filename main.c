#include <stdio.h>
#include "pico/stdlib.h"
#include "ads131a04.h"

int main(void) {
    stdio_init_all();

    // Let USB CDC come up
    for (int i = 0; i < 10; ++i) {
        printf("Tick %d\r\n", i);
        sleep_ms(500);
    }

    printf("Phase 1: ads131_init() via SPI...\r\n");
    ads131_status_t st = ads131_init();
    if (st != ADS131_OK) {
        printf("ads131_init() failed: %d\r\n", (int)st);
        while (1) {
            sleep_ms(1000);
        }
    }
    printf("ads131_init() returned: OK\r\n");

    printf("Phase 2: starting PIO+DMA double-buffered capture...\r\n");
    ads131_start_pio_dma_capture();

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

        printf("Frames: total=%llu  delta=%llu  dt=%lld us  rate=%.1f SPS  dma_errors=%llu\r\n",
               (unsigned long long)fb->total_frames,
               (unsigned long long)delta_frames,
               (long long)dt_us,
               rate,
               (unsigned long long)fb->dma_errors);

        for (int b = 0; b < ADS131_NUM_BUFFERS; ++b) {
            if (fb->buffer_full[b]) {
                printf("Buffer %d full (contains %d frames)\r\n",
                       b, ADS131_FRAMES_PER_BUFFER);
                // TODO: here you can process or forward fb->frames[b][...]
                fb->buffer_full[b] = false;
            }
        }
    }

    return 0;
}
