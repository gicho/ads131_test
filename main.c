#include <stdio.h>
#include "pico/stdlib.h"
#include "ads131a04.h"

int main(void) {
    stdio_init_all();

    // Give USB CDC some time to enumerate
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

    printf("sleeping...\r\n");
    sleep_ms(10);

    printf("Phase 2: starting PIO+DMA double-buffered capture...\r\n");
    ads131_start_pio_dma_capture();

    ads131_frame_buffers_t *fb = ads131_get_frame_buffers();

    uint64_t last_total_frames = 0;
    absolute_time_t last_time = get_absolute_time();

    while (true) {
        sleep_ms(1);

        uint64_t current_total = fb->total_frames;
        uint64_t delta_frames  = current_total - last_total_frames;
        last_total_frames = current_total;

        absolute_time_t now = get_absolute_time();
        int64_t dt_us = absolute_time_diff_us(last_time, now);
        last_time = now;

        double rate = 0.0;
        if (dt_us > 0) {
            rate = (double)delta_frames * 1e6 / (double)dt_us;
        }

        uint32_t dmaA = 0, dmaB = 0;
        ads131_get_dma_counts(&dmaA, &dmaB);

        printf("Frames: total=%llu  delta=%llu  dt=%lld us  rate=%.1f SPS"
               "  dma_errors=%llu  dmaA_count=%u  dmaB_count=%u\r\n",
               (unsigned long long)fb->total_frames,
               (unsigned long long)delta_frames,
               (long long)dt_us,
               rate,
               (unsigned long long)fb->dma_errors,
               dmaA, dmaB);

        // Peek first few words of buffer 0/1 as a sanity check
        if (fb->buffer_full[0]) {
            uint32_t w0 = fb->frames[0][0][0];
            uint32_t w1 = fb->frames[0][0][1];
            printf("Buffer 0 full: first frame words[0]=0x%08lx words[1]=0x%08lx\r\n",
                   (unsigned long)w0, (unsigned long)w1);
            fb->buffer_full[0] = false;
        }
        if (fb->buffer_full[1]) {
            uint32_t w0 = fb->frames[1][0][0];
            uint32_t w1 = fb->frames[1][0][1];
            printf("Buffer 1 full: first frame words[0]=0x%08lx words[1]=0x%08lx\r\n",
                   (unsigned long)w0, (unsigned long)w1);
            fb->buffer_full[1] = false;
        }
    }

    return 0;
}
