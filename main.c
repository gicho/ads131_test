#include <stdio.h>
#include "pico/stdlib.h"
#include "ads131a04.h"

int main(void) {
    stdio_init_all();

    // USB bring-up
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

    const uint32_t N_SAMPLES = 1024;
    uint32_t batch = 0;

    printf("Starting DMA throughput benchmark: N=%u frames per batch\r\n", N_SAMPLES);
    printf("Each frame = status + 4x24-bit channels (15 bytes clocked)\r\n");

    while (true) {
        uint32_t ok = 0;
        uint32_t timeouts = 0;
        uint16_t status = 0;
        int32_t ch[4];

        uint64_t t0 = time_us_64();
        for (uint32_t i = 0; i < N_SAMPLES; ++i) {
            if (ads131_read_frame_dma(&status, ch)) {
                ok++;
            } else {
                timeouts++;
            }
        }
        uint64_t t1 = time_us_64();
        uint64_t dt_us = t1 - t0;

        uint64_t rate_x10 = 0;
        if (dt_us > 0 && ok > 0) {
            rate_x10 = (uint64_t)ok * 10000000ull / dt_us;
        }
        uint32_t sps_int = (uint32_t)(rate_x10 / 10);
        uint32_t sps_frac = (uint32_t)(rate_x10 % 10);

        printf("Batch %lu: N=%u ok=%u timeouts=%u dt=%llu us  rate=%lu.%lu SPS\r\n",
               (unsigned long)batch,
               N_SAMPLES,
               ok,
               timeouts,
               (unsigned long long)dt_us,
               (unsigned long)sps_int,
               (unsigned long)sps_frac);

        batch++;
        sleep_ms(500);
    }

    return 0;
}
