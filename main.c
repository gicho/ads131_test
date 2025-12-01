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

    printf("Phase 2: starting simple PIO capture (no DMA)...\r\n");
    ads131_start_pio_simple_capture();

    // We never return
    while (1) {
        sleep_ms(1000);
    }

    return 0;
}
