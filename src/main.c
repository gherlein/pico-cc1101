#include <inttypes.h>
#include <math.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cc1101.h"

uint8_t syncWord2[2] = {0xFF, 10};
bool packetWaiting;

void printByteWise(uint64_t num) {
    printf("%lld\n", num);

    printf("byte 0 = %u\n", (num & 0xFF00000000000000));
    printf("byte 1 = %u\n", (num & 0x00FF000000000000));
    printf("byte 2 = %u\n", (num & 0x0000FF0000000000));
    printf("byte 3 = %u\n", (num & 0x000000FF00000000));
    printf("byte 4 = %u\n", (num & 0x00000000FF000000));
    printf("byte 5 = %u\n", (num & 0x0000000000FF0000));
    printf("byte 6 = %u\n", (num & 0x000000000000FF00));
    printf("byte 7 = %u\n", (num & 0x00000000000000FF));
    printf("\n");
}

int main() {
    stdout_uart_init();

    printf("started!\n");

    // uint32_t basefreq = 300000000UL;
    uint32_t basefreq = 433000000UL;

    init(basefreq, MODE_LOW_SPEED);

    uint8_t c = 0;
    const char *message = "hello world";
    CCPACKET packet;
    packet.length = strlen(message) + 1;
    strncpy((char *)packet.data, message, packet.length);

    disableCCA();
    while (1) {
        // uint32_t f = basefreq + (c * 500000UL);
        //  setCarrierFreq(f);
        sleep_us(500);
        c++;
        printf("%d ", c);
        sendData(packet);
    }
}
