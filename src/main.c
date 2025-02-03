#include <inttypes.h>
#include <math.h>
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

    uint64_t f = 26000000UL / ((1UL << 16) * 433000000UL);
    // printByteWise(f);
    f = 26000000UL / ((1UL << 16) * 915000000UL);
    // printByteWise(f);
    uint32_t x = 0x0010A762;
    printf("%ul\n", x);
    x = 0x00220E0F;
    printf("%ul\n", x);

    init(CFREQ_433, MODE_LOW_SPEED);
    // setSyncWord(syncWord2[0], syncWord2[1]);
    // setCarrierFreq(CFREQ_433);
    // disableAddressCheck();
    // setTxPowerAmp(PA_LongDistance);

    const char *message = "hello world";
    printf("message: %s\n", message);
    CCPACKET packet;
    // We also need to include the 0 byte at the end of the string
    packet.length = strlen(message) + 1;
    strncpy((char *)packet.data, message, packet.length);

    while (1) {
        printf("sending packet...\n");
        sendData(packet);
        sleep_ms(100);
        printf("sent packet\n");
    }
    // sendData(packet);
}
