#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cc1101.h"

uint8_t syncWord2[2] = {0xFF, 10};
bool packetWaiting;

int main() {
    stdout_uart_init();

    printf("started!\n");

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
