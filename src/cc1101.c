/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 * Copyright (c) 2016 Tyler Sommer <contact@tylersommer.pro>
 * Copyright (c) 2025 Greg Herlein <gherlein@herlein.com>
 *
 * This file is derived from the CC1101 project.  It's ported to C using the RPi Pico SDK.
 *
 * CC1101 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 *
 * CC1101 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with CC1101; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 * USA
 *
 * Author: Greg Herlein
 * Creation date: 02/01/2025
 */

#include "cc1101.h"

#include <hardware/spi.h>
#include <pico/time.h>
#include <stdint.h>

spi_inst_t *spi = spi0;
bool cc1101_rdy;
uint8_t cc1101_state;
uint8_t rx_fifo_bytes;
uint8_t tx_fifo_bytes;
uint8_t csb;

#define CC1101_STATE_IDLE 0
#define CC1101_STATE_RX 1
#define CC1101_STATE_TX 2
#define CC1101_STATE_FSTXON 3
#define CC1101_STATE_CALIBRATE 4
#define CC1101_STATE_SETTLING 5
#define CC1101_STATE_RX_OVERFLOW 6
#define CC1101_STATE_TX_UNDERFLOW 7

bool startSPI() {
    // Initialize CS pin high
    gpio_init(SS);
    gpio_set_dir(SS, GPIO_OUT);
    gpio_put(SS, 1);

    gpio_set_dir(CC1101_GDO0, GPIO_IN);

    // Initialize SPI port at 1 MHz
    spi_init(spi, 1000 * 1000);

    // Set SPI format
    spi_set_format(spi0,  // SPI instance
                   8,     // Number of bits per transfer
                   1,     // Polarity (CPOL)
                   1,     // Phase (CPHA)
                   SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(CLK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MISO, GPIO_FUNC_SPI);

    printf("CLK: %d   MOSI:  %d   MISO:  %d\n", CLK, MOSI, MISO);

    printf("SPI initialized\n");
    sleep_ms(10);
    return true;
}

/**
 * Macros
 */
// Select (SPI) CC1101
void cc1101_Select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SS, 0);
    asm volatile("nop \n nop \n nop");
}
// Deselect (SPI) CC1101
void cc1101_Deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SS, 1);
    asm volatile("nop \n nop \n nop");
}
// Wait until SPI MISO line goes low
#define wait_Miso() while (gpio_get(MISO) != 0)
// Get GDO0 pin state
#define getGDO0state() digitalRead(GPIO17)
// Wait until GDO0 line goes high
#define wait_GDO0_high() while (gpio_get(GPIO17) != 1)
// Wait until GDO0 line goes low
#define wait_GDO0_low() while (gpio_get(GPIO17) != 0)

bool bitRead(uint8_t *x, char n) { return (*x & (1 << n)) ? 1 : 0; }

/**
 * PATABLE
 */
// const uint8_t paTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};

uint8_t rfState;
uint32_t carrierFreq;
uint8_t workMode;
uint8_t channel;
uint8_t syncWord[2];
uint8_t devAddress;

/**
 * wakeUp
 *
 * Wake up CC1101 from Power Down state
 */
void wakeUp(void) {
    cc1101_Select();  // Select CC1101
    sleep_ms(10);
    wait_Miso();        // Wait until MISO goes low
    cc1101_Deselect();  // Deselect CC1101
    sleep_ms(10);
}

/**
 * writeReg
 *
 * Write single register into the CC1101 IC via SPI
 *
 * 'regAddr'	Register address
 * 'value'	Value to be writen
 */
void writeReg(uint8_t regAddr, uint8_t value) {
    cc1101_Select();  // Select CC1101
    wait_Miso();      // Wait until MISO goes low
    // SPI.transfer(regAddr);  // Send register address
    // SPI.transfer(value);    // Send value
    // printf("writeReg: [0x%02X] [0x%02X]\n", regAddr, value);

    csb = spi_write_read_blocking(spi, &regAddr, &csb, 1);
    // printf("writeReg: [0x%02X] csb: [0x%02X]\n", regAddr, csb);
    cc1101_rdy = spi_write_read_blocking(spi, &value, &csb, 1);
    // printf("writeReg: [0x%02X] csb: [0x%02X]\n", value, csb);
    // parseCSB(csb, value);
    cc1101_Deselect();  // Deselect CC1101
}

/**
 * writeBurstReg
 *
 * Write multiple registers into the CC1101 IC via SPI
 *
 * 'regAddr'	Register address
 * 'buffer'	Data to be writen
 * 'len'	Data length
 */
void writeBurstReg(uint8_t regAddr, uint8_t *buffer, uint8_t len) {
    uint8_t addr, i;
    // printf("writeBurstReg: [0x%02X] len: %d\n", regAddr, len);

    addr = regAddr | WRITE_BURST;  // Enable burst transfer
    cc1101_Select();               // Select CC1101
    wait_Miso();                   // Wait until MISO goes low
    // SPI.transfer(addr);            // Send register address
    spi_write_read_blocking(spi, &regAddr, &csb, 1);

    for (i = 0; i < len; i++) {
        // SPI.transfer(buffer[i]);  // Send value
        spi_write_read_blocking(spi, &buffer[i], &csb, 1);
        // parseCSB(csb, buffer[i]);
    }
    //
    // int n = spi_write_blocking(spi, buffer, len);
    // printf("wrote %d bytes from buffer: %s\n", len, buffer);
    cc1101_Deselect();  // Deselect CC1101
}

/**
 * cmdStrobe
 *
 * Send command strobe to the CC1101 IC via SPI
 *
 * 'cmd'	Command strobe
 */
void cmdStrobe(uint8_t cmd) {
    cc1101_Select();  // Select CC1101
    wait_Miso();      // Wait until MISO goes low
                      // SPI.transfer(cmd);  // Send strobe command
    int n = spi_write_read_blocking(spi, &cmd, &csb, 1);
    // printf("cmdStrobe: wrote [0x%02X] %d bytes\n", cmd, n);
    //   parseCSB(csb, cmd);
    cc1101_Deselect();  // Deselect CC1101
}

/**
 * readReg
 *
 * Read CC1101 register via SPI
 *
 * 'regAddr'	Register address
 * 'regType'	Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 *
 * Return:
 * 	Data uint8_t returned by the CC1101 IC
 */
uint8_t readReg(uint8_t regAddr, uint8_t regType) {
    uint8_t addr;
    uint8_t val = 0xFF;

    addr = regAddr | regType | 0x80;
    // printf("addr: 0x%02X\n", addr);

    cc1101_Select();  // Select CC1101
    wait_Miso();      // Wait until MISO goes low
    // SPI.transfer(addr);        // Send register address
    // val = SPI.transfer(0x00);  // Read result
    int num_bytes_read = 0;

    int n = spi_write_read_blocking(spi, &addr, &csb, 1);
    // printf("readReg: wrote [0x%02X] (%d) bytes to reg [0x%0X]\n", addr, n, regAddr);
    // printf("csb: [0x%02X] \n", csb);
    spi_read_blocking(spi, 0x00, &val, 1);
    // printf("val: [0x%02X]\n", val);
    cc1101_Deselect();
    // printf("Waiting on MISO...");
    wait_Miso();
    // printf("done\n");
    return val;
}

/**
 * readBurstReg
 *
 * Read burst data from CC1101 via SPI
 *
 * 'buffer'	Buffer where to copy the result to
 * 'regAddr'	Register address
 * 'len'	Data length
 */
void readBurstReg(uint8_t *buffer, uint8_t regAddr, uint8_t len) {
    uint8_t addr, i;

    addr = regAddr | READ_BURST;
    cc1101_Select();  // Select CC1101
    wait_Miso();      // Wait until MISO goes low
                      // SPI.transfer(addr);                                        // Send register address
    // for (i = 0; i < len; i++) buffer[i] = SPI.transfer(0x00);  // Read result uint8_t by uint8_t
    spi_write_blocking(spi, &addr, 1);
    for (i = 0; i < len; i++) {
        // buffer[i] = SPI.transfer(0x00);
        spi_read_blocking(spi, 0, &buffer[i], 1);
    }
    cc1101_Deselect();  // Deselect CC1101
}

/**
 * reset
 *
 * Reset CC1101
 */
void reset(void) {
    cc1101_Deselect();  // Deselect CC1101
    sleep_ms(250);
    cc1101_Select();  // Select CC1101
    sleep_ms(250);
    cc1101_Deselect();  // Deselect CC1101
    sleep_ms(250);
    cc1101_Select();  // Select CC1101

    wait_Miso();  // Wait until MISO goes low
                  // SPI.transfer(CC1101_SRES);  // Send reset command strobe
    cmdStrobe(CC1101_SRES);
    wait_Miso();        // Wait until MISO goes low
    cc1101_Deselect();  // Deselect CC1101

    uint8_t val = readStatusReg(CC1101_PARTNUM);
    printf("Partnum: [0x%02X]\n", val);

    setCCregs();  // Reconfigure CC1101

    setTxState();
}

/**
 * setCCregs
 *
 * Configure CC1101 registers
 */
void setCCregs(void) {
    writeReg(CC1101_IOCFG2, CC1101_DEFVAL_IOCFG2);
    writeReg(CC1101_IOCFG1, CC1101_DEFVAL_IOCFG1);
    writeReg(CC1101_IOCFG0, CC1101_DEFVAL_IOCFG0);
    writeReg(CC1101_FIFOTHR, CC1101_DEFVAL_FIFOTHR);
    writeReg(CC1101_PKTLEN, CC1101_DEFVAL_PKTLEN);
    writeReg(CC1101_PKTCTRL1, CC1101_DEFVAL_PKTCTRL1);
    writeReg(CC1101_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);

    // Set default synchronization word
    setSyncWord(syncWord[0], syncWord[1]);

    // Set default device address
    setDevAddress(devAddress);

    // Set default frequency channel
    setChannel(channel);

    writeReg(CC1101_FSCTRL1, CC1101_DEFVAL_FSCTRL1);
    writeReg(CC1101_FSCTRL0, CC1101_DEFVAL_FSCTRL0);

    // Set default carrier frequency = 868 MHz
    setCarrierFreq(carrierFreq);
    printf("setCarrierFreq: %d\n", carrierFreq);
    setChannel(0);

    // RF speed
    if (workMode == MODE_LOW_SPEED)
        writeReg(CC1101_MDMCFG4, CC1101_DEFVAL_MDMCFG4_4800);
    else
        writeReg(CC1101_MDMCFG4, CC1101_DEFVAL_MDMCFG4_38400);

    writeReg(CC1101_MDMCFG3, CC1101_DEFVAL_MDMCFG3);
    writeReg(CC1101_MDMCFG2, CC1101_DEFVAL_MDMCFG2);
    writeReg(CC1101_MDMCFG1, CC1101_DEFVAL_MDMCFG1);
    writeReg(CC1101_MDMCFG0, CC1101_DEFVAL_MDMCFG0);
    writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
    writeReg(CC1101_MCSM2, CC1101_DEFVAL_MCSM2);
    writeReg(CC1101_MCSM1, CC1101_DEFVAL_MCSM1);
    writeReg(CC1101_MCSM0, CC1101_DEFVAL_MCSM0);
    writeReg(CC1101_FOCCFG, CC1101_DEFVAL_FOCCFG);
    writeReg(CC1101_BSCFG, CC1101_DEFVAL_BSCFG);
    writeReg(CC1101_AGCCTRL2, CC1101_DEFVAL_AGCCTRL2);
    writeReg(CC1101_AGCCTRL1, CC1101_DEFVAL_AGCCTRL1);
    writeReg(CC1101_AGCCTRL0, CC1101_DEFVAL_AGCCTRL0);
    writeReg(CC1101_WOREVT1, CC1101_DEFVAL_WOREVT1);
    writeReg(CC1101_WOREVT0, CC1101_DEFVAL_WOREVT0);
    writeReg(CC1101_WORCTRL, CC1101_DEFVAL_WORCTRL);
    writeReg(CC1101_FREND1, CC1101_DEFVAL_FREND1);
    writeReg(CC1101_FREND0, CC1101_DEFVAL_FREND0);
    writeReg(CC1101_FSCAL3, CC1101_DEFVAL_FSCAL3);
    writeReg(CC1101_FSCAL2, CC1101_DEFVAL_FSCAL2);
    writeReg(CC1101_FSCAL1, CC1101_DEFVAL_FSCAL1);
    writeReg(CC1101_FSCAL0, CC1101_DEFVAL_FSCAL0);
    writeReg(CC1101_RCCTRL1, CC1101_DEFVAL_RCCTRL1);
    writeReg(CC1101_RCCTRL0, CC1101_DEFVAL_RCCTRL0);
    writeReg(CC1101_FSTEST, CC1101_DEFVAL_FSTEST);
    writeReg(CC1101_PTEST, CC1101_DEFVAL_PTEST);
    writeReg(CC1101_AGCTEST, CC1101_DEFVAL_AGCTEST);
    writeReg(CC1101_TEST2, CC1101_DEFVAL_TEST2);
    writeReg(CC1101_TEST1, CC1101_DEFVAL_TEST1);
    writeReg(CC1101_TEST0, CC1101_DEFVAL_TEST0);

    // Send empty packet
    CCPACKET packet;
    packet.length = 0;
    sendData(packet);
    printf("sent empty packet\n");
}

/**
 * init
 *
 * Initialize CC1101 radio
 *
 * @param freq Carrier frequency
 * @param mode Working mode (speed, ...)
 */
void init(uint32_t freq, uint8_t mode) {
    channel = CC1101_DEFVAL_CHANNR;
    syncWord[0] = CC1101_DEFVAL_SYNC1;
    syncWord[1] = CC1101_DEFVAL_SYNC0;
    devAddress = CC1101_DEFVAL_ADDR;

    carrierFreq = freq;
    workMode = mode;
    // SPI.begin();                  // Initialize SPI interface
    startSPI();
    reset();  // Reset CC1101

    // wakeUp();  // Wake up CC1101 from Power Down state

    // Configure PATABLE
    setTxPowerAmp(PA_LowPower);
    printf("radio initialized\n");
}

/**
 * setSyncWord
 *
 * Set synchronization word
 *
 * 'syncH'	Synchronization word - High uint8_t
 * 'syncL'	Synchronization word - Low uint8_t
 */
void setSyncWord(uint8_t syncH, uint8_t syncL) {
    writeReg(CC1101_SYNC1, syncH);
    writeReg(CC1101_SYNC0, syncL);
    syncWord[0] = syncH;
    syncWord[1] = syncL;
}

/**
 * setDevAddress
 *
 * Set device address
 *
 * @param addr	Device address
 */
void setDevAddress(uint8_t addr) {
    writeReg(CC1101_ADDR, addr);
    devAddress = addr;
}

/**
 * setChannel
 *
 * Set frequency channel
 *
 * 'chnl'	Frequency channel
 */
void setChannel(uint8_t chnl) {
    writeReg(CC1101_CHANNR, chnl);
    channel = chnl;
}

/**
 * setCarrierFreq
 *
 * Set carrier frequency
 *
 * 'freq'	New carrier frequency
 */
void setCarrierFreq(uint32_t freq) {
    // uint32_t freq = 433000000;
    carrierFreq = freq;
    uint32_t fs = freq / (26000000 / (1 << 16));
    printf("setCarrierFreq: %ul", fs);

// #define ALWAYS_433
#ifdef ALWAYS_433
    uint8_t f2 = 0x10;
    uint8_t f1 = 0xA7;
    uint8_t f0 = 0x62;
#else
    uint8_t f2 = (fs & 0x00FF0000) >> 16;
    uint8_t f1 = (fs & 0x0000FF00) >> 8;
    uint8_t f0 = (fs & 0x000000FF);
#endif
    printf("f: %02X f2: %02X   f1:  %02X   f0: %02X\n", fs, f2, f1, f0);

    carrierFreq = freq;
    //  writeReg(CC1101_FREQ2, f2);
    //  writeReg(CC1101_FREQ1, f1);
    //  writeReg(CC1101_FREQ0, f0);
    writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_433);
    writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_433);
    writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_433);

    // while (1) {
    // }
    return;
#if 1
    switch (freq) {
        case CFREQ_915:
            writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_915);
            writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_915);
            writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_915);
            break;
        case CFREQ_433:
            writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_433);
            writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_433);
            writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_433);
            break;
        case CFREQ_918:
            writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_918);
            writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_918);
            writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_918);
            break;
        default:
            writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2_868);
            writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1_868);
            writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0_868);
            break;
    }

    carrierFreq = freq;
#endif
}

/**
 * setPowerDownState
 *
 * Put CC1101 into power-down state
 */
void setPowerDownState() {
    printf("setPowerDownState\n");
    //  Comming from RX state, we need to enter the IDLE state first
    cmdStrobe(CC1101_SIDLE);
    // Enter Power-down state
    cmdStrobe(CC1101_SPWD);
}

bool sendData(CCPACKET packet) {
    uint8_t marcState;
    bool res = false;

    // Declare to be in Tx state. This will avoid receiving packets whilst
    // transmitting
    rfState = RFSTATE_TX;

    // Enter RX state
    // testing setRxState();

    printf("sendData\n");
    if (packet.length > 0) {
        printf("sendData: is %d bytes\n", packet.length);
    } else {
        printf("sendData: is 0 bytes\n");
    }

    int tries = 0;
    // Check that the RX state has been entered
    // while (tries++ < 1000 && ((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D) {
#if 0
    marcState = readStatusReg(CC1101_MARCSTATE);
    printf("sendData MarcState in RX: [0x%02X]\n", marcState);
    while (tries++ < 1000 && ((marcState & 0x1F) != 0x0D)) {
        if (marcState == 0x11)  // RX_OVERFLOW
            flushRxFifo();      // flush receive queue
    }
    if (tries >= 1000) {
        // TODO: MarcState sometimes never enters the expected state; this is a hack workaround.
        printf("MarcState never entered expected state\n");
        return false;
    }

    // sleep_us(500);

    if (packet.length > 0) {
        printf("sendData: sending %d bytes\n", packet.length);
        // Set data length at the first position of the TX FIFO
        writeReg(CC1101_TXFIFO, packet.length);
        // Write data into the TX FIFO
        writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);

        // CCA enabled: will enter TX state only if the channel is clear
        setTxState();
    }
#endif

    setTxState();
    for (int x = 0; x < packet.length; x++) {
        printf("%02X ", packet.data[x]);
    }
    printf("\n");
    // Check that TX state is being entered (state = RXTX_SETTLING)
    marcState = readStatusReg(CC1101_MARCSTATE);
    printf("sendData MarcState in TX: [0x%02X]\n", marcState);

    marcState = marcState & 0x1F;
    if ((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15)) {
        setIdleState();  // Enter IDLE state
        printf("--- flushing the FIFO\n");
        flushTxFifo();  // Flush Tx FIFO
        setRxState();   // Back to RX state

        // Declare to be in Rx state
        rfState = RFSTATE_RX;
        printf("--- entering RX state, returning false\n");
        return false;
    }
    // printf("CS off\n");
    // cc1101_Deselect();
    //  Wait for the sync word to be transmitted
    printf("waiting for sync word transmit...\n");
    // wait_GDO0_high();
    while (gpio_get(17) != 1) {
    }
    // Wait until the end of the packet transmission
    while (gpio_get(17) != 0) {
    }
    // Check that the TX FIFO is empty
    printf("checing if the TX FIFO is empty\n");
    if ((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0) res = true;

    printf("sendData: entereting IDLE state\n");
    setIdleState();  // Enter IDLE state
    flushTxFifo();   // Flush Tx FIFO

    // Enter back into RX state
    printf("sendData: entereting RX state\n");
    setRxState();

    // Declare to be in Rx state
    rfState = RFSTATE_RX;

    return res;
}

/**
 * sendData2
 *
 * Send data packet via RF
 *
 * 'packet'	Packet to be transmitted. First uint8_t is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool sendData2(CCPACKET packet) {
    uint8_t marcState;
    bool res = false;
    int txfifo;

    // Declare to be in Tx state. This will avoid receiving packets whilst
    // transmitting
    rfState = RFSTATE_TX;

    // Enter RX state
    setRxState();

    int tries = 0;
    // Check that the RX state has been entered
    while (tries++ < 1000 && ((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D) {
        if (marcState == 0x11)  // RX_OVERFLOW
            flushRxFifo();      // flush receive queue
    }
    if (tries >= 1000) {
        // TODO: MarcState sometimes never enters the expected state; this is a hack workaround.
        return false;
    }
    sleep_us(500);

    if (packet.length <= 0) return 0;
    //  printf("writing data to radio buffer\n");
    // Set data length at the first position of the TX FIFO
    writeReg(CC1101_TXFIFO, packet.length);
    // Write data into the TX FIFO
    writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);

    txfifo = readStatusReg(CC1101_TXBYTES) & 0x7F;
    printf(("bytes in tx fifo: %d\n"), txfifo);

    // CCA enabled: will enter TX state only if the channel is clear
    printf("sending data\n");
    setTxState();

#if 1
    while (1) {
        printf(".");
        sleep_us(500);
        marcState = readStatusReg(CC1101_MARCSTATE);
        // mask upp the upper bits - unused
        marcState = marcState & 0x1F;
        // printf("sendData MarcState in TX: [0x%02X] csb [0x%02X]\n", marcState, csb);
        //  if ((marcState == 0x13) || (marcState == 0x14) || (marcState == 0x15)) {
        if ((marcState == 0x0D) || (marcState == 0x0E) || (marcState == 0x0F)) {
            printf("************************************ gack\n");
            setIdleState();  // Enter IDLE state
            flushTxFifo();   // Flush Tx FIFO
            setRxState();    // Back to RX state
                             // Declare to be in Rx state
            rfState = RFSTATE_RX;
            return false;
        } else if (marcState == 0x16) {
            // TXFIFOUNDERFLOW
            printf("***** TXFIFO_UNDERFLOW *****\n");
            int txfifo = readStatusReg(CC1101_TXBYTES) & 0x7F;
            printf(("tx fifo bytes: %d\n"), txfifo);
            flushTxFifo();
            return false;
        } else {
            break;
        }
    }
#endif

    while (gpio_get(17) != 1) {
        // printf("waiting for GDO0 to go high\n");
    }
    // Wait until the end of the packet transmission
    while (gpio_get(17) != 0) {
    }
    sleep_us(500);
    printf("transmission happened...\n");

    // Check that the TX FIFO is empty
    // printf("checing if the TX FIFO is empty\n");
    txfifo = readStatusReg(CC1101_TXBYTES) & 0x7F;
    printf(("tx fifo bytes: %d\n"), txfifo);
    if (txfifo == 0) {
        res = true;
    } else {
        printf("flushing TX FIFO\n");
        flushTxFifo();
        res = false;
    }
    setIdleState();
    flushTxFifo();
    setRxState();
    rfState = RFSTATE_RX;
    // marcState = readStatusReg(CC1101_MARCSTATE);
    //  sleep_ms(250);

    return res;
}

/**
 * receiveData
 *
 * Read data packet from RX FIFO
 *
 * 'packet'	Container for the packet received
 *
 * Return:
 * 	Amount of uint8_ts received
 */
uint8_t receiveData(CCPACKET *packet) {
    uint8_t val;
    uint8_t rxuint8_ts = readStatusReg(CC1101_RXBYTES);

    // Any uint8_t waiting to be read and no overflow?
    if (rxuint8_ts & 0x7F && !(rxuint8_ts & 0x80)) {
        // Read data length
        packet->length = readConfigReg(CC1101_RXFIFO);
        // If packet is too long
        if (packet->length > CCPACKET_DATA_LEN)
            packet->length = 0;  // Discard packet
        else {
            // Read data packet
            readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
            // Read RSSI
            packet->rssi = readConfigReg(CC1101_RXFIFO);
            // Read LQI and CRC_OK
            val = readConfigReg(CC1101_RXFIFO);
            packet->lqi = val & 0x7F;
            packet->crc_ok = bitRead(&val, 7);
        }
    } else
        packet->length = 0;

    setIdleState();  // Enter IDLE state
    flushRxFifo();   // Flush Rx FIFO
    // cmdStrobe(CC1101_SCAL);

    // Back to RX state
    setRxState();

    return packet->length;
}

/**
 * setRxState
 *
 * Enter Rx state
 */
void setRxState(void) {
    // printf("setRxState\n");
    cmdStrobe(CC1101_SRX);
    rfState = RFSTATE_RX;
}

/**
 * setTxState
 *
 * Enter Tx state
 */
void setTxState(void) {
    // printf("setTxState()\n");
    cmdStrobe(CC1101_STX);

    rfState = RFSTATE_TX;
}
