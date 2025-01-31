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

spi_inst_t *spi = spi0;

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

    printf("SPI initialized\n");
    return true;
}

/**
 * Macros
 */
// Select (SPI) CC1101
#define cc1101_Select() digitalWrite(SS, LOW)
// Deselect (SPI) CC1101
#define cc1101_Deselect() digitalWrite(SS, HIGH)
// Wait until SPI MISO line goes low
#define wait_Miso() while (digitalRead(MISO) > 0)
// Get GDO0 pin state
#define getGDO0state() digitalRead(GDO0)
// Wait until GDO0 line goes high
#define wait_GDO0_high() while (!getGDO0state())
// Wait until GDO0 line goes low
#define wait_GDO0_low() while (getGDO0state())

bool bitRead(uint8_t *x, char n) { return (*x & (1 << n)) ? 1 : 0; }

/**
 * PATABLE
 */
// const uint8_t paTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};

uint8_t rfState;
uint8_t carrierFreq;
uint8_t workMode;
uint8_t channel;
uint8_t syncWord[2];
uint8_t devAddress;

bool digitalWrite(uint8_t pin, bool value) {
    gpio_put(pin, value);
    return true;
}
bool digitalRead(uint8_t pin) { return gpio_get(pin); }

/**
 * wakeUp
 *
 * Wake up CC1101 from Power Down state
 */
void wakeUp(void) {
    cc1101_Select();    // Select CC1101
    wait_Miso();        // Wait until MISO goes low
    cc1101_Deselect();  // Deselect CC1101
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
    spi_write_blocking(spi, &regAddr, 1);
    spi_write_blocking(spi, &value, 1);
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

    addr = regAddr | WRITE_BURST;  // Enable burst transfer
    cc1101_Select();               // Select CC1101
    wait_Miso();                   // Wait until MISO goes low
    // SPI.transfer(addr);            // Send register address
    spi_write_blocking(spi, &regAddr, 1);
    for (i = 0; i < len; i++) {
        // SPI.transfer(buffer[i]);  // Send value
        spi_write_blocking(spi, &buffer[i], 1);
    }
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
    int n = spi_write_blocking(spi, &cmd, 1);
    printf("wrote [%x] %d bytes\n", cmd, n);
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
    uint8_t addr, val;

    addr = regAddr | regType;
    cc1101_Select();  // Select CC1101
    wait_Miso();      // Wait until MISO goes low
    // SPI.transfer(addr);        // Send register address
    // val = SPI.transfer(0x00);  // Read result
    int num_bytes_read = 0;
    spi_write_blocking(spi, &addr, 1);
    spi_read_blocking(spi, 0, &val, 1);
    printf("read [%x] from reg [%x]\n", val, regAddr);
    cc1101_Deselect();  // Deselect CC1101
    // FIXME: This is a hack to get the value out of the buffer
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
    sleep_us(5);
    cc1101_Select();  // Select CC1101
    sleep_us(10);
    cc1101_Deselect();  // Deselect CC1101
    sleep_us(41);
    cc1101_Select();  // Select CC1101

    wait_Miso();  // Wait until MISO goes low
                  // SPI.transfer(CC1101_SRES);  // Send reset command strobe
    uint8_t cmd = CC1101_SRES;
    spi_write_blocking(spi, &cmd, 1);
    wait_Miso();  // Wait until MISO goes low

    cc1101_Deselect();  // Deselect CC1101

    setCCregs();  // Reconfigure CC1101
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
}

/**
 * init
 *
 * Initialize CC1101 radio
 *
 * @param freq Carrier frequency
 * @param mode Working mode (speed, ...)
 */
void init(uint8_t freq, uint8_t mode) {
    channel = CC1101_DEFVAL_CHANNR;
    syncWord[0] = CC1101_DEFVAL_SYNC1;
    syncWord[1] = CC1101_DEFVAL_SYNC0;
    devAddress = CC1101_DEFVAL_ADDR;

    carrierFreq = freq;
    workMode = mode;
    // SPI.begin();                  // Initialize SPI interface
    startSPI();
    reset();  // Reset CC1101

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
void setCarrierFreq(uint8_t freq) {
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
}

/**
 * setPowerDownState
 *
 * Put CC1101 into power-down state
 */
void setPowerDownState() {
    // Comming from RX state, we need to enter the IDLE state first
    cmdStrobe(CC1101_SIDLE);
    // Enter Power-down state
    cmdStrobe(CC1101_SPWD);
}

/**
 * sendData
 *
 * Send data packet via RF
 *
 * 'packet'	Packet to be transmitted. First uint8_t is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool sendData(CCPACKET packet) {
    uint8_t marcState;
    bool res = false;

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
        printf("MarcState never entered expected state\n");
        return false;
    }

    sleep_us(500);

    if (packet.length > 0) {
        // Set data length at the first position of the TX FIFO
        writeReg(CC1101_TXFIFO, packet.length);
        // Write data into the TX FIFO
        writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);

        // CCA enabled: will enter TX state only if the channel is clear
        setTxState();
    }

    // Check that TX state is being entered (state = RXTX_SETTLING)
    marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
    if ((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15)) {
        setIdleState();  // Enter IDLE state
        flushTxFifo();   // Flush Tx FIFO
        setRxState();    // Back to RX state

        // Declare to be in Rx state
        rfState = RFSTATE_RX;
        return false;
    }

    // Wait for the sync word to be transmitted
    wait_GDO0_high();

    // Wait until the end of the packet transmission
    wait_GDO0_low();

    // Check that the TX FIFO is empty
    if ((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0) res = true;

    setIdleState();  // Enter IDLE state
    flushTxFifo();   // Flush Tx FIFO

    // Enter back into RX state
    setRxState();

    // Declare to be in Rx state
    rfState = RFSTATE_RX;

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
    cmdStrobe(CC1101_SRX);
    rfState = RFSTATE_RX;
}

/**
 * setTxState
 *
 * Enter Tx state
 */
void setTxState(void) {
    cmdStrobe(CC1101_STX);
    rfState = RFSTATE_TX;
}
