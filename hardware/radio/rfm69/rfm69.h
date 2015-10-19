/* **********************************************************************************
 * Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
 * **********************************************************************************
 * Copyright Felix Rusu (2014), felix@lowpowerlab.com
 * http://lowpowerlab.com/
 * Copyright (c) 2015 by Meinhard Ritscher <unreachable@gmx.net> ported to using pain C avr-gcc
 *
 * **********************************************************************************
 * License
 * **********************************************************************************
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General
 * Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Licence can be viewed at
 * http://www.gnu.org/licenses/gpl-3.0.txt
 *
 * Please maintain this license information along with authorship
 * and copyright notices in any redistribution of this code
 * ********************************************************************************** */
#ifndef _RFM69_H
#define _RFM69_H

#include <stdbool.h>
#include "config.h"

#ifdef DEBUG_RFM69_LOGGER
# include "core/debug.h"
# define RFM69DEBUG(a...)  debug_printf("rfm69: " a)
#else
# define RFM69DEBUG(a...)
#endif

#define rfm69_int_enable()  \
  _EIMSK |= _BV(RFM69_IRQ_PIN);
#define rfm69_int_disable()     \
  _EIMSK &= ~_BV(RFM69_IRQ_PIN);

#define PIN_READ(pin) (PIN_CHAR(pin ## _PORT) & _BV(pin ## _PIN))
#define SET_SPI_MODE0      {SPCR&=~(_BV(CPOL)|_BV(CPHA));}
#define SET_SPI_MSBFIRST   {SPCR&=~ _BV(DORD);}
#define SET_SPI_CLK_DIV4   {SPCR&=~(_BV(SPR1)|_BV(SPR0));}


#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)

/*
 * INT0 or INT1 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
 *
 * This is configured within the appropriate hardware's pinning m4 file
 */

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

static volatile uint8_t RFM69_DATA[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
static volatile uint8_t RFM69_DATALEN;
static volatile uint8_t SENDERID;
static volatile uint8_t TARGETID; // should match _address
static volatile uint8_t PAYLOADLEN;
static volatile uint8_t ACK_REQUESTED;
// should be polled immediately after sending a packet with ACK request
static volatile uint8_t ACK_RECEIVED;
// most accurate RSSI during reception (closest to the reception)
static volatile int16_t RSSI;
// current transceiver state
static volatile uint8_t _mode = RF69_MODE_STANDBY; // should be protected?

bool rfm69_initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID);
void rfm69_setAddress(uint8_t addr);
void rfm69_setNetwork(uint8_t networkID);
bool rfm69_canSend();
void rfm69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
bool rfm69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime); // 40ms roundtrip req for 61byte packets
bool rfm69_receiveDone();
bool rfm69_ACKReceived(uint8_t fromNodeID);
bool rfm69_ACKRequested();
void rfm69_sendACK(const void* buffer, uint8_t bufferSize);
uint32_t rfm69_getFrequency();
void rfm69_setFrequency(uint32_t freqHz);
void rfm69_encrypt(const char* key);
void rfm69_setCS(uint8_t newSPISlaveSelect);
int16_t rfm69_readRSSI(bool forceTrigger);
void rfm69_promiscuous(bool onOff);
void rfm69_setHighPower(bool onOFF); // has to be called after initialize() for RFM69HW
void rfm69_setPowerLevel(uint8_t level); // reduce/increase transmit power level
void rfm69_sleep();
uint8_t rfm69_readTemperature(uint8_t calFactor); // get CMOS temperature (8bit)
void rfm69_rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
uint8_t rfm69_readReg(uint8_t addr);
void rfm69_writeReg(uint8_t addr, uint8_t val);
uint8_t rfm69_getMode();
// protected
/*
static void sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK, bool sendACK);

static void receiveBegin();
static void setMode(uint8_t mode);
static void setHighPowerRegs(bool onOff);
static void select();
static void unselect();
*/

#endif // _RFM69_H
