/* **********************************************************************************
 * Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
 * **********************************************************************************
 * Copyright Felix Rusu (2014), felix@lowpowerlab.com
 * http://lowpowerlab.com/
 * Copyright (c) 2015 by Meinhard Ritscher <unreachable@gmx.net> ported to using plain C avr-gcc
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
#include <avr/interrupt.h>

#include "rfm69.h"
#include "rfm69_registers.h"
#include "core/spi.h"

#ifdef USE_STOPWATCH
#include "core/stopwatch.h"
#else
#include "services/clock/clock.h"
#define millis() (clock_get_uptime())
#endif

uint8_t _address
#ifdef RFM69_NODE_ID
 = RFM69_NODE_ID
#endif
;

bool _promiscuousMode = false;
uint8_t _powerLevel = 24; // 4dBm
bool _isRFM69HW = false;
uint8_t _SPCR;
uint8_t _SPSR;


/* module-local prototypes */
static void sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK, bool sendACK);
static void receiveBegin();
static void setMode(uint8_t mode);
static void setHighPowerRegs(bool onOff);
static void select();
static void unselect();


/*volatile uint8_t _mode;
volatile uint8_t DATALEN;
volatile uint8_t SENDERID;
volatile uint8_t TARGETID;     // should match _address
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
volatile int16_t RSSI;          // most accurate RSSI during reception (closest to the reception)
*/

bool rfm69_initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  // RFM69's CS pin is set to HIGH in spi_init
  // RFM69's CS port is configured at start of main via port mask in global.h
  // this pin is used in sendFrame function below as well as for the interrupt
  rfm69_int_enable();

  do rfm69_writeReg(REG_SYNCVALUE1, 0xAA); while (rfm69_readReg(REG_SYNCVALUE1) != 0xAA);
  do rfm69_writeReg(REG_SYNCVALUE1, 0x55); while (rfm69_readReg(REG_SYNCVALUE1) != 0x55);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    rfm69_writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  rfm69_encrypt(0);

  rfm69_setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  setMode(RF69_MODE_STANDBY);
  while ((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  // interrupt is handled by ISR below

  _address = nodeID;
  return true;
}

// return the frequency (in Hz)
uint32_t rfm69_getFrequency()
{
  return RF69_FSTEP * (((uint32_t) rfm69_readReg(REG_FRFMSB) << 16) + ((uint16_t) rfm69_readReg(REG_FRFMID) << 8) + rfm69_readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void rfm69_setFrequency(uint32_t freqHz)
{
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX) {
    setMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  rfm69_writeReg(REG_FRFMSB, freqHz >> 16);
  rfm69_writeReg(REG_FRFMID, freqHz >> 8);
  rfm69_writeReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX) {
    setMode(RF69_MODE_SYNTH);
  }
  setMode(oldMode);
}

static
void setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (_isRFM69HW) setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (_isRFM69HW) setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}

// debugging only
uint8_t rfm69_getMode()
{
  return (rfm69_readReg(REG_OPMODE) & 0xE3);
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call rfm69_receiveDone()
void rfm69_sleep() {
  setMode(RF69_MODE_SLEEP);
}

//set this node's address
void rfm69_setAddress(uint8_t addr)
{
  _address = addr;
  rfm69_writeReg(REG_NODEADRS, _address);
}

//set this node's network id
void rfm69_setNetwork(uint8_t networkID)
{
  rfm69_writeReg(REG_SYNCVALUE2, networkID);
}

// set *transmit/TX* output power: 0=min, 31=max
// see above for the equation to calculate dBm - basically -18 + powerLevel for RFM69W
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void rfm69_setPowerLevel(uint8_t powerLevel)
{
  _powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  if (_isRFM69HW) _powerLevel /= 2;
  rfm69_writeReg(REG_PALEVEL, (rfm69_readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
}

bool rfm69_canSend()
{
  if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && rfm69_readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

// suggested default by cpp upstream requestACK: false
void rfm69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
  rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
#ifdef USE_STOPWATCH
  stopwatch_start();
#endif
  uint32_t now = millis();
  while (!rfm69_canSend() && millis() - now < RF69_CSMA_LIMIT_MS)
    rfm69_receiveDone();
#ifdef USE_STOPWATCH
  stopwatch_stop();
#endif
  sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// to increase the chance of getting a packet across, call this function instead of send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5..8ms at 50kbps@915MHz
// suggested defaults by cpp upstream: retries: 2 retryWaitTime: 40
bool rfm69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
  uint32_t sentTime;
  for (uint8_t i = 0; i <= retries; i++)
  {
    rfm69_send(toAddress, buffer, bufferSize, true);
#ifdef USE_STOPWATCH
    stopwatch_start();
#endif
    sentTime = millis();
    while (millis() - sentTime < retryWaitTime)
    {
      if (rfm69_ACKReceived(toAddress))
      {
        RFM69DEBUG(" ~ms: %u \n", millis() - sentTime);
#ifdef USE_STOPWATCH
        stopwatch_stop();
#endif
        return true;
      }
    }
#ifdef USE_STOPWATCH
    stopwatch_stop();
#endif
    RFM69DEBUG(" RETRY# %u \n", i + 1);
  }
  return false;
}

// should be polled immediately after sending a packet with ACK request
bool rfm69_ACKReceived(uint8_t fromNodeID) {
  if (rfm69_receiveDone())
    return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
  return false;
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
bool rfm69_ACKRequested() {
  return ACK_REQUESTED && (TARGETID != RF69_BROADCAST_ADDR);
}

// should be called immediately after reception in case sender wants ACK
// defaults in cpp upstream buffer: "", uint8_t bufferSize: 0
void rfm69_sendACK(const void* buffer, uint8_t bufferSize) {
  uint8_t sender = SENDERID;
  int16_t _RSSI = RSSI; // save payload received RSSI value
  rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
#ifdef USE_STOPWATCH
  stopwatch_start();
#endif
  uint32_t now = millis();
  while (!rfm69_canSend() && millis() - now < RF69_CSMA_LIMIT_MS)
    rfm69_receiveDone();
#ifdef USE_STOPWATCH
  stopwatch_stop();
#endif
  sendFrame(sender, buffer, bufferSize, false, true);
  RSSI = _RSSI; // restore payload RSSI
}

// internal function
// suggested defaults by cpp upstream: requestACK: false, sendACK: false
static
void sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  rfm69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;
  PIN_SET(STATUSLED_RFM69_TX);

  // control byte
  uint8_t CTLbyte = 0x00;
  if (sendACK)
    CTLbyte = 0x80;
  else if (requestACK)
    CTLbyte = 0x40;

  // write to FIFO
  select();
  spi_send(REG_FIFO | 0x80);
  spi_send(bufferSize + 3);
  spi_send(toAddress);
  spi_send(_address);
  spi_send(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
    spi_send(((uint8_t*) buffer)[i]);
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
#ifdef USE_STOPWATCH
  stopwatch_start();
#endif
  uint32_t txStart = millis();
  while (PIN_READ(RFM69_IRQ) == 0 && millis() - txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
#ifdef USE_STOPWATCH
  stopwatch_stop();
#endif
  PIN_CLEAR(STATUSLED_RFM69_TX);
  //while (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  setMode(RF69_MODE_STANDBY);
}

// internal function - interrupt gets called when a packet is received
#ifdef RFM69_INT_VECTOR
ISR(RFM69_INT_VECTOR)
{
  PIN_SET(STATUSLED_RFM69_RX);

  if (_mode == RF69_MODE_RX && (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    //RSSI = rfm69_readRSSI(false);
    setMode(RF69_MODE_STANDBY);
    select();
    spi_send(REG_FIFO & 0x7F);
    PAYLOADLEN = spi_send(0);
    PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
    TARGETID = spi_send(0);
    if(!(_promiscuousMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
       || PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      PAYLOADLEN = 0;
      unselect();
      receiveBegin();

    } else {

      RFM69_DATALEN = PAYLOADLEN - 3;
      SENDERID = spi_send(0);
      uint8_t CTLbyte = spi_send(0);

      ACK_RECEIVED = CTLbyte & 0x80; // extract ACK-received flag
      ACK_REQUESTED = CTLbyte & 0x40; // extract ACK-requested flag

      for (uint8_t i = 0; i < RFM69_DATALEN; i++)
      {
        RFM69_DATA[i] = spi_send(0);
      }
      if (RFM69_DATALEN < RF69_MAX_DATA_LEN) RFM69_DATA[RFM69_DATALEN] = 0; // add null at end of string
      unselect();
      setMode(RF69_MODE_RX);
    }
    RSSI = rfm69_readRSSI(false);
  }
  PIN_CLEAR(STATUSLED_RFM69_RX);

}
#endif //RFM69_INT_HANDLER


// internal function
static
void receiveBegin() {
  RFM69_DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  RSSI = 0;
  if (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  rfm69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool rfm69_receiveDone() {
//ATOMIC_BLOCK(ATOMIC_FORCEON)
//{
  cli(); // re-enabled in unselect() via setMode() or via receiveBegin()
  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    sei(); // explicitly re-enable interrupts
    return false;
  }
  receiveBegin();
  return false;
//}
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void rfm69_encrypt(const char* key) {
  setMode(RF69_MODE_STANDBY);
  if (key != 0)
  {
    select();
    spi_send(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      spi_send(key[i]);
    unselect();
  }
  rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
// defaults at cpp upstream: forceTrigger: false
int16_t rfm69_readRSSI(bool forceTrigger) {
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    rfm69_writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((rfm69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -rfm69_readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t rfm69_readReg(uint8_t addr)
{
  // 0x7F tells the RFM69 module that
  // the next access is a read command
  select();

  spi_send(addr & 0x7F);
  uint8_t regval = spi_send(0);

  unselect();

  return regval;
}

void rfm69_writeReg(uint8_t addr, uint8_t value)
{
  // 0x80 tells the RFM69 module that
  // the next access is a write command
  select();

  spi_send(addr | 0x80);
  spi_send(value);

  unselect();

}

// select the RFM69 transceiver (save SPI settings, set CS low)
static
void select() {
  cli();
  // save current SPI settings
  _SPCR = SPCR;
  _SPSR = SPSR;
  // set RFM69 SPI settings
  SET_SPI_MODE0 // setDataMode(SPI_MODE0);
  SET_SPI_MSBFIRST // setBitOrder(MSBFIRST); set to 0 => MSBFIRST
  SET_SPI_CLK_DIV4 // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  PIN_CLEAR(SPI_CS_RFM69);
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
static
void unselect() {
  PIN_SET(SPI_CS_RFM69);
  // restore SPI settings to what they were before talking to RFM69
  SPCR = _SPCR;
  SPSR = _SPSR;
  sei();
}

// true  = disable filtering to capture all frames on network
// false = enable node/broadcast filtering to capture only frames sent to this/broadcast address
// suggested default by cpp upstream: onOff: true
void rfm69_promiscuous(bool onOff) {
  _promiscuousMode = onOff;
  //rfm69_writeReg(REG_PACKETCONFIG1, (rfm69_readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

// for RFM69HW only: you must call rfm69_setHighPower(true) after initialize() or else transmission won't work
// suggested default by cpp upstream: onOff: true
void rfm69_setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  // why is over current protection disabled for the HW variants
  // datasheet seems to suggest different
  rfm69_writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    rfm69_writeReg(REG_PALEVEL, (rfm69_readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    rfm69_writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

// internal function
static
void setHighPowerRegs(bool onOff) {
  rfm69_writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  rfm69_writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

// suggested default by cpp upstream: calFactor: 0
uint8_t rfm69_readTemperature(uint8_t calFactor) // returns centigrade
{
  setMode(RF69_MODE_STANDBY);
  rfm69_writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((rfm69_readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~rfm69_readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void rfm69_rcCalibration()
{
  // setMode(RF69_MODE_STANDBY); according to datasheet
  rfm69_writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((rfm69_readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}
