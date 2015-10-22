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
#include "core/util/byte2bin.h"

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
static void print_rfm69_mode(uint8_t mode_value);

/*
 * Setting up the module
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
    // connecting LEDs to DIO1, DIO2 and DIO3 for debugging:
    // DIO1 00 => FifoLevel | DIO2 00 => FifoNotEmpty | DIO3 00 => FifoFull (Table 21 in datasheet)
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

  //  DDR_CONFIG_IN(RFM69_IRQ);
  // this pin is used in sendFrame function below as well as for the interrupt
  //rfm69_int_enable();
  // enbale interrupt on rising edge
  _EIMSK |= _BV(INT1);
  _EICRA |= _BV(ISC11) | _BV(ISC10);

#ifdef USE_STOPWATCH
  stopwatch_start();
#endif
  uint32_t start = millis();
  uint8_t timeout = 50;
  do rfm69_writeReg(REG_SYNCVALUE1, 0xAA); while (rfm69_readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
  start = millis();
  do rfm69_writeReg(REG_SYNCVALUE1, 0x55); while (rfm69_readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    rfm69_writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  rfm69_encrypt(0);

  rfm69_setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  setMode(RF69_MODE_STANDBY);
  while ((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  start = millis();
  while (((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
#ifdef USE_STOPWATCH
  stopwatch_stop();
#endif
  if (millis()-start >= timeout){
    return false;
  }
  // interrupt is handled by ISR below

  _address = nodeID;
  return true;
}

/*
 * return the frequency (in Hz)
 */
uint32_t rfm69_getFrequency()
{
  return RF69_FSTEP * (((uint32_t) rfm69_readReg(REG_FRFMSB) << 16) + ((uint16_t) rfm69_readReg(REG_FRFMID) << 8) + rfm69_readReg(REG_FRFLSB));
}

/*
 * set the frequency (in Hz)
 */
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

// internal function for debugging
// checks that our local _model shadowing variable
// reflects the RegOpMode's mode setting
// returns 0 if equal, the register's content otherwise
// adds a 1 in the unused bit 0 for RF69_MODE_SLEEP
static
uint8_t checkMode()
{
  uint8_t modeRegVal = rfm69_readReg(REG_OPMODE);
  switch (modeRegVal & 0x1C) {
    case RF_OPMODE_TRANSMITTER:
      if(_mode == RF69_MODE_TX) return 0;
      _mode = RF69_MODE_TX;
      break;
    case RF_OPMODE_RECEIVER:
      if(_mode == RF69_MODE_RX) return 0;
      _mode = RF69_MODE_RX;
      break;
    case RF_OPMODE_SYNTHESIZER:
      if(_mode == RF69_MODE_SYNTH) return 0;
      _mode = RF69_MODE_SYNTH;
      break;
    case RF_OPMODE_STANDBY:
      if(_mode == RF69_MODE_STANDBY) return 0;
      _mode = RF69_MODE_STANDBY;
      break;
    case RF_OPMODE_SLEEP:
      if(_mode == RF69_MODE_SLEEP) return 0;
      _mode = RF69_MODE_SLEEP;
      modeRegVal |= 0x01; // just to make sure it is > 0
      break;
    default:
      ;
  }
  return modeRegVal;
}

// internal function
// switches to the desired mode
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

  // we are using packet mode (in contrast to continuous mode), so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}


/*
 * put transceiver in sleep mode to save battery
 * to wake or resume receiving just call rfm69_receiveDone()
 */
void rfm69_sleep() {
  setMode(RF69_MODE_SLEEP);
}

/*
 * set this node's address
 * This library uses "soft" address filtering
 * instead of the module's ability to filter to
 * make broadcasts possible (see lowpowerlab forum)
 */
void rfm69_setAddress(uint8_t addr)
{
  _address = addr;
  rfm69_writeReg(REG_NODEADRS, _address);
}

/*
 * set this node's network id
 * network id is second sync word
 */
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

/*
 * not documented so far
 */
bool rfm69_canSend()
{
  if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && rfm69_readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

/*
 * Send a package without retries
 * suggested default by cpp upstream requestACK: false
 */
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

/*
 * to increase the chance of getting a packet across, call this function instead of send
 * and it handles all the ACK requesting/retrying for you :)
 * The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
 * The reason for the semi-automaton is that the lib is interrupt driven and
 * requires user action to read the received data and decide what to do with it
 * replies usually take only 5..8ms at 50kbps@915MHz
 * suggested defaults by cpp upstream: retries: 2 retryWaitTime: 40
 */
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

/*
 * should be polled immediately after sending a packet with ACK request
 */
bool rfm69_ACKReceived(uint8_t fromNodeID) {
  if (rfm69_receiveDone())
    return (RFM69_SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
  return false;
}

/*
 * check whether an ACK was requested in the last received packet (non-broadcasted packet)
 */
bool rfm69_ACKRequested() {
  return ACK_REQUESTED && (TARGETID != RF69_BROADCAST_ADDR);
}

/*
 * should be called immediately after reception in case sender wants ACK
 * defaults in cpp upstream buffer: "", uint8_t bufferSize: 0
 */
void rfm69_sendACK(const void* buffer, uint8_t bufferSize) {
  ACK_REQUESTED = 0;   // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
  uint8_t sender = RFM69_SENDERID;
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
  PIN_SET(STATUSLED_RFM69_TX);
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  rfm69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 ); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // control byte
  uint8_t CTLbyte = 0x00;
  if (sendACK)
    CTLbyte = RFM69_CTL_SENDACK;
  else if (requestACK)
    CTLbyte = RFM69_CTL_REQACK;

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

  // we do not want DIO0 turning high to trigger an interrupt
  rfm69_int_disable();
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

// internal function
// interrupt gets called when a packet is received
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
    // match this node's address, or broadcast address or anything in promiscuous mode
    if(!(_promiscuousMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR)
       || PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      // PAYLOADLEN = 0;
      unselect();
      receiveBegin();
    } else {

      RFM69_DATALEN = PAYLOADLEN - 3;
      RFM69_SENDERID = spi_send(0);
      uint8_t CTLbyte = spi_send(0);

      ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
      ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag

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
// initial setup for RX
static
void receiveBegin() {
  RFM69_DATALEN = 0;
  RFM69_SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  RSSI = 0;
  if (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  // set DIO0 to "PAYLOADREADY" in receive mode
  rfm69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 );
  rfm69_int_enable();
  setMode(RF69_MODE_RX);
}

/*
 * Checks if a packet was received and/or
 *   puts transceiver in receive (ie RX or listen) mode.
 * Call this to start receiving and periodically check
 * if new data has arrived by calling this function.
 *
 */
bool rfm69_receiveDone() {

  cli(); // re-enabled in unselect() via setMode() or via receiveBegin()

  if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    sei(); // explicitly re-enable interrupts

    // in case the receiver locks up
    // this may be helpful for debugging
    /*
    uint8_t val = rfm69_readReg(REG_IRQFLAGS2) & ( RF_IRQFLAGS2_FIFOFULL | RF_IRQFLAGS2_FIFONOTEMPTY | RF_IRQFLAGS2_FIFOLEVEL );
    if (val)
    {
      //RFM69DEBUG("Fifo not empty? But PAYLOADLEN = 0 - %s - \n", byte2bin(val));

      uint8_t mode_rc = checkMode();
      if( mode_rc > 0)
      {
        RFM69DEBUG(" ==> wrong mode %s \n", byte2bin(mode_rc));
        receiveBegin();
      }
      // clear Fifo
      if(val & RF_IRQFLAGS2_FIFOFULL)
      {
        rfm69_writeReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
        RFM69DEBUG("fifo cleared %s \n", byte2bin(val));
      }
      if(val & RF_IRQFLAGS2_FIFOLEVEL)
      {
        RFM69DEBUG("fifo level beyond threshold %s \n", byte2bin(val));
        receiveBegin();
      }
    } */
    return false;
  }
  receiveBegin();
  return false;

}

/*
 * Copy the data from the (not yet internal) buffer
 * buffer: the storage the data is copied to
 * rx_length: size of the buffer and how many bytes where copied
 * returns: count of data not fitting in provided buffer
 */
uint8_t rfm69_getData(char *buffer, uint8_t *rx_length)
{
  uint8_t leftover = 0;
  if(*rx_length > RFM69_DATALEN)
  {
    *rx_length = RFM69_DATALEN;
  } else {
    leftover = RFM69_DATALEN - *rx_length;
  }
  for (uint8_t i = 0; i < *rx_length ; i++)
  {
    buffer[i] = RFM69_DATA[i];
  }
  return leftover;
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

// internal function
// select the RFM69 transceiver (save SPI settings, set CS low)
static
void select() {
  cli();
  // save current SPI settings
  _SPCR = _SPCR0;
  _SPSR = _SPSR0;
  // set RFM69 SPI settings
  SET_SPI_MODE0 // setDataMode(SPI_MODE0);
  SET_SPI_MSBFIRST // setBitOrder(MSBFIRST); set to 0 => MSBFIRST
  SET_SPI_CLK_DIV4 // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  PIN_CLEAR(SPI_CS_RFM69);
}

// internal function
// unselect the RFM69 transceiver (set CS high, restore SPI settings)
static
void unselect() {
  PIN_SET(SPI_CS_RFM69);
  // restore SPI settings to what they were before talking to RFM69
  _SPCR0 = _SPCR;
  _SPSR0 = _SPSR;
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

/*
 * reads the modul's temperature
 * suggested default by cpp upstream: calFactor: 0
 */
uint8_t rfm69_readTemperature(uint8_t calFactor) // returns centigrade
{
  setMode(RF69_MODE_STANDBY);
  rfm69_writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((rfm69_readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~rfm69_readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

/*
 * activly force a RC recalibration
 */
void rfm69_rcCalibration()
{
  // setMode(RF69_MODE_STANDBY); according to datasheet
  rfm69_writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((rfm69_readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

// for debugging
static
void print_rfm69_mode(uint8_t mode_value)
{
  RFM69DEBUG("\nTransceiver's operating modes:\nMode : ");
  if ( mode_value == 0b000 ) {
      RFM69DEBUG ( "000 -> Sleep mode (SLEEP)\n" );
  } else if ( mode_value == 0b001 ) {
      RFM69DEBUG ( "001 -> Standby mode (STDBY)\n" );
  } else if ( mode_value == 0b010 ) {
      RFM69DEBUG ( "010 -> Frequency Synthesizer mode (FS)\n" );
  } else if ( mode_value == 0b011 ) {
      RFM69DEBUG ( "011 -> Transmitter mode (TX)\n" );
  } else if ( mode_value == 0b100 ) {
      RFM69DEBUG ( "100 -> Receiver Mode (RX)\n" );
  } else {
      RFM69DEBUG ( " %s -> RESERVED\n", byte2bin(mode_value) );
  }
  RFM69DEBUG ( "\n" );
}

/*
 * prints the values of all rfm69_registers
 * if RFM69_REGISTER_DETAIL is defined via CONFIG
 * it prints details of and explanations for the
 * values too
 */
void rfm69_readAllRegs()
{
  uint8_t regVal;

#ifdef RFM69_REGISTER_DETAIL
  int capVal;

  //... State Variables for intelligent decoding
  uint8_t modeFSK = 0;
  int bitRate = 0;
  int freqDev = 0;
  long freqCenter = 0;
#endif // RFM69_REGISTER_DETAIL

  RFM69DEBUG("Address - HEX - BIN\n");
  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    select();
    spi_send(regAddr & 0x7F);// send address + r/w bit (read)
    regVal = spi_send(0);
    unselect();

    RFM69DEBUG("0x%02x - 0x%02x - %s \n", regAddr, regVal, byte2bin(regVal));

#ifdef RFM69_REGISTER_DETAIL
    switch ( regAddr )
    {
        case 0x1 : {
            RFM69DEBUG ( "Controls the automatic Sequencer ( see section 4.2 )\nSequencerOff : " );
            if ( 0x80 & regVal ) {
                RFM69DEBUG ( "1 -> Mode is forced by the user\n" );
            } else {
                RFM69DEBUG ( "0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n" );
            }

            RFM69DEBUG( "\nEnables Listen mode, should be enabled whilst in Standby mode:\nListenOn : " );
            if ( 0x40 & regVal ) {
                RFM69DEBUG ( "1 -> On\n" );
            } else {
                RFM69DEBUG ( "0 -> Off ( see section 4.3)\n" );
            }

            RFM69DEBUG( "\nAborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n" );
            if ( 0x20 & regVal ) {
                RFM69DEBUG ( "ERROR - ListenAbort should NEVER return 1 this is a write only register\n" );
            }

            capVal = (regVal >> 2) & 0x7;
            print_rfm69_mode(capVal);
            break;
        }

        case 0x2 : {

            RFM69DEBUG("Data Processing mode:\nDataMode : ");
            capVal = (regVal >> 5) & 0x3;
            if ( capVal == 0b00 ) {
                RFM69DEBUG ( "00 -> Packet mode\n" );
            } else if ( capVal == 0b01 ) {
                RFM69DEBUG ( "01 -> reserved\n" );
            } else if ( capVal == 0b10 ) {
                RFM69DEBUG ( "10 -> Continuous mode with bit synchronizer\n" );
            } else if ( capVal == 0b11 ) {
                RFM69DEBUG ( "11 -> Continuous mode without bit synchronizer\n" );
            }

            RFM69DEBUG("\nModulation scheme:\nModulation Type : ");
            capVal = (regVal >> 3) & 0x3;
            if ( capVal == 0b00 ) {
                RFM69DEBUG ( "00 -> FSK\n" );
                modeFSK = 1;
            } else if ( capVal == 0b01 ) {
                RFM69DEBUG ( "01 -> OOK\n" );
            } else if ( capVal == 0b10 ) {
                RFM69DEBUG ( "10 -> reserved\n" );
            } else if ( capVal == 0b11 ) {
                RFM69DEBUG ( "11 -> reserved\n" );
            }

            RFM69DEBUG("\nData shaping: ");
            if ( modeFSK ) {
                RFM69DEBUG( "in FSK:\n" );
            } else {
                RFM69DEBUG( "in OOK:\n" );
            }
            RFM69DEBUG ("ModulationShaping : ");
            capVal = regVal & 0x3;
            if ( modeFSK ) {
                if ( capVal == 0b00 ) {
                    RFM69DEBUG ( "00 -> no shaping\n" );
                } else if ( capVal == 0b01 ) {
                    RFM69DEBUG ( "01 -> Gaussian filter, BT = 1.0\n" );
                } else if ( capVal == 0b10 ) {
                    RFM69DEBUG ( "10 -> Gaussian filter, BT = 0.5\n" );
                } else if ( capVal == 0b11 ) {
                    RFM69DEBUG ( "11 -> Gaussian filter, BT = 0.3\n" );
                }
            } else {
                if ( capVal == 0b00 ) {
                    RFM69DEBUG ( "00 -> no shaping\n" );
                } else if ( capVal == 0b01 ) {
                    RFM69DEBUG ( "01 -> filtering with f(cutoff) = BR\n" );
                } else if ( capVal == 0b10 ) {
                    RFM69DEBUG ( "10 -> filtering with f(cutoff) = 2*BR\n" );
                } else if ( capVal == 0b11 ) {
                    RFM69DEBUG ( "ERROR - 11 is reserved\n" );
                }
            }

            RFM69DEBUG ( "\n" );
            break;
        }

        case 0x3 : {
            bitRate = (regVal << 8);
            break;
        }

        case 0x4 : {
            bitRate |= regVal;
            unsigned long val = 32UL * 1000UL * 1000UL / bitRate;
            RFM69DEBUG ( "Bit Rate (Chip Rate when Manchester encoding is enabled)\nBitRate : %u\n", val);
            break;
        }

        case 0x5 : {
            freqDev = ( (regVal & 0x3f) << 8 );
            break;
        }

        case 0x6 : {
            freqDev |= regVal;
            unsigned long val = 61UL * freqDev;
            RFM69DEBUG( "Frequency deviation\nFdev : %u \n", val );
            break;
        }

        case 0x7 : {
            unsigned long tempVal = regVal;
            freqCenter = ( tempVal << 16 );
            break;
        }

        case 0x8 : {
            unsigned long tempVal = regVal;
            freqCenter = freqCenter | ( tempVal << 8 );
            break;
        }

        case 0x9 : {
            freqCenter = freqCenter | regVal;
            unsigned long val = 61UL * freqCenter;
            RFM69DEBUG ( "RF Carrier frequency\nFRF : %u\n", val);
            break;
        }

        case 0xa : {
            RFM69DEBUG ( "RC calibration control & status\nRcCalDone : " );
            if ( 0x40 & regVal ) {
                RFM69DEBUG ( "1 -> RC calibration is over\n" );
            } else {
                RFM69DEBUG ( "0 -> RC calibration is in progress\n" );
            }

            RFM69DEBUG ( "\n" );
            break;
        }

        case 0xb : {
            RFM69DEBUG ( "Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\nAfcLowBetaOn : " );
            if ( 0x20 & regVal ) {
                RFM69DEBUG ( "1 -> Improved AFC routine\n" );
            } else {
                RFM69DEBUG ( "0 -> Standard AFC routine\n" );
            }
            RFM69DEBUG ( "\n" );
            break;
        }

        case 0xc : {
            RFM69DEBUG ( "Reserved\n\n" );
            break;
        }

        case 0xd : {
            uint8_t val = regVal >> 6;
            RFM69DEBUG ( "Resolution of Listen mode Idle time (calibrated RC osc):\nListenResolIdle : " );
            if ( val == 0b00 ) {
                RFM69DEBUG ( "00 -> reserved\n" );
            } else if ( val == 0b01 ) {
                RFM69DEBUG ( "01 -> 64 us\n" );
            } else if ( val == 0b10 ) {
                RFM69DEBUG ( "10 -> 4.1 ms\n" );
            } else if ( val == 0b11 ) {
                RFM69DEBUG ( "11 -> 262 ms\n" );
            }

            RFM69DEBUG ( "\nResolution of Listen mode Rx time (calibrated RC osc):\nListenResolRx : " );
            val = (regVal >> 4) & 0x3;
            if ( val == 0b00 ) {
                RFM69DEBUG ( "00 -> reserved\n" );
            } else if ( val == 0b01 ) {
                RFM69DEBUG ( "01 -> 64 us\n" );
            } else if ( val == 0b10 ) {
                RFM69DEBUG ( "10 -> 4.1 ms\n" );
            } else if ( val == 0b11 ) {
                RFM69DEBUG ( "11 -> 262 ms\n" );
            }

            RFM69DEBUG ( "\nCriteria for packet acceptance in Listen mode:\nListenCriteria : " );
            if ( 0x8 & regVal ) {
                RFM69DEBUG ( "1 -> signal strength is above RssiThreshold and SyncAddress matched\n" );
            } else {
                RFM69DEBUG ( "0 -> signal strength is above RssiThreshold\n" );
            }

            RFM69DEBUG ( "\nAction taken after acceptance of a packet in Listen mode:\nListenEnd : " );
            val = (regVal >> 1 ) & 0x3;
            if ( val == 0b00 ) {
                RFM69DEBUG ( "00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n" );
            } else if ( val == 0b01 ) {
                RFM69DEBUG ( "01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n" );
            } else if ( val == 0b10 ) {
                RFM69DEBUG ( "10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n" );
            } else if ( val == 0b11 ) {
                RFM69DEBUG ( "11 -> Reserved\n" );
            }

            RFM69DEBUG ( "\n" );
            break;
        }

        case 0x25 : {
            uint8_t val = (regVal >> 6) & 0x3;
            RFM69DEBUG ( "DIO0 mapped %s \n", byte2bin(val));
        }

        default : {
        }
    }
#endif //RFM69_REGISTER_DETAIL
  }

}
