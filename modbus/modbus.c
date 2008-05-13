/* vim:fdm=marker ts=4 et ai
 * {{{
 *
 * Copyright (c) 2008 by Christian Dietrich <stettberger@dokucode.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 }}} */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <string.h>
#include "../eeprom.h"
#include "../net/yport_net.h"
#include "../bit-macros.h"
#include "../usart.h"
#include "../config.h"
#include "../net/modbus_net.h"
#include "modbus.h"

#include "../pinning.c"

#ifdef MODBUS_SUPPORT

volatile struct modbus_buffer modbus_send_buffer;
uint8_t modbus_recv_timer = 0;

uint16_t 
modbus_crc_calc(uint8_t *data, uint8_t len) 
{
  uint16_t crc = 0xffff;
  uint8_t i = 0;
  while(i < len)
    crc = _crc16_update(crc, data[i++]);
  return crc;
}

void
modbus_init(void)
{
    /* Initialize the usart module */
    usart_init();


    /* Enable RX/TX Swtich as Output */
    DDR_CONFIG_OUT(MODBUS_TX);
    PIN_CLEAR(MODBUS_TX);

    modbus_send_buffer.len = 0;
    modbus_send_buffer.sent = 0;
    modbus_send_buffer.crc_len = 0;
    
    /* set baud rate */
    _UBRRH_UART0 = HI8(MODBUS_UART_UBRR);
    _UBRRL_UART0 = LO8(MODBUS_UART_UBRR);
}

void
modbus_periodic(void)
{
  if (modbus_recv_timer == 0) return;
  modbus_recv_timer--;

  if (!modbus_conn) return;
  if (!modbus_conn->appstate.modbus.waiting_for_answer) return;
  if (modbus_recv_timer != 0) return;
  modbus_conn->appstate.modbus.waiting_for_answer = 0;
  modbus_conn->appstate.modbus.new_data = 1;
  modbus_conn = NULL;
}

uint8_t 
modbus_rxstart(uint8_t *data, uint8_t len) {
  if (modbus_send_buffer.crc_len != 0) return 0; /* There is an packet on the way */

  /* enable the transmitter */
  PIN_SET(MODBUS_TX);

  modbus_send_buffer.crc = modbus_crc_calc(data, len);
  modbus_send_buffer.crc_len = 2;

  modbus_send_buffer.data = data;
  modbus_send_buffer.len = len;

  /* Enable the tx interrupt and send the first character */
  modbus_send_buffer.sent = 1;
  _UCSRB_UART0 |= _BV(_TXCIE_UART0);
  _UDR_UART0 = data[0];

  return 1;
}

SIGNAL(USART0_TX_vect)
{
  if (modbus_send_buffer.sent < modbus_send_buffer.len) {
    _UDR_UART0 = modbus_send_buffer.data[modbus_send_buffer.sent++];
  } else if (modbus_send_buffer.crc_len != 0) {
    /* Send the crc checksum */
    _UDR_UART0 = modbus_send_buffer.crc >> (( 1 - (--modbus_send_buffer.crc_len)) * 8);
  } else {
    /* Disable this interrupt */
    _UCSRB_UART0 &= ~(_BV(_TXCIE_UART0));
    /* Disable the transmitter */
    PIN_CLEAR(MODBUS_TX);
    /* free the modbus_conn */
    modbus_conn->appstate.modbus.must_send = 0;
    modbus_conn->appstate.modbus.waiting_for_answer = 1;
    modbus_conn->appstate.modbus.len = 0;
    modbus_recv_timer = 4;
  }
}

SIGNAL(USART0_RX_vect)
{
  /* Ignore errors */
  if ((_UCSRA_UART0 & _BV(DOR0)) || (_UCSRA_UART0 & _BV(FE0))) {
    uint8_t v = _UDR_UART0;
    (void) v;
    return; 
  }
  uint8_t data = _UDR_UART0;

  if (!modbus_conn) return;
  // Are we waiting for an answer?
  if (!modbus_conn->appstate.modbus.waiting_for_answer) return;
  // Is the buffer big enough
  if (modbus_conn->appstate.modbus.len >= MODBUS_BUFFER_LEN) return;

  modbus_conn->appstate.modbus.data[modbus_conn->appstate.modbus.len++] = data;

  modbus_recv_timer = 2;
}

#endif
