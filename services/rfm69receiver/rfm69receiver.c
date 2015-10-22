/*
 * Copyright (c) 2015 by Meinhard Ritscher <unreachable@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
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
 */

#include "config.h"
#include "rfm69receiver.h"


#ifdef RFM69_SUPPORT
  #include "hardware/radio/rfm69/rfm69.h"
#else
  #error: RFM69_SUPPORT missing
#endif

  uint8_t counter = 0;
  uint16_t packet_nr = 0;
  char * rfm_receiver_buffer[30];
/*
 * setup the rmf69 module for receiving
 */
int8_t
rfm69_receiver_init(void)
{
  RFM69RECEIVERDEBUG("rmf init ...\n");
  rfm69_initialize(RF69_868MHZ, RFM69_NODE_ID, RFM69_NET_ID);
  rfm69_setPowerLevel(20);
  rfm69_setFrequency(RFM69_FREQUENCY);

  RFM69RECEIVERDEBUG("            ... finished. \n");
  RFM69RECEIVERDEBUG("RFM69 frequency: %lu\n", rfm69_getFrequency());
  RFM69RECEIVERDEBUG("RFM69 temperature: %u\n",rfm69_readTemperature(0));
  return 0;
}

/*
 * periodically check if the RMF69 module did receive
 * some data and if so, collect the data
 */
int8_t
rfm69_receiver_receive(void)
{

  if(rfm69_receiveDone())
  {
    counter = 0;
    uint8_t rx_length = 30;
    uint8_t sender_id = RFM69_SENDERID;
    rfm69_getData(rfm_receiver_buffer, &rx_length);
    if(rfm69_ACKRequested())
    {
      rfm69_sendACK("", 0);
    }
    rfm69_receiveDone();
    if(rx_length > 10)
    {
      RFM69RECEIVERDEBUG("[%05u]received %u bytes from %u: %s \n", ++packet_nr, rx_length, sender_id, rfm_receiver_buffer);
    }
    PIN_CLEAR(STATUSLED_RFM69_TX);
    rfm69_receiveDone();
  } else {
    // set status led if nothing was received for quite a while
    if(++counter > 50) {
      PIN_SET(STATUSLED_RFM69_TX);
      counter = 0;
    }
  }
  return 0;
}


/*
  -- Ethersex META --
  header(services/rfm69receiver/rfm69receiver.h)
  init(rfm69_receiver_init)
  timer(20, rfm69_receiver_receive())
*/
