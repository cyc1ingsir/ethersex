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
#include "core/util/string_parsing.h"


#ifdef RFM69_SUPPORT
  #include "hardware/radio/rfm69/rfm69.h"
#else
  #error: RFM69_SUPPORT missing
#endif

  uint8_t counter = 0;
  uint16_t packet_nr = 0;
  uint16_t missed_packets = 0;
  char rfm_receiver_buffer[34];
  char * receiver_buffer_ptr;
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
    receiver_buffer_ptr = rfm_receiver_buffer;
    counter = 0;
    uint8_t rx_length = 34;
    uint8_t sender_id = RFM69_SENDERID;
    rfm69_getData(receiver_buffer_ptr, &rx_length);
    if(rx_length < 34)
      rfm_receiver_buffer[rx_length] = 0;
    if(rfm69_ACKRequested())
    {
      rfm69_sendACK("", 0);
    }
    rfm69_receiveDone();
    if(rx_length > 5)
    {
      if(rfm_receiver_buffer[0]== '[' )
      {
        uint16_t sender_packet_nr;
        if(next_uint16( ++receiver_buffer_ptr, &sender_packet_nr))
        {
          if(sender_packet_nr != packet_nr)
          {
            if (sender_packet_nr - packet_nr > 1)
              missed_packets++;
            packet_nr = sender_packet_nr;
            RFM69RECEIVERDEBUG("[%05u]received %u bytes from %u: %s \n", packet_nr, rx_length, sender_id, ++receiver_buffer_ptr);
          }
          else
          {
            // this might be a packet already received but not with a ACK send too late for the sender
          }
        }
      }
      else
      {
        RFM69RECEIVERDEBUG("received packet with unknown format: %s \n", receiver_buffer_ptr);
      }
      if( missed_packets > 0 )
        RFM69RECEIVERDEBUG("missed_packets: %u\n", missed_packets);
    }
    PIN_CLEAR(STATUSLED_RFM69_TX);
    rfm69_receiveDone();
  } else {
    // set status led if nothing was received for quite a while
    if(++counter > 200)
    {
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
