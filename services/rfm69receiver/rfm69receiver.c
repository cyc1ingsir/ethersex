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

/*
 * setup the rmf69 module for receiving
 */
int8_t
rfm69_receiver_init(void)
{
  RFM69RECEIVERDEBUG("rmf init ...\n");
  rfm_init();
  RFM69RECEIVERDEBUG("            ... finished. \n");
  rfm_rxon();
  return 0;
}

/*
 * periodically check if the RMF69 module did receive
 * some data and if so, collect the data
 */
int8_t
rfm69_receiver_receive(void)
{

  char data[MAX_ARRAYSIZE + 1];
  uint8_t rx_length, counter = 0;

  rx_length = 0;

  if (rfm_receiving()) {
    rfm_receive(data, &rx_length);
    data[ ( rx_length+2 <= MAX_ARRAYSIZE )? rx_length + 2 : MAX_ARRAYSIZE ]  = 0;
    RFM69RECEIVERDEBUG("received %d bytes: %s \n", rx_length, data);
    rfm_rxon();
    PIN_CLEAR(STATUSLED_RFM69_TX);
  } else {
    // set status led if nothing was received for quite a while
    if(counter++ > 100) {
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
  timer(10, rfm69_receiver_receive())
*/
