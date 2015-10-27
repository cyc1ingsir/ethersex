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
#include "climatercv.h"
#include "services/climaterecorder/climate.h"

#include "services/clock/clock.h"
#ifdef RFM69_SUPPORT
  #include "hardware/radio/rfm69/rfm69.h"
#endif

uint8_t rx_length, counter = 0;
char receiver_buffer[MAX_CLIMATE_MESSAGE + 1];
/*
 * parses a integer number to number
 * and returns a pointer to the delimiting ';'
 * behind the parsed number
 */
static char*
parse_number(char* data, int16_t *number, int8_t c, uint8_t len)
{
  char *end, *start = 0;
  end = data;
  for (uint8_t i = 0; i < len; i++)
  {
    if (*end == c)
    {
      start = end;
    }
    if (*end == 0) {
      CLIMATERCVDEBUG("break at i: %d | (%x data) (%x start)  (%x end)\n", i, data, start, end);
      break;
    }
    end++;
  }
  if (start == 0)
    return 0;

  if(*(++start) != ':') return 0;

  // parsing number using horner scheme
  int8_t sign = 1;

  if(*(++start) == '-')
  {
    sign = -1;
    start++;
  }

  uint8_t digit = 0;
  while(start < end)
  {
    if (digit > 9) // can't get smaller than 0 due to unsigend int
    {
//      CLIMATERCVDEBUG("number: %d, digit %d - pointing to %s \n", *number, digit, start);
      *number = 0;
      return 0;
    }
    *number *= 10;
    *number += digit;
    if(*start == ';')
      break;
    digit = *start - '0';
    start++;
  }
  if(*start != ';'){
    // terminating ; missing => not valid
//    CLIMATERCVDEBUG("number: %d, digit %d - pointing to %s  (%x end)\n", *number, digit, start, end);
    *number = 0;
    return 0;
  }
  *number *= sign;
//  CLIMATERCVDEBUG("parsed %d , pointing to %s \n", *number, start);
  return start;
}

static uint8_t
parse_record(char* data, uint8_t len)
{
  int16_t uid = 0;
  char *start = parse_number(data, &uid, 'u', len );
  if( ! start ) return 1;
  if (!clock_last_sync()){
    /*
     * we do not need to parse this further
     * as we can't attach a timestamp to this record
     * still we should acknowledge that we have received a
     * record from this node and retransmitting is not
     * necessary at the moment
     */
    return 99;
  }

  int16_t temp = 0;
  start = parse_number(start, &temp, 't', data + len - start);
  if( ! start ) return 2;

  int16_t humid = 0;
  start = parse_number(start, &humid, 'h', data + len - start );
  if( ! start ) return 3;

  for (uint8_t i = 0; i < climate_nodes_count; i++)
  {
    if (climate_nodes[i].uid == (uint8_t) uid)
    {
      climate_nodes[i].records[climate_nodes[i].current].temp = temp;
      climate_nodes[i].records[climate_nodes[i].current].humid = (uint16_t) humid;
      climate_nodes[i].records[climate_nodes[i].current].timestamp = clock_get_time();
      CLIMATERCVDEBUG("received and stored record in [%d] from node: %d temp: %d humid: %d at %lu\n",
                      climate_nodes[i].current,
                      uid,
                      temp,
                      humid,
                      climate_nodes[i].records[climate_nodes[i].current].timestamp);
      break;
    }
  }
  return 0;
}

/*
 * setup the rmf69 module for receiving
 */
int8_t
climate_receiver_init(void)
{
  CLIMATERCVDEBUG("rmf init ...\n");
  rfm69_initialize(RF69_868MHZ, RFM69_NODE_ID, RFM69_NET_ID);
  rfm69_setPowerLevel(20);
  rfm69_setFrequency(RFM69_FREQUENCY);
  CLIMATERCVDEBUG("            ... finished. \n");
  return 0;
}

/*
 * periodically check if the RMF69 module did receive
 * some data and if so, collect the data
 */
int8_t
climate_receiver_receive(void)
{

  if(rfm69_receiveDone())
  {
    receiver_buffer_ptr = receiver_buffer;
    counter = 0;
    uint8_t rx_length = MAX_CLIMATE_MESSAGE;
    uint8_t sender_id = RFM69_SENDERID;
    rfm69_getData(receiver_buffer_ptr, &rx_length);
    receiver_buffer[ ( rx_length+2 <= MAX_CLIMATE_MESSAGE )? rx_length + 2 : MAX_CLIMATE_MESSAGE ]  = 0;
    if(rfm69_ACKRequested())
    {
      rfm69_sendACK("", 0);
    }
    rfm69_receiveDone();
    if(rx_length > 5)
    {
      CLIMATERCVDEBUG("received %d bytes: %s \n", rx_length, receiver_buffer_ptr);
      uint8_t rc = parse_record(receiver_buffer_ptr, rx_length);
      if( rc )
      {
        CLIMATERCVDEBUG("not able to parse received record rc=%d\n", rc);
      }
    }
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
  header(services/climatereceiver/climatercv.h)
  init(climate_receiver_init)
  timer(10, climate_receiver_receive())
*/
