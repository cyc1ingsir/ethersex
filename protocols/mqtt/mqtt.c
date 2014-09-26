/*
 * Copyright (c) 2014 by Philip Matura <ike@tura-home.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
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
 */


/*
 * A simple MQTT client.
 *
 * With inspiration from
 * https://github.com/knolleary/pubsubclient
 *
 * *Usage*:
 *
 *  - create a mqtt_connection_config_t structure somewhere
 *  - call mqtt_set_connection_config(.)
 *  - the connection will be established in the next uip_poll cycle
 *  - the connack_callback will be fired (if supplied)
 *  - you may subscribe to topics using construct_subscribe_packet(.)
 *  - the poll_callback will be fired each uip_poll cycle (if supplied)
 *  - the publish_callback will be fired when a publish packet arrives
 *    (if supplied)
 *  - the close_callback will be fired on a connection close/abort
 *    (if supplied)
 *
 * make sure to only write packets when the connection is established
 * (mqtt_is_connected() returns true), which is guaranteed during the
 * connack, poll, publish callbacks.
 *
 * The MQTT_SENBUFFER_LENGTH can be configured freely, however do not increase
 * its value above 256. For values >256 the state variables need to be 16 bit.
 *
 * May an explanation of the buffer layout is in order:
 *
 * The send_buffer is used for multiple purposes. It may contain (in this
 * order):
 *
 *  - The last sent packet (stored for a uip retransmit request)
 *    Its length is stored in send_buffer_last_length (and may be 0)
 *
 *  - The send queue. Binary data that will be sent at the next uip poll
 *    request. send_buffer_current_head stores the index behind this data
 *    block, so writing new data to the buffer becomes
 *
 *      send_buffer[send_buffer_current_head++] = new_byte;
 *
 *  - Unused buffer space.
 *
 *  - Receive buffer. Fragmented packets are stored and will be rebuilt here.
 *    When new data is added the whole receive buffer segment will be copied
 *    the need amount of bytes backward.
 *
 *  *Note*:
 *
 *  Because of the implementation, the code can actually work with incoming
 *  packets larger than MQTT_SENDBUFFER_LENGTH, but only if they arrive within
 *  one TCP segment.
 *
 */

// TODO check:
// what happens if uip_send() is called after uip_abort()?


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "config.h"
#include "protocols/uip/uip.h"
#include "mqtt.h"
#include "mqtt_state.h"


#define STATE (&con_state)


// BUFFER VARIABLES

static uint8_t send_buffer[MQTT_SENDBUFFER_LENGTH];
static uint8_t send_buffer_last_length;  // length of last packet
                                         //   (for uip-retransmit)
static uint8_t send_buffer_current_head; // current buffer head
static uint8_t receive_buffer_length;    // length of data for received buffer
static uint16_t receive_packet_length;   // length of data for received buffer

// MQTT PROTOCOL STATE

static uint16_t nextMsgId;
static uint16_t lastOutActivity;
static uint16_t lastInActivity;
static bool pingOutstanding;

// GENERAL STATE STRUCTURES

static mqtt_connection_config_t const *con_config = NULL;
static mqtt_connection_state_t con_state;
static uip_conn_t *mqtt_conn;

static uint16_t timer_counter = 0;


/********************
 *                  *
 *   Declarations   *
 *                  *
 ********************/

static inline void abort_connection(void);
static inline uint8_t MQTT_LF_LENGTH(uint16_t length);
static inline void mqtt_reset(void);
static inline void make_new_message_id(void);
static inline uint16_t minimum(uint16_t a, uint16_t b);

static inline void mqtt_buffer_write_data(const void *data, uint16_t length);
static void mqtt_buffer_write_string(char const *data);
static inline bool mqtt_buffer_free(uint16_t length);
static void mqtt_flush_buffer(void);
static inline void mqtt_retransmit(void);
static inline void mqtt_received_ack(void);
static uint8_t mqtt_buffer_write_length_field(uint8_t *buffer,
    uint16_t length);
static bool mqtt_write_to_receive_buffer(const void *data, uint16_t length);

static bool construct_connect_packet(void);
bool construct_publish_packet(char const *topic, const void *payload,
    uint16_t payload_length, bool retain);
bool construct_subscribe_packet(char const *topic);
bool construct_unsubscribe_packet(char const *topic);
bool construct_zerolength_packet(uint8_t msg_type);
bool construct_ack_packet(uint8_t msg_type, uint16_t msgid);

static void mqtt_handle_packet(const void *data, uint8_t llen,
    uint16_t packet_length);
static uint8_t mqtt_parse_length_field(uint16_t *length, const void *buffer,
    uint16_t max_read);
static void mqtt_parse(void);

static void mqtt_poll(void);
static void mqtt_main(void);
static void mqtt_init(void);
void mqtt_periodic(void);

void mqtt_set_connection_config(mqtt_connection_config_t const *config);


/*********************
 *                   *
 *      Misc         *
 *                   *
 *********************/

static inline void
abort_connection(void)
{
  uip_abort();
  mqtt_conn = NULL;

  if (con_config && con_config->close_callback)
    con_config->close_callback();
}

// return the length of a mqtt variable length field
static inline uint8_t
MQTT_LF_LENGTH(uint16_t length)
{
  return (length / 128) + 1;
}

// reset state
static inline void
mqtt_reset(void)
{
  send_buffer_last_length = send_buffer_current_head = receive_buffer_length
    = receive_packet_length = 0;
  nextMsgId = 0;
  pingOutstanding = false;
  lastInActivity = lastOutActivity = timer_counter;
}

// the message id must not be 0
static inline void
make_new_message_id(void)
{
  if (++nextMsgId == 0)
    ++nextMsgId;
}

// simple mathematics
static inline uint16_t
minimum(uint16_t a, uint16_t b) { return a < b ? a : b; }


/*********************
 *                   *
 *  Buffer Handling  *
 *                   *
 *********************/

// write data to the buffer (no free space check)
static inline void
mqtt_buffer_write_data(const void *data, uint16_t length)
{
  memcpy(send_buffer + send_buffer_current_head, data, length);
  send_buffer_current_head += length;
}

// write a c-string to the buffer, inserting length field
// (no free space check)
static void
mqtt_buffer_write_string(char const *data)
{
  char const *idp = data;
  uint16_t i = 0;
  send_buffer_current_head += 2;
  while (*idp) {
     send_buffer[send_buffer_current_head++] = *idp++;
     i++;
  }
  send_buffer[send_buffer_current_head-i-2] = (i >> 8);
  send_buffer[send_buffer_current_head-i-1] = (i & 0xFF);
}

// return whether the buffer has enough storage room for `length` bytes
static inline bool
mqtt_buffer_free(uint16_t length)
{
  return send_buffer_current_head + length + receive_buffer_length
    <= MQTT_SENDBUFFER_LENGTH;
}

// flush the send buffer (uip_send) if there is data
static void
mqtt_flush_buffer(void)
{
  if (send_buffer_last_length == 0 // no data waiting for a uip_ack
      && send_buffer_current_head > 0)
  {
    send_buffer_last_length = minimum(send_buffer_current_head, uip_mss());
    uip_send(send_buffer, send_buffer_last_length);
    lastOutActivity = timer_counter;
  }
}

// uip wanted a retransmit
static inline void
mqtt_retransmit(void)
{
  if (send_buffer_last_length > 0)
    uip_send(send_buffer, send_buffer_last_length);
}

// respond to a received uip_ack
static inline void
mqtt_received_ack(void)
{
  // discard last sent data
  if (send_buffer_last_length > 0)
  {
    // copy queue to beginning of buffer
    memcpy(send_buffer, send_buffer + send_buffer_last_length,
        send_buffer_current_head - send_buffer_last_length);
    send_buffer_current_head -= send_buffer_last_length;
    send_buffer_last_length = 0;
  }
}

// write mqtt length field to buffer
// return number of bytes written
static uint8_t
mqtt_buffer_write_length_field(uint8_t *buffer, uint16_t length)
{
  uint8_t llen = 0;
  uint8_t digit;
  uint8_t pos = 0;
  do {
    digit = length % 128;
    length = length / 128;
    if (length > 0) {
      digit |= 0x80;
    }
    buffer[pos++] = digit;
    llen++;
  } while(length>0);
  return llen;
}

// write some data into the receive buffer
// return false if there is not enough free buffer space
static bool
mqtt_write_to_receive_buffer(const void *data, uint16_t length)
{
  if (!mqtt_buffer_free(length))
    return false;

  // move receive buffer backward
  memcpy(
      send_buffer + MQTT_SENDBUFFER_LENGTH - receive_buffer_length - length,
      send_buffer + MQTT_SENDBUFFER_LENGTH - receive_buffer_length,
      receive_buffer_length);
  // copy new data to the end
  memcpy(
      send_buffer + MQTT_SENDBUFFER_LENGTH - length,
      data,
      length);
  // increase data counter
  receive_buffer_length += length;

  return true;
}


/*********************
 *                   *
 *  Sending Packets  *
 *                   *
 *********************/

static bool
construct_connect_packet(void)
{
  uint8_t protocol_string[9] =
    {0x00,0x06,'M','Q','I','s','d','p',MQTTPROTOCOLVERSION};

  // calculate length
  uint16_t length = sizeof(protocol_string)
    + 1                                 // connect flags
    + strlen(con_config->client_id) + 2 // client id
    + 2;                                // keep alive
  if (con_config->will_topic)
    length += strlen(con_config->will_topic)
      + strlen(con_config->will_message) + 4;
  if (con_config->user)
    length += strlen(con_config->user) + 2;
  if (con_config->pass)
    length += strlen(con_config->pass) + 2;

  // packet length + length field + header flags
  if (!mqtt_buffer_free(length + MQTT_LF_LENGTH(length) + 1))
    return false; // this should not happen (first paket sent)

  // fixed header
  send_buffer[send_buffer_current_head++] = MQTTCONNECT;
  send_buffer_current_head += mqtt_buffer_write_length_field(
      send_buffer + send_buffer_current_head,
      length);

  // protocol string
  unsigned int j;
  for (j = 0;j<9;j++) {
    send_buffer[send_buffer_current_head++] = protocol_string[j];
  }

  // connect flags
  uint8_t connect_flags;
  if (con_config->will_topic) {
    connect_flags = 0x06
      | (con_config->will_qos<<3)
      | ((con_config->will_retain?1:0)<<5);
  } else {
    connect_flags = 0x02;
  }

  if(con_config->user != NULL) {
    connect_flags = connect_flags|0x80;

    if(con_config->pass != NULL) {
       connect_flags = connect_flags|(0x80>>1);
    }
  }

  send_buffer[send_buffer_current_head++] = connect_flags;

  // keep alive
  send_buffer[send_buffer_current_head++] = ((MQTT_KEEPALIVE) >> 8);
  send_buffer[send_buffer_current_head++] = ((MQTT_KEEPALIVE) & 0xFF);

  // client id
  mqtt_buffer_write_string(con_config->client_id);

  // will
  if (con_config->will_topic) {
    mqtt_buffer_write_string(con_config->will_topic);
    mqtt_buffer_write_string(con_config->will_message);
  }

  // user / pass
  if(con_config->user != NULL) {
    mqtt_buffer_write_string(con_config->user);
    if(con_config->pass != NULL) {
      mqtt_buffer_write_string(con_config->pass);
    }
  }

  return true;
}


bool
construct_publish_packet(char const *topic, const void *payload,
    uint16_t payload_length, bool retain)
{
  // maybe make this a parameter (at least qos=1 should already be operational)
  const uint8_t qos = 0;

  uint16_t length = strlen(topic) + 2
    + payload_length;
  if (qos > 0)
    length += 2; // message id

  if (!mqtt_buffer_free(length + MQTT_LF_LENGTH(length) + 1))
    return false;

  // header flags
  uint8_t header = MQTTPUBLISH | qos<<1;
  if (retain)
    header |= 1<<0;

  // fixed header
  send_buffer[send_buffer_current_head++] = header;
  send_buffer_current_head += mqtt_buffer_write_length_field(
      send_buffer + send_buffer_current_head, length);

  // topic
  mqtt_buffer_write_string(topic);

  // message id
  if (qos > 0)
  {
    send_buffer[send_buffer_current_head++] = (nextMsgId >> 8);
    send_buffer[send_buffer_current_head++] = (nextMsgId & 0xFF);
    make_new_message_id();
  }

  mqtt_buffer_write_data(payload, payload_length);

  return true;
}


bool
construct_subscribe_packet(char const *topic)
{
  uint16_t length = 2   // message id
    + strlen(topic) + 2 // topic
    + 1;                // qos

  if (!mqtt_buffer_free(length + MQTT_LF_LENGTH(length) + 1))
    return false;

  // fixed header
  const uint8_t qos = 1;
  send_buffer[send_buffer_current_head++] = MQTTSUBSCRIBE | qos<<1;
  send_buffer_current_head += mqtt_buffer_write_length_field(
      send_buffer + send_buffer_current_head, length);

  // message id
  send_buffer[send_buffer_current_head++] = (nextMsgId >> 8);
  send_buffer[send_buffer_current_head++] = (nextMsgId & 0xFF);
  make_new_message_id();

  // payload: topic + requested qos
  mqtt_buffer_write_string(topic);
  send_buffer[send_buffer_current_head++] = 0; // requested qos level

  return true;
}


bool
construct_unsubscribe_packet(char const *topic)
{
  uint16_t length = 2    // message id
    + strlen(topic) + 2; // topic

  if (!mqtt_buffer_free(length + MQTT_LF_LENGTH(length) + 1))
    return false;

  // fixed header
  const uint8_t qos = 1;
  send_buffer[send_buffer_current_head++] = MQTTUNSUBSCRIBE | qos<<1;
  send_buffer_current_head += mqtt_buffer_write_length_field(send_buffer
      + send_buffer_current_head, length);

  // message id
  send_buffer[send_buffer_current_head++] = (nextMsgId >> 8);
  send_buffer[send_buffer_current_head++] = (nextMsgId & 0xFF);
  make_new_message_id();

  // payload: topic
  mqtt_buffer_write_string(topic);

  return true;
}


// msg_type may be one of:
// MQTTPINGREQ
// MQTTPINGRESP
// MQTTDISCONNECT
bool
construct_zerolength_packet(uint8_t msg_type)
{
  if (!mqtt_buffer_free(2))
    return false;

  // fixed header
  send_buffer[send_buffer_current_head++] = msg_type;
  send_buffer[send_buffer_current_head++] = 0; // length field

  return true;
}


// msg_type may be one of:
// MQTTPUBACK
// MQTTPUBREC
// MQTTPUBREL // qos of 1 is automatically added
// MQTTPUBCOMP
bool
construct_ack_packet(uint8_t msg_type, uint16_t msgid)
{
  if (!mqtt_buffer_free(4))
    return false;

  uint8_t header_flags = msg_type;
  if (msg_type == MQTTPUBREL)
    header_flags |= MQTTQOS1;

  // fixed header
  send_buffer[send_buffer_current_head++] = header_flags;
  send_buffer[send_buffer_current_head++] = 2; // length field

  // message id
  send_buffer[send_buffer_current_head++] = (msgid >> 8);
  send_buffer[send_buffer_current_head++] = (msgid & 0xFF);

  return true;
}


/***********************
 *                     *
 *  Receiving Packets  *
 *                     *
 ***********************/

// parse and react to a received packet
static void
mqtt_handle_packet(const void *data, uint8_t llen, uint16_t packet_length)
{
  lastInActivity = timer_counter;

  const uint8_t *packet = data + llen + 1;
  uint8_t header = *(uint8_t*) data;


  //
  // STATE: CONNECT
  //

  if (STATE->stage == MQTT_STATE_CONNECT)
  {
    // only accept connack packets
    if ((header & 0xf0) != MQTTCONNACK)
    {
      MQTTDEBUG ("expected connack packet, aborting\n");
      abort_connection();
      return;
    }

    // assert packet length
    if (packet_length < 2)
    {
      MQTTDEBUG ("packet length assert connack, aborting\n");
      abort_connection();
      return;
    }

    // check return code
    if (packet[1] != 0)
    {
      MQTTDEBUG ("connection request error code, aborting\n");
      abort_connection();
      return;
    }

    MQTTDEBUG ("connack received\n");
    STATE->stage = MQTT_STATE_CONNECTED;

    // auto subscribe
    if (con_config->auto_subscribe_topics)
      for (uint8_t i=0; con_config->auto_subscribe_topics[i] != NULL; i++)
        construct_subscribe_packet(con_config->auto_subscribe_topics[i]);

    if (con_config && con_config->connack_callback)
      con_config->connack_callback();
  }


  //
  // STATE: CONNECTED
  //

  else if (STATE->stage == MQTT_STATE_CONNECTED)
  {
    switch (header & 0xf0)
    {
      case MQTTPUBLISH:

        ; // a declaration can't be the first statement after a case label
        uint8_t qos = (header & 0x06) >> 1;

        // assert packet length
        if (packet_length < 2 + (qos ? 2:0))
        {
          MQTTDEBUG ("packet length assert pub1, aborting\n");
          abort_connection();
          return;
        }

        uint16_t topic_length = packet[0] * 256 + packet[1];

        // assert packet length again
        if (packet_length < 2 + (qos ? 2:0) + topic_length)
        {
          MQTTDEBUG ("packet length assert pub2, aborting\n");
          abort_connection();
          return;
        }

        if (con_config && con_config->publish_callback)
        {
          const void *payload = packet + 2 + topic_length;
          uint16_t payload_length = packet_length - (2 + topic_length);

          if (qos > 0)
          {
            payload += 2;
            payload_length -= 2;
          }

          con_config->publish_callback((char*) packet + 2, topic_length,
              payload, payload_length);
        }

        // check for qos level, send ack
        if (qos > 0)
        {
          uint16_t msgid = packet[2 + topic_length] * 256
            + packet[2 + topic_length + 1];

          if (qos == 1)
            construct_ack_packet(MQTTPUBACK, msgid);

          else if (qos == 2)
            construct_ack_packet(MQTTPUBREC, msgid);
        }

        MQTTDEBUG ("publish received\n");

        break;


      case MQTTPUBACK:
        break; // hmm 'kay


      case MQTTPUBREC:

        // assert packet length
        if (packet_length < 2)
        {
          MQTTDEBUG ("packet length assert rec, aborting\n");
          abort_connection();
          return;
        }

        {
          uint16_t msgid = packet[0] * 256 + packet[1];

          construct_ack_packet(MQTTPUBREL, msgid);
        }

        break;


      case MQTTPUBREL:

        // assert packet length
        if (packet_length < 2)
        {
          MQTTDEBUG ("packet length assert rel, aborting\n");
          abort_connection();
          return;
        }

        {
          uint16_t msgid = packet[0] * 256 + packet[1];

          construct_ack_packet(MQTTPUBCOMP, msgid);
        }

        break;


      case MQTTPUBCOMP:
        break; // hmm 'kay


      case MQTTSUBACK:
      case MQTTUNSUBACK:
        break; // hmm 'kay


      case MQTTPINGREQ:
        MQTTDEBUG ("pingreq received\n");
        construct_zerolength_packet(MQTTPINGRESP);
        break;


      case MQTTPINGRESP:
        MQTTDEBUG ("pingresp received\n");
        pingOutstanding = false;
        break;


      case MQTTCONNACK:
      case MQTTCONNECT:
      case MQTTSUBSCRIBE:
      case MQTTUNSUBSCRIBE:
      case MQTTDISCONNECT:
        break; // ?
    }
  }


  //
  // STATE: UNKNOWN
  //

  else
  {
    MQTTDEBUG ("unkown state\n"); // uhmm
  }
}

// parse the variable length field
// return number of bytes read
static uint8_t
mqtt_parse_length_field(uint16_t *length, const void *buffer, uint16_t max_read)
{
  uint16_t l = 0;

  for (uint8_t i=0; i<max_read; i++)
  {
    l += ( ((uint8_t*)buffer)[i] & (~0x80) ) << (i*7);
    if ( ( ((uint8_t*)buffer)[i] & 0x80) == 0x00 )
    {
      *length = l;
      return i + 1;
    }
  }

  return 0; // length field is not complete
}

// parse the input buffer/stream, rebuild fragmented packets, delegate
// handle_packet when a full packet is received
static void
mqtt_parse(void)
{
  uint16_t remaining_length;
  uint16_t bytes_read = 0;

  while (bytes_read < uip_len)
  {
    MQTTPARSEDEBUG ("loop\n");
    remaining_length = uip_len - bytes_read;

    // no data in receive buffer
    if (!receive_buffer_length)
    {
      MQTTPARSEDEBUG ("no_buffer\n");
      // try to parse the length field
      uint16_t packet_length = 0;
      // the +1 and -1 come from the first header byte
      uint8_t llen = mqtt_parse_length_field(
          &packet_length,
          uip_appdata + bytes_read + 1,
          remaining_length - 1);

      // length field is not completely received
      if (llen == 0)
      {
        MQTTPARSEDEBUG ("length field incomplete\n");
        if (!mqtt_write_to_receive_buffer(uip_appdata + bytes_read,
              remaining_length))
        {
          MQTTPARSEDEBUG ("buffer_full1\n");
          abort_connection();
          return;
        }
        bytes_read += remaining_length;
      }

      // the complete packet is in the uip buffer
      else if (packet_length + llen + 1 <= remaining_length)
      {
        MQTTPARSEDEBUG ("handle_packet\n");
        mqtt_handle_packet(uip_appdata + bytes_read, llen, packet_length);
        bytes_read += packet_length + llen + 1;
      }

      // length field received, packet incomplete
      // write it to receive buffer
      else
      {
        receive_packet_length = packet_length;
        if (!mqtt_write_to_receive_buffer(uip_appdata + bytes_read,
              remaining_length))
        {
          MQTTPARSEDEBUG ("buffer_full2\n");
          abort_connection();
          return;
        }
        bytes_read += remaining_length;
      }
    }

    // packet length already known
    else if (receive_packet_length != 0)
    {
      MQTTPARSEDEBUG ("length_known\n");
      // check whether we now have the complete packet
      if (receive_buffer_length + remaining_length >= receive_packet_length)
      {
        uint16_t fragment_length = receive_packet_length
          - receive_buffer_length;

        // write this last fragment to the receive buffer
        if (!mqtt_write_to_receive_buffer(uip_appdata + bytes_read,
              fragment_length))
        {
          MQTTPARSEDEBUG ("buffer_full3\n");
          abort_connection();
          return;
        }

        mqtt_handle_packet(
            send_buffer + MQTT_SENDBUFFER_LENGTH - receive_buffer_length,
            MQTT_LF_LENGTH(receive_packet_length),
            receive_packet_length);
        bytes_read += fragment_length;
      }

      else
      {
        // write this new fragment to the receive buffer
        if (!mqtt_write_to_receive_buffer(uip_appdata + bytes_read,
              remaining_length))
        {
          MQTTPARSEDEBUG ("buffer_full4\n");
          abort_connection();
          return;
        }
        bytes_read += remaining_length;
      }
    }

    // packet length not known
    else
    {
      MQTTPARSEDEBUG ("length_unknown\n");
      // write one byte to the receive buffer, and try to parse the length
      // field again
      if (!mqtt_write_to_receive_buffer(uip_appdata + bytes_read, 1))
        {
          MQTTPARSEDEBUG ("buffer_full5\n");
          abort_connection();
          return;
        }
      bytes_read += 1;

      // try to parse the length field
      uint16_t packet_length = 0;
      // the +1 and -1 come from the first header byte
      uint8_t llen = mqtt_parse_length_field(
          &packet_length,
          send_buffer + MQTT_SENDBUFFER_LENGTH - receive_buffer_length + 1,
          receive_buffer_length - 1);

      if (llen != 0)
        receive_packet_length = packet_length;

      // now just fall through the loop
    }
  }
}


/********************
 *                  *
 *    Main Logic    *
 *                  *
 ********************/

// periodically called function, detect and react to timeouts
static void
mqtt_poll(void)
{
  if (
      (timer_counter - lastInActivity > MQTT_KEEPALIVE * TIMER_TICKS_PER_SECOND)
      ||
      (timer_counter - lastOutActivity > MQTT_KEEPALIVE * TIMER_TICKS_PER_SECOND)
     )
  {
    if (pingOutstanding)
    {
      MQTTDEBUG ("missed ping, aborting\n");
      abort_connection();
      return;
    }
    else
    {
      MQTTDEBUG ("ping request\n");
      construct_zerolength_packet(MQTTPINGREQ);

      // reset counter, wait another keepalive period
      lastInActivity = timer_counter;
      pingOutstanding = true;
    }
  }

  if (con_config && con_config->poll_callback)
    con_config->poll_callback();
}

// mqtt uip callback, manage connection state, delegate I/O
static void
mqtt_main(void)
{

  if (uip_aborted() || uip_timedout()) {
    MQTTDEBUG ("connection aborted\n");
    mqtt_conn = NULL;

    if (con_config && con_config->close_callback)
      con_config->close_callback();

    return;
  }

  if (uip_closed()) {
    MQTTDEBUG ("connection closed\n");
    mqtt_conn = NULL;
    return;
  }

  if (uip_connected())
  {
    MQTTDEBUG ("new connection\n");

    construct_connect_packet();

    // init
    nextMsgId = 1;
    STATE->stage = MQTT_STATE_CONNECT;

    // send
    mqtt_flush_buffer();
  }

  if (uip_acked())
  {
    MQTTDEBUG ("acked\n");
    mqtt_received_ack();
  }

  if (uip_rexmit ())
  {
    MQTTDEBUG ("mqtt main rexmit\n");
    mqtt_retransmit();
  }

  else if (uip_newdata() && uip_len) {
    MQTTDEBUG ("received data: \n");
    mqtt_parse();
  }

  else if (uip_poll() && STATE->stage == MQTT_STATE_CONNECTED)
  {
    MQTTDEBUG ("mqtt main poll\n");
    mqtt_poll();
    mqtt_flush_buffer();
  }

  else if (uip_poll() && STATE->stage == MQTT_STATE_CONNECT)
  {
    if (timer_counter - lastInActivity
        > MQTT_KEEPALIVE * TIMER_TICKS_PER_SECOND)
    {
      MQTTDEBUG ("connect request timed out\n");
      abort_connection();
      return;
    }
  }
}


// periodic mqtt timer callback, initialize connection if non-active
void
mqtt_periodic(void)
{
  timer_counter++;

  if (! mqtt_conn)
      mqtt_init();
}


// initialize mqtt connection if config data has been supplied
static void
mqtt_init(void)
{
  if (!mqtt_conn)
  {
    if (!con_config)
    {
      MQTTDEBUG ("cannot initialize mqtt client, no config\n");
      return;
    }

    MQTTDEBUG ("initializing mqtt client\n");

    mqtt_reset(); // reset state
    // uip_connect wants a non-constant pointer
    mqtt_conn = uip_connect((uip_ipaddr_t*) &con_config->target_ip, HTONS(1883), mqtt_main);

    if (! mqtt_conn) {
      MQTTDEBUG ("no uip_conn available.\n");
      return;
    }
  }
}


// Set the connection config for mqtt
void
mqtt_set_connection_config(mqtt_connection_config_t const *config)
{
  con_config = config;
}

/*
  -- Ethersex META --
  header(protocols/mqtt/mqtt.h)
  timer(1, mqtt_periodic())
*/
