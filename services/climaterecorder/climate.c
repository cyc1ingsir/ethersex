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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "climate.h"

#include "core/util/fixedpoint.h"
#include "core/periodic.h"      /* for HZ */
#include "hardware/dht/dht.h"
#include "services/filelogger/filelogger.h"
#include "services/clock/clock.h"

#define NELEMS(x) (sizeof(x)/sizeof(x[0]))


/* global variables */
#include "node_config.h"
uint8_t climate_nodes_count = NELEMS(climate_nodes);

const uint8_t CLIMATE_LOGGING_INTERVAL = 60;
uint8_t is_init;

/*
 *
 */
static int8_t
climate_recorder_init(void)
{
  interval = CLIMATE_LOGGING_INTERVAL * HZ / 300;

  // would be very nice if this function could be called
  // after the system clock has been synced
  if( filelogger_init("climate") == 0)
  {
    is_init = 1;
    CLIMATEDEBUG("successfully init filelogger from climaterecorder\n");
    return 0;
  }
  return 1;
}

int8_t
climate_recorder_update(void)
{
  for (uint8_t i = 0; i < NELEMS(climate_nodes); i++)
  {
    climate_node_t *node = &climate_nodes[i];
    if (node->records[node->current].timestamp == 0) // nothing has been logged yet
      continue;

    climate_node_record_t *current_record =  &(node->records[node->current]);
    // written record is either record[0] if current==1
    //                       or record[1] if current==0 => (current + 1) % 2
    climate_node_record_t *written_record =  &(node->records[(node->current + 1) % 2]);

    // write entry every 10 minutes at least
    // this covers the case "nothing logged since boot" too
    if ( current_record->timestamp > written_record->timestamp &&
         (current_record->timestamp - written_record->timestamp) > 600000)
    {
      CLIMATEDEBUG("write regular record after every 10 minutes %lu - %lu\n",
                   current_record->timestamp,
                   written_record->timestamp
                  );
      write_entry(node);
      continue;
    }
    // test if temperature changed by at least 0.2 Degree or
    // if humidity at least by 0.5 %
    // if so, write logentry with shorter frequency than 10 minutes
    if ( abs(current_record->temp - written_record->temp) > 1
      || abs(current_record->humid - written_record->humid) > 4)
    {
      CLIMATEDEBUG("write in between record due to dramatic ;-) value difference\n");
      write_entry(node);
      continue;
    }
  }
  return 0;
}

int8_t
write_entry(climate_node_t* node)
{
  climate_node_record_t *current_record =  &(node->records[node->current]);
  char datastring[20];
  char *strptr = &datastring[0];

  int8_t len = snprintf_P(strptr, 9, PSTR("[%06d]"), node->uid);
  len += itoa_fixedpoint(current_record->temp, 1, strptr+len, 5);
  datastring[len++] = ';';
  len += itoa_fixedpoint(current_record->humid, 1, strptr+len, 5);
  datastring[len++] = '\n';
  datastring[len] = '\0';
  CLIMATEDEBUG("log record: %s", strptr);
  filelogger_log(strptr, len, current_record->timestamp);
  node->current = (node->current + 1) % 2;
  return 0;
}

static void
update_local_node(void)
{
  if (dht_sensors[0].humid == 0)
    return; // no plausible value
  uint8_t current = climate_nodes[0].current;
  climate_nodes[0].records[current].humid = dht_sensors[0].humid;
  climate_nodes[0].records[current].temp = dht_sensors[0].temp;
  climate_nodes[0].records[current].timestamp = clock_get_time();
}

/*
 * this function is called periodically
 */
int8_t
climate_recorder_write_entry(void)
{
  // extend (max possible) delay
  if(interval > 0 )
  {
    interval--;
    return 0;
  }
  interval = CLIMATE_LOGGING_INTERVAL * HZ / 300;
  if (!is_init)
  {
    if(climate_recorder_init())
    {
      CLIMATEDEBUG("init not successful yet\n");
      return 1;
    }
  }

  if (dht_sensors[0].humid == 0)
  {
    CLIMATEDEBUG("no plausible value\n");
    return 1;
  }

  update_local_node();
  climate_recorder_update();

  /*
  char datastring[20];
  char *strptr = &datastring[0];

  int8_t len = itoa_fixedpoint(dht_sensors[0].temp, 1, strptr, 5);
  datastring[len++] = ';';
  len = len + itoa_fixedpoint(dht_sensors[0].humid, 1, strptr+len, 5);
  datastring[len++] = '\n';
  datastring[len] = '\0';

  filelogger_log(strptr, len);
  */

  return 0;
}

/*
  -- Ethersex META --
  header(services/climaterecorder/climate.h)
  timer(300, climate_recorder_write_entry())
*/
