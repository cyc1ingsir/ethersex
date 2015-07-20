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

  char datastring[20];
  char *strptr = &datastring[0];

  if (dht_sensors[0].humid == 0)
  {
    CLIMATEDEBUG("no plausible value\n");
    return 1;
  }

  int8_t len = itoa_fixedpoint(dht_sensors[0].temp, 1, strptr, 5);
  datastring[len++] = ';';
  len = len + itoa_fixedpoint(dht_sensors[0].humid, 1, strptr+len, 5);
  datastring[len++] = '\n';
  datastring[len] = '\0';

  filelogger_log(strptr, len);

  return 0;
}

/*
  -- Ethersex META --
  header(services/climaterecorder/climate.h)
  timer(300, climate_recorder_write_entry())
*/
