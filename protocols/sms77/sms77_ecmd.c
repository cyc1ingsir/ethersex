/*
 * Copyright (c) 2009 by Stefan Müller <mueller@cos-gbr.de>
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

#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "sms77.h"

#include "protocols/ecmd/ecmd-base.h"


int16_t parse_cmd_sm (char *cmd, char *output, uint16_t len) 
{
  if (sms77_send(cmd)) 
    return ECMD_FINAL_OK;
  return ECMD_FINAL(snprintf_P(cmd, len, PSTR("sending failed")));
}

/*
  -- Ethersex META --
  block([[SMS77]])
  ecmd_feature(sm, "sms77 ",MESSAGE,Send MESSAGE to compiled in sms77 service)
*/
