/*
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
 * Copyright (c) 2015 by M. Ritscher <unreachable@gmx.net>
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

#include <avr/io.h>

#include "config.h"
#include "climate.h"
#include "protocols/ecmd/ecmd-base.h"


int16_t parse_cmd_climate_recorder_write_entry(char *cmd, char *output, uint16_t len)
{
  climate_recorder_write_entry();
  return ECMD_FINAL_OK;
}


/*
-- Ethersex META --
block([[Climate_Recorder]])
ecmd_feature(climate_recorder_write_entry, "climate_log",, Log a )
*/
