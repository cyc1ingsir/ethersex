/*
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
 * Copyright (c) 2015 by Meinhard Ritscher   <unreachable@gmx.net>
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
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "filelogger.h"
#include "protocols/ecmd/ecmd-base.h"


int16_t parse_cmd_ls(char *cmd, char *output, uint16_t len)
{
  return filelogger_init( "xzy" ) == 0 ?
          ECMD_FINAL_OK : ECMD_FINAL(snprintf_P(output, len, PSTR("logger init failed")));
}

int16_t parse_cmd_log(char *cmd, char *output, uint16_t len)
{
  return filelogger_log ("Sample log entry\n", 17) == 0 ?
          ECMD_FINAL_OK : ECMD_FINAL(snprintf_P(output, len, PSTR("no valid file handle?")));
}

int16_t parse_cmd_closelog(char *cmd, char *output, uint16_t len)
{
  return filelogger_close ()== 0 ?
          ECMD_FINAL_OK : ECMD_FINAL(snprintf_P(output, len, PSTR("file not open?")));
}

/*
-- Ethersex META --
block([[File_Logger]])
ecmd_feature(log, "log",, write dummy log entry)
ecmd_feature(ls, "ls",, inits and creates log file plus directory)
ecmd_feature(closelog, "closelog",, closes log file)
*/

