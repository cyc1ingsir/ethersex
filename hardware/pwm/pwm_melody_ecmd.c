/*
 * Copyright (c) 2009 by Christian Dietrich <stettberger@dokucode.de>
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
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

#include "config.h"
#include "core/debug.h"
#include "pwm_melody.h"
#include <stdlib.h>

#include "protocols/ecmd/ecmd-base.h"


#ifdef PWM_MELODY_SUPPORT

int16_t
parse_cmd_pwm_melody_play(char *cmd, char *output, uint16_t len)
{
  uint8_t song = atoi(cmd);
  pwm_melody_init(song);
  return ECMD_FINAL_OK;
}

#endif  /* PWM_SUPPORT */
