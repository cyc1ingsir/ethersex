/*
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
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

#ifndef HAVE_CLIMATE_H
#define HAVE_CLIMATE_H

int8_t
climate_recorder_write_entry(void);

uint8_t interval;

#include "config.h"
#ifdef DEBUG_CLIMATE_RECORDER
# include "core/debug.h"
# define CLIMATEDEBUG(a...)  debug_printf("climate recorder: " a)
#else
# define CLIMATEDEBUG(a...)
#endif

#endif  /* HAVE_CLIMATE_H */
