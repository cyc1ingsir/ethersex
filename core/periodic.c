/*
 *
 * Copyright (c) by Alexander Neumann <alexander@bumpern.de>
 * Copyright (c) by Stefan Siegl <stesie@brokenpipe.de>
 * Copyright (c) by David Gräff <david.graeff@web.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License (either version 2 or
 * version 3) as published by the Free Software Foundation.
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

#include <avr/interrupt.h>

#include "core/periodic.h"
#include "config.h"
#include "network.h"
#include "core/debug.h"
#include "protocols/uip/uip.h"
#include "protocols/uip/uip_arp.h"
#include "protocols/uip/uip_neighbor.h"
#include "protocols/uip/uip_router.h"
#include "protocols/ecmd/via_i2c/ecmd_i2c.h"
#include "control6/control6.h"
#include "hardware/radio/fs20/fs20.h"
#include "services/watchcat/watchcat.h"
#include "services/clock/clock.h"
#include "services/cron/cron.h"
#include "services/cron/cron_static.h"
#include "protocols/uip/ipv6.h"
#include "hardware/input/ps2/ps2.h"
#include "hardware/radio/rfm12/rfm12.h"
#include "protocols/syslog/syslog.h"
#include "mcuf/mcuf.h"
#include "protocols/usb/usb.h"
#include "protocols/modbus/modbus.h"
#include "protocols/zbus/zbus.h"
#include "protocols/mysql/mysql.h"
#include "services/jabber/jabber.h"

#ifdef BOOTLOADER_SUPPORT
uint8_t bootload_delay = CONF_BOOTLOAD_DELAY;
#endif

void periodic_init(void)
{

    /* init timer1 to expire after ~20ms, with CTC enabled */
    TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
    OCR1A = (F_CPU/1024/50);

}

/*
  -- Ethersex META --
  header(core/periodic.h)
  init(periodic_init)
*/
