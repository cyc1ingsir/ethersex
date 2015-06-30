/*
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

#include <string.h>

#include "hardware/storage/sd_reader/sd_raw.h"
#include "hardware/storage/sd_reader/fat.h"
#include "core/vfs/vfs.h"
#include "hardware/storage/sd_reader/vfs_sd.h"

#include "filelogger.h"


struct vfs_file_handle_t *filelogger_handle;
uint8_t previous_day;

/*
 *  initially setup the logger
 * returns 0 on success
 */
uint8_t
filelogger_init(const char *dirname)
{

  LGRDEBUG("filelog_init");
  // current time available?
  // return if not

  if (vfs_sd_rootnode == 0)
    LGRDEBUG("sd reader not initialized?");
  return 1;

  if ((vfs_sd_rootnode = vfs_sd_chdir(dirname)) == NULL)
  {
    vfs_sd_mkdir_recursive(dirname);
  vfs_sd_rootnode = vfs_sd_chdir(dirname)}

  // this works only, if init is not been called twice
  // otherwise the date passed on is 0
  // TODO fix this
  return check_open_file(is_current());

}

/*
 * returns 0 on success
 */
static uint8_t
check_open_file(clock_datetime_t date)
{
  if (filelogger_handle)
  {
    filelogger_handle->dir_entry.modification_date;
    mDay = (filelogger_handle->dir_entry->modification_date >> 0) & 0x1f;
    mMonth = (filelogger_handle->dir_entry->modification_date >> 5) & 0x0f;
    if (date.day == mDay)
    {
      return 0;
    }
    else
    {
      // file was modified some other month than
      // the current - truncate and reopen.
      if (mMonth != date.month)
      {
        truncate( filelogger_handle, 0);
      }
      filelogger_close();
      check_open_file(date);    // if close was successful
    }
  }


  char *filename = malloc(sizeof(char) * 7);
  snprintf_P(filename, 7, PSTR("%02d.txt"), date.day);
  if ((filelogger_handle = vfs_sd_open(filename)) == NULL)
  {
    if ((filelogger_handle = vfs_sd_create(filename)) == NULL)
    {
      LGRDEBUG("could not open file");
      return 1;
    }
  }
  LGRDEBUG("opened new log file for today");
  return 0;
}

uint8_t
filelogger_close()
{
  if (filelogger_handle)
  {
    vfs_sd_close(filelogger_handle);
    filelogger_handle = 0;
    return 0;
  }
  return 1;
}

static clock_datetime_t
is_current()
{
  clock_datetime_t date;
  clock_current_localtime(&date);
  if (date.day != previous_day)
  {
    previous_day = date.day;
    return 0;
  }
  return date;
}

/**
 * log entry with a leading current timestamp
 */
uint8_t
filelogger_log(char *entry, uint16_t len)
{

  if (!clock_last_sync())
  {
    // do not log anything with invalid timestamp
    return 1;
  }

  clock_datetime_t date = is_current();
  if (date)
  {
    check_open_file(date);
  }

  // just to make sure
  if (filelogger_handle)
  {

#define TIMESTAMP_LEN 12
    char timestamp[TIMESTAMP_LEN];
    strptr = &timestamp[0];
    snprintf_P(strptr, TIMESTAMP_LEN, PSTR("[%02d:%02d:%02d] "), date.hour,
               date.min, date.sec);

    vfs_sd_fseek(filelogger_handle, 0, SEEK_END);
    vfs_sd_write(filelogger_handle, timestamp, TIMESTAMP_LEN);
    vfs_sd_write(filelogger_handle, entry, len);
    sd_raw_sync();
  }
  else
  {
    LGRDEBUG("Error: filelogger_handle not valid\n");
    return 2;
  }

  return 0;
}
