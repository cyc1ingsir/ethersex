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
#include "services/clock/clock.h"

#include "filelogger.h"


struct vfs_file_handle_t *filelogger_handle;
uint8_t previous_day;
char * folder_name;

/*
 * returns 0 on success
 */
static uint8_t
check_open_file(clock_datetime_t *date)
{
  if (filelogger_handle)
  {
    uint16_t mDate = filelogger_handle->u.sd->dir_entry.modification_date;
    uint8_t mDay = (mDate >> 0) & 0x1f;
    uint8_t mMonth = (mDate >> 5) & 0x0f;
    if (date->day == mDay)
    {
      return 0;
    }
    else
    {
      // file was modified some other month than
      // the current - truncate and reopen.
      if (mMonth != date->month)
      {
        vfs_fseek_truncate_close(1, filelogger_handle, 0, SEEK_SET);
      }
      filelogger_close();
      return check_open_file(date);    // if close was successful
    }
  }
  uint8_t len = sizeof(char) * 9 + strlen(folder_name);
  char *filename = malloc(len);
  snprintf_P(filename, len, PSTR("/%s/%02d.txt"), folder_name, date->day);
  if ((filelogger_handle = vfs_sd_open(filename)) == NULL)
  {
    if ((filelogger_handle = vfs_sd_create(filename)) == NULL)
    {
      LGRDEBUG("could not open file");
      free(filename);
      return 1;
    }
    timestamp_t current = clock_get_time();
    // write to file
#define TIMESTAMP_LEN 33
    char timestamp[TIMESTAMP_LEN];
    char *strptr = &timestamp[0];
    snprintf_P(strptr, TIMESTAMP_LEN, PSTR("#%u\n"), current);
    vfs_sd_write(filelogger_handle, timestamp, strlen(strptr));
  }
  LGRDEBUG("opened new log file for today\n");
  free(filename);
  return 0;
}

/*
 * returns 0 if last call of this function was
 *           made on the same day
 *         1 if the last call was made on any
 *           other day
 */
static uint8_t
is_current(clock_datetime_t *date)
{
  clock_current_localtime(date);
  if (date->day != previous_day)
  {
    previous_day = date->day;
    return 1;
  }
  return 0;
}

/*
 * initially setup the logger
 * returns 0 on success
 * This function may be called only once!
 */
uint8_t
filelogger_init(const char *dirname)
{

  if (vfs_sd_rootnode == 0){
    LGRDEBUG("sd reader not initialized?\n");
    return 1;
  }

  // TODO check that vfs_sd_rootnode is not dirname already ....
  // this might work without dir_node!
  struct fat_dir_struct *dir_node;
  if ((dir_node = vfs_sd_chdir(dirname)) == NULL)
  {
    vfs_sd_mkdir_recursive(dirname);
    if ((dir_node = vfs_sd_chdir(dirname)) == NULL)
    {
      LGRDEBUG("unable to create folder?\n");
      return 2;
    }
  }
  folder_name = strdup(dirname);
  if(folder_name == NULL)
  {
    LGRDEBUG("can't store folder name to RAM!\n");
  }

  //fat_reset_dir(vfs_sd_rootnode);

  if (!clock_last_sync())
  {
    // can't open the file until the the system clock is in sync
    // we do not have the current day yet
    // the file will be opened when filelogger_log is called
    LGRDEBUG("logfile not opened yet but logging will care about that later\n");
    return 0;
  }

  clock_datetime_t date;
  if(is_current(&date))
  {
    check_open_file(&date);
  }
  return 0;
}

/**
 *
 */
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

/**
 * log entry with a leading current timestamp
 * filelog_init needs to be called ONCE prior to using
 * the filelogger
 */
uint8_t
filelogger_log(char *entry, uint16_t len)
{

  if (!clock_last_sync())
  {
    // do not log anything with invalid timestamp
    return 1;
  }

  clock_datetime_t date;
  if(is_current(&date))
  {
    check_open_file(&date);
  }

  // just to make sure
  if (filelogger_handle)
  {

#define TIMESTRING_LEN 12
    uint8_t len = len + TIMESTRING_LEN;
    char line[len];
    char *strptr = &line[0];
    snprintf_P(strptr, len, PSTR("[%02d:%02d:%02d] %s"),
               date.hour,
               date.min,
               date.sec,
               entry
              );

    vfs_sd_fseek(filelogger_handle, 0, SEEK_END);
    vfs_sd_write(filelogger_handle, line, len);
    sd_raw_sync();
  }
  else
  {
    LGRDEBUG("Error: filelogger_handle not valid\n");
    return 2;
  }

  return 0;
}
