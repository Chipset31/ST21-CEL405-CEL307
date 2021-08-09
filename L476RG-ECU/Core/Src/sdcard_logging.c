/*
 * sdcard_logging.c
 *
 *  Created on: Aug 1, 2021
 *      Author: User
 */

#include "sdcard_logging.h"
#include <stdio.h>
#include <canard.h>

static FATFS sd_fs;
static FIL logfile;



/*
 * Gets RTC time and formats it into ISO8601
 * Includes 3 digits for milliseconds, a space and a null terminator
 * e.g
 * "2021-08-03T17:43:41.000 "
 */
static TCHAR* getRTCTimestamp(void)
{
	static TCHAR ts[25] = {0};

	RTC_TimeTypeDef time = {0};
	RTC_DateTypeDef date = {0};

	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	uint32_t ms = time.SubSeconds * (((float)1) / (time.SecondFraction + 1)) * 1000;

	sprintf(ts, "%u-%02u-%02uT%02u:%02u:%02u.%03lu ",
			(date.Year+2000), date.Month, date.Date,
			time.Hours, time.Minutes, time.Seconds, ms);
	return ts;
}

/**
  * @brief  Init function for SD card logging, mounts the device.
  * @param  drivePath Logical drive path, see FatFS documentation. If null will default to FatFs default drive "".
  * @retval FatFS error code.
  */
FRESULT SDLogInit(const TCHAR* drivePath)
{
	FRESULT res = FR_INVALID_PARAMETER;

	if((drivePath == NULL))
	{
		res = f_mount(&sd_fs, "", 0);
	}
	else
	{
		res = f_mount(&sd_fs, drivePath, 0);
	}

	/*
	res = f_open(&logfile, LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);

	if(res == FR_OK)
	{
		f_printf(&logfile, "\n----------------------- Log Start at %s-----------------------\n", getRTCTimestamp());

		res = f_close(&logfile);
	}
	 */
	return res;
}

FRESULT SDLogDeInit(void)
{
	f_close(&logfile);

	return f_mount(NULL, "0:", 0);
}

FRESULT SDLogWrite(const TCHAR* str)
{
	FRESULT res;
	uint32_t byteswritten;

	res = f_open(&logfile, LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);

	if(res != FR_OK)
	{
		return res;
	}

	f_puts(getRTCTimestamp(), &logfile);

	byteswritten = f_puts(str, &logfile);

	if(byteswritten > 0)
	{
		res = f_close(&logfile);
	}

	return res;
}

/*
 *	@brief  Logs a received message transfer containing a uavcan_si_sample type.
 * 		Function expects to log some measured value of type float and a uint64_t timestamp (in microseconds).
 * 	@param  transfer	The complete received canard transfer containing things which will be logged,
 * 		specifically the local timestamp (µs) on reception, node ID, port ID, transfer ID and payload size (bytes).
 * 	@param	name		A string containing the DSDL name of the received type. Typically defined in the type header.
 * 	@param	value		The value of the data sample.
 * 	@param	timestamp	The timestamp (µs) which was transmitted along with the value, i.e the remote node time when the value was sampled.
 * 	@retval FatFS error code.
 */
FRESULT SDLogSampleScalar(const CanardTransfer* const transfer, const TCHAR* name, float value, uint64_t timestamp)
{
	FRESULT res = FR_INVALID_PARAMETER;
	FIL log;
	TCHAR filename[8] = {0};

	//Log each node ID to a separate file
	sprintf(&filename,"%u.txt", (uint32_t)transfer->remote_node_id);

	res = f_open(&log, filename, FA_OPEN_APPEND | FA_WRITE);
	if(res != FR_OK)
	{
		return res;
	}

	//The f_printf implementation in FatFs doesn't support printing floats ("%f"), so we use sprintf first.
	TCHAR valueString[50] = {0};
	sprintf(valueString, "%f", value);

	f_printf(&log, "%lu\t\tUAVCAN Message transfer received\n", (uint64_t)transfer->timestamp_usec);

	f_printf(&log, "\tNode ID: %u\tSubject ID: %u\tTransfer ID: %u\t\tPayload size: %u bytes\n",
			(uint32_t)transfer->remote_node_id, (uint32_t)transfer->port_id, (uint32_t)transfer->transfer_id, (uint32_t)transfer->payload_size);

	f_printf(&log, "\t%s\tValue:\t%s\t\tTimestamp:\t%lu\n\n", name, valueString, timestamp);

	res = f_close(&log);
	return res;
}


