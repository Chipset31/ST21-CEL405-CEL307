/*
 * sdcard_logging.h
 *
 *  Created on: Aug 1, 2021
 *      Author: User
 */

#ifndef INC_SDCARD_LOGGING_H_
#define INC_SDCARD_LOGGING_H_

#include "ff.h"
#include "stm32l4xx_hal_rtc.h"
#include <canard.h>
//#include "fatfs.h"

/*
 *	Publicly accessible defines for filenames, file extensions and file paths on the SD card
 *	Feel free to modify these according to preference
 */
#define LOG_FILENAME "log.txt"
#define LOG_FILEPATH ""				//Empty string is handled as root directory, see FatFS docs for info on valid file paths

#define LOG_MAX_FILESIZE 2500000		//in bytes, if the logfile exceeds this size a new file will be created. (not currently implemented)


FRESULT SDLogInit(const char* drivePath);
FRESULT SDLogDeInit(void);
FRESULT SDLogWrite(const char* message);
FRESULT SDLogSampleScalar(const CanardTransfer* const transfer, const TCHAR* name, float value, uint64_t timestamp);
FRESULT SDLogSampleScalarToCSV(const CanardTransfer* const transfer, const TCHAR* name, float value, uint64_t timestamp);



#endif /* INC_SDCARD_LOGGING_H_ */
