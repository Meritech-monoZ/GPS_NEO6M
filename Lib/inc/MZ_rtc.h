/** @file MZ_rtc.h
 * @date Aug 3, 2021
 * @author SKM
 * @brief This file is responsible for real time clock operations
 */

#ifndef MZ_RTC_H_
#define MZ_RTC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "MZ_error_handler.h"

/**
 * @struct mzd_time
 * Structure for Defining the Date and time for Monoz
 */
typedef struct
{
	char YY;																	/*!< Year */
	char MM;																	/*!< Month */
	char DD;																	/*!< Day */
	char hh;																	/*!< Hour */
	char mm;																	/*!< Minute */
	char ss;																	/*!< Second */
	char zz;																	/*!< +- Time Zone */
}mzd_time;

/** @defgroup RTC RTC
 * Real time clock operations
 * @{
 *
 * @fn mz_error_t MZ_RTC_Init(mzd_time _dt)
 * @brief Monoz real time clock initialization
 * @param _dt mzd_time
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_RTC_Init(mzd_time _dt);

/**
 * @fn mz_error_t MZ_RTC_ReInit(mzd_time _dt)
 * @brief Monoz real time clock Re-initialization
 * @param _dt mzd_time
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_RTC_ReInit(mzd_time _dt);

/**
 * @fn mz_error_t MZ_RTC_read(mzd_time * _dt)
 * @brief Monoz real time clock Read
 * @param _dt mzd_time*
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_RTC_read(mzd_time * _dt);

/**
 * @fn mz_error_t MZ_RTC_set(mzd_time _dt)
 * @brief Monoz real time clock set
 * @param _dt mzd_time*
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_RTC_set(mzd_time _dt);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_RTC_H_ */
