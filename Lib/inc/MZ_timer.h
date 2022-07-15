/** @file MZ_timer.h
 *  @date Apr 27, 2021
 *  @author SKM
 * @brief Monoz Timer
 * This file contains APIs for create,delete,start and stop of timers
 */


#ifndef MZ_TIMER_H_
#define MZ_TIMER_H_																///< Define Monoz Timer
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_error_handler.h"
#include "MZ_sys_cmsis_os2.h"

// Time MACROS
#define	AT_TIME_1SEC		(1000)													///< Define 1 Second AT command Time
#define	AT_TIME_2SEC		(AT_TIME_1SEC * 2)										///< Define 2 Second AT command Time
#define	AT_TIME_5SEC		(AT_TIME_1SEC * 5)										///< Define 5 Second AT command Time
#define	AT_TIME_10SEC		(AT_TIME_1SEC * 10)										///< Define 10 Second AT command Time
#define	AT_TIME_15SEC		(AT_TIME_1SEC * 15)										///< Define 15 Second AT command Time
#define	AT_TIME_20SEC		(AT_TIME_1SEC * 20)										///< Define 20 Second AT command Time
#define	AT_TIME_25SEC		(AT_TIME_1SEC * 25)										///< Define 25 Second AT command Time
#define	AT_TIME_30SEC		(AT_TIME_1SEC * 30)										///< Define 30 Second AT command Time
#define	AT_TIME_1MIN		(AT_TIME_1SEC * 60)										///< Define 60 Second AT command Time
#define	AT_TIME_2MIN		(AT_TIME_1MIN * 2)										///< Define 2 Min AT command Time
#define	AT_TIME_5MIN		(AT_TIME_1MIN * 5)										///< Define 5 Min Second AT command Time
#define	AT_TIME_10MIN		(AT_TIME_1MIN * 10)										///< Define 10 Min AT command Time

/** @enum en_mz_tm
 * @brief   Enumeration for Monoz Timer
 */
typedef enum
{
	MZ_TM_EMPTY,				/*!< Monoz timer empty */
	MZ_TM_START,				/*!< Monoz timer start */
	MZ_TM_RUNNING,				/*!< Monoz timer running */
	MZ_TM_STOP,					/*!< Monoz timer stop */
}en_mz_tm;

#define MZ_TIMER_NOT_AVAILABLE				(0xFFFF)								///< Defines Availability of Monoz Timer

typedef mz_timer_fp	mz_tm_cb;														/*!< mz_tm_cb variable of type mz_timer_fp */


/** 
 * @defgroup TIMER TIMER
 * Timer operations
 * @{
 * @fn size_t mz_tm_create(char* _name,TickType_t _tick,UBaseType_t _auto,mz_tm_cb _cb)
 * @brief Create Monoz Timer
 * @param _name char
 * @param _tick TickType_t
 * @param _auto UBaseType_t
 * @param _cb mz_tm_cb
 * @return function returns _id
 */ 
size_t mz_tm_create(char* _name,TickType_t _tick,UBaseType_t _auto,mz_tm_cb _cb);

/** 
 * @fn size_t mz_tm_create_one(char* _name,TickType_t _tick,mz_tm_cb _cb)
 * @brief Create a one shot timer
 * @param _name char
 * @param _tick TickType_t
 * @param _cb mz_tm_cb
 * @return function returns mz_tm_create(_name, _tick, pdFALSE, _cb)
 */ 
size_t mz_tm_create_one(char* _name,TickType_t _tick,mz_tm_cb _cb);


/** @fn size_t mz_tm_create_recursive(char* _name,TickType_t _tick,mz_tm_cb _cb)
 * @brief Create a recursive timer
 * @param _name char
 * @param _tick TickType_t
 * @param _cb mz_tm_cb
 * @return function returns mz_tm_create(_name, _tick, pdTRUE, _cb)
 */ 
size_t mz_tm_create_recursive(char* _name,TickType_t _tick,mz_tm_cb _cb);


/** @fn mz_error_t mz_tm_start(size_t _id)
 * @brief Start the timer
 * @param _id size_t
 * @return function returns _ret
 */ 
mz_error_t mz_tm_start(size_t _id);


/** @fn mz_error_t mz_tm_create_start(char* _name,TickType_t _tick,UBaseType_t _auto,mz_tm_cb _cb)
 * @brief Create and Start the timer
 * @param _name char
 * @param _tick TickType_t
 * @param _auto UBaseType_t
 * @param _cb mz_tm_cb
 * @return function returns MZ_FAIL
 */ 
mz_error_t mz_tm_create_start(char* _name,TickType_t _tick,UBaseType_t _auto,mz_tm_cb _cb);


/** @fn mz_error_t mz_tm_create_start_one(char* _name,TickType_t _tick,mz_tm_cb _cb)
 * @brief Create and Start the one shot timer
 * @param _name char
 * @param _tick TickType_t
 * @param _cb mz_tm_cb
 * @return function returns mz_tm_create_start(_name, _tick, pdFALSE, _cb) value
 */ 							
mz_error_t mz_tm_create_start_one(char* _name,TickType_t _tick,mz_tm_cb _cb);


/** @fn mz_error_t mz_tm_create_start_recursive(char* _name,TickType_t _tick,mz_tm_cb _cb)
 * @brief Create and Start the recursive timer
 * @param _name char
 * @param _tick TickType_t
 * @param _cb mz_tm_cb
 * @return function returns the value mz_tm_create_start(_name, _tick, pdTRUE, _cb)
 */ 
mz_error_t mz_tm_create_start_recursive(char* _name,TickType_t _tick,mz_tm_cb _cb);


/** @fn mz_error_t mz_tm_stop(size_t _id)
 * @brief Stop the timer
 * @param _id Size_t
 * @return function returns the result of pdPASS != xTimerStop(_tmr[_id].timer,0) ? MZ_FAIL : MZ_OK
 */ 
mz_error_t mz_tm_stop(size_t _id);


/** @fn mz_error_t mz_tm_delete(size_t _id)
 * @brief Delete the timer
 * @param _id Size_t
 * @return function returns the result of pdPASS != xTimerDelete(_tmr[_id].timer,0) ? MZ_FAIL : MZ_OK
 */
mz_error_t mz_tm_delete(size_t _id);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_TIMER_H_ */
