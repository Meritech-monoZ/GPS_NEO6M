/**
  * @file     MZ_error_handler.h
  * @author   SKM
  * @date     Mar 22, 2021
  * @brief Monoz Error Handler
*/

#ifndef MZ_ERROR_HANDLER_H_
#define MZ_ERROR_HANDLER_H_													///< Defines Inc Monoz Error Handler
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_tool_config.h"
/**
 * @enum mz_error_t
 * @brief Enumeration of Monoz Error Flags
 */
typedef enum
{
	MZ_OK,																		/*!< OK */
	MZ_FAIL,																	/*!< Generic Fail */

	MZ_CALLBACK_REG_FAIL,														/*!< Failed to initialize the callback functionality */

	// Memory related error flag
	MZ_MEM_ALLOCATION_FAIL,														/*!< Failed to allocate memory */

	// UART related error flags
	MZ_UART_INVALID_NO,															/*!< Invalid UART number */
	MZ_UART_ALREADY_REG,														/*!< UART is already registered */
	MZ_UART_INIT_FAIL,															/*!< UART initialization failed */
	MZ_UART_NOT_REG,															/*!< UART is not registered */

	// RTC related error flags
	MZ_RTC_INIT_FAIL,															/*!< RTC initialization failed */
	MZ_RTC_NOT_REG,																/*!< RTC is not registered */
	MZ_RTC_READ_FAIL,															/*!< RTC Read Failed */
	MZ_RTC_WRITE_FAIL,															/*!< RTC Write Failed */

	// I2C related error flags
	MZ_I2C_INVALID_NO,															/*!< Invalid I2C number */
	MZ_I2C_ALREADY_REG,															/*!< I2C is already registered */
	MZ_I2C_INIT_FAIL,															/*!< I2C initialization failed */
	MZ_I2C_NOT_REG,																/*!< I2C is not registered */

	// System related error flags
	MZ_SEMA_CREATE_FAIL,														/*!< Semaphore creation failed */
	MZ_SEMA_ACQUIRE_FAIL,														/*!< Semaphore acquire failed */
	MZ_MUTEX_CREATE_FAIL,														/*!< Mutex creation failed */
	MZ_MUTEX_ACQUIRE_FAIL,														/*!< Mutex acquire failed */
	MZ_MAILBOX_CREATE_FAIL,														/*!< Mailbox creation failed */
	MZ_THREAD_CREATE_FAIL,														/*!< Thread creation failed */

	// Function related Error flags
	MZ_INVALID_ARGUMENT,														/*!< Invalid Argument */
	MZ_INVALID_SIZE,															/*!< Invalid Size */

	// Initialization Error flags
	MZ_INTERFACE_NOT_DEFINED,													/*!< Interface is not defined */

	// AT cmd related error flags
	MZ_AT_CMD_CREATE_FAIL,														/*!< AT cmd creation failed */
	MZ_AT_CMD_TYPE_WRONG,														/*!< wrong AT cmd type */
	MZ_AT_CMD_NOTSUPPORT,														/*!< AT cmd not supported */

	// AT Core related error flags
	MZ_AT_CORE_INIT_FAILED,														/*!< AT core initialization failed */
	MZ_AT_SLOT_COUNT_ERROR,														/*!< Wrong number of slot configured in software */

	// LWM2M related error flags
	MZ_LW_INVALID_OBJECT,														/*!< Invalid Lwm2m Object reference */
	MZ_LW_INVALID_OBJ_INSTANCE,													/*!< Invalid Lwm2m Object Instance */
	MZ_LW_INVALID_RESOURCE,														/*!< Invalid Lwm2m Resource reference */
	MZ_LW_INVALID_RES_INSTANCE,													/*!< Invalid Lwm2m Resource Instance */
	MZ_LW_INVALID_TOKEN,														/*!< Token not present for the resource */

#if(MZ_FLASH_DRIVER_ENABLE == 1)
	// Flash related error flags
	MZ_FLASH_UNLOCK_FAILED,														/*!< Flash Unlock failed */
	MZ_FLASH_LOCK_FAILED,														/*!< Flash lock failed */
	MZ_FLASH_ADD_OUTSIDE,														/*!< Flash Address outside of MCU */
	MZ_FLASH_PAGE_OUTSIDE_BANK,													/*!< Flash Page outside of BANK */
	MZ_FLASH_ADD_OUTSIDE_CTX,													/*!< Flash Address outside context */
	MZ_FLASH_PAGE_OUTSIDE_CTX,													/*!< Flash Page outside context */
	MZ_FLASH_ERASE_FAILED,														/*!< Flash Erase failed */
	MZ_FLASH_INVALID_WRITE_SIZE,												/*!< Flash Invalid Write Size */
	MZ_FLASH_WRITE_FAILED,														/*!< Flash Write failed */
	MZ_FLASH_NOT_READY,															/*!< Flash Not Ready */
#if(MZ_FLASH_CONFIG_ENABLE == 1)
	MZ_CONFIG_CTX_INIT_FAILED,													/*!< Flash Context Init Failed */
	MZ_CONFIG_PLACEMENT_ERROR,													/*!< Configuration not placed in correct address */
	MZ_CONFIG_INVALID_VERSION,													/*!< Configuration Version is Invalid */

#endif //(MZ_FLASH_CONFIG_ENABLE == 1)
#endif //(MZ_FLASH_DRIVER_ENABLE == 1)

	MZ_INVALID_VERSION,															/*!< Invalid Version */

}mz_error_t;

/** 
 * @defgroup ERROR_HANDLER ERROR HANDLER
 * Monoz Error Handler API
 * @{
 * 
 * @fn const char * mz_error_to_str(mz_error_t e)
 * @brief Monoz Error to String representation
 * @param e mz_error_t
 * @return Error_Message
*/
const char * mz_error_to_str(mz_error_t e);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_ERROR_HANDLER_H_ */
