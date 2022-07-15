/** @file  MZ_flash.h
 *  @date Aug 2, 2021
 *  @author SKM
 *  @brief This file contains the Monoz flash structure and also responsible for flash initialization,
 *  flash storage and flash erase
 */

#ifndef MZ_FLASH_H_
#define MZ_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "MZ_BoardCfg.h"
#include "MZ_common.h"
#include "MZ_sys_cmsis_os2.h"
#include "MZ_error_handler.h"
#if(MZ_FLASH_DRIVER_ENABLE == 1)

/**
 * @enum en_mz_fl_op
 * @brief Enables the Monoz flash operations
 */
typedef enum
{
	MZ_FL_NOT_READY,												/*!< Monoz flash driver is not Ready */
	MZ_FL_READY,													/*!< Monoz flash driver is Ready */
	MZ_FL_WRITE,													/*!< Monoz flash write */
}en_mz_fl_op;

/**
 * @struct st_mz_flash
 * @brief Structure for Monoz Flash
 */
typedef struct
{
	mzUint32			base_addr;												/*!< Base address of managed flash */
	mzUint32			no_of_bank_pages;										/*!< Number of pages */
	mzUint32			bank_number;											/*!< Bank number of the address */
	mzUint32			page_number;											/*!< page number of the address */
	en_mz_fl_op			current_op;												/*!< Current operation */
}st_mz_flash;

/**
 * @defgroup MONOZ_FLASH MONOZ FLASH
 * Monoz Flash APIs
 * @{
 *
 * @fn mz_error_t mz_f_init(st_mz_flash * _ctx, mzUint32 _addr, mzUint32 _n_page)
 * @brief Monoz flash initialization
 * @param _ctx *st_mz_flash
 * @param _addr mzUint32
 * @param _n_page mzUint32
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_f_init(st_mz_flash * _ctx, mzUint32 _addr, mzUint32 _n_page);

/** @fn mz_error_t mz_f_store(st_mz_flash * _ctx, mzUint32 Write_addr,const void * data, mzUint32 size)
 * @brief Monoz flash storage
 * @param _ctx *st_mz_flash
 * @param Write_addr mzUint32
 * @param data const void*
 * @param size mzUint32
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_f_store(st_mz_flash * _ctx, mzUint32 Write_addr,const void * data, mzUint32 size);

/** @fn mz_error_t mz_f_erase_full(st_mz_flash * _ctx)
 * @brief Fully erase Monoz flash
 * @param _ctx st_mz_flash
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_f_erase_full(st_mz_flash * _ctx);

// relative page_no - start with 0 for 1st page
/** @fn mz_error_t mz_f_erase_ctx_relative_page_no(st_mz_flash * _ctx, mzUint32 page_no)
 * @brief Erase relative pages in the Monoz flash
 * @param _ctx st_mz_flash
 * @param page_no mzUint32
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_f_erase_ctx_relative_page_no(st_mz_flash * _ctx, mzUint32 page_no);
/** @} */
#endif //(MZ_FLASH_DRIVER_ENABLE == 1)

#ifdef __cplusplus
}
#endif
#endif /* MZ_FLASH_H_ */
