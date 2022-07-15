/**
 * @file MZ_main.h
 * @author SKM
 * @date Feb 2 2021 
 * @brief Monoz Main file
 */

#ifndef MZ_MAIN_H_
#define MZ_MAIN_H_                       ///< Defines the Monoz Main Inc file
#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"
#include "stm32l4xx_hal.h"
#include "MZ_version.h"

#define MZ_SW_VERSION_MAJOR			(1)     ///< Defines the major monoz software version
#define MZ_SW_VERSION_MINOR			(0)     ///< Defines the minor monoz software version
#define MZ_SW_VERSION_PATCH			(7)     ///< Defines the patch version


/** @defgroup MAIN MAIN
 * Main function of the Application
 * @{
 *
 * @fn mz_error_t MZ_init(mz_version * _ver)
 * @brief Main function Initialization
 * @param _ver mz_version
 * @return MZ_OK/MZ_FAIL
 */

/*
 * This API is used to Initialize the MonoZ Library.
 *
 * Sample use of Initializing MonoZ Library is as follows.
 *
 *  mz_version ver = {
 *		  ._major = MZ_SW_VERSION_MAJOR,
 *		  ._minor = MZ_SW_VERSION_MINOR,
 *		  ._patch = MZ_SW_VERSION_PATCH
 *  };
 *  if(MZ_OK != MZ_init(&ver))
 *  {
 *	  Error_Handler();
 *  }
 */
mz_error_t MZ_init(mz_version * _ver);

/**
 * @fn mz_error_t MZ_modem_config_set(_mz_fp _fp)
 * @brief Check the modem configuration is set or not set
 * @param _fp _mz_fp
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_modem_config_set(_mz_fp _fp);

/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_MAIN_H_ */
