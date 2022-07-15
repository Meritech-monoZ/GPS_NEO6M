/**
  * @file     MZ_common.h
  * @author   SKM
  * @version  V1.0
  * @date     Sep 15, 2020
  * @brief    Monoz Lib Common.
*/


#ifndef MZ_COMMON_H
#define MZ_COMMON_H                                                       ///< Define Monoz Common
#ifdef __cplusplus
extern "C" {
#endif

#define MZ_IFNULLGOCLEAN(X) do{if(0 == X) goto clean;}while(0);           ///< Monoz If Null Go and Clean
#define MZ_IFNOTNULLGOCLEAN(X) do{if(0 != X) goto clean;}while(0);        ///< Monoz If Not Null Go and Clean
#define MZ_IFRETNOTMZ_OKGOCLEAN(X) \
	do{_ret = X; if(MZ_OK != _ret)goto clean;}while(0);             ///< Monoz if not return MZ_OK Go and Clean

#define __INTERNAL	static
#define __EXTERNAL

#include "stm32l4xx_hal.h"
#include "MZ_BoardCfg.h"
#include "MZ_tool_config.h"
#include "stddef.h"
//#include "task.h"
#include "string.h"


typedef void (*_mz_fp)(void *);                                           /*!< Common reference function pointer */                             

/** @brief 1 Byte data type 
 * 
 * \a mz_fn variable of type \a _mz_fp 
 */ 
typedef _mz_fp	mz_fn;                                                                                           

/** @brief 1 Byte data type
 * 
 * \a mzInt8 variable of type \a char
 */
typedef char 					mzInt8;

/** @brief 1 Byte data type 
 * 
 * \a mzSint8 variable of type \a signed \a char
 */
typedef signed char				mzSint8;

/** @brief 1 Byte data type 
 * 
 * \a mzUint8 variable of type \a unsigned \a char 
 */
typedef unsigned char			mzUint8;


/** @brief 2 Byte data type 
 * 
 * \a mzInt16 variable of type \a short \a int
 */
typedef short int				mzInt16;

/** @brief 2 Byte data type
 * 
 * \a mzSint16 variable of type \a signed \a short \a int
 */
typedef signed short int		mzSint16;

/** @brief 2 Byte data type
 * 
 * \a mzUint16 variable of type \a unsigned \a short \a int 
 */
typedef unsigned short int		mzUint16;


/** @brief 4 Byte data type
 * 
 * \a mzInt32 variable of type \a long \a int 
 */
typedef long int				mzInt32;

/** @brief 4 Byte data type
 * 
 * \a mzSint32 variable of type \a signed \a long \a int 
 */
typedef signed long int			mzSint32;

/** @brief 4 Byte data type
 * 
 * \a mzUint32 variable of type \a unsigned \a long \a int 
 */
typedef unsigned long int		mzUint32;


/* 8 Byte data type */
/** @brief 8 Byte data type
 * 
 * \a mzInt64 variable of type \a long \a long
 */
typedef long long				mzInt64;

/** @brief 8 Byte data type
 * 
 * \a mzSint64 variable of type \a signed \a long \a long
 */
typedef signed long long		mzSint64;


/** @brief 8 Byte data type
 * 
 * \a mzUint64 variable of type \a unsigned \a long \a long
 */
typedef unsigned long long		mzUint64;

/**
 * @enum mzReg
 * @brief Monoz Register and Deregister
 */ 
typedef enum
{
	MZ_DEREG,                             /*!< Monoz Deregister */
	MZ_REG,                               /*!< Monoz Register */
}mzReg;

//#include "MZ_public.h"

#define MZ_VAR_NOINIT 	__attribute__((section(".mz_RamBlock_noinit")))   ///< Defines the mz_RamBlock_noinit
#define MZ_VAR_INIT 	__attribute__((section(".mz_RamBlock_init")))       ///< Defines the mz_RamBlock_init
#define MZ_VAR_CONST 	__attribute__((section(".mz_const_section")))       ///< Defines monoz constant variable
#define MZ_FUN 			__attribute__((section(".mz_FunctionBlock")))         ///< Defines the monoz function
#define MZ_VAR_CONFIG 	__attribute__((section(".mz_config_table")))      ///< Defines the monoz configuration table

#ifdef __cplusplus
}
#endif
#endif
