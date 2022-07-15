/** @file MZ_version.h
 * @date Aug 16, 2021
 * @author SKM
 * @brief This file contains the API for compare the versions of two inputs
 */

#ifndef MZ_VERSION_H_
#define MZ_VERSION_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "MZ_error_handler.h"
#include "MZ_common.h"

/** @struct mz_version
 * @brief Monoz Versions
 */
typedef struct
{
	mzUint8 _major;						/*!< Major version */
	mzUint8 _minor;						/*!< Minor version */
	mzUint8 _patch;						/*!< patch version */
}mz_version;


// Input both versions to compare and return the result
// MZ_OK / MZ_INVALID_VERSION
/**
 * @defgroup VERSION VERSION
 * Checking version
 * @{
 *
 * @fn mz_error_t mz_version_check(mz_version * _in1,mz_version * _in2)
 * @brief comparing the two versions
 * @param _in1 mz_version
 * @param _in2 mz_version
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_version_check(mz_version * _in1,mz_version * _in2);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_VERSION_H_ */
