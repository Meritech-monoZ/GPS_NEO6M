/** 
 * @file MZ_Modem_public.h
 *  @date Apr 23, 2021
 *  @author SKM
 * @brief Monoz Modem public Protocol
 */

#ifndef MZ_AT_INIT_H_
#define MZ_AT_INIT_H_									///< Monoz AT Initialization
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_error_handler.h"
#include "MZ_Common.h"


/** @struct st_att
 * @brief Structure for a Single AT Cmd
 */
typedef struct
{
	mzInt8		*_cmd;															/*!< Individual AT CMD */
	mzInt32		_tO;															/*!< Time out set for the AT CMD */
	mzInt32		_delay;															/*!< Mandatory Waiting period after this AT CMD finished */
}st_att;

/** 
 * @defgroup MODEM MODEM 
 * Monoz Modem
 * @{
 * 
 * @fn mz_error_t mz_send_AT(void * arg)
 * @brief Send AT command
 * @param arg void *
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_send_AT(void * arg);

/** 
 * @fn mz_error_t MZ_init_cmd_direct(char * dbuffer, mzInt32 to, mzInt32 twn)
 * @brief Initialize AT Command
 * @param dbuffer char *
 * @param to mzInt32
 * @param twn mzInt32
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t MZ_init_cmd_direct(char * dbuffer, mzInt32 to, mzInt32 twn);

/** 
 * @fn mz_error_t MZ_lwm2m_cmd_direct(char * dbuffer, mzInt32 to)
 * @brief Initialize LWM2M Command
 * @param dbuffer char *
 * @param to mzInt32
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t MZ_lwm2m_cmd_direct(char * dbuffer, mzInt32 to);

/** @fn mz_error_t mz_at_core_init_complete(void)
 * @brief Initialize the Monoz AT Core completely
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_at_core_init_complete(void);

/** @fn mz_error_t mz_at_set_init_count(int in)
 * @brief Set the Initial Count for monoz AT
 * @param in int
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_at_set_init_count(int in);

/** @fn void mz_at_set_at_debug_enable(int in)
 * @brief Debug enable for monoz AT set
 * @param in int
 */
void mz_at_set_at_debug_enable(int in);

/** @fn int mz_at_get_at_debug_enable(void)
 * @brief Debug enable to get the Monoz AT
 * @return 1/0
 */
int mz_at_get_at_debug_enable(void);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_AT_INIT_H_ */
